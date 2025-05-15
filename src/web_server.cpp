#include "web_server.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Web server on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Mutex for data protection
SemaphoreHandle_t webDataSemaphore = NULL;

// Current data for the web server
WebPageData currentWebData;

// Task handle
TaskHandle_t webServerTaskHandle = NULL;

// Flag indicating if we have new data to send
volatile bool newDataAvailable = false;

// WiFi connection timeout (in milliseconds)
const unsigned long WIFI_TIMEOUT = 20000;

// Forward declarations
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void notifyClients();
const char* generateHTML();
void webServerUpdateTask(void* parameter);

// HTML content (will be stored in program memory to save RAM)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>ESP32 Sensor Dashboard</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial, Helvetica, sans-serif; margin: 0; padding: 20px; }
    h1 { color: #333366; }
    .card { background-color: white; box-shadow: 0 4px 8px 0 rgba(0,0,0,0.2); margin: 10px; padding: 15px; border-radius: 5px; }
    .sensor { display: inline-block; margin: 5px; text-align: center; padding: 10px; border: 1px solid #ddd; border-radius: 4px; width: 120px; }
    .sensor.ok { background-color: #dff0d8; }
    .sensor.fail { background-color: #f2dede; }
    .gauge { width: 100px; height: 20px; position: relative; background-color: #f3f3f3; border-radius: 10px; margin: 5px auto; }
    .gauge-fill { height: 100%; border-radius: 10px; background-color: #4CAF50; transition: width 0.3s ease; }
    .rc-value { font-weight: bold; }
    .offline { background-color: #f2dede; padding: 10px; text-align: center; display: none; }
  </style>
</head>
<body>
  <h1>ESP32 Drone Sensor Dashboard</h1>
  
  <div id="offline-alert" class="offline">
    <strong>Connection Lost!</strong> Trying to reconnect...
  </div>
  
  <div class="card">
    <h2>Distance Sensors</h2>
    <div id="sensors-container"></div>
  </div>
  
  <div class="card">
    <h2>Gyroscope</h2>
    <div>
      <p>Roll: <span id="gyro-roll">0.0</span>°</p>
      <p>Pitch: <span id="gyro-pitch">0.0</span>°</p>
      <p>Yaw: <span id="gyro-yaw">0.0</span>°</p>
    </div>
  </div>
  
  <div class="card">
    <h2>RC Controller</h2>
    <div>
      <p>Throttle: <span id="rc-throttle" class="rc-value">0</span>
        <div class="gauge"><div id="throttle-gauge" class="gauge-fill" style="width:0%"></div></div>
      </p>
      <p>Yaw: <span id="rc-yaw" class="rc-value">0</span>
        <div class="gauge"><div id="yaw-gauge" class="gauge-fill" style="width:50%"></div></div>
      </p>
      <p>Pitch: <span id="rc-pitch" class="rc-value">0</span>
        <div class="gauge"><div id="pitch-gauge" class="gauge-fill" style="width:50%"></div></div>
      </p>
      <p>Roll: <span id="rc-roll" class="rc-value">0</span>
        <div class="gauge"><div id="roll-gauge" class="gauge-fill" style="width:50%"></div></div>
      </p>
      <p>Auxiliary channels: <span id="rc-aux">None</span></p>
      <p>Last updated: <span id="rc-lastupdate">Never</span></p>
    </div>
  </div>

  <script>
    const gateway = `ws://${window.location.hostname}/ws`;
    let websocket;
    let lastRcUpdate = 0;
    let lastSensorUpdate = 0;
    let reconnectInterval = null;
    const sensorElements = {};
    const positions = ["LEFT", "FRONT", "RIGHT", "BACK", "BOTTOM"];
    
    // Initialize the sensor elements
    function initSensors() {
      const container = document.getElementById('sensors-container');
      for (const pos of positions) {
        const sensorDiv = document.createElement('div');
        sensorDiv.className = 'sensor';
        sensorDiv.innerHTML = `
          <h3>${pos}</h3>
          <p>Distance: <span id="sensor-${pos}">N/A</span> mm</p>
          <p>Status: <span id="status-${pos}">Unknown</span></p>
        `;
        container.appendChild(sensorDiv);
        sensorElements[pos] = {
          element: sensorDiv,
          value: document.getElementById(`sensor-${pos}`),
          status: document.getElementById(`status-${pos}`)
        };
      }
    }
    
    function initWebSocket() {
      console.log('Trying to open a WebSocket connection...');
      websocket = new WebSocket(gateway);
      websocket.onopen = onOpen;
      websocket.onclose = onClose;
      websocket.onmessage = onMessage;
    }
    
    function onOpen(event) {
      console.log('Connection opened');
      document.getElementById('offline-alert').style.display = 'none';
      if (reconnectInterval) {
        clearInterval(reconnectInterval);
        reconnectInterval = null;
      }
    }
    
    function onClose(event) {
      console.log('Connection closed');
      document.getElementById('offline-alert').style.display = 'block';
      if (!reconnectInterval) {
        reconnectInterval = setInterval(initWebSocket, 2000);
      }
    }
    
    function onMessage(event) {
      try {
        const data = JSON.parse(event.data);
        
        // Update RC data
        if (data.rc) {
          document.getElementById('rc-throttle').textContent = data.rc.throttle;
          document.getElementById('rc-yaw').textContent = data.rc.yaw;
          document.getElementById('rc-pitch').textContent = data.rc.pitch;
          document.getElementById('rc-roll').textContent = data.rc.roll;
          
          // Normalize values for gauges (assuming 1000-2000 range for RC values)
          const normalizeRcValue = (val) => Math.min(Math.max(((val - 1000) / 1000) * 100, 0), 100);
          document.getElementById('throttle-gauge').style.width = `${normalizeRcValue(data.rc.throttle)}%`;
          document.getElementById('yaw-gauge').style.width = `${normalizeRcValue(data.rc.yaw)}%`;
          document.getElementById('pitch-gauge').style.width = `${normalizeRcValue(data.rc.pitch)}%`;
          document.getElementById('roll-gauge').style.width = `${normalizeRcValue(data.rc.roll)}%`;
          
          if (data.rc.aux && data.rc.aux.length > 0) {
            document.getElementById('rc-aux').textContent = data.rc.aux.join(', ');
          } else {
            document.getElementById('rc-aux').textContent = 'None';
          }
          
          lastRcUpdate = new Date().getTime();
          document.getElementById('rc-lastupdate').textContent = new Date().toLocaleTimeString();
        }
        
        // Update sensor data
        if (data.sensors) {
          for (const pos of positions) {
            const index = positions.indexOf(pos);
            if (index >= 0 && index < data.sensors.distances.length) {
              const distance = data.sensors.distances[index];
              const status = data.sensors.status[index];
              
              sensorElements[pos].value.textContent = (distance < 65535) ? distance : 'Out of range';
              sensorElements[pos].status.textContent = status ? 'OK' : 'FAIL';
              sensorElements[pos].element.className = `sensor ${status ? 'ok' : 'fail'}`;
            }
          }
          lastSensorUpdate = new Date().getTime();
        }
        
        // Update gyroscope data
        if (data.gyro) {
          document.getElementById('gyro-roll').textContent = data.gyro.roll.toFixed(2);
          document.getElementById('gyro-pitch').textContent = data.gyro.pitch.toFixed(2);
          document.getElementById('gyro-yaw').textContent = data.gyro.yaw.toFixed(2);
        }
        
      } catch (e) {
        console.error('Error parsing WebSocket message', e);
      }
    }
    
    window.addEventListener('load', (event) => {
      initSensors();
      initWebSocket();
    });
  </script>
</body>
</html>
)rawliteral";

// Helper function to convert sensor position to string
const char* sensorPositionToStringWeb(SensorPosition pos) {
    switch (pos) {
        case LEFT:   return "LEFT";
        case FRONT:  return "FRONT";
        case RIGHT:  return "RIGHT";
        case BACK:   return "BACK";
        case BOTTOM: return "BOTTOM";
        default:     return "UNKNOWN";
    }
}

// Initialize the web server and start the task
void startWebServerTask(const char* ssid, const char* password) {
    // Create the semaphore for data protection
    webDataSemaphore = xSemaphoreCreateMutex();
    if (webDataSemaphore == NULL) {
        Serial.println("Error creating webDataSemaphore!");
        return;
    }
    
    // Initialize webData with defaults
    if (xSemaphoreTake(webDataSemaphore, portMAX_DELAY) == pdTRUE) {
        memset(&currentWebData, 0, sizeof(WebPageData));
        for (int i = 0; i < COUNT_SENSORS; i++) {
            currentWebData.sensorDistances[i] = 0xFFFF; // Max value to indicate no reading
            currentWebData.sensorStatus[i] = false;     // Default to not active
        }
        currentWebData.gyroRoll = 0.0f;
        currentWebData.gyroPitch = 0.0f;
        currentWebData.gyroYaw = 0.0f;
        currentWebData.lastRcUpdate = 0;
        currentWebData.lastSensorUpdate = 0;
        xSemaphoreGive(webDataSemaphore);
    }
    
    // Create the web server task
    xTaskCreatePinnedToCore(
        webServerUpdateTask,    // Function
        "WebServerTask",        // Name
        8192,                   // Stack size (increased for web server)
        (void*)new String[2]{String(ssid), String(password)},  // Parameters: WiFi credentials
        1,                      // Priority
        &webServerTaskHandle,   // Handle
        1                       // Core 1
    );
}

// Task to run the web server and send updates to clients
void webServerUpdateTask(void* parameter) {
    // Get WiFi credentials from parameters
    String* credentials = (String*)parameter;
    String ssid = credentials[0];
    String password = credentials[1];
    delete[] credentials;  // Clean up allocated memory
    
    Serial.println("Web server task started");
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);
    
    // Connect to WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());
    
    // Wait for WiFi connection with timeout
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < WIFI_TIMEOUT) {
        delay(100);
        Serial.print(".");
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nFailed to connect to WiFi. Web server will not start.");
        vTaskDelete(NULL);  // Delete this task
        return;
    }
    
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    
    // Configure WebSocket events
    ws.onEvent(onEvent);
    server.addHandler(&ws);
    
    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", index_html);
    });
    
    // Start server
    server.begin();
    Serial.println("HTTP server started");
    
    // Main task loop
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 100ms update interval
    
    for(;;) {
        // Clean WebSocket clients
        ws.cleanupClients();
        
        // Check if we have new data to send
        if (newDataAvailable) {
            notifyClients();
            newDataAvailable = false;
        }
        
        // Periodic check of WiFi connection
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi disconnected! Attempting to reconnect...");
            WiFi.reconnect();
        }
        
        // Wait for the next update time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Update the web server data from elsewhere in the code
void updateWebServerData(const WebPageData& newData) {
    if (xSemaphoreTake(webDataSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Update the data
        currentWebData = newData;
        newDataAvailable = true;
        xSemaphoreGive(webDataSemaphore);
    }
}

// Notify all connected WebSocket clients with current data
void notifyClients() {
    // Take semaphore to safely access data
    if (xSemaphoreTake(webDataSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Create JSON document
        StaticJsonDocument<1024> jsonDoc;  // Size depends on your data
        
        // Add RC data
        JsonObject rcJson = jsonDoc.createNestedObject("rc");
        rcJson["throttle"] = currentWebData.rc.throttle;
        rcJson["yaw"] = currentWebData.rc.yaw;
        rcJson["pitch"] = currentWebData.rc.pitch;
        rcJson["roll"] = currentWebData.rc.roll;
        
        // Add aux channels if available
        if (currentWebData.rc.Naux > 0 && currentWebData.rc.aux != nullptr) {
            JsonArray auxJson = rcJson.createNestedArray("aux");
            for (int i = 0; i < currentWebData.rc.Naux; i++) {
                auxJson.add(currentWebData.rc.aux[i]);
            }
        }
        
        // Add sensors data
        JsonObject sensorsJson = jsonDoc.createNestedObject("sensors");
        JsonArray distancesJson = sensorsJson.createNestedArray("distances");
        JsonArray statusJson = sensorsJson.createNestedArray("status");
        
        for (int i = 0; i < COUNT_SENSORS; i++) {
            distancesJson.add(currentWebData.sensorDistances[i]);
            statusJson.add(currentWebData.sensorStatus[i]);
        }
        
        // Add gyro data
        JsonObject gyroJson = jsonDoc.createNestedObject("gyro");
        gyroJson["roll"] = currentWebData.gyroRoll;
        gyroJson["pitch"] = currentWebData.gyroPitch;
        gyroJson["yaw"] = currentWebData.gyroYaw;
        
        // Add timestamps
        jsonDoc["lastRcUpdate"] = currentWebData.lastRcUpdate;
        jsonDoc["lastSensorUpdate"] = currentWebData.lastSensorUpdate;
        
        // Serialize JSON to string
        String jsonString;
        serializeJson(jsonDoc, jsonString);
        
        // Send to all clients
        ws.textAll(jsonString);
        
        // Release semaphore
        xSemaphoreGive(webDataSemaphore);
    }
}

// Handle WebSocket events
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
            break;
        case WS_EVT_DISCONNECT:
            Serial.printf("WebSocket client #%u disconnected\n", client->id());
            break;
        case WS_EVT_DATA:
            handleWebSocketMessage(arg, data, len);
            break;
        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            break;
    }
}

// Handle incoming WebSocket messages
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    // For now, we just receive data, no commands from client
    // You could add command handling here if needed
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        data[len] = 0;
        Serial.printf("Received WebSocket message: %s\n", (char*)data);
    }
}