#include "web_server.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
 
// Mutex for data protection
SemaphoreHandle_t webDataSemaphore = NULL;

// Current data for the web server
WebPageData currentWebData;

// Flag indicating if we have new data to send
volatile bool newDataAvailable = false;

// Reference to the websocket for client communication
AsyncWebSocket* wsPtr = NULL;

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

// Event handler for WebSocket events
void handleWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
            break;
        case WS_EVT_DISCONNECT:
            Serial.printf("WebSocket client #%u disconnected\n", client->id());
            break;
        case WS_EVT_DATA:
            // Handle incoming messages if needed
            if (((AwsFrameInfo*)arg)->final && ((AwsFrameInfo*)arg)->index == 0 && ((AwsFrameInfo*)arg)->len == len && ((AwsFrameInfo*)arg)->opcode == WS_TEXT) {
                data[len] = 0;
                Serial.printf("Received WebSocket message: %s\n", (char*)data);
            }
            break;
        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            break;
    }
}

// Initialize the web server and websocket system
void initWebServer(AsyncWebServer* server, AsyncWebSocket* ws, const char* ssid, const char* password) {
    // Store global reference to websocket
    wsPtr = ws;
    
    // Create the semaphore for data protection if not already created
    if (webDataSemaphore == NULL) {
        webDataSemaphore = xSemaphoreCreateMutex();
        if (webDataSemaphore == NULL) {
            Serial.println("Error creating webDataSemaphore!");
            return;
        }
    }
    
    // Initialize webData with defaults
    if (xSemaphoreTake(webDataSemaphore, portMAX_DELAY) == pdTRUE) {
        memset(&currentWebData, 0, sizeof(WebPageData));
        for (int i = 0; i < 5; i++) {
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
    
    // Set WiFi mode and create Access Point
    WiFi.mode(WIFI_AP);
    delay(100);
    
    if (!WiFi.softAP(ssid, password)) {
        Serial.println("Failed to create WiFi Access Point!");
        return;
    }
    
    Serial.print("WiFi Access Point created. SSID: ");
    Serial.println(ssid);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
    
    // Setup WebSocket handler
    ws->onEvent(handleWebSocketEvent);
    server->addHandler(ws);
    
    // Route for root / web page
    server->on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", index_html);
    });
    
    // Start server
    server->begin();
    Serial.println("HTTP server started");
}

// Variables para control de actualización periódica
TaskHandle_t webUpdateTaskHandle = NULL;
volatile bool forceUpdate = false;

// Función para actualizar datos en el servidor web
void updateWebServerData(const WebPageData& newData) {
    if (webDataSemaphore == NULL) {
        // Crear el semáforo si no existe
        webDataSemaphore = xSemaphoreCreateMutex();
        if (webDataSemaphore == NULL) {
            Serial.println("Error creating webDataSemaphore!");
            return;
        }
    }
    
    // Simplemente actualiza los datos en currentWebData
    if (xSemaphoreTake(webDataSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Actualizar los datos
        currentWebData = newData;
        // Establecer flag para forzar actualización si es necesario (opcional)
        // forceUpdate = true;  // Descomenta si quieres actualizaciones instantáneas en ciertos casos
        xSemaphoreGive(webDataSemaphore);
    }
}

// Tarea para enviar actualizaciones periódicamente
void webUpdateTask(void* parameter) {
    int updateInterval = *((int*)parameter);
    delete (int*)parameter; // Liberar memoria del parámetro
    
    Serial.printf("Web update task started with interval: %d ms\n", updateInterval);
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(updateInterval);
    
    for (;;) {
        // Verificar si hay clientes conectados antes de procesar
        if (wsPtr && wsPtr->count() > 0) {
            if (xSemaphoreTake(webDataSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
                // Preparar JSON
                JsonDocument jsonDoc;
                
                // Añadir datos RC
                auto rcJson = jsonDoc["rc"].to<JsonObject>();
                rcJson["throttle"] = currentWebData.rc.throttle;
                rcJson["yaw"] = currentWebData.rc.yaw;
                rcJson["pitch"] = currentWebData.rc.pitch;
                rcJson["roll"] = currentWebData.rc.roll;
                
                // Añadir canales aux si disponibles
                if (currentWebData.rc.Naux > 0 && currentWebData.rc.aux != nullptr) {
                    auto auxJson = rcJson["aux"].to<JsonArray>();
                    for (int i = 0; i < currentWebData.rc.Naux; i++) {
                        auxJson.add(currentWebData.rc.aux[i]);
                    }
                }
                
                // Añadir datos de sensores
                auto sensorsJson = jsonDoc["sensors"].to<JsonObject>();
                auto distancesJson = sensorsJson["distances"].to<JsonArray>();
                auto statusJson = sensorsJson["status"].to<JsonArray>();
                
                for (int i = 0; i < 5; i++) {
                    distancesJson.add(currentWebData.sensorDistances[i]);
                    statusJson.add(currentWebData.sensorStatus[i]);
                }
                
                // Añadir datos del giroscopio
                auto gyroJson = jsonDoc["gyro"].to<JsonObject>();
                gyroJson["roll"] = currentWebData.gyroRoll;
                gyroJson["pitch"] = currentWebData.gyroPitch;
                gyroJson["yaw"] = currentWebData.gyroYaw;
                
                // Añadir timestamps
                jsonDoc["lastRcUpdate"] = currentWebData.lastRcUpdate;
                jsonDoc["lastSensorUpdate"] = currentWebData.lastSensorUpdate;
                jsonDoc["serverTime"] = millis(); // Tiempo actual del servidor
                
                // Serializar JSON a string
                String jsonString;
                serializeJson(jsonDoc, jsonString);
                
                // Soltar semáforo antes de enviar para no bloquear
                xSemaphoreGive(webDataSemaphore);
                
                // Enviar a todos los clientes
                try {
                    wsPtr->textAll(jsonString);
                    Serial.println("WS data sent");
                } catch (...) {
                    Serial.println("Error sending WebSocket data");
                }
            }
        }
        
        // Esperar hasta el próximo intervalo
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Iniciar tarea de actualizaciones web
void startWebUpdateTask(int updateIntervalMs) {
    // Crear parámetro dinámico (se libera en la tarea)
    int* interval = new int(updateIntervalMs);
    
    // Crear tarea dedicada para actualizaciones web
    xTaskCreatePinnedToCore(
        webUpdateTask,
        "WebUpdateTask",
        4096,               // Stack size
        (void*)interval,    // Pasar intervalo como parámetro
        1,                  // Prioridad baja
        &webUpdateTaskHandle,
        0                   // Core 1 (app core)
    );
}