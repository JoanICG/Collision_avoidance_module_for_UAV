#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

// Define I2C pins for ESP32
#define SDA_PIN 21
#define SCL_PIN 22

// Define the cores
#define SENSOR_CORE 0
#define SERVER_CORE 1

// WiFi credentials for Access Point
const char* ssid = "ESP32_MPU6050_Server";
const char* password = "12345678";  // At least 8 characters

// Sensor update rate
#define SENSOR_RATE_MS 20 // 50Hz

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Create MPU6050 instance
MPU6050 mpu;

// Struct to hold all MPU6050 data
struct SensorData {
  // Acceleration
  int16_t ax, ay, az;
  // Gyro
  int16_t gx, gy, gz;
  // Temperature
  float temperature;
  // Calculated angles from accelerometer
  float accel_roll, accel_pitch;
  // Calculated angles with complementary filter
  float roll, pitch, yaw;
  // Calibration status
  bool calibrated;
};

// Shared data between cores
SensorData sensorData;

// Mutex for protecting shared data access
SemaphoreHandle_t xSensorDataMutex;

// Variables for angle calculation
unsigned long lastTime = 0;
float dt = 0;
float gyroScale = 131.0; // For sensitivity +/- 250 degrees/s

// Calibration values
int16_t ax_offset, ay_offset, az_offset;
int16_t gx_offset, gy_offset, gz_offset;

// Function to handle WebSocket events
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  }
}

// HTML content for the web page
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>ESP32 MPU6050 Data</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial, sans-serif; text-align: center; margin: 0px auto; padding: 20px; }
    .card { background-color: #F8F7F9; box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5); 
            padding: 20px; border-radius: 5px; margin-bottom: 20px; }
    .readings { font-size: 1.5rem; }
    canvas { margin-top: 20px; border: 1px solid #ddd; }
    .flex-container { display: flex; flex-wrap: wrap; justify-content: space-around; }
    .sensor-value { margin: 10px; min-width: 150px; }
  </style>
</head>
<body>
  <div class="card">
    <h2>ESP32 MPU6050 Sensor Data</h2>
    <div class="flex-container">
      <div class="sensor-value">
        <h3>Accelerometer</h3>
        <div class="readings">
          <div>X: <span id="accel_x">0</span></div>
          <div>Y: <span id="accel_y">0</span></div>
          <div>Z: <span id="accel_z">0</span></div>
        </div>
      </div>
      <div class="sensor-value">
        <h3>Gyroscope</h3>
        <div class="readings">
          <div>X: <span id="gyro_x">0</span></div>
          <div>Y: <span id="gyro_y">0</span></div>
          <div>Z: <span id="gyro_z">0</span></div>
        </div>
      </div>
      <div class="sensor-value">
        <h3>Angles (deg)</h3>
        <div class="readings">
          <div>Roll: <span id="roll">0</span>°</div>
          <div>Pitch: <span id="pitch">0</span>°</div>
          <div>Yaw: <span id="yaw">0</span>°</div>
        </div>
      </div>
    </div>
    <div class="sensor-value">
      <h3>Temperature</h3>
      <div class="readings">
        <span id="temp">0</span>°C
      </div>
    </div>
  </div>
  
  <div class="card">
    <h2>3D Orientation</h2>
    <canvas id="canvas" width="300" height="300"></canvas>
  </div>

<script>
  var gateway = `ws://${window.location.hostname}/ws`;
  var websocket;
  
  window.addEventListener('load', onLoad);
  
  function initWebSocket() {
    console.log('Trying to open a WebSocket connection...');
    websocket = new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
  }
  
  function onOpen(event) {
    console.log('Connection opened');
  }
  
  function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
  }
  
  function onMessage(event) {
    var data = JSON.parse(event.data);
    
    // Update accelerometer values
    document.getElementById('accel_x').innerHTML = data.ax;
    document.getElementById('accel_y').innerHTML = data.ay;
    document.getElementById('accel_z').innerHTML = data.az;
    
    // Update gyroscope values
    document.getElementById('gyro_x').innerHTML = data.gx;
    document.getElementById('gyro_y').innerHTML = data.gy;
    document.getElementById('gyro_z').innerHTML = data.gz;
    
    // Update angle values
    document.getElementById('roll').innerHTML = data.roll.toFixed(2);
    document.getElementById('pitch').innerHTML = data.pitch.toFixed(2);
    document.getElementById('yaw').innerHTML = data.yaw.toFixed(2);
    
    // Update temperature value
    document.getElementById('temp').innerHTML = data.temp.toFixed(2);
    
    // Update 3D visualization
    drawOrientation(data.roll, data.pitch, data.yaw);
  }
  
  function onLoad(event) {
    initWebSocket();
    initCanvas();
  }
  
  // Simple 3D cube visualization
  var canvas, ctx;
  function initCanvas() {
    canvas = document.getElementById('canvas');
    ctx = canvas.getContext('2d');
    ctx.lineWidth = 2;
    ctx.strokeStyle = '#000000';
    ctx.fillStyle = 'rgba(0, 150, 255, 0.5)';
  }
  
  function drawOrientation(roll, pitch, yaw) {
    const size = 100;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Move to center of canvas
    ctx.save();
    ctx.translate(canvas.width / 2, canvas.height / 2);
    
    // Convert degrees to radians
    let rollRad = roll * Math.PI / 180;
    let pitchRad = pitch * Math.PI / 180;
    let yawRad = yaw * Math.PI / 180;
    
    // Rotate context based on orientation
    ctx.rotate(yawRad);
    
    // Draw a simple representation of the sensor
    // This is a simplified model - a more complex 3D model would require a 3D library
    ctx.beginPath();
    ctx.rect(-size/2, -size/2, size, size);
    ctx.stroke();
    ctx.fill();
    
    // Draw direction indicators
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(0, -size/2 - 20);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(0, -size/2 - 25, 5, 0, 2 * Math.PI);
    ctx.fill();
    
    // Roll and pitch indicators
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(20 * Math.sin(rollRad), -20 * Math.cos(rollRad));
    ctx.stroke();
    
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(-20 * Math.sin(pitchRad), -20 * Math.cos(pitchRad));
    ctx.stroke();
    
    ctx.restore();
  }
</script>
</body>
</html>
)rawliteral";

// Function to calibrate the MPU6050
void calibrateMPU6050() {
  Serial.println("Calibrating MPU6050...");
  
  // Wait for sensor to stabilize
  delay(1000);
  
  // Use the built-in calibration function from the MPU6050 library
  mpu.CalibrateAccel(10);
  mpu.CalibrateGyro(10);
  
  // Get the calibrated offsets
  int16_t* offsets = mpu.GetActiveOffsets();
  ax_offset = offsets[0];
  ay_offset = offsets[1];
  az_offset = offsets[2];
  gx_offset = offsets[3];
  gy_offset = offsets[4];
  gz_offset = offsets[5];
  
  Serial.println("Calibration complete!");
  Serial.println("Accel offsets:");
  Serial.print("X="); Serial.print(ax_offset);
  Serial.print(" Y="); Serial.print(ay_offset);
  Serial.print(" Z="); Serial.println(az_offset);
  Serial.println("Gyro offsets:");
  Serial.print("X="); Serial.print(gx_offset);
  Serial.print(" Y="); Serial.print(gy_offset);
  Serial.print(" Z="); Serial.println(gz_offset);
}

// Task for reading MPU6050 sensor data
void SensorTask(void *pvParameters) {
  // Initialize I2C communication
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  
  // Verify connection
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
    vTaskDelete(NULL); // Delete task if sensor connection fails
    return;
  }
  
  // Configure MPU6050
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250); // Range = ±250 degrees/sec
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // Range = ±2g
  mpu.setDLPFMode(MPU6050_DLPF_BW_20); // Low pass filter = 20Hz
  
  // Calibrate the MPU6050
  calibrateMPU6050();
  
  // Set initial time
  lastTime = micros();
  
  while (1) {
    // Calculate time since last reading
    unsigned long currentTime = micros();
    dt = (currentTime - lastTime) / 1000000.0; // Convert to seconds
    lastTime = currentTime;
    
    // Take semaphore to access shared data
    if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
      // Read raw sensor values
      mpu.getMotion6(&sensorData.ax, &sensorData.ay, &sensorData.az, 
                     &sensorData.gx, &sensorData.gy, &sensorData.gz);
      
      // Read temperature
      int16_t rawTemp = mpu.getTemperature();
      sensorData.temperature = (float)rawTemp / 340.0 + 36.53; // Formula from datasheet
      
      // Calculate angles from accelerometer data
      float accel_x = sensorData.ax;
      float accel_y = sensorData.ay;
      float accel_z = sensorData.az;
      
      // Calculate pitch and roll from accelerometer (in degrees)
      sensorData.accel_roll = atan2(accel_y, accel_z) * 180.0 / PI;
      sensorData.accel_pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / PI;
      
      // Convert gyro values to degrees per second
      float gyro_x = sensorData.gx / gyroScale;
      float gyro_y = sensorData.gy / gyroScale;
      float gyro_z = sensorData.gz / gyroScale;
      
      // Complementary filter for roll, pitch and yaw
      // Alpha parameter determines the weight given to each source
      float alpha = 0.98;
      
      // If this is the first reading, initialize with accelerometer values
      static bool firstReading = true;
      if (firstReading) {
        sensorData.roll = sensorData.accel_roll;
        sensorData.pitch = sensorData.accel_pitch;
        sensorData.yaw = 0;
        firstReading = false;
      } else {
        // Apply complementary filter
        sensorData.roll = alpha * (sensorData.roll + gyro_x * dt) + (1 - alpha) * sensorData.accel_roll;
        sensorData.pitch = alpha * (sensorData.pitch + gyro_y * dt) + (1 - alpha) * sensorData.accel_pitch;
        sensorData.yaw += gyro_z * dt; // Gyro only for yaw
      }
      
      // Set calibrated flag
      sensorData.calibrated = true;
      
      // Release semaphore
      xSemaphoreGive(xSensorDataMutex);
    }
    
    // Delay for sensor update rate
    vTaskDelay(SENSOR_RATE_MS / portTICK_PERIOD_MS);
  }
}

// Task for WebServer
void ServerTask(void *pvParameters) {
  // Initialize WiFi Access Point
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  // Initialize WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  
  // Setup web server routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });
  
  // Start server
  server.begin();
  Serial.println("HTTP server started");
  
  // JSON buffer for WebSocket messages
  StaticJsonDocument<256> jsonDoc;
  char jsonBuffer[256];
  
  while (1) {
    // Take semaphore to access shared data
    if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
      // Only send data if calibrated
      if (sensorData.calibrated && ws.count() > 0) {
        // Prepare JSON document
        jsonDoc["ax"] = sensorData.ax;
        jsonDoc["ay"] = sensorData.ay;
        jsonDoc["az"] = sensorData.az;
        jsonDoc["gx"] = sensorData.gx;
        jsonDoc["gy"] = sensorData.gy;
        jsonDoc["gz"] = sensorData.gz;
        jsonDoc["roll"] = sensorData.roll;
        jsonDoc["pitch"] = sensorData.pitch;
        jsonDoc["yaw"] = sensorData.yaw;
        jsonDoc["temp"] = sensorData.temperature;
        
        // Serialize JSON to buffer
        serializeJson(jsonDoc, jsonBuffer);
        
        // Send to all WebSocket clients
        ws.textAll(jsonBuffer);
      }
      
      // Release semaphore
      xSemaphoreGive(xSensorDataMutex);
    }
    
    // Delay for WebSocket update rate (50ms = 20Hz)
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000); // Allow serial to initialize
  
  // Initialize the mutex
  xSensorDataMutex = xSemaphoreCreateMutex();
  
  // Create tasks for each core
  xTaskCreatePinnedToCore(
    SensorTask,   // Function to implement the task
    "SensorTask", // Name of the task
    4096,         // Stack size in words
    NULL,         // Task input parameter
    1,            // Priority of the task
    NULL,         // Task handle
    SENSOR_CORE   // Core where the task should run
  );
  
  xTaskCreatePinnedToCore(
    ServerTask,   // Function to implement the task
    "ServerTask", // Name of the task
    4096,         // Stack size in words
    NULL,         // Task input parameter
    1,            // Priority of the task
    NULL,         // Task handle
    SERVER_CORE   // Core where the task should run
  );
}

void loop() {
  // Nothing to do here, everything is handled by the tasks
  delay(1000);
}