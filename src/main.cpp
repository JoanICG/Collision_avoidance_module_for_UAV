#include <Arduino.h>
#include "../lib/RcDriver/RcDriver.h"
#include "../lib/RcDriver/inc/RcDriveriBus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <vl53l0x_manager.h>
#include "web_server.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// WiFi credentials - ¡Cambiar estos valores!
const char* WIFI_SSID = "ESP32-Drone";
const char* WIFI_PASSWORD = "12345678";

// Web data structure and semaphore
WebPageData masterWebData;
SemaphoreHandle_t masterWebDataSemaphore = NULL;

// Create an instance of the iBus strategy
RcDriveriBus ibusStrategy;
RcDriver rcDriver;

// FreeRTOS handles
TaskHandle_t rcTaskHandle = NULL;
TaskHandle_t displayTaskHandle = NULL;
SemaphoreHandle_t rcDataSemaphore = NULL;

// Reordena las declaraciones - primero las estructuras de medición
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
VL53L0X_RangingMeasurementData_t measure4;
VL53L0X_RangingMeasurementData_t measure5;

// Global interrupt flags for sensors
volatile bool sensorLeftInterrupted = false;
volatile bool sensorFrontInterrupted = false;
volatile bool sensorRightInterrupted = false;
volatile bool sensorBackInterrupted = false;
volatile bool sensorBottomInterrupted = false;

// Declara las variables de distancia ANTES de usarlas como punteros
uint16_t distanceLeft = 0;
uint16_t distanceFront = 0;
uint16_t distanceRight = 0;
uint16_t distanceBack = 0;
uint16_t distanceBottom = 0;

// Ahora puedes crear los arrays de punteros que hacen referencia a esas variables
VL53L0X_RangingMeasurementData_t* sensorMeasures[] = {
    &measure1, &measure2, &measure3, &measure4, &measure5
};

uint16_t* sensorDistances[] = {
    &distanceLeft, &distanceFront, &distanceRight, &distanceBack, &distanceBottom
};

// Task handle for sensor monitoring
TaskHandle_t sensorTaskHandle = NULL;

// Shared variables protected by semaphore
RcInfo currentRcInfo;
unsigned long lastFrameTime = 0;
unsigned long frameAge = 0;

// Display interval
const unsigned long DISPLAY_INTERVAL = 1000;  // Display data every second

// Web server and websocket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Forward references for functions used by websocket
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

// HTML content for the web page - referenced from web_server.cpp
extern const char index_html[] PROGMEM;

// Interrupt handlers for each sensor (IRAM_ATTR ensures they run in RAM)
void IRAM_ATTR sensorLeftInterruptHandler() {
    sensorLeftInterrupted = true;
}

void IRAM_ATTR sensorFrontInterruptHandler() {
    sensorFrontInterrupted = true;
}

void IRAM_ATTR sensorRightInterruptHandler() {
    sensorRightInterrupted = true;
}

void IRAM_ATTR sensorBackInterruptHandler() {
    sensorBackInterrupted = true;
}

void IRAM_ATTR sensorBottomInterruptHandler() {
    sensorBottomInterrupted = true;
}

// Primero define una estructura para guardar la información de configuración
typedef struct {
    SensorPosition position;
    const char* name;
    void (*interruptHandler)(void);
    volatile bool* interruptFlag;
} SensorConfig;

// Luego crea un array con la configuración de todos los sensores
const SensorConfig sensorConfigs[] = {
    {LEFT,   "LEFT",   &sensorLeftInterruptHandler,   &sensorLeftInterrupted},
    {FRONT,  "FRONT",  &sensorFrontInterruptHandler,  &sensorFrontInterrupted},
    {RIGHT,  "RIGHT",  &sensorRightInterruptHandler,  &sensorRightInterrupted},
    {BACK,   "BACK",   &sensorBackInterruptHandler,   &sensorBackInterrupted},
    {BOTTOM, "BOTTOM", &sensorBottomInterruptHandler, &sensorBottomInterrupted}
};

// External constant defined in vl53l0x_manager.cpp
extern const int COUNT_SENSORS;

// Watchdog for VL53L0X sensors
unsigned long* lastInterruptTimes = nullptr; // Will be dynamically allocated
const unsigned long SENSOR_WATCHDOG_TIMEOUT = 5000; // 5 seconds without interrupts

// Task for handling RC updates - runs at higher frequency for responsiveness
void rcUpdateTask(void *parameter) {
    // Initialize task timing
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(4); // 4ms - critical for RC responsiveness
    
    for(;;) {
        // Update RC driver to process incoming data
        rcDriver.update();
        
        // Take semaphore to safely access shared data
        if (xSemaphoreTake(rcDataSemaphore, pdMS_TO_TICKS(5)) == pdTRUE) {
            // Get and modify RC data
            rcDriver.getRcInfo(currentRcInfo);
            
            // Modify data if needed (examples)
            // currentRcInfo.throttle = 2000 - currentRcInfo.throttle;
            
            // Send modified RC data back
            rcDriver.sendRcInfo(currentRcInfo);
            
            // Update timing info
            lastFrameTime = millis();
            frameAge = lastFrameTime - currentRcInfo.timestamp;
            
            // Release semaphore
            xSemaphoreGive(rcDataSemaphore);
        }
        
        // After processing RC data
        if (xSemaphoreTake(masterWebDataSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Copy RC data to masterWebData
            masterWebData.rc = currentRcInfo;
            masterWebData.lastRcUpdate = millis();
            
            // No enviar directamente, solo actualizar los datos
            // WebPageData webDataCopy = masterWebData;
            xSemaphoreGive(masterWebDataSemaphore);
            
            // Eliminar esta línea:
            // updateWebServerData(webDataCopy);
        }

        // Precisely time the task execution
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task for displaying data at lower priority and frequency
void displayTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(DISPLAY_INTERVAL);
    
    for(;;) {
        // Access shared data safely using semaphore
        if (xSemaphoreTake(rcDataSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Display RC data
            Serial.println("\n--------- RC Data ----------");
            Serial.print("Throttle: ");
            Serial.print(currentRcInfo.throttle);
            Serial.print(" | Yaw: ");
            Serial.print(currentRcInfo.yaw);
            Serial.print(" | Pitch: ");
            Serial.print(currentRcInfo.pitch);
            Serial.print(" | Roll: ");
            Serial.println(currentRcInfo.roll);
            
            // Display aux channels if available
            if (currentRcInfo.Naux > 0 && currentRcInfo.aux != nullptr) {
                Serial.print("Aux channels: ");
                for (int i = 0; i < currentRcInfo.Naux; i++) {
                    Serial.print(currentRcInfo.aux[i]);
                    if (i < currentRcInfo.Naux - 1) Serial.print(", ");
                }
                Serial.println();
            }
            
            // Display timing information
            Serial.print("Frame age: ");
            Serial.print(frameAge);
            Serial.println(" ms");

            unsigned long timeSinceLastPacket = rcDriver.getTimeSinceLastRx();
            Serial.print("Time since last packet: ");
            Serial.print(timeSinceLastPacket);
            Serial.println(" ms");
            
            if (timeSinceLastPacket > 500) {
                //TODO, handle what to dowhen data lost
                Serial.println("⚠️ WARNING: RC SIGNAL LOST! ⚠️");
            }
            
            Serial.println("----------------------------");
            
            // Release semaphore
            xSemaphoreGive(rcDataSemaphore);
        }
        
        if (xSemaphoreTake(masterWebDataSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Update sensor data in masterWebData
            masterWebData.sensorDistances[LEFT] = distanceLeft;
            masterWebData.sensorDistances[FRONT] = distanceFront;
            masterWebData.sensorDistances[RIGHT] = distanceRight;
            masterWebData.sensorDistances[BACK] = distanceBack;
            masterWebData.sensorDistances[BOTTOM] = distanceBottom;
            
            // Update sensor status
            for(int i=0; i<COUNT_SENSORS; ++i) {
                masterWebData.sensorStatus[i] = vl53l0xSensors.isSensorEnabled(static_cast<SensorPosition>(i));
            }
            
            // Update timestamp
            masterWebData.lastSensorUpdate = millis();
            
            // Eliminar estas líneas:
            // Create a copy for the web server
            WebPageData webDataCopy = masterWebData;
            xSemaphoreGive(masterWebDataSemaphore);
            
            // Eliminar esta línea:
             updateWebServerData(webDataCopy);
        }

        // Wait for next display interval
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task to monitor sensor interrupts and process measurements
void sensorMonitorTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // Check every 20ms
    
    for(;;) {
        unsigned long currentTime = millis();
        
        // Procesar todas las interrupciones de sensores en un solo bucle
        for (int i = 0; i < COUNT_SENSORS; i++) {
            const SensorConfig& config = sensorConfigs[i];
            
            // Si hay interrupción pendiente para este sensor
            if (*(config.interruptFlag)) {
                // Record successful interrupt time
                lastInterruptTimes[i] = currentTime;
                
                if (vl53l0xSensors.isSensorEnabled(config.position)) {
                    // Obtener medición
                    vl53l0xSensors.getMesaurement(config.position, sensorMeasures[i]);
                    
                    // Almacenar medición en la variable correspondiente
                    *sensorDistances[i] = sensorMeasures[i]->RangeMilliMeter;
                    
                    // Mostrar resultado
                    Serial.printf("%s sensor triggered: %d mm\n", 
                                 config.name, 
                                 sensorMeasures[i]->RangeMilliMeter);
                    
                    // Double clear the interrupt to ensure it's fully cleared
                    vl53l0xSensors.clearInterrupt(config.position);
                    delay(1);
                    vl53l0xSensors.clearInterrupt(config.position);
                }
                // Restablecer bandera de interrupción
                *(config.interruptFlag) = false;
            }
            
            // Check for sensor watchdog timeout - specially focusing on problematic sensors
            // (pins 32 and 26 correspond to RIGHT and BOTTOM positions based on your initialization)
            if (vl53l0xSensors.isSensorEnabled(config.position) && 
                lastInterruptTimes[i] > 0 &&  // Only check if sensor has triggered at least once
                currentTime - lastInterruptTimes[i] > SENSOR_WATCHDOG_TIMEOUT) {
                
                // Special focus on problematic sensors (pins 32 and 26)
                bool isProblemSensor = false;
                
                // Check if this is one of the problematic sensors (RIGHT uses pin 32, BOTTOM uses pin 26)
                if (config.position == RIGHT || config.position == BOTTOM) {
                    isProblemSensor = true;
                }
                
                Serial.printf("WARNING: %s sensor hasn't triggered for %lu ms. %s\n", 
                             config.name, 
                             currentTime - lastInterruptTimes[i],
                             isProblemSensor ? "Resetting problematic sensor..." : "Resetting...");
                
                // Use the built-in sensor reset function from vl53l0x_manager.cpp
                if (vl53l0xSensors.resetSensor(config.position) == 0) {
                    // Setup the interrupt again after reset
                    vl53l0xSensors.setupSensorInterrupt(
                        config.position,
                        300,  // Same threshold as original setup
                        config.interruptHandler
                    );
                    
                    Serial.printf("%s sensor reset and reinitialized successfully\n", config.name);
                } else {
                    Serial.printf("Failed to reset %s sensor\n", config.name);
                }
                
                // Reset the timer to prevent continuous reset attempts
                lastInterruptTimes[i] = currentTime;
            }
        }
        
        // Actualizar datos para la interfaz web
        if (xSemaphoreTake(masterWebDataSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Actualizar todos los datos de sensores de una vez
            for (int i = 0; i < COUNT_SENSORS; i++) {
                masterWebData.sensorDistances[i] = *sensorDistances[i];
                masterWebData.sensorStatus[i] = vl53l0xSensors.isSensorEnabled(
                    static_cast<SensorPosition>(i)
                );
            }
            
            masterWebData.lastSensorUpdate = millis();
            xSemaphoreGive(masterWebDataSemaphore);
        }
        
        // Wait until next check time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Handle incoming WebSocket messages

void setup() {
    // Initialize serial for debug
    Serial.begin(115200);
    delay(1000); // Give time for serial port to stabilize
    
    Serial.println("\n=== FreeRTOS RC Driver System ===");
    
    Wire.begin(); // Initialize I2C bus for VL53L0X sensors
    delay(100); // Give time for I2C bus to stabilize
  
    // Create semaphore for data protection
    rcDataSemaphore = xSemaphoreCreateMutex();
    if (!rcDataSemaphore) {
        Serial.println("Error creating semaphore!");
        while(1) { delay(100); } // Fatal error
    }
    
    // Initialize RC Driver
    Serial.println("Initializing RC Driver...");
    rcDriver.setStrategy(&ibusStrategy);
        
    rcDriver.begin(115200);
    
    // Initialize VL53L0X sensors
    Serial.println("Initializing VL53L0X sensors...");
    if (vl53l0xSensors.begin() == 0) {
        Serial.println("Error initializing VL53L0X sensors!");
    }

    for(int i=0; i<COUNT_SENSORS; i++) {
        Serial.printf("Sensor %d status: %d\n", i, sensors[i].sensor_status);
    }

    // Set up sensor interrupts with threshold of 300mm
    Serial.println("Setting up sensor interrupts...");

    // Configure each sensor for interrupt operation
    for (const auto& config : sensorConfigs) {
        if (vl53l0xSensors.isSensorEnabled(config.position)) {
            vl53l0xSensors.setupSensorInterrupt(
                config.position, 
                300,  // Threshold en mm
                config.interruptHandler
            );
            Serial.printf("%s sensor interrupt configured\n", config.name);
        }
    }

    // Initialize sensor watchdog array
    lastInterruptTimes = new unsigned long[COUNT_SENSORS]{0};
    if (!lastInterruptTimes) {
        Serial.println("Failed to allocate memory for sensor watchdog!");
    }

    // Create semaphore for master web data
    masterWebDataSemaphore = xSemaphoreCreateMutex();
    if (masterWebDataSemaphore == NULL) {
        Serial.println("Error creating masterWebDataSemaphore!");
    }
    
    // Initialize masterWebData (default values)
    if (xSemaphoreTake(masterWebDataSemaphore, portMAX_DELAY) == pdTRUE) {
        memset(&masterWebData, 0, sizeof(WebPageData));
        for(int i=0; i<COUNT_SENSORS; ++i) {
            masterWebData.sensorDistances[i] = 0xFFFF; // Indicate no reading
            masterWebData.sensorStatus[i] = vl53l0xSensors.isSensorEnabled(static_cast<SensorPosition>(i));
        }
        xSemaphoreGive(masterWebDataSemaphore);
    }
    
    // Initialize the web server system
    Serial.println("Initializing Web Server...");
    initWebServer(&server, &ws, WIFI_SSID, WIFI_PASSWORD);
    
    // Start the web server task
    Serial.println("Starting Web Server Task...");

    // Create tasks
    Serial.println("Creating FreeRTOS tasks...");
    
    // RC update task - higher priority, on core 0 (background)
    xTaskCreatePinnedToCore(
        rcUpdateTask,        // Function
        "RCTask",            // Name
        4096,                // Stack size
        NULL,                // Parameters
        3,                   // Priority (higher number = higher priority)
        &rcTaskHandle,       // Handle
        0                    // Core (0 = background/WiFi core)
    );
    
    // Display task - lower priority, on core 1 (application)
    xTaskCreatePinnedToCore(
        displayTask,         // Function
        "DisplayTask",       // Name
        4096,                // Stack size
        NULL,                // Parameters
        1,                   // Priority (lower than RC task)
        &displayTaskHandle,  // Handle
        1                    // Core (1 = application core)
    );

    // Create sensor monitoring task
    xTaskCreatePinnedToCore(
        sensorMonitorTask,    // Function
        "SensorTask",         // Name
        4096,                 // Stack size
        NULL,                 // Parameters
        2,                    // Priority (between RC and display)
        &sensorTaskHandle,    // Handle
        0                     // Core (0 = background core)
    );
    
    // Iniciar tarea periódica de actualizaciones web (cada 100ms = 10Hz)
    startWebUpdateTask(100);
    
    Serial.println("FreeRTOS scheduler started");
    Serial.println("=============================================");
}

void loop() {
    // Nothing to do here - FreeRTOS tasks handle everything
    delay(1000);
}