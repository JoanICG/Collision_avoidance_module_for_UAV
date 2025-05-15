#include <Arduino.h>
#include "../lib/RcDriver/RcDriver.h"
#include "../lib/RcDriver/inc/RcDriveriBus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <vl53l0x_manager.h>

// Create an instance of the iBus strategy
RcDriveriBus ibusStrategy;
RcDriver rcDriver;

// FreeRTOS handles
TaskHandle_t rcTaskHandle = NULL;
TaskHandle_t displayTaskHandle = NULL;
SemaphoreHandle_t rcDataSemaphore = NULL;

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

// Latest distance measurements
uint16_t distanceLeft = 0;
uint16_t distanceFront = 0;
uint16_t distanceRight = 0;
uint16_t distanceBack = 0;
uint16_t distanceBottom = 0;

// Task handle for sensor monitoring
TaskHandle_t sensorTaskHandle = NULL;

// No es necesario crear una nueva instancia ya que usaremos la definida en vl53l0x_manager.cpp
// VL53L0X_Manager vl53l0xManager;  <-- Eliminar esta línea

// Shared variables protected by semaphore
RcInfo currentRcInfo;
unsigned long lastFrameTime = 0;
unsigned long frameAge = 0;

// Display interval
const unsigned long DISPLAY_INTERVAL = 1000;  // Display data every second

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

// Task for handling RC updates - runs at higher frequency for responsiveness
void rcUpdateTask(void *parameter) {
    // Initialize task timing
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(4); // 10ms - critical for RC responsiveness
    
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
                Serial.println("⚠️ WARNING: RC SIGNAL LOST! ⚠️");
            }
            
            Serial.println("----------------------------");
            
            // Release semaphore
            xSemaphoreGive(rcDataSemaphore);
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
        // Process LEFT sensor
        if(sensorLeftInterrupted) {
            if(vl53l0xSensors.isSensorEnabled(LEFT)) {
                vl53l0xSensors.getMesaurement(LEFT, &measure1);
                distanceLeft = measure1.RangeMilliMeter;
                Serial.printf("LEFT sensor triggered: %d mm\n", measure1.RangeMilliMeter);
                vl53l0xSensors.clearInterrupt(LEFT);
            }
            sensorLeftInterrupted = false;
        }
        
        // Process FRONT sensor
        if(sensorFrontInterrupted) {
            if(vl53l0xSensors.isSensorEnabled(FRONT)) {
                distanceFront = vl53l0xSensors.getMesaurement(FRONT, &measure2);
                Serial.printf("FRONT sensor triggered: %d mm\n", measure2.RangeMilliMeter);
                vl53l0xSensors.clearInterrupt(FRONT);
            }
            sensorFrontInterrupted = false;
        }
        
        // Process RIGHT sensor
        if(sensorRightInterrupted) {
            if(vl53l0xSensors.isSensorEnabled(RIGHT)) {
                distanceRight = vl53l0xSensors.getMesaurement(RIGHT, &measure3);
                Serial.printf("RIGHT sensor triggered: %d mm\n", measure3.RangeMilliMeter);
                vl53l0xSensors.clearInterrupt(RIGHT);
            }
            sensorRightInterrupted = false;
        }
        
        // Process BACK sensor
        if(sensorBackInterrupted) {
            if(vl53l0xSensors.isSensorEnabled(BACK)) {
                distanceBack = vl53l0xSensors.getMesaurement(BACK, &measure4);
                Serial.printf("BACK sensor triggered: %d mm\n", measure4.RangeMilliMeter);
                vl53l0xSensors.clearInterrupt(BACK);
            }
            sensorBackInterrupted = false;
        }
        
        // Process BOTTOM sensor
        if(sensorBottomInterrupted) {
            if(vl53l0xSensors.isSensorEnabled(BOTTOM)) {
                distanceBottom = vl53l0xSensors.getMesaurement(BOTTOM, &measure5);
                Serial.printf("BOTTOM sensor triggered: %d mm\n", measure5.RangeMilliMeter);
                vl53l0xSensors.clearInterrupt(BOTTOM);
            }
            sensorBottomInterrupted = false;
        }
        
        // Wait until next check time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup() {
    // Initialize serial for debug
    Serial.begin(115200);
    delay(1000); // Give time for serial port to stabilize
    
    Serial.println("\n=== FreeRTOS RC Driver System ===");
    
    Wire.begin(); // Initialize I2C bus for VL53L0X sensors
    delay(100); // Give time for I2C bus to stabilize

    // Configure GPIO12 safely before anything else
    pinMode(12, OUTPUT);
    digitalWrite(12, LOW);
    delay(10);
    
    // Create semaphore for data protection
    rcDataSemaphore = xSemaphoreCreateMutex();
    if (!rcDataSemaphore) {
        Serial.println("Error creating semaphore!");
        while(1) { delay(100); } // Fatal error
    }
    
    // Initialize RC Driver
    Serial.println("Initializing RC Driver...");
    rcDriver.setStrategy(&ibusStrategy);
    
    // Use alternative pins if you're having flash issues
    // Serial2.begin(115200, SERIAL_8N1, 25, 23); // Use safer GPIO25 (RX) and 23 (TX)
    
    rcDriver.begin(115200);
    
    // Initialize VL53L0X sensors
    Serial.println("Initializing VL53L0X sensors...");
    if (vl53l0xSensors.begin() == 0) {  // Usar vl53l0xSensors en lugar de vl53l0xManager
        Serial.println("Error initializing VL53L0X sensors!");
        //while(1) { delay(100); } // Fatal error
    }

    for(int i=0; i<COUNT_SENSORS; i++) {
        Serial.printf("Sensor %d status: %d\n", i, sensors[i].sensor_status);
    }

    // Set up sensor interrupts with threshold of 300mm
    Serial.println("Setting up sensor interrupts...");

    // Configure each sensor for interrupt operation
    if(vl53l0xSensors.isSensorEnabled(LEFT)) {
        vl53l0xSensors.setupSensorInterrupt(LEFT, 300, sensorLeftInterruptHandler);
        Serial.println("LEFT sensor interrupt configured");
    }

    if(vl53l0xSensors.isSensorEnabled(FRONT)) {
        vl53l0xSensors.setupSensorInterrupt(FRONT, 300, sensorFrontInterruptHandler);
        Serial.println("FRONT sensor interrupt configured");
    }

    if(vl53l0xSensors.isSensorEnabled(RIGHT)) {
        vl53l0xSensors.setupSensorInterrupt(RIGHT, 300, sensorRightInterruptHandler);
        Serial.println("RIGHT sensor interrupt configured");
    }

    if(vl53l0xSensors.isSensorEnabled(BACK)) {
        vl53l0xSensors.setupSensorInterrupt(BACK, 300, sensorBackInterruptHandler);
        Serial.println("BACK sensor interrupt configured");
    }

    if(vl53l0xSensors.isSensorEnabled(BOTTOM)) {
        vl53l0xSensors.setupSensorInterrupt(BOTTOM, 300, sensorBottomInterruptHandler);
        Serial.println("BOTTOM sensor interrupt configured");
    }

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
        0                     // Core (1 = application core)
    );
    
    Serial.println("FreeRTOS scheduler started");
    Serial.println("=============================================");
}

void loop() {
    // Nothing to do here - FreeRTOS tasks handle everything
    // This task still runs at lowest priority
    delay(1000);
}