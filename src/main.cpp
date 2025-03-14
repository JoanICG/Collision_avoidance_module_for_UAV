#include <Arduino.h>
#include <Wire.h>
#include "../lib/RcDriver/RcDriver.h"
#include "../lib/RcDriver/inc/RcDriveriBus.h"

// Create an instance of RcDriver
RcDriver rcBus;
TaskHandle_t rcTaskHandle = NULL;

// Task function for RC data collection
void rcDataCollectionTask(void* parameter) {
    RcDriver* driver = static_cast<RcDriver*>(parameter);
    
    while(true) {
        driver->decode();  // Decode the incoming data
        // Short delay to prevent CPU hogging
        vTaskDelay(1);  // Minimal delay for continuous reading
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Hello World");
  
    // Set the strategy
    rcBus.setStrategy(new RcDriveriBus());
    
    // Initialize serial for RX2 pin
    rcBus.beginSerial(115200);
  
    // Start RC data collection task on core 1
    xTaskCreatePinnedToCore(
        rcDataCollectionTask,  // Task function
        "RcDataTask",          // Name
        4096,                  // Stack size
        &rcBus,                // Parameter (pointer to RcDriver)
        1,                     // Priority
        &rcTaskHandle,         // Task handle
        1                      // Core (1 = application core)
    );
}

void loop() {
    // Access RC channel data
    const RcChannelData* data = rcBus.getData();
    
    // if (data->isValid) {
        // Display first 8 channels
        for (int i = 0; i < 8; i++) {
            Serial.print("CH");
            Serial.print(i);
            Serial.print(": ");
            Serial.print(data->channels[i]);
            Serial.print(" ");
        }
        Serial.println();
    // }
    
    // delay(1000);
}