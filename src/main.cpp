#include <Arduino.h>
#include "../lib/RcDriver/RcDriver.h"
#include "../lib/RcDriver/inc/RcDriveriBus.h"

// Create an instance of the iBus strategy
RcDriveriBus ibusStrategy;
RcDriver rcDriver;

unsigned long lastDisplayTime = 0;
const unsigned long DISPLAY_INTERVAL = 1000;  // Display data every second

// Send modified RC data
void sendModifiedRcData() {
    // Create a struct RcInfo local
    RcInfo info;
    
    // Get the current RC data
    rcDriver.getRcInfo(info);

    // Modify the data if needed (example: invert throttle)
    // info.throttle = 2000 - info.throttle;

    // Send the modified data
    int sent = rcDriver.sendRcInfo(info);
    
    // No need to report failures due to timing constraints
    if (sent == 0) {
        // Uncomment for debugging
        // Serial.println("Sent modified RC data");
    }
    
    // DO NOT free memory here - RcDriver handles memory management
}

void setup() {
    Serial.begin(115200);  // Initialize primary serial for debug output
    
    Serial.println("RC Driver Test");
    Serial.println("Initializing RC Driver...");
    
    rcDriver.setStrategy(&ibusStrategy);  // Set the strategy to iBus
    // Initialize RC Driver with default iBus strategy
    rcDriver.begin(115200);
}

void loop() {
    // Update RC driver to process incoming data
    rcDriver.update();
    
    // Call the function to send modified RC data
    sendModifiedRcData();
    
    // Display data periodically
    unsigned long currentTime = millis();
    if (currentTime - lastDisplayTime >= DISPLAY_INTERVAL) {
        // Create a struct RcInfo local
        RcInfo info;
        
        // Get the current RC data
        rcDriver.getRcInfo(info);
        
        // Calculate time since last frame
        unsigned long frameAge = currentTime - info.timestamp;
        
        // Display RC data
        Serial.println("--------- RC Data ----------");
        Serial.print("Throttle: ");
        Serial.print(info.throttle);
        Serial.print(" | Yaw: ");
        Serial.print(info.yaw);
        Serial.print(" | Pitch: ");
        Serial.print(info.pitch);
        Serial.print(" | Roll: ");
        Serial.println(info.roll);
        
        // Display aux channels if available
        if (info.Naux > 0 && info.aux != nullptr) {
            Serial.print("Aux channels: ");
            for (int i = 0; i < info.Naux; i++) {
                Serial.print(info.aux[i]);
                if (i < info.Naux - 1) Serial.print(", ");
            }
            Serial.println();
        }
        
        Serial.print("Frame age: ");
        Serial.print(frameAge);
        Serial.println(" ms");

        unsigned long timeSinceLastPacket = rcDriver.getTimeSinceLastRx();
        Serial.print("Time since last packet: ");
        Serial.print(timeSinceLastPacket);
        Serial.println(" ms");
        
        Serial.println("----------------------------");
        
        // DO NOT free memory here - RcDriver handles memory management
        
        lastDisplayTime = currentTime;
    }
}