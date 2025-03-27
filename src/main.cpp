#include <Arduino.h>
#include "../lib/RcDriver/RcDriver.h"

RcDriver rcDriver;
unsigned long lastDisplayTime = 0;
const unsigned long DISPLAY_INTERVAL = 1000;  // Display data every second

// Create a function to send RC data
void sendModifiedRcData() {
    // Get current RC data
    RcInfo info = rcDriver.getRcInfo();
    
    // Modify the data (example: invert throttle)
    info.throttle = 255 - info.throttle;
    
    // Send modified data back out
    bool sent = rcDriver.sendRcInfo(info);
    
    // Don't need to report failures due to timing restrictions
    if (sent) {
        // Uncomment for debugging
        // Serial.println("Sent modified RC data");
    }
}

void setup() {
    Serial.begin(115200);  // Initialize primary serial for debug output
    
    Serial.println("RC Driver Test");
    Serial.println("Initializing RC Driver...");
    
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
        // Get the latest RC data
        RcInfo info = rcDriver.getRcInfo();
        
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
        
        lastDisplayTime = currentTime;
    }
}