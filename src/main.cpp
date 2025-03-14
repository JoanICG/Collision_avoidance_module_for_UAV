#include <Arduino.h>
#include <Wire.h>
#include "../lib/RcDriver/RcDriver.h"
#include "../lib/RcDriver/inc/RcDriveriBus.h"

// Create an instance of RcDriveriBus
RcDriver rcBus;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Hello World");
  
  // Use the RcDriveriBus implementation
  rcBus.setStrategy(new RcDriveriBus());
}

void loop() {
  rcBus.decode();
  rcBus.encode();
  delay(1000);
}