#pragma once

#include <Adafruit_VL53L0X.h>

#define DEBUG_VL  // Descomentar para habilitar mensajes de depuración

#define NSENSORS 5  // Número total de sensores VL53L0X
#define VL53L0X_FIRST_ADDRESS 0x29 // Dirección I2C por defecto


//TODO, reorder for proper pin identification
enum SensorPosition {
  LEFT = 0,
  FRONT = 1,
  RIGHT = 2,
  BACK = 3,
  BOTTOM = 4
};

// Definir pines XSHUT para los 5 sensores
const uint8_t xshutPins[] = {33, 23, 35, 18, 27}; 

// Definir pines de interrupción para los 5 sensores
const uint8_t interruptPins[] = {32, 25, 34, 19, 26};

// Valor en que se activará la interrupción (en mm)
#define THRESHOLD_VALUE 200

// Modos de operación del sensor
enum SensorMode {
  MODE_STANDARD,
  MODE_LONG_RANGE,
  MODE_HIGH_PRECISION
};

// Tipo de función callback para interrupciones
typedef void (*VL53L0XCallbackFunction)();

class VL53L0X_Manager {
private:
  // Instancias de los sensores VL53L0X

  // Variables para habilitar sensores
  bool sensorEnabled[NSENSORS]; 

public:
  VL53L0X_Manager();
  Adafruit_VL53L0X sensors[NSENSORS];

  // Inicialización
  int begin();
  
  // Configuración
  int configureSensorMode(SensorPosition sensor, SensorMode mode);
  int setAllSensorsMode(SensorMode mode);
  
    // Configurar interrupciones en un sensor individual
  int setupSensorInterrupt(SensorPosition sensor, uint16_t threshold, VL53L0XCallbackFunction callback);
 
  int clearInterrupt(SensorPosition sensor);
  
  // Getters para comprobar si los sensores están habilitados
  bool isSensorEnabled(SensorPosition sensor);
                         
  // Limpiar interrupciones en todos los sensores habilitados
  void clearAllInterrupts();


  // Set operation mode
  //For now, change for all sensors
  void setStandardMode();
  void setLongRangeMode();
  void setPrecisionMode();

};

extern VL53L0X_Manager vl53l0xSensors;
