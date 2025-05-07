#pragma once

#include <Adafruit_VL53L0X.h>

//#define DEBUG

#define NSENSORS 5  // Número total de sensores VL53L0X

//TODO, reorder for proper pin identification
enum SensorPosition {
  LEFT = 0,
  FRONT = 1,
  RIGHT = 2,
  BACK = 3,
  BOTTOM = 4
};

// Definir pines XSHUT para los 5 sensores
#define XSHUT_1   33  // IO15
#define XSHUT_2   13   // IO4
#define XSHUT_3   35  // IO13
#define XSHUT_4   18  // IO14
#define XSHUT_5   27  // IO27

const uint8_t xshutPins[] = {33, 13, 35, 18, 27}; 

// Pines de interrupción
#define INTERRUPT_1 32  
#define INTERRUPT_2 12 
#define INTERRUPT_3 34 
#define INTERRUPT_4 19 
#define INTERRUPT_5 26 

const uint8_t interruptPins[] = {32, 12, 34, 19, 26};

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
  Adafruit_VL53L0X sensors[NSENSORS];

  // Variables para habilitar sensores
  bool sensorEnabled[NSENSORS]; 

public:
  VL53L0X_Manager();
  
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
