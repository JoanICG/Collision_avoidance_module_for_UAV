#pragma once

#include <Adafruit_VL53L0X.h>
#include <Wire.h>

#define DEBUG_VL  // Descomentar para habilitar mensajes de depuración

#define VL53L0X_DEFAULT_ADDRESS 0x29 // Dirección I2C por defecto


//TODO, reorder for proper pin identification
enum SensorPosition {
  LEFT = 0,
  FRONT = 1,
  RIGHT = 2,
  BACK = 3,
  BOTTOM = 4
};

// Valor en que se activará la interrupción (en mm)
#define THRESHOLD_VALUE 200

// Modos de operación del sensor
enum SensorMode {
  MODE_STANDARD,
  MODE_LONG_RANGE,
  MODE_HIGH_PRECISION
};

typedef struct {
  Adafruit_VL53L0X *psensor; // pointer to object
  TwoWire *pwire; // I2C wire object
  SensorPosition position;            // id for the sensor as its position
  int address; // I2C address
  int shutdown_pin;  // which pin for shutdown;
  int interrupt_pin; // which pin to use for interrupts.
  Adafruit_VL53L0X::VL53L0X_Sense_config_t
      sensor_config;     // options for how to use the sensor
  uint16_t range;        // range value used in continuous mode stuff.
  int sensor_status; /**
                            status from last ranging in continuous
                          **/ 
} sensorList_t;

// Declare sensors and COUNT_SENSORS as extern
extern sensorList_t sensors[];
extern const int COUNT_SENSORS;

// Tipo de función callback para interrupciones
typedef void (*VL53L0XCallbackFunction)();

class VL53L0X_Manager {
private:
 

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

  // Iniciar medición continua en un sensor específico
  int getMesaurement(SensorPosition sensor, VL53L0X_RangingMeasurementData_t *measure);

  // Set operation mode
  //For now, change for all sensors
  void setStandardMode();
  void setLongRangeMode();
  void setPrecisionMode();

};

extern VL53L0X_Manager vl53l0xSensors;
