#ifndef VL53L0X_SENSOR_H
#define VL53L0X_SENSOR_H

#include <Adafruit_VL53L0X.h>

// Definir pines XSHUT para los 5 sensores
#define XSHUT_1   15  // IO15
#define XSHUT_2   4   // IO4
#define XSHUT_3   13  // IO13
#define XSHUT_4   14  // IO14
#define XSHUT_5   27  // IO27

// Pin de interrupción para el sensor trasero (sensor 4)
#define GPIO_INTERRUPT 19  // IO19 (es el pin de interrupción, conectado al GPIO1 del sensor 4)

// Definir pines de interrupción para cada sensor
#define GPIO_INTERRUPT_1 18  // Izquierda
#define GPIO_INTERRUPT_2 2   // Frontal
#define GPIO_INTERRUPT_3 5   // Derecha
#define GPIO_INTERRUPT_4 19  // Trasero (mantenemos el actual)
#define GPIO_INTERRUPT_5 23  // Inferior

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
  Adafruit_VL53L0X sensor1;
  Adafruit_VL53L0X sensor2;
  Adafruit_VL53L0X sensor3; 
  Adafruit_VL53L0X sensor4; 
  Adafruit_VL53L0X sensor5;

  bool sensor1Enabled;
  bool sensor2Enabled;
  bool sensor3Enabled;
  bool sensor4Enabled;
  bool sensor5Enabled;
  
  // Variables para la interrupción
  bool interruptEnabled;
  uint16_t thresholdDistance;
  VL53L0XCallbackFunction callbackFunction;

  // Estados de interrupción para cada sensor
  bool interrupt1Enabled;
  bool interrupt2Enabled;
  bool interrupt3Enabled;
  bool interrupt4Enabled;
  bool interrupt5Enabled;
  
  // Callbacks específicos para cada sensor
  VL53L0XCallbackFunction callback1;
  VL53L0XCallbackFunction callback2;
  VL53L0XCallbackFunction callback3;
  VL53L0XCallbackFunction callback4;
  VL53L0XCallbackFunction callback5;

  // Puntero al sensor que maneja las interrupciones
  Adafruit_VL53L0X* sensorWithInterrupt;
  
  // Función para obtener un sensor por su dirección I2C
  Adafruit_VL53L0X* getSensorByAddress(uint8_t address);

  // Método privado para configurar interrupciones en un sensor individual
  bool setupSensorInterrupt(Adafruit_VL53L0X &sensor, uint16_t threshold);

  // Configuración de los diferentes modos
  void configureStandardMode(Adafruit_VL53L0X &sensor);
  void configureLongRangeMode(Adafruit_VL53L0X &sensor);
  void configurePrecisionMode(Adafruit_VL53L0X &sensor);

public:
  VL53L0X_Manager();
  
  // Inicialización
  bool begin();
  void scanI2C();
  
  // Configuración
  void configureSensorMode(Adafruit_VL53L0X &sensor, SensorMode mode);
  void setAllSensorsMode(SensorMode mode);
  
  // Toma de medidas
  bool getMeasurement(VL53L0X_RangingMeasurementData_t &measure1, 
                      VL53L0X_RangingMeasurementData_t &measure2,
                      VL53L0X_RangingMeasurementData_t &measure3,
                      VL53L0X_RangingMeasurementData_t &measure4,
                      VL53L0X_RangingMeasurementData_t &measure5);
  
  // Funciones para manejo de interrupciones (para el sensor trasero - sensor4)
  bool setupInterrupt(uint16_t thresholdDistance, VL53L0XCallbackFunction callback);
  bool clearInterrupt();
  void processInterrupt();

  // Métodos para configurar interrupciones individuales
  bool setupInterrupt1(uint16_t threshold, VL53L0XCallbackFunction callback);
  bool setupInterrupt2(uint16_t threshold, VL53L0XCallbackFunction callback);
  bool setupInterrupt3(uint16_t threshold, VL53L0XCallbackFunction callback);
  bool setupInterrupt4(uint16_t threshold, VL53L0XCallbackFunction callback); 
  bool setupInterrupt5(uint16_t threshold, VL53L0XCallbackFunction callback);
  
  // Método para configurar todas las interrupciones a la vez
  bool setupAllInterrupts(uint16_t threshold1, uint16_t threshold2,
                          uint16_t threshold3, uint16_t threshold4, 
                          uint16_t threshold5, 
                          VL53L0XCallbackFunction callback1,
                          VL53L0XCallbackFunction callback2,
                          VL53L0XCallbackFunction callback3,
                          VL53L0XCallbackFunction callback4,
                          VL53L0XCallbackFunction callback5);

  // Accesores
  Adafruit_VL53L0X& getSensor1() { return sensor1; }
  Adafruit_VL53L0X& getSensor2() { return sensor2; }
  Adafruit_VL53L0X& getSensor3() { return sensor3; } 
  Adafruit_VL53L0X& getSensor4() { return sensor4; } 
  Adafruit_VL53L0X& getSensor5() { return sensor5; }

  bool isSensor1Enabled() { return sensor1Enabled; }
  bool isSensor2Enabled() { return sensor2Enabled; }
  bool isSensor3Enabled() { return sensor3Enabled; }
  bool isSensor4Enabled() { return sensor4Enabled; }
  bool isSensor5Enabled() { return sensor5Enabled; }
  
  bool isInterruptEnabled() { return interruptEnabled; }
  uint16_t getThresholdDistance() { return thresholdDistance; }

  // Getters para comprobar si los sensores están habilitados
  bool isSensor1Enabled() const { return sensor1Enabled; }
  bool isSensor2Enabled() const { return sensor2Enabled; }
  bool isSensor3Enabled() const { return sensor3Enabled; }
  bool isSensor4Enabled() const { return sensor4Enabled; }
  bool isSensor5Enabled() const { return sensor5Enabled; }
  
  // Configurar interrupciones en todos los sensores habilitados
  bool setupAllInterrupts(uint16_t threshold1, uint16_t threshold2,
                          uint16_t threshold3, uint16_t threshold4, 
                          uint16_t threshold5, VL53L0XCallbackFunction callback);
                          
  // Limpiar interrupciones en todos los sensores habilitados
  void clearAllInterrupts();

  // Añadir este método para limpiar interrupciones por sensor individual
  bool clearSensorInterrupt(int sensorNum);
};

extern VL53L0X_Manager vl53l0xSensors;

#endif // VL53L0X_SENSOR_H