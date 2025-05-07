#pragma once

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>

// Definir el tipo de la función de callback para la interrupción
typedef void (*MPU6050InterruptCallback)(void);

// Constantes para mejorar legibilidad
#define MPU_X_AXIS 0
#define MPU_Y_AXIS 1
#define MPU_Z_AXIS 2

// Pin de interrupción DATA_READY del MPU6050
#define MPU_INT_PIN 5

// Frecuencia de muestreo y actualización
#define MPU_SAMPLE_RATE 100  // Hz

// Estructura para datos de orientación completos
struct OrientationData {
    // Ángulos de Euler
    float pitch;    // Cabeceo (inclinación adelante-atrás)
    float roll;     // Alabeo (inclinación lateral)
    float yaw;      // Guiñada (rotación sobre eje vertical)
    
    // Cuaternión para cálculos 3D avanzados
    float quaternion[4];  // w, x, y, z
    
    // Aceleración lineal (sin gravedad)
    float linearAccelX;
    float linearAccelY;
    float linearAccelZ;
    
    // Aceleración total
    float accelMagnitude;
    
    // Timestamp
    unsigned long timestamp;
};

class MPU6050_Manager {
private:
  Adafruit_MPU6050 mpu;
  Madgwick madgwickFilter;
  
  // Datos procesados y crudos
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float roll, pitch, yaw;
  
  // Variables internas
  unsigned long lastUpdateTime;
  float deltaTime;
  volatile bool newDataAvailable;
  bool interruptMode;
  
  // Calibración
  float gyroOffsetX, gyroOffsetY, gyroOffsetZ;
  float accelScaleX, accelScaleY, accelScaleZ;
  
  // Diagnóstico
  uint32_t readErrorCount;
  bool sensorPresent;
  bool isCalibrated;
  
  // Estructura para almacenar datos procesados actualizados
  OrientationData currentOrientation;

  // Métodos internos
  void processNewData();
  bool calibrateSensor();

public:
  MPU6050_Manager();
  
  // Inicialización con opciones
  bool begin(bool useInterrupts = true, bool performCalibration = true, 
             MPU6050InterruptCallback interruptCallback = nullptr);
  
  // *** MÉTODO PRINCIPAL - OBTENER TODO DE UNA VEZ ***
  // Devuelve true si hay datos nuevos, false si no
  bool getOrientation(OrientationData &data);
  
  // *** MÉTODOS INDIVIDUALES (PARA RETROCOMPATIBILIDAD) ***
  bool hasNewData();
  float getPitch();
  float getRoll();
  float getYaw();
  float getRawAccel(uint8_t axis);
  float getRawGyro(uint8_t axis);
  void getQuaternion(float* q);
  
  // Sistema de interrupciones
  void setDataFlag();
  
  // Diagnóstico y recuperación
  bool isHealthy();
  bool reset();
  bool recalibrate();
};

// Instancia global
extern MPU6050_Manager mpuSensor;