#ifndef MPU6050_SENSOR_H
#define MPU6050_SENSOR_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

class MPU6050_Manager {
private:
  Adafruit_MPU6050 mpu;
  
  // Variables para almacenar datos y cálculos
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float angleX, angleY;
  float filteredAngleX, filteredAngleY;
  unsigned long lastUpdateTime;
  
  // Factor de filtro complementario
  const float alpha = 0.96;

public:
  MPU6050_Manager();
  
  // Inicialización y configuración
  bool begin();
  
  // Procesamiento de datos
  void update();
  
  // Accesores para datos procesados
  float getAccelX() { return accelX; }
  float getAccelY() { return accelY; }
  float getAccelZ() { return accelZ; }
  float getGyroX() { return gyroX; }
  float getGyroY() { return gyroY; }
  float getGyroZ() { return gyroZ; }
  float getPitch() { return filteredAngleX; }
  float getRoll() { return filteredAngleY; }
  
  // Acceso directo al objeto Adafruit_MPU6050
  Adafruit_MPU6050& getSensor() { return mpu; }
};

extern MPU6050_Manager mpuSensor;

#endif // MPU6050_SENSOR_H