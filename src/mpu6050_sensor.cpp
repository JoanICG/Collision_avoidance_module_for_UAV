#include "mpu6050_sensor.h"
#include <Arduino.h>

MPU6050_Manager mpuSensor;

MPU6050_Manager::MPU6050_Manager() 
  : accelX(0), accelY(0), accelZ(0),
    gyroX(0), gyroY(0), gyroZ(0),
    angleX(0), angleY(0),
    filteredAngleX(0), filteredAngleY(0),
    lastUpdateTime(0) {
}

bool MPU6050_Manager::begin() {
  Serial.println("Inicializando MPU6050...");
  
  if (!mpu.begin()) {
    Serial.println("Error al iniciar el MPU6050. Comprueba las conexiones.");
    return false;
  }
  
  Serial.println("MPU6050 iniciado correctamente.");
  
  // Configurar el MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);    // ±8G
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);         // ±500 grados/s
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);      // Filtro interno de 21 Hz
  
  lastUpdateTime = millis();
  return true;
}

void MPU6050_Manager::update() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Calcular intervalo de tiempo
  unsigned long currentTime = millis();
  float dt = (currentTime - lastUpdateTime) / 1000.0;
  lastUpdateTime = currentTime;
  
  // Evitar divisiones por cero
  if (dt <= 0) dt = 0.01;
  
  // Guardar valores de aceleración y giroscopio
  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;
  
  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;
  
  // Calcular ángulos con acelerómetro
  // (simplificado para pequeñas inclinaciones)
  angleX = atan2(accelY, accelZ) * 180.0 / PI;
  angleY = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
  
  // Filtro complementario - combina acelerómetro y giroscopio
  // El giroscopio proporciona cambios rápidos, el acelerómetro corrige a largo plazo
  filteredAngleX = alpha * (filteredAngleX + gyroX * dt) + (1 - alpha) * angleX;
  filteredAngleY = alpha * (filteredAngleY + gyroY * dt) + (1 - alpha) * angleY;
}