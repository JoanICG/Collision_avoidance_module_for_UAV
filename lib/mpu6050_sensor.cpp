#include "mpu6050_sensor.h"
#include <Arduino.h>
#include <Wire.h>

// Instancia global
MPU6050_Manager mpuSensor;

MPU6050_Manager::MPU6050_Manager() : 
    madgwickFilter(MPU_SAMPLE_RATE) {  // Inicializar filtro Madgwick con la frecuencia correcta
    
    accelX = accelY = accelZ = 0.0f;
    gyroX = gyroY = gyroZ = 0.0f;
    roll = pitch = yaw = 0.0f;
    lastUpdateTime = 0;
    deltaTime = 0.0f;
    
    newDataAvailable = false;
    interruptMode = true;
    
    // Valores para calibración inicializados a neutro
    gyroOffsetX = gyroOffsetY = gyroOffsetZ = 0.0f;
    accelScaleX = accelScaleY = accelScaleZ = 1.0f;
    
    readErrorCount = 0;
    sensorPresent = false;
    isCalibrated = false;
    
    // Ajustar el filtro Madgwick para máxima precisión
    madgwickFilter.begin(0.1f);  // Valor que proporciona buen balance
}

bool MPU6050_Manager::begin(bool useInterrupts, bool performCalibration, 
                           MPU6050InterruptCallback interruptCallback) {
    // Inicializar I2C con mayor velocidad
    Wire.begin();
    Wire.setClock(400000);  // 400 kHz
    
    // Intentar comunicarse con el MPU6050
    if (!mpu.begin()) {
        Serial.println("Error: No se pudo encontrar el MPU6050");
        sensorPresent = false;
        return false;
    }
    
    sensorPresent = true;
    Serial.println("MPU6050 encontrado!");
    
    // Configuración optimizada para precisión
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);    // Menor rango = mayor precisión
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);         // Menor rango = mayor precisión 
    mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);      // Mejor filtrado de ruido
    
    // Guardar modo de operación
    interruptMode = useInterrupts;
    
    if (interruptMode) {
        // Verificar que se proporcionó una función de callback
        if (interruptCallback == nullptr) {
            Serial.println("Error: No se proporcionó una función de callback para la interrupción");
            interruptMode = false;  // Caer al modo polling
        } else {
            // Configurar MPU6050 para generar interrupciones DATA_READY
            Wire.beginTransmission(0x68);
            Wire.write(0x38);  // Registro INT_ENABLE
            Wire.write(0x01);  // Habilitar DATA_READY
            Wire.endTransmission();
            
            // Limpiar interrupciones pendientes
            Wire.beginTransmission(0x68);
            Wire.write(0x3A);  // Registro INT_STATUS
            Wire.endTransmission(false);
            Wire.requestFrom(0x68, 1);
            Wire.read();
            
            // Configurar pin de interrupción con la función de callback proporcionada
            pinMode(MPU_INT_PIN, INPUT_PULLUP);
            attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), interruptCallback, RISING);
            
            Serial.println("MPU6050: Modo interrupciones activado (pin " + String(MPU_INT_PIN) + ")");
        }
    }
    
    if (!interruptMode) {
        Serial.println("MPU6050: Modo polling activado");
    }
    
    // Realizar calibración si se solicitó
    if (performCalibration) {
        if (calibrateSensor()) {
            Serial.println("Calibración completada con éxito");
            isCalibrated = true;
        } else {
            Serial.println("Error en calibración, usando valores por defecto");
            isCalibrated = false;
        }
    }
    
    lastUpdateTime = micros();  // Usar micros para mayor precisión
    
    return true;
}

// El resto del archivo permanece igual