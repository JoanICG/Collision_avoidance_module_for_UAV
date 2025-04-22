#include "../vl53l0x_sensor.h"
#include <Wire.h>

// Variables globales para los callbacks, necesarias para ISRs estáticas
static VL53L0XCallbackFunction globalCallback1 = nullptr;
static VL53L0XCallbackFunction globalCallback2 = nullptr;
static VL53L0XCallbackFunction globalCallback3 = nullptr;
static VL53L0XCallbackFunction globalCallback4 = nullptr;
static VL53L0XCallbackFunction globalCallback5 = nullptr;

// Global callback for general interrupt handler
static VL53L0XCallbackFunction globalCallback = nullptr;

// ISRs estáticas para cada sensor
void IRAM_ATTR handleVL53L0XInterrupt1() {
  if (globalCallback1 != nullptr) globalCallback1();
}

void IRAM_ATTR handleVL53L0XInterrupt2() {
  if (globalCallback2 != nullptr) globalCallback2();
}

void IRAM_ATTR handleVL53L0XInterrupt3() {
  if (globalCallback3 != nullptr) globalCallback3();
}

void IRAM_ATTR handleVL53L0XInterrupt4() {
  if (globalCallback4 != nullptr) globalCallback4();
}

void IRAM_ATTR handleVL53L0XInterrupt5() {
  if (globalCallback5 != nullptr) globalCallback5();
}

// General interrupt handler
void IRAM_ATTR handleVL53L0XInterrupt() {
  if (globalCallback != nullptr) globalCallback();
}

VL53L0X_Manager vl53l0xSensors;

// Constructor actualizado
VL53L0X_Manager::VL53L0X_Manager() 
  : sensor1Enabled(false), sensor2Enabled(false), 
    sensor3Enabled(false), sensor4Enabled(false), sensor5Enabled(false),
    interrupt1Enabled(false), interrupt2Enabled(false), 
    interrupt3Enabled(false), interrupt4Enabled(false), interrupt5Enabled(false),
    callback1(nullptr), callback2(nullptr), 
    callback3(nullptr), callback4(nullptr), callback5(nullptr) {
}

// Modifica la función begin() para asignar explícitamente dirección 0x33 al sensor trasero:
bool VL53L0X_Manager::begin() {
  // Configurar todos los pines XSHUT como salida
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);
  pinMode(XSHUT_4, OUTPUT);
  pinMode(XSHUT_5, OUTPUT);
  
  // Apagar TODOS los sensores VL53L0X
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);
  digitalWrite(XSHUT_4, LOW);
  digitalWrite(XSHUT_5, LOW);
  delay(100);
  
  Serial.println("Todos los sensores apagados");

  // SOLO ACTIVAMOS el sensor 5 (inferior)
  digitalWrite(XSHUT_5, HIGH);
  delay(100);
  
  Serial.println("Inicializando SOLO el sensor 5 (Inferior)");
  
  if (!sensor5.begin(0x29)) { // Usamos la dirección predeterminada
    Serial.println("Error al iniciar sensor 5 (Inferior)");
    sensor5Enabled = false;
    digitalWrite(XSHUT_5, LOW);
    return false;
  }
  
  Serial.println("Sensor 5 (Inferior) iniciado correctamente en dirección 0x29");
  sensor5Enabled = true;
  
  // Configurar en modo estándar
  configureSensorMode(sensor5, MODE_STANDARD);
  
  // Optimizar timing budget para detección rápida
  sensor5.setMeasurementTimingBudgetMicroSeconds(50000); // 50ms
  
  // TODOS los demás sensores permanecen deshabilitados
  sensor1Enabled = false;
  sensor2Enabled = false;
  sensor3Enabled = false;
  sensor4Enabled = false;
  
  return sensor5Enabled;
}

void VL53L0X_Manager::scanI2C() {
  Serial.println("Escaneando bus I2C...");
  byte error, address;
  int nDevices = 0;
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Dispositivo encontrado en dirección 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  
  if (nDevices == 0)
    Serial.println("No se encontraron dispositivos I2C");
  else
    Serial.println("Escaneo I2C completado");
}

void VL53L0X_Manager::configureStandardMode(Adafruit_VL53L0X &sensor) {
  // Restablecer a valores por defecto
  sensor.setMeasurementTimingBudgetMicroSeconds(33000); // 33ms
  
  // Valores por defecto para el resto de parámetros
  sensor.setLimitCheckValue(VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 0.25);
  sensor.setLimitCheckValue(VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 18.0);
  
  // VCSEL pulse periods por defecto
  sensor.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 14);
  sensor.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 10);
}

void VL53L0X_Manager::configureLongRangeMode(Adafruit_VL53L0X &sensor) {
  // Aumenta el tiempo de medición
  sensor.setMeasurementTimingBudgetMicroSeconds(200000); // 200ms
  
  // Ajusta para detectar señales más débiles
  sensor.setLimitCheckValue(VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 0.1);
  sensor.setLimitCheckValue(VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 60.0);
  
  // Aumenta la duración de los pulsos láser
  sensor.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
  sensor.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
}

void VL53L0X_Manager::configurePrecisionMode(Adafruit_VL53L0X &sensor) {
  // Aumenta sustancialmente el tiempo de medición
  sensor.setMeasurementTimingBudgetMicroSeconds(300000); // 300ms
  
  // Ajusta para mayor precisión
  sensor.setLimitCheckValue(VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 0.5);
  sensor.setLimitCheckValue(VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 15.0);
  
  // Valores VCSEL por defecto
  sensor.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 14);
  sensor.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 10);
}

void VL53L0X_Manager::configureSensorMode(Adafruit_VL53L0X &sensor, SensorMode mode) {
  switch (mode) {
    case MODE_STANDARD:
      configureStandardMode(sensor);
      Serial.println("Configurado en modo estándar");
      break;
    case MODE_LONG_RANGE:
      configureLongRangeMode(sensor);
      Serial.println("Configurado en modo largo alcance");
      break;
    case MODE_HIGH_PRECISION:
      configurePrecisionMode(sensor);
      Serial.println("Configurado en modo alta precisión");
      break;
  }
}

void VL53L0X_Manager::setAllSensorsMode(SensorMode mode) {
  if (sensor1Enabled) configureSensorMode(sensor1, mode);
  if (sensor2Enabled) configureSensorMode(sensor2, mode);
  if (sensor3Enabled) configureSensorMode(sensor3, mode); // Nuevo
  if (sensor4Enabled) configureSensorMode(sensor4, mode); // Nuevo
  if (sensor5Enabled) configureSensorMode(sensor5, mode); // Nuevo
}

// Actualizado para 5 sensores
bool VL53L0X_Manager::getMeasurement(VL53L0X_RangingMeasurementData_t &measure1, 
                                    VL53L0X_RangingMeasurementData_t &measure2,
                                    VL53L0X_RangingMeasurementData_t &measure3, // Nuevo
                                    VL53L0X_RangingMeasurementData_t &measure4, // Nuevo
                                    VL53L0X_RangingMeasurementData_t &measure5  // Nuevo
                                    ) {
  if (sensor1Enabled) sensor1.rangingTest(&measure1, false);
  if (sensor2Enabled) sensor2.rangingTest(&measure2, false);
  if (sensor3Enabled) sensor3.rangingTest(&measure3, false); // Nuevo
  if (sensor4Enabled) sensor4.rangingTest(&measure4, false); // Nuevo
  if (sensor5Enabled) sensor5.rangingTest(&measure5, false); // Nuevo
  
  // Devuelve true si al menos un sensor está habilitado (implica que se intentó leer)
  return sensor1Enabled || sensor2Enabled || sensor3Enabled || sensor4Enabled || sensor5Enabled;
}

// Añade esta función para identificar qué sensor tiene la dirección 0x33
Adafruit_VL53L0X* VL53L0X_Manager::getSensorByAddress(uint8_t address) {
  // Necesitamos determinar qué objeto sensor está en la dirección indicada
  // Esta función solo funcionará después de la inicialización
  
  // Realizamos una prueba de comunicación con cada sensor en la dirección indicada
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    // El dispositivo existe en esta dirección
    // Ahora necesitamos saber cuál de nuestros sensores está en esa dirección
    
    // Esto es una aproximación - idealmente deberíamos almacenar las direcciones
    // de cada sensor durante la inicialización
    if (sensor1Enabled) {
      Wire.beginTransmission(address);
      Wire.write(0x00); // Intentamos leer un registro
      if (Wire.endTransmission() == 0) {
        return &sensor1;
      }
    }
    
    if (sensor2Enabled) {
      Wire.beginTransmission(address);
      Wire.write(0x00);
      if (Wire.endTransmission() == 0) {
        return &sensor2;
      }
    }
    
    if (sensor3Enabled) {
      Wire.beginTransmission(address);
      Wire.write(0x00);
      if (Wire.endTransmission() == 0) {
        return &sensor3;
      }
    }
    
    if (sensor4Enabled) {
      Wire.beginTransmission(address);
      Wire.write(0x00);
      if (Wire.endTransmission() == 0) {
        return &sensor4;
      }
    }
    
    if (sensor5Enabled) {
      Wire.beginTransmission(address);
      Wire.write(0x00);
      if (Wire.endTransmission() == 0) {
        return &sensor5;
      }
    }
  }
  
  return nullptr; // No se encontró sensor con esa dirección
}

// Modifica setupInterrupt para usar el sensor con dirección 0x33
bool VL53L0X_Manager::setupInterrupt(uint16_t threshold, VL53L0XCallbackFunction callback) {
  // Usar directamente sensor4 (trasero) para interrupciones, sin importar su dirección
  if (!sensor4Enabled) {
    Serial.println("Error: El sensor trasero (sensor 4) no está habilitado");
    return false;
  }
  
  // Podemos verificar su dirección para diagnóstico
  Wire.beginTransmission(0x33);
  bool direccionCorrecta = (Wire.endTransmission() == 0);
  
  Serial.print("Configurando interrupciones en el sensor trasero (dirección 0x33 ");
  Serial.print(direccionCorrecta ? "verificada" : "NO VERIFICADA");
  Serial.println(")");
  
  // Guardar parámetros
  thresholdDistance = threshold;
  callbackFunction = callback;
  globalCallback = callback;
  sensorWithInterrupt = &sensor4; // Usamos directamente sensor4
  
  // Configurar el pin de interrupción como entrada con pull-up
  pinMode(GPIO_INTERRUPT, INPUT_PULLUP);
  
  // CLAVE 1: Configurar GPIO para interrupciones
  Serial.println("Configurando GPIO para interrupciones...");
  VL53L0X_Error status = sensor4.setGpioConfig(
    VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
    VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW,
    VL53L0X_INTERRUPTPOLARITY_LOW);
  
  if (status != VL53L0X_ERROR_NONE) {
    Serial.print("Error al configurar GPIO de interrupción: ");
    Serial.println(status);
    return false;
  }
  
  // CLAVE 2: Establecer umbrales exactamente como en el ejemplo
  FixPoint1616_t lowThreshold = (threshold * 65536.0);
  FixPoint1616_t highThreshold = ((threshold + 50) * 65536.0);
  
  Serial.println("Configurando umbrales de interrupción...");
  status = sensor4.setInterruptThresholds(lowThreshold, highThreshold, true);
  if (status != VL53L0X_ERROR_NONE) {
    Serial.print("Error al configurar umbrales de interrupción: ");
    Serial.println(status);
    return false;
  }
  
  // CLAVE 3: Configurar modo continuo
  Serial.println("Configurando modo continuo...");
  status = sensor4.setDeviceMode(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, false);
  if (status != VL53L0X_ERROR_NONE) {
    Serial.print("Error al configurar modo continuo: ");
    Serial.println(status);
    return false;
  }
  
  // CLAVE 4: Iniciar medición - CRÍTICO
  Serial.println("Iniciando medición continua...");
  status = sensor4.startMeasurement();
  if (status != VL53L0X_ERROR_NONE) {
    Serial.print("Error al iniciar medición: ");
    Serial.println(status);
    return false;
  }
  
  // Verificar estado del pin después de configurar
  Serial.print("Estado del pin interrupción: ");
  Serial.println(digitalRead(GPIO_INTERRUPT) ? "HIGH" : "LOW");
  
  // Si está en LOW, limpiar
  if (digitalRead(GPIO_INTERRUPT) == LOW) {
    Serial.println("Pin ya activo, limpiando interrupciones iniciales...");
    sensor4.clearInterruptMask(false);
    delay(100);
  }
  
  // Configurar interrupción ESP32
  Serial.println("Adjuntando ISR a pin de interrupción...");
  attachInterrupt(digitalPinToInterrupt(GPIO_INTERRUPT), handleVL53L0XInterrupt, FALLING);
  interruptEnabled = true;
  
  Serial.println("Configuración de interrupción completada");
  return true;
}

// Configurar interrupción para sensor 1 (Izquierdo)
bool VL53L0X_Manager::setupInterrupt1(uint16_t threshold, VL53L0XCallbackFunction callback) {
  if (!sensor1Enabled) {
    Serial.println("Error: Sensor 1 (Izquierdo) no está habilitado");
    return false;
  }
  
  // Guardar callback
  callback1 = callback;
  globalCallback1 = callback;
  
  // Configurar pin de interrupción
  pinMode(GPIO_INTERRUPT_1, INPUT_PULLUP);
  
  // Configurar sensor
  bool success = setupSensorInterrupt(sensor1, threshold);
  
  if (success) {
    // Adjuntar interrupción
    attachInterrupt(digitalPinToInterrupt(GPIO_INTERRUPT_1), handleVL53L0XInterrupt1, FALLING);
    interrupt1Enabled = true;
    Serial.println("Interrupción configurada para sensor 1 (Izquierdo)");
  }
  
  return success;
}

// Configurar interrupción para sensor 2 (Frontal)
bool VL53L0X_Manager::setupInterrupt2(uint16_t threshold, VL53L0XCallbackFunction callback) {
  if (!sensor2Enabled) {
    Serial.println("Error: Sensor 2 (Frontal) no está habilitado");
    return false;
  }
  
  // Guardar callback
  callback2 = callback;
  globalCallback2 = callback;
  
  // Configurar pin de interrupción
  pinMode(GPIO_INTERRUPT_2, INPUT_PULLUP);
  
  // Configurar sensor
  bool success = setupSensorInterrupt(sensor2, threshold);
  
  if (success) {
    // Adjuntar interrupción
    attachInterrupt(digitalPinToInterrupt(GPIO_INTERRUPT_2), handleVL53L0XInterrupt2, FALLING);
    interrupt2Enabled = true;
    Serial.println("Interrupción configurada para sensor 2 (Frontal)");
  }
  
  return success;
}

// Configurar interrupción para sensor 3 (Derecho)
bool VL53L0X_Manager::setupInterrupt3(uint16_t threshold, VL53L0XCallbackFunction callback) {
  if (!sensor3Enabled) {
    Serial.println("Error: Sensor 3 (Derecho) no está habilitado");
    return false;
  }
  
  // Guardar callback
  callback3 = callback;
  globalCallback3 = callback;
  
  // Configurar pin de interrupción
  pinMode(GPIO_INTERRUPT_3, INPUT_PULLUP);
  
  // Configurar sensor
  bool success = setupSensorInterrupt(sensor3, threshold);
  
  if (success) {
    // Adjuntar interrupción
    attachInterrupt(digitalPinToInterrupt(GPIO_INTERRUPT_3), handleVL53L0XInterrupt3, FALLING);
    interrupt3Enabled = true;
    Serial.println("Interrupción configurada para sensor 3 (Derecho)");
  }
  
  return success;
}

// Configurar interrupción para sensor 4 (Trasero)
bool VL53L0X_Manager::setupInterrupt4(uint16_t threshold, VL53L0XCallbackFunction callback) {
  if (!sensor4Enabled) {
    Serial.println("Error: Sensor 4 (Trasero) no está habilitado");
    return false;
  }
  
  // Guardar callback
  callback4 = callback;
  globalCallback4 = callback;
  
  // Configurar pin de interrupción
  pinMode(GPIO_INTERRUPT_4, INPUT_PULLUP);
  
  // Configurar sensor
  bool success = setupSensorInterrupt(sensor4, threshold);
  
  if (success) {
    // Adjuntar interrupción
    attachInterrupt(digitalPinToInterrupt(GPIO_INTERRUPT_4), handleVL53L0XInterrupt4, FALLING);
    interrupt4Enabled = true;
    Serial.println("Interrupción configurada para sensor 4 (Trasero)");
  }
  
  return success;
}

// Configurar interrupción para sensor 5 (Inferior)
bool VL53L0X_Manager::setupInterrupt5(uint16_t threshold, VL53L0XCallbackFunction callback) {
  Serial.println("CONFIGURANDO EXCLUSIVAMENTE EL SENSOR #5 (INFERIOR)");
  
  if (!sensor5Enabled) {
    Serial.println("ERROR: Sensor inferior no está habilitado");
    return false;
  }
  
  // Guardar callback
  callback5 = callback;
  globalCallback5 = callback;
  
  // Verificar IO23 está configurado como entrada con pull-up
  Serial.println("Configurando pin IO23 para interrupciones del sensor inferior");
  pinMode(GPIO_INTERRUPT_5, INPUT_PULLUP);
  
  // Mostrar estado inicial del pin de interrupción
  Serial.print("Estado inicial de IO23: ");
  Serial.println(digitalRead(GPIO_INTERRUPT_5) ? "HIGH" : "LOW");
  
  // Configurar sensor - misma configuración óptima que el sensor trasero
  Serial.println("Aplicando configuración optimizada al sensor inferior:");
  bool success = setupSensorInterrupt(sensor5, threshold);
  
  if (success) {
    // Adjuntar interrupción
    attachInterrupt(digitalPinToInterrupt(GPIO_INTERRUPT_5), handleVL53L0XInterrupt5, FALLING);
    interrupt5Enabled = true;
    Serial.println("✓ Interrupción del sensor inferior configurada correctamente en IO23");
  } else {
    Serial.println("✗ Error al configurar interrupción del sensor inferior");
  }
  
  return success;
}

// Implementa los métodos similares para los sensores 3, 4 y 5...

// Método para configurar todas las interrupciones
bool VL53L0X_Manager::setupAllInterrupts(uint16_t threshold1, uint16_t threshold2,
                                        uint16_t threshold3, uint16_t threshold4, 
                                        uint16_t threshold5,
                                        VL53L0XCallbackFunction callback1,
                                        VL53L0XCallbackFunction callback2,
                                        VL53L0XCallbackFunction callback3,
                                        VL53L0XCallbackFunction callback4,
                                        VL53L0XCallbackFunction callback5) {
  bool success = true;
  
  if (sensor1Enabled) success &= setupInterrupt1(threshold1, callback1);
  if (sensor2Enabled) success &= setupInterrupt2(threshold2, callback2);
  if (sensor3Enabled) success &= setupInterrupt3(threshold3, callback3);
  if (sensor4Enabled) success &= setupInterrupt4(threshold4, callback4);
  if (sensor5Enabled) success &= setupInterrupt5(threshold5, callback5);
  
  return success;
}

// Modificar clearInterrupt para seguir exactamente el ejemplo
bool VL53L0X_Manager::clearInterrupt() {
  if (!interruptEnabled || !sensor4Enabled) {
    Serial.println("No se puede limpiar: interrupción no habilitada o sensor trasero deshabilitado");
    return false;
  }
  
  Serial.println("Limpiando máscara de interrupción...");
  VL53L0X_Error status = sensor4.clearInterruptMask(false);
  
  if (status != VL53L0X_ERROR_NONE) {
    Serial.print("Error al limpiar interrupción: ");
    Serial.println(status);
    return false;
  }
  
  Serial.println("Interrupción limpiada correctamente");
  return true;
}

void VL53L0X_Manager::processInterrupt() {
  if (!interruptEnabled || !sensor4Enabled) {
    Serial.println("No se puede procesar: interrupción no habilitada o sensor trasero deshabilitado");
    return;
  }
  
  // Obtener la medición
  VL53L0X_RangingMeasurementData_t measure;
  Serial.print("Leyendo medición... ");
  sensor4.getRangingMeasurement(&measure, false);
  
  if (measure.RangeStatus != 4) {
    Serial.print("Distancia (mm): ");
    Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println("Fuera de rango");
  }
  
  // Limpiar interrupción después de leer
  clearInterrupt();
}

// Implementación de los nuevos métodos:

bool VL53L0X_Manager::setupAllInterrupts(uint16_t threshold1, uint16_t threshold2,
                                        uint16_t threshold3, uint16_t threshold4, 
                                        uint16_t threshold5, VL53L0XCallbackFunction callback) {
  // Guardar callback global
  globalCallback = callback;
  
  // Configurar el pin de interrupción como entrada con pull-up
  pinMode(GPIO_INTERRUPT, INPUT_PULLUP);
  
  bool success = true;
  
  // Configurar interrupciones en los sensores habilitados
  if (sensor1Enabled) {
    Serial.print("Configurando interrupción en Sensor 1 con umbral de ");
    Serial.print(threshold1);
    Serial.println(" mm");
    success &= setupSensorInterrupt(sensor1, threshold1);
  }
  
  if (sensor2Enabled) {
    Serial.print("Configurando interrupción en Sensor 2 con umbral de ");
    Serial.print(threshold2);
    Serial.println(" mm");
    success &= setupSensorInterrupt(sensor2, threshold2);
  }
  
  if (sensor3Enabled) {
    Serial.print("Configurando interrupción en Sensor 3 con umbral de ");
    Serial.print(threshold3);
    Serial.println(" mm");
    success &= setupSensorInterrupt(sensor3, threshold3);
  }
  
  if (sensor4Enabled) {
    Serial.print("Configurando interrupción en Sensor 4 con umbral de ");
    Serial.print(threshold4);
    Serial.println(" mm");
    success &= setupSensorInterrupt(sensor4, threshold4);
  }
  
  if (sensor5Enabled) {
    Serial.print("Configurando interrupción en Sensor 5 con umbral de ");
    Serial.print(threshold5);
    Serial.println(" mm");
    success &= setupSensorInterrupt(sensor5, threshold5);
  }
  
  // Configurar interrupción ESP32
  Serial.println("Adjuntando ISR a pin de interrupción...");
  attachInterrupt(digitalPinToInterrupt(GPIO_INTERRUPT), handleVL53L0XInterrupt, FALLING);
  interruptEnabled = true;
  
  return success;
}

// Método privado para configurar interrupción en un sensor individual
bool VL53L0X_Manager::setupSensorInterrupt(Adafruit_VL53L0X &sensor, uint16_t threshold) {
  // CLAVE 1: Configuración exactamente igual que el sensor trasero
  Serial.println("Configurando GPIO para interrupciones...");
  VL53L0X_Error status = sensor.setGpioConfig(
    VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
    VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW,
    VL53L0X_INTERRUPTPOLARITY_LOW);
  
  if (status != VL53L0X_ERROR_NONE) {
    Serial.print("Error al configurar GPIO de interrupción: ");
    Serial.println(status);
    return false;
  }
  
  // CLAVE 2: Establecer umbrales exactamente igual que el sensor trasero
  FixPoint1616_t lowThreshold = (threshold * 65536.0);
  FixPoint1616_t highThreshold = ((threshold + 50) * 65536.0); // Añadir margen de 50mm como en setupInterrupt()
  
  Serial.println("Configurando umbrales de interrupción...");
  status = sensor.setInterruptThresholds(lowThreshold, highThreshold, true); // Igual que el sensor trasero
  if (status != VL53L0X_ERROR_NONE) {
    Serial.print("Error al configurar umbrales de interrupción: ");
    Serial.println(status);
    return false;
  }
  
  // CLAVE 3: Modo continuo - idéntico al sensor trasero
  Serial.println("Configurando modo continuo...");
  status = sensor.setDeviceMode(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, false);
  if (status != VL53L0X_ERROR_NONE) {
    Serial.print("Error al configurar modo continuo: ");
    Serial.println(status);
    return false;
  }
  
  // CLAVE 4: Iniciar medición - crítico para el funcionamiento
  Serial.println("Iniciando medición continua...");
  status = sensor.startMeasurement();
  if (status != VL53L0X_ERROR_NONE) {
    Serial.print("Error al iniciar medición: ");
    Serial.println(status);
    return false;
  }
  
  // Verificar estado del pin después de configurar - igual que en sensor trasero
  Serial.print("Sensor configurado correctamente");
  
  return true;
}

// Limpiar interrupciones en todos los sensores habilitados
void VL53L0X_Manager::clearAllInterrupts() {
  // Primero limpiamos todas las interrupciones
  if (sensor1Enabled) {
    sensor1.clearInterruptMask(false);
    // Reiniciar medición continua
    sensor1.startMeasurement();
  }
  
  if (sensor2Enabled) {
    sensor2.clearInterruptMask(false);
    // Reiniciar medición continua
    sensor2.startMeasurement();
  }
  
  if (sensor3Enabled) {
    sensor3.clearInterruptMask(false);
    // Reiniciar medición continua
    sensor3.startMeasurement();
  }
  
  if (sensor4Enabled) {
    sensor4.clearInterruptMask(false);
    // Reiniciar medición continua
    sensor4.startMeasurement();
  }
  
  if (sensor5Enabled) {
    sensor5.clearInterruptMask(false);
    // Reiniciar medición continua
    sensor5.startMeasurement();
  }
  
  // Agregamos un pequeño retardo para que los sensores se estabilicen
  delayMicroseconds(100);
}

// Add after processInterrupt():
bool VL53L0X_Manager::clearSensorInterrupt(int sensorNum) {
  VL53L0X_Error status = VL53L0X_ERROR_NONE;
  
  switch (sensorNum) {
    case 1:
      if (sensor1Enabled) {
        status = sensor1.clearInterruptMask(false);
        sensor1.startMeasurement(); // Restart measurement
      }
      break;
    case 2:
      if (sensor2Enabled) {
        status = sensor2.clearInterruptMask(false);
        sensor2.startMeasurement();
      }
      break;
    case 3:
      if (sensor3Enabled) {
        status = sensor3.clearInterruptMask(false);
        sensor3.startMeasurement();
      }
      break;
    case 4:
      if (sensor4Enabled) {
        status = sensor4.clearInterruptMask(false);
        sensor4.startMeasurement();
      }
      break;
    case 5:
      if (sensor5Enabled) {
        status = sensor5.clearInterruptMask(false);
        sensor5.startMeasurement();
      }
      break;
    default:
      return false;
  }
  
  return (status == VL53L0X_ERROR_NONE);
}