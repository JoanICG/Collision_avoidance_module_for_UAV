#include <Wire.h>
#include "vl53l0x_manager.h"
#include <Arduino.h> // For pinMode, digitalWrite, delay, Serial


// Define and initialize the global sensors array
sensorList_t sensors[]{
    {new Adafruit_VL53L0X(), &Wire, LEFT, 0x30, 35, 34,
     Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
    {new Adafruit_VL53L0X(), &Wire, FRONT, 0x31, 23, 25,
     Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
    {new Adafruit_VL53L0X(), &Wire, RIGHT, 0x32, 33, 32,
     Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
    {new Adafruit_VL53L0X(), &Wire, BACK, 0x33, 18, 19,
     Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
    {new Adafruit_VL53L0X(), &Wire, BOTTOM, 0x34, 27, 26,
     Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0}
};

// Define COUNT_SENSORS
const int COUNT_SENSORS = sizeof(sensors) / sizeof(sensors[0]);

// Definition for the global manager instance
VL53L0X_Manager vl53l0xSensors;

// --- Constructor ---
VL53L0X_Manager::VL53L0X_Manager() {
    // Inicialización de variables miembro si es necesario
    for (int i = 0; i < COUNT_SENSORS; ++i) {
        sensors[i].sensor_status = -1; // Inicializar el estado del sensor como no detectado
        pinMode(sensors[i].shutdown_pin, OUTPUT);
        pinMode(sensors[i].interrupt_pin, INPUT_PULLUP); // Configurar pines de interrupción
    }
}

// --- Inicialización ---
int VL53L0X_Manager::begin() {
    #ifdef DEBUG_VL
        Serial.println("Iniciando gestor de sensores VL53L0X...");
    #endif

    // 1. Poner todos los sensores en reset (XSHUT LOW) y configurar pines
    #ifdef DEBUG_VL
        Serial.println("Reseteando todos los sensores VL53L0X y configurando pines...");
    #endif
    for (int i = 0; i < COUNT_SENSORS; ++i) 
        digitalWrite(sensors[i].shutdown_pin, LOW); // Poner en reset 
    delay(50); // Pequeña pausa tras resetear todos
    
    // 2. Inicializar y asignar direcciones I2C únicas secuencialmente
    int sensors_initialized_count = 0;

    for (int i = 0; i < COUNT_SENSORS; ++i) {
        #ifdef DEBUG_VL
                Serial.printf("--- Inicializando sensor %d (Posición: %d, Pin XSHUT: %d) ---\n",
                            i + 1, static_cast<SensorPosition>(i), sensors[i].shutdown_pin);
        #endif

        // Sacar SOLO el sensor actual del reset
        digitalWrite(sensors[i].shutdown_pin, HIGH);
        delay(10); // Dar tiempo al sensor para arrancar

        // Pre-check: Ping I2C address 0x29 (default address)
        Wire.beginTransmission(VL53L0X_DEFAULT_ADDRESS);
        if (Wire.endTransmission() != 0) {
            #ifdef DEBUG_VL
                Serial.printf("ERROR: Sensor %d (Pos: %d) no detectado en 0x29 . No se llamará a sensors[%d].begin().\n",
                              i + 1, sensors[i].shutdown_pin, i);
            #endif
            digitalWrite(sensors[i].shutdown_pin, LOW); // Mantener en reset si no se detecta
            sensors[i].sensor_status = -1; // Marcar como no detectado
            continue; // Saltar al siguiente sensor
        }

        #ifdef DEBUG_VL
        Serial.printf("Sensor %d (Pos: %d) detectado en 0x29 (pre-check OK). Llamando a sensors[%d].begin()...\n",
                      i + 1, sensors[i].shutdown_pin, i);
        #endif
        if (!sensors[i].psensor->begin(sensors[i].address, false, sensors[i].pwire,
                                        sensors[i].sensor_config)) { // Explicitly pass default address and Wire
            #ifdef DEBUG_VL
                Serial.printf("ERROR: Fallo en sensors[%d].begin() para sensor %d (Pos: %d) (después de pre-check OK).\n",
                              i, i + 1, sensors[i].address);
            #endif
            digitalWrite(sensors[i].shutdown_pin, LOW); // Poner en reset si begin() falla
            sensors[i].sensor_status = -1;
            continue; // Saltar al siguiente sensor
        }

        #ifdef DEBUG_VL
            Serial.printf("Sensor %d (Pos: %d) inicializado correctamente en 0x29.\n", i + 1, sensors[i].address);
        #endif

        // Verificar si el sensor responde en la nueva (o actual si era 0x29) dirección
        Wire.beginTransmission(sensors[i].address);
        byte address_check_error = Wire.endTransmission();

        if (address_check_error == 0) {
            #ifdef DEBUG_VL
                Serial.printf("Sensor %d (Pos: %d) responde correctamente en 0x%02X.\n",
                              i + 1, sensors[i].address, sensors[i].address);
            #endif
            sensors[i].sensor_status = 0; // Marcar como inicializado]
            sensors_initialized_count++;
            // Configurar modo por defecto (ej: Standard)
        } else {
            #ifdef DEBUG_VL
                Serial.printf("ERROR: Sensor %d (Pos: %d) no responde en la dirección 0x%02X después del cambio/confirmación (Error I2C: %d).\n",
                              i + 1, sensors[i].address, sensors[i].address, address_check_error);
            #endif
            digitalWrite(sensors[i].shutdown_pin, LOW); // Poner en reset si no responde en la nueva dirección
            sensors[i].sensor_status = -1;
        }
        #ifdef DEBUG_VL
            Serial.println("---");
        #endif
    }

    #ifdef DEBUG_VL
    if (sensors_initialized_count == COUNT_SENSORS) {
        Serial.println("Todos los sensores VL53L0X disponibles inicializados correctamente.");
    } else if (sensors_initialized_count > 0) {
        Serial.printf("ADVERTENCIA: Se inicializaron %d de %d sensores: ", sensors_initialized_count, COUNT_SENSORS);
        for (int k = 0; k < COUNT_SENSORS; ++k) {
            if (sensors[k].sensor_status >= 0) {
                Serial.printf("%d(0x%02X) ", k + 1, VL53L0X_DEFAULT_ADDRESS + k);
            }
        }
        Serial.println(".");
    } else {
        Serial.println("ERROR CRÍTICO: Ningún sensor VL53L0X pudo ser inicializado.");
    }
    #endif

    return (sensors_initialized_count > 0) ? 0 : -1; // Retornar 0 si al menos un sensor está habilitado, -1 si ninguno
}

// --- Configuración ---
int VL53L0X_Manager::configureSensorMode(SensorPosition sensor, SensorMode mode) {
    // Código para configurar el modo de un sensor específico
    #ifdef DEBUG_VL
        Serial.printf("Configurando sensor %d en modo %d...\n", sensor, mode);
    #endif
    if(sensors[sensor].sensor_status < 0) {
        #ifdef DEBUG_VL
            Serial.printf("Sensor %d no habilitado. No se puede configurar el modo.\n", sensor);    
        #endif
        return -1; // Si el sensor no está habilitado, salir de la función
    }
    switch (mode) {
        case MODE_STANDARD:
            sensors[sensor].psensor->setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 14);
            sensors[sensor].psensor->setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 10);
            sensors[sensor].psensor->setMeasurementTimingBudgetMicroSeconds(33000);
            break;
        case MODE_LONG_RANGE:
            sensors[sensor].psensor->setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
            sensors[sensor].psensor->setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
            sensors[sensor].psensor->setMeasurementTimingBudgetMicroSeconds(200000);
            break;
        case MODE_HIGH_PRECISION:
            sensors[sensor].psensor->setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 14);
            sensors[sensor].psensor->setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 10);
            sensors[sensor].psensor->setMeasurementTimingBudgetMicroSeconds(200000); // Aumentar el tiempo de medición
            break;
    }
    return 0;
}

int VL53L0X_Manager::setAllSensorsMode(SensorMode mode) {
    #ifdef DEBUG_VL
        Serial.printf("Configurando todos los sensores en modo %d...\n", mode);
    #endif
    bool error = false;
    // Código para configurar el modo de todos los sensores
    for (int i = 0; i < COUNT_SENSORS; ++i) {
        if(configureSensorMode(static_cast<SensorPosition>(i), mode) != 0) {
            error = true; // Si falla al configurar un sensor, marcar error
        }
    }
    return error ? -1 : 0; 
}

// Configura interrupción para un sensor específico
int VL53L0X_Manager::setupSensorInterrupt(SensorPosition sensor, uint16_t threshold, VL53L0XCallbackFunction callback) {
    #ifdef DEBUG_VL
        Serial.printf("Configurando interrupción para sensor %d con umbral %d...\n", sensor, threshold);
    #endif
    if(sensors[sensor].sensor_status != 0) {
        #ifdef DEBUG_VL
            Serial.printf("Sensor %d no habilitado. No se puede configurar la interrupción.\n", sensor);    
        #endif
        return -1; // Si el sensor no está habilitado, salir de la función
    }
    Serial.printf("Configurando interrupción para el sensor %d...\n", sensor);
    // Attach interrupt callback function
    attachInterrupt(digitalPinToInterrupt(sensors[sensor].interrupt_pin), callback, CHANGE);
    
    // Configurar el sensor para la interrupción
    Wire.beginTransmission(sensors[sensor].address);
    if (Wire.endTransmission() != 0) {
        #ifdef DEBUG_VL
        Serial.printf("ERROR: Sensor %d no responde en la dirección I2C 0x%02X.\n", sensor, sensors[sensor].address);
        #endif
        return -1; // Salir con error
    }
    
    Serial.printf("Configurando asdasd para el sensor %d...\n", sensor);
    sensors[sensor].psensor->setGpioConfig(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
                                   VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW,
                                   VL53L0X_INTERRUPTPOLARITY_LOW);

    // Configurar los límites de interrupción
    // 50mm as low threshold - converting to FixPoint1616_t format (Q16.16)
    FixPoint1616_t lowThreshold = threshold << 16; // Left shift by 16 bits for fixed-point format
    FixPoint1616_t highThreshold = threshold << 16; // Convert to FixPoint1616_t format
    #ifdef DEBUG_VL
        Serial.printf("Configurando límites de interrupción: Bajo %d, Alto %d...\n", lowThreshold, highThreshold);
        Serial.println("Set Mode VL53L0X_DEVICEMODE_CONTINUOUS_RANGING... ");        
    #endif
    sensors[sensor].psensor->setInterruptThresholds(lowThreshold, highThreshold, true);
    sensors[sensor].psensor->setDeviceMode(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, true);

    Serial.printf("Configurando bbb para el sensor %d...\n", sensor);
    sensors[sensor].psensor->startMeasurement();

    return 0;
}

int VL53L0X_Manager::clearInterrupt(SensorPosition sensor) {
    #ifdef DEBUG_VL
        Serial.printf("Limpiando interrupción para sensor %d...\n", sensor);
    #endif

    if(sensors[sensor].sensor_status < 0) {
        #ifdef DEBUG_VL
            Serial.printf("Sensor %d no habilitado. No se puede limpiar la interrupción.\n", sensor);    
        #endif
        return -1; // Si el sensor no está habilitado, salir de la función
    }
    
    sensors[sensor].psensor->clearInterruptMask(false);
    Serial.printf("Limpiado\n");
    return 0; // Placeholder
}


bool VL53L0X_Manager::isSensorEnabled(SensorPosition sensor) {
    if(sensors[sensor].sensor_status >= 0) {
        Serial.printf("Sensor %d está habilitado.\n", sensor);
        return true; // Sensor está habilitado
    } else {
        Serial.printf("Sensor %d NO está habilitado.\n", sensor);

        return false; // Sensor no está habilitado
    }
}

// Add VL53L0X_Manager:: to the function definition
int VL53L0X_Manager::getMesaurement(SensorPosition sensor, VL53L0X_RangingMeasurementData_t *measure){

    #ifdef DEBUG_VL
        Serial.printf("Obteniendo medición del sensor %d...\n", static_cast<int>(sensor)); // Cast enum to int for printf
    #endif

    if(sensors[sensor].sensor_status < 0) { // Access sensorEnabled as a class member
        #ifdef DEBUG_VL
            Serial.printf("Sensor %d no habilitado. No se puede obtener la medición.\n", static_cast<int>(sensor));    
        #endif
        return -1; // Si el sensor no está habilitado, salir de la función
    }
    // The Wire.beginTransmission/endTransmission check here is redundant if the sensor is already known to be enabled
    // and its address is correctly set. This check is more for initial discovery or if I2C bus is unstable.
    // Consider if this check is needed every time or only if an error is suspected.
    // For now, keeping it as per your original code.
   
    sensors[sensor].psensor->getRangingMeasurement(measure);
    return measure->RangeMilliMeter;
}


// --- Modos de Operación (Aplicar a todos) ---

void VL53L0X_Manager::setStandardMode() {
    #ifdef DEBUG_VL
        Serial.println("Configurando todos los sensores en modo estándar...");
    #endif
    for (int i = 0; i < COUNT_SENSORS; ++i) {
        configureSensorMode(static_cast<SensorPosition>(i), MODE_STANDARD);
    }
}

void VL53L0X_Manager::setLongRangeMode() {
    #ifdef DEBUG_VL
        Serial.println("Configurando todos los sensores en modo de largo alcance...");
    #endif
    for (int i = 0; i < COUNT_SENSORS; ++i) {
        configureSensorMode(static_cast<SensorPosition>(i), MODE_LONG_RANGE);
    }
}

void VL53L0X_Manager::setPrecisionMode() {
    #ifdef DEBUG_VL
        Serial.println("Configurando todos los sensores en modo de alta precisión...");
    #endif
    for (int i = 0; i < COUNT_SENSORS; ++i) {
        configureSensorMode(static_cast<SensorPosition>(i), MODE_HIGH_PRECISION);
    }
}