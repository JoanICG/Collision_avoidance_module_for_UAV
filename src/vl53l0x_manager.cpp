#include <Wire.h>
#include "vl53l0x_manager.h"
// Definición de la instancia global
VL53L0X_Manager vl53l0xSensors;

// --- Constructor ---
VL53L0X_Manager::VL53L0X_Manager() {
    // Inicialización de variables miembro si es necesario
    for (int i = 0; i < NSENSORS; ++i) {
        sensorEnabled[i] = false;
    }
}

// --- Inicialización ---
int VL53L0X_Manager::begin() {
    // Asumiendo que Wire.begin() y Wire.setClock() se llaman en main.cpp/setup()

    #ifdef DEBUG_VL
        Serial.println("Iniciando gestor de sensores VL53L0X...");
    #endif

    // 1. Poner todos los sensores en reset (XSHUT LOW) usando el array xshutPins
    #ifdef DEBUG_VL
        Serial.println("Reseteando todos los sensores VL53L0X...");
    #endif
    for (int i = 0; i < NSENSORS; ++i) {
        pinMode(xshutPins[i], OUTPUT);
        pinMode(interruptPins[i], INPUT_PULLUP); // Configurar pines de interrupción como INPUT_PULLUP
        digitalWrite(xshutPins[i], LOW);
    }
    delay(50); // Pequeña pausa tras resetear

    // 2. Inicializar y asignar direcciones I2C únicas secuencialmente
    bool all_sensors_ok = true;
    // bool sensors_found[NSENSORS] = {false, false, false, false, false}; // sensors_found is now part of the class or implicitly handled by sensorEnabled
    for (int i = 0; i < NSENSORS; ++i) {
        // Usar el índice 'i' que corresponde a SensorPosition (LEFT=0, FRONT=1, etc.)
        #ifdef DEBUG_VL
                SensorPosition currentPosition = static_cast<SensorPosition>(i);
                Serial.printf("--- Inicializando sensor %d (Posición: %d, Pin XSHUT: %d) ---\n",
                            i + 1, currentPosition, xshutPins[i]);
        #endif

        // Sacar el sensor actual del reset usando el array xshutPins
        digitalWrite(xshutPins[i], HIGH);
        delay(150); // Dar tiempo al sensor para arrancar (increased from 50ms)

        // Pre-check: Ping I2C address 0x29 before calling library's begin()
        Wire.beginTransmission(0x29);
        byte pre_check_error = Wire.endTransmission();

        if (pre_check_error != 0) {
            #ifdef DEBUG_VL
                Serial.printf("ERROR: Sensor %d no detectado en 0x29 (Error I2C pre-check: %d). No se llamará a sensors[%d].begin().\n", i + 1, pre_check_error, i);
            #endif
            digitalWrite(xshutPins[i], LOW); // Apagarlo para no interferir
            all_sensors_ok = false;
            sensorEnabled[i] = false;
            continue; // Saltar al siguiente sensor
        }

        // Intentar inicializar el sensor en la dirección I2C por defecto (0x29)
        #ifdef DEBUG_VL
        Serial.printf("Sensor %d detectado en 0x29 (pre-check OK). Llamando a sensors[%d].begin()...\n", i + 1, i);
        #endif
        if (!sensors[i].begin()) { // La librería Adafruit maneja la inicialización básica
            #ifdef DEBUG_VL
                        Serial.printf("ERROR: Fallo en sensors[%d].begin() (después de pre-check OK). Verifique conexiones.\n", i);
            #endif
            // Apagarlo de nuevo para no interferir con los siguientes
            digitalWrite(xshutPins[i], LOW);
            all_sensors_ok = false;
            sensorEnabled[i] = false; // Ensure it's marked as not enabled
            continue; // Saltar al siguiente sensor
        }

        // Sensor fue encontrado y sensors[i].begin() tuvo éxito
        #ifdef DEBUG_VL
                Serial.printf("Sensor %d inicializado correctamente en 0x29.\n", i + 1);
        #endif
        // sensors_found[i] = true; // Mark as found if you still use this array

        // Asignar nueva dirección I2C única (0x2A, 0x2B, ...)
        uint8_t newAddress = VL53L0X_FIRST_ADDRESS + i; // Asegúrate de definir VL53L0X_FIRST_ADDRESS (e.g., 0x2A)
        #ifdef DEBUG_VL
                Serial.printf("Cambiando dirección del sensor %d a 0x%02X...\n", i + 1, newAddress);
        #endif
        sensors[i].setAddress(newAddress);
        delay(10); // Pausa después de cambiar la dirección

        // Verificar si el sensor responde en la nueva dirección (importante)
        Wire.beginTransmission(newAddress);
        byte error = Wire.endTransmission();
        if (error == 0) {
            #ifdef DEBUG_VL
                        Serial.printf("Sensor %d responde correctamente en 0x%02X.\n", i + 1, newAddress);
            #endif
             sensorEnabled[i] = true; // Marcar como habilitado
             // sensors_found[i] = true; // Marcar como encontrado
             // Configurar modo por defecto (ej: Standard)
             // Es mejor llamar a configureSensorMode aquí si ya está implementada
             // Uncomment to use the configureSensorMode function instead of manual configuration
             configureSensorMode(static_cast<SensorPosition>(i), MODE_STANDARD);

            #ifdef DEBUG_VL
                        Serial.printf("Pin de interrupción %d (GPIO %d) configurado como INPUT_PULLUP.\n", i + 1, interruptPins[i]);
            #endif

            } else {
                #ifdef DEBUG_VL
                            Serial.printf("ERROR: Sensor %d no responde en la nueva dirección 0x%02X después del cambio (Error I2C: %d).\n", i + 1, newAddress, error);
                #endif
                // Apagarlo para no interferir
                digitalWrite(xshutPins[i], LOW);
                all_sensors_ok = false;
                // Dejar el sensor como no habilitado (sensorEnabled[i] ya es false)
            }
            #ifdef DEBUG_VL
                    Serial.println("---");
            #endif
    }

#ifdef DEBUG_VL
    if (all_sensors_ok == NSENSORS) {
        Serial.println("Todos los sensores VL53L0X disponibles inicializados correctamente.");
    } else if (all_sensors_ok < NSENSORS) {
         Serial.printf("ADVERTENCIA: Se inicializaron solo los sensores: ");
            for (int i = 0; i < NSENSORS; ++i) {
                if (sensorEnabled[i]) {
                    Serial.printf("%d ", i + 1); // Mostrar solo los habilitados
                }
            }
            Serial.println(".");  
    } else {
         Serial.println("ERROR CRÍTICO: Ningún sensor VL53L0X pudo ser inicializado.");
    }
#endif

    // Decidir qué retornar: ¿éxito solo si todos funcionan, o si al menos uno funciona?
    return all_sensors_ok ? 0 : -1; // Retornar 0 si todos los sensores están habilitados, -1 si no
}

// --- Configuración ---
int VL53L0X_Manager::configureSensorMode(SensorPosition sensor, SensorMode mode) {
    // Código para configurar el modo de un sensor específico
    #ifdef DEBUG_VL
        Serial.printf("Configurando sensor %d en modo %d...\n", sensor, mode);
    #endif
    if(sensorEnabled[sensor] == false) {
        #ifdef DEBUG_VL
            Serial.printf("Sensor %d no habilitado. No se puede configurar el modo.\n", sensor);    
        #endif
        return -1; // Si el sensor no está habilitado, salir de la función
    }
    switch (mode) {
        case MODE_STANDARD:
            sensors[sensor].setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 14);
            sensors[sensor].setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 10);
            sensors[sensor].setMeasurementTimingBudgetMicroSeconds(33000);
            break;
        case MODE_LONG_RANGE:
            sensors[sensor].setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
            sensors[sensor].setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
            sensors[sensor].setMeasurementTimingBudgetMicroSeconds(200000);
            break;
        case MODE_HIGH_PRECISION:
            sensors[sensor].setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 14);
            sensors[sensor].setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 10);
            sensors[sensor].setMeasurementTimingBudgetMicroSeconds(200000); // Aumentar el tiempo de medición
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
    for (int i = 0; i < NSENSORS; ++i) {
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
    if(sensorEnabled[sensor] == false) {
        #ifdef DEBUG_VL
            Serial.printf("Sensor %d no habilitado. No se puede configurar la interrupción.\n", sensor);    
        #endif
        return -1; // Si el sensor no está habilitado, salir de la función
    }

    // Attach interrupt callback function
    attachInterrupt(digitalPinToInterrupt(interruptPins[sensor]), callback, CHANGE);
    
    // Configurar el sensor para la interrupción
    sensors[sensor].setGpioConfig(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
                                   VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW,
                                   VL53L0X_INTERRUPTPOLARITY_LOW);

    // Configurar los límites de interrupción
    // 50mm as low threshold - converting to FixPoint1616_t format (Q16.16)
    FixPoint1616_t lowThreshold = 50 << 16; // Left shift by 16 bits for fixed-point format
    FixPoint1616_t highThreshold = threshold << 16; // Convert to FixPoint1616_t format
    #ifdef DEBUG_VL
        Serial.printf("Configurando límites de interrupción: Bajo %d, Alto %d...\n", lowThreshold, highThreshold);
        sensors[sensor].setInterruptThresholds(lowThreshold, highThreshold, true);
        Serial.println("Set Mode VL53L0X_DEVICEMODE_CONTINUOUS_RANGING... ");
        sensors[sensor].setDeviceMode(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, true);
    #else
        sensors[sensor].setInterruptThresholds(lowThreshold, highThreshold, false);
        sensors[sensor].setDeviceMode(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, false);
    #endif
    // Iniciar medición continua
    sensors[sensor].startMeasurement();

    return 0;
}

int VL53L0X_Manager::clearInterrupt(SensorPosition sensor) {
    #ifdef DEBUG_VL
        Serial.printf("Limpiando interrupción para sensor %d...\n", sensor);
    #endif

    if(sensorEnabled[sensor] == false) {
        #ifdef DEBUG_VL
            Serial.printf("Sensor %d no habilitado. No se puede limpiar la interrupción.\n", sensor);    
        #endif
        return -1; // Si el sensor no está habilitado, salir de la función
    }
    
    sensors[sensor].clearInterruptMask(false);

    return 0; // Placeholder
}


bool VL53L0X_Manager::isSensorEnabled(SensorPosition sensor) {
    // Código para verificar si un sensor está habilitado
   
    return sensorEnabled[sensor];

}

// --- Modos de Operación (Aplicar a todos) ---

void VL53L0X_Manager::setStandardMode() {
    #ifdef DEBUG_VL
        Serial.println("Configurando todos los sensores en modo estándar...");
    #endif
    for (int i = 0; i < NSENSORS; ++i) {
        configureSensorMode(static_cast<SensorPosition>(i), MODE_STANDARD);
    }
}

void VL53L0X_Manager::setLongRangeMode() {
    #ifdef DEBUG_VL
        Serial.println("Configurando todos los sensores en modo de largo alcance...");
    #endif
    for (int i = 0; i < NSENSORS; ++i) {
        configureSensorMode(static_cast<SensorPosition>(i), MODE_LONG_RANGE);
    }
}

void VL53L0X_Manager::setPrecisionMode() {
    #ifdef DEBUG_VL
        Serial.println("Configurando todos los sensores en modo de alta precisión...");
    #endif
    for (int i = 0; i < NSENSORS; ++i) {
        configureSensorMode(static_cast<SensorPosition>(i), MODE_HIGH_PRECISION);
    }
}