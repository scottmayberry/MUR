#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Wire.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_DPS310.h>
#include "kxtj3-1057.h"
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>
#include <MPU6500_WE.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "hscdtd008a.h"
#include "MS5837.h"
#include <FlexCAN_T4.h>
#include <RadioLib.h>

class SensorManager {
public:
    SensorManager();
    void setupSensors();
    bool processSensors(DynamicJsonDocument& transmitDoc);
    void reset_timers();

private:
    bool message_ready_to_transmit;

    // Publish Rates for Sensors
    float pose_update_rate_hz = 50;
    bool pose_group_toggle = false;
    elapsedMicros pose_elapsed_micros;

    float env_update_rate_hz = 5;
    elapsedMicros env_elapsed_micros;

    void setupLeakPins();
    static void leak_detected_interrupt();
    void add_leak_detected_to_json(DynamicJsonDocument& transmitDoc);
    void setupLSM6DS3TR();
    void add_LSM6DS3TR_to_json(DynamicJsonDocument& transmitDoc);
    void setupMPU6500();
    void add_MPU6500_to_json(DynamicJsonDocument& transmitDoc);
    void setupKXTJ3_1057();
    void add_KXTJ3_1057_to_json(DynamicJsonDocument& transmitDoc);
    void setupLIS2MDL();
    void add_LIS2MDL_to_json(DynamicJsonDocument& transmitDoc);
    void setupHSCDTD008A();
    void add_HSCDTD_to_json(DynamicJsonDocument& transmitDoc);
    void setupBNO055();
    void add_BNO055_to_json(DynamicJsonDocument& transmitDoc);
    void setupAHT20();
    void add_AHT20_to_json(DynamicJsonDocument& transmitDoc);
    void setupDPS310();
    void add_DPS310_to_json(DynamicJsonDocument& transmitDoc);
    void setupMS5837();
    void add_MS5837_to_json(DynamicJsonDocument& transmitDoc);
    void processPoseSensors(DynamicJsonDocument& transmitDoc);
    void processEnv(DynamicJsonDocument& transmitDoc);
    void setupMagIntPin();
    void add_Mag_Int_to_json(DynamicJsonDocument& transmitDoc);
    static void mag_int_detected_interrupt();

    float hz_to_microseconds_p_period(float);

    static bool _leak_detected;
    static bool _mag_switch_on;
};

#endif // SENSOR_MANAGER_H
