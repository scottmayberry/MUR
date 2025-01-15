// SensorManager.h
// Header file for the SensorManager class.
// Manages various sensors, their initialization, data processing, and JSON serialization
// for the MUR underwater robot's compute and battery fuselage boards.

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

// Include necessary libraries and sensor headers
#include <Wire.h>                      // I2C communication library
#include <SPI.h>                       // SPI communication library
#include <ArduinoJson.h>               // JSON handling library
#include <Adafruit_AHTX0.h>            // AHT20 humidity and temperature sensor library
#include <Adafruit_DPS310.h>           // DPS310 pressure and temperature sensor library
#include "kxtj3-1057.h"                // KXTJ3_1057 accelerometer library
#include <Adafruit_LIS2MDL.h>          // LIS2MDL magnetometer library
#include <Adafruit_Sensor.h>           // Adafruit sensor abstraction library
#include <MPU6500_WE.h>                // MPU6500 accelerometer and gyroscope library
#include <Adafruit_LSM6DS3TRC.h>       // LSM6DS3TR-C accelerometer and gyroscope library
#include <Adafruit_BNO055.h>           // BNO055 absolute orientation sensor library
#include <utility/imumaths.h>          // IMU math utilities
#include "hscdtd008a.h"                // HSCDTD008A humidity and temperature sensor library
#include "MS5837.h"                    // MS5837 pressure sensor library
#include <FlexCAN_T4.h>                // FlexCAN library for CAN bus communication
#include <RadioLib.h>                  // Radio library for CC1101 module

class SensorManager {
public:
    // Constructor: Initializes the SensorManager
    SensorManager();

    // Initializes and sets up all sensors
    void setupSensors();

    // Processes sensor data and prepares it for transmission
    // Returns true if data is ready to transmit
    bool processSensors(DynamicJsonDocument& transmitDoc);

    // Resets the timers for sensor updates
    void reset_timers();

private:
    bool message_ready_to_transmit; // Flag indicating if a message is ready to transmit

    // Publish Rates for Sensors
    float pose_update_rate_hz = 50;     // Update rate in Hz for pose-related sensors
    bool pose_group_toggle = false;      // Toggle to alternate sensor data addition
    elapsedMicros pose_elapsed_micros;  // Timer for pose-related sensors

    float env_update_rate_hz = 5;        // Update rate in Hz for environmental sensors
    elapsedMicros env_elapsed_micros;    // Timer for environmental sensors

    // Sensor Setup Methods
    void setupLeakPins();                        // Sets up leak detection pins and interrupts
    static void leak_detected_interrupt();       // ISR for leak detection
    void add_leak_detected_to_json(DynamicJsonDocument& transmitDoc); // Adds leak detection status to JSON

    void setupLSM6DS3TR();                       // Sets up LSM6DS3TR-C sensor
    void add_LSM6DS3TR_to_json(DynamicJsonDocument& transmitDoc); // Adds LSM6DS3TR-C data to JSON

    void setupMPU6500();                         // Sets up MPU6500 sensor
    void add_MPU6500_to_json(DynamicJsonDocument& transmitDoc);   // Adds MPU6500 data to JSON

    void setupKXTJ3_1057();                      // Sets up KXTJ3_1057 accelerometer
    void add_KXTJ3_1057_to_json(DynamicJsonDocument& transmitDoc); // Adds KXTJ3_1057 data to JSON

    void setupLIS2MDL();                         // Sets up LIS2MDL magnetometer
    void add_LIS2MDL_to_json(DynamicJsonDocument& transmitDoc);    // Adds LIS2MDL data to JSON

    void setupHSCDTD008A();                      // Sets up HSCDTD008A sensor
    void add_HSCDTD_to_json(DynamicJsonDocument& transmitDoc);     // Adds HSCDTD008A data to JSON

    void setupBNO055();                          // Sets up BNO055 sensor
    void add_BNO055_to_json(DynamicJsonDocument& transmitDoc);      // Adds BNO055 data to JSON

    void setupAHT20();                           // Sets up AHT20 sensor
    void add_AHT20_to_json(DynamicJsonDocument& transmitDoc);        // Adds AHT20 data to JSON

    void setupDPS310();                          // Sets up DPS310 sensor
    void add_DPS310_to_json(DynamicJsonDocument& transmitDoc);       // Adds DPS310 data to JSON

    void setupMS5837();                          // Sets up MS5837 sensor
    void add_MS5837_to_json(DynamicJsonDocument& transmitDoc);       // Adds MS5837 data to JSON

    void processPoseSensors(DynamicJsonDocument& transmitDoc);       // Processes pose-related sensors
    void processEnv(DynamicJsonDocument& transmitDoc);               // Processes environmental sensors

    void setupMagIntPin();                       // Sets up magnetometer interrupt pin
    void add_Mag_Int_to_json(DynamicJsonDocument& transmitDoc);     // Adds magnetometer interrupt status to JSON
    static void mag_int_detected_interrupt();    // ISR for magnetometer interrupt

    // Utility Method
    float hz_to_microseconds_p_period(float);    // Converts Hz to microseconds per period

    // Static Member Variables
    static bool _leak_detected;                  // Static flag for leak detection
    static bool _mag_switch_on;                  // Static flag for magnetometer switch status
};

#endif // SENSOR_MANAGER_H
