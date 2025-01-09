// SensorManager.cpp
// Implementation of the SensorManager class for managing various sensors
// on the MUR underwater robot's compute and battery fuselage boards.

#include "config.h"               // Include configuration settings
#include "SensorManager.h"        // Include the SensorManager class definition
#include <Adafruit_Sensor.h>      // Include Adafruit sensor library

// Conditional compilation based on the board type (Compute or Fuselage)
#ifdef BOARD_COMPUTE
    // Define pin numbers specific to the Compute board
    #define LEAK_DETECTOR_pin 22            // Pin connected to the leak detector
    HardwareSerial& gpsSerial = Serial2;     // GPS serial communication on Serial2

    // Radio module configuration
    #define GDO0_pin 15                     // GDO0 pin for the radio module
    #define GDO2_pin 14                     // GDO2 pin for the radio module
    #define RADIO_CS_pin 16                 // Chip Select pin for the radio module
    CC1101 radio = new Module(RADIO_CS_pin, GDO0_pin, RADIOLIB_NC, GDO2_pin);  // Initialize CC1101 radio module

    // Magnetometer switch pin
    #define MAG_INT_pin 17                  // Interrupt pin for the magnetometer switch

    // Initialize CAN bus on CAN2 with specified buffer sizes
    FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can;  // Using CAN2 for Compute board

#elif defined(BOARD_FUSELAGE)
    // Define pin numbers specific to the Fuselage board
    #define LEAK_DETECTOR_pin 14            // Pin connected to the leak detector
    HardwareSerial& gpsSerial = Serial4;     // GPS serial communication on Serial4

    // Initialize CAN bus on CAN1 with specified buffer sizes
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;  // Using CAN1 for Fuselage board

    HardwareSerial& extSerial2 = Serial1;      // Additional external serial communication on Serial1
#endif

// Sensor Instances Initialization

// Accelerometer & Gyroscope (LSM6DS3TR-C)
Adafruit_LSM6DS3TRC lsm6ds3trc;                // Initialize LSM6DS3TR-C sensor
bool LSM6DS3TR_enabled_flag = true;            // Flag to indicate if LSM6DS3TR-C is enabled

// MPU6500 Gyroscope & Accelerometer
#define MPU6500_ADDR 0x68                       // I2C address for MPU6500
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR); // Initialize MPU6500 sensor
bool MPU6500_enabled_flag = true;               // Flag to indicate if MPU6500 is enabled

// Accelerometer KXTJ3_1057
#define HIGH_RESOLUTION                         // Enable high-resolution mode
#define KXTJ3_1057_ADDR 0x0E                    // I2C address for KXTJ3_1057
float KXTJ3_1057_sample_rate = 50;              // Sample rate in Hz for KXTJ3_1057
uint8_t KXTJ3_1057_accelRange = 2;              // Accelerometer range for KXTJ3_1057
bool KXTJ3_1057_enabled_flag = true;            // Flag to indicate if KXTJ3_1057 is enabled
KXTJ3 myIMU(KXTJ3_1057_ADDR);                   // Initialize KXTJ3_1057 sensor

// Magnetometer (LIS2MDL)
Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(12345); // Initialize LIS2MDL magnetometer with unique identifier
bool LIS2MDL_enabled_flag = true;                   // Flag to indicate if LIS2MDL is enabled

// HSCDTD008A Humidity and Temperature Sensor
HSCDTD008A hscdt;                            // Initialize HSCDTD008A sensor
hscdtd_status_t hscdtd_status;               // Variable to hold HSCDTD008A status
bool HSCDT_enabled_flag = true;               // Flag to indicate if HSCDTD008A is enabled

// IMU - BNO055 Absolute Orientation Sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // Initialize BNO055 with sensor ID and I2C address
bool BNO055_enabled_flag = true;                // Flag to indicate if BNO055 is enabled

// GPS Module
bool gps_enabled_flag = true;               // Flag to indicate if GPS is enabled
const char gps_header[] = "\"GPS\":";        // JSON header for GPS data
char gps_buffer[1024];                       // Buffer to store incoming GPS data
int gps_buffer_index = 0;                    // Index for GPS buffer
int gps_text_flag_counter = 0;               // Counter for GPS text flags

// Humidity & Temperature Sensor (AHT20)
Adafruit_AHTX0 aht;                           // Initialize AHT20 sensor
bool AHT20_enabled_flag = true;               // Flag to indicate if AHT20 is enabled

// Pressure & Temperature Sensor (DPS310)
Adafruit_DPS310 dps;                          // Initialize DPS310 sensor
bool DPS310_enabled_flag = true;              // Flag to indicate if DPS310 is enabled

// Pressure Sensor (MS5837)
MS5837 ms5837_sensor;                         // Initialize MS5837 sensor
bool MS5837_enabled_flag = true;              // Flag to indicate if MS5837 is enabled

// External Serial Communication
HardwareSerial& extSerial1 = Serial5;         // External serial communication on Serial5

// Static Member Variables Initialization
bool SensorManager::_leak_detected = false;   // Static flag to indicate if a leak is detected
bool SensorManager::_mag_switch_on = false;   // Static flag to indicate if magnetometer switch is on

// Constructor for the SensorManager class
SensorManager::SensorManager() {}

// Sets up all sensors and initializes communication interfaces
void SensorManager::setupSensors() {
    setupLeakPins();          // Configure leak detection pins and interrupts
    Wire.begin();             // Initialize I2C communication
    SPI.begin();              // Initialize SPI communication

    delay(10000);             // Wait for 10 seconds to allow sensors to stabilize
    setupAHT20();             // Initialize AHT20 sensor
    setupDPS310();            // Initialize DPS310 sensor
    setupKXTJ3_1057();        // Initialize KXTJ3_1057 accelerometer
    setupLIS2MDL();           // Initialize LIS2MDL magnetometer
    setupHSCDTD008A();        // Initialize HSCDTD008A sensor
    setupMPU6500();           // Initialize MPU6500 sensor
    setupLSM6DS3TR();         // Initialize LSM6DS3TR-C sensor
    setupBNO055();            // Initialize BNO055 sensor
    setupMS5837();            // Initialize MS5837 pressure sensor
    gpsSerial.begin(9600);    // Start GPS serial communication at 9600 baud rate

    #ifdef BOARD_COMPUTE
    setupMagIntPin();          // Configure magnetometer interrupt pin if on Compute board
    #endif

    pose_elapsed_micros = 0;  // Reset pose sensor timer
    env_elapsed_micros = 0;   // Reset environmental sensor timer
}

// Processes sensor data and prepares it for transmission
bool SensorManager::processSensors(DynamicJsonDocument& transmitDoc) {
    processPoseSensors(transmitDoc);  // Process pose-related sensors
    processEnv(transmitDoc);          // Process environmental sensors

    if(message_ready_to_transmit){
        message_ready_to_transmit = false; // Reset the transmission flag
        return true;                        // Indicate that data is ready to transmit
    }
    return false;                           // Indicate that no data is ready to transmit
}

// Sets up leak detection pins and attaches interrupts
void SensorManager::setupLeakPins() {
    pinMode(LEAK_DETECTOR_pin, INPUT);                           // Set leak detector pin as input
    attachInterrupt(digitalPinToInterrupt(LEAK_DETECTOR_pin), leak_detected_interrupt, RISING); // Attach interrupt on rising edge
}

// Interrupt Service Routine for leak detection
void SensorManager::leak_detected_interrupt() {
    SensorManager::_leak_detected = true; // Set the leak detected flag
}

// Adds leak detection status to the JSON document
void SensorManager::add_leak_detected_to_json(DynamicJsonDocument& transmitDoc) {
    if (!SensorManager::_leak_detected) return;    // Exit if no leak detected
    transmitDoc["leak"]["on"] = SensorManager::_leak_detected; // Add leak status to JSON
    SensorManager::_leak_detected = false;        // Reset the leak detected flag
}

// Sets up the LSM6DS3TR-C accelerometer and gyroscope
void SensorManager::setupLSM6DS3TR() {
    if (!lsm6ds3trc.begin_I2C()) { // Initialize LSM6DS3TR-C via I2C
        Serial.println("[LSM6DS3TR] Failed to find LSM6DS3TR-C chip"); // Print error if initialization fails
        LSM6DS3TR_enabled_flag = false; // Disable LSM6DS3TR-C if not found
    } else {
        Serial.println("[LSM6DS3TR] Initialized"); // Confirm successful initialization
    }
}

// Adds LSM6DS3TR-C sensor data to the JSON document
void SensorManager::add_LSM6DS3TR_to_json(DynamicJsonDocument& transmitDoc) {
    if (!LSM6DS3TR_enabled_flag) return; // Exit if LSM6DS3TR-C is disabled
    sensors_event_t accel, gyro, temp;
    lsm6ds3trc.getEvent(&accel, &gyro, &temp); // Retrieve accelerometer, gyroscope, and temperature data
    transmitDoc["LSM6DS3"]["acc_x"] = accel.acceleration.x; // Add accelerometer X-axis data
    transmitDoc["LSM6DS3"]["acc_y"] = accel.acceleration.y; // Add accelerometer Y-axis data
    transmitDoc["LSM6DS3"]["acc_z"] = accel.acceleration.z; // Add accelerometer Z-axis data
    transmitDoc["LSM6DS3"]["gyr_x"] = gyro.gyro.x;          // Add gyroscope X-axis data
    transmitDoc["LSM6DS3"]["gyr_y"] = gyro.gyro.y;          // Add gyroscope Y-axis data
    transmitDoc["LSM6DS3"]["gyr_z"] = gyro.gyro.z;          // Add gyroscope Z-axis data
    transmitDoc["LSM6DS3"]["temp"] = temp.temperature;      // Add temperature data
}

// Sets up the MPU6500 accelerometer and gyroscope
void SensorManager::setupMPU6500() {
    if (!myMPU6500.init()) { // Initialize MPU6500
        Serial.println("[MPU6500] Failed to initialize"); // Print error if initialization fails
        MPU6500_enabled_flag = false; // Disable MPU6500 if not initialized
    } else {
        Serial.println("[MPU6500] Connected"); // Confirm successful connection
        delay(2000); // Wait for 2 seconds
        myMPU6500.autoOffsets(); // Automatically calculate and set offsets
        myMPU6500.enableGyrDLPF(); // Enable Gyroscope Digital Low Pass Filter
        myMPU6500.setGyrDLPF(MPU6500_DLPF_3); // Set Gyroscope DLPF to setting 3
        myMPU6500.setSampleRateDivider(5); // Set sample rate divider
        myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250); // Set gyroscope range to ±250 degrees/sec
        myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G); // Set accelerometer range to ±2g
        myMPU6500.enableAccDLPF(true); // Enable Accelerometer DLPF
        myMPU6500.setAccDLPF(MPU6500_DLPF_3); // Set Accelerometer DLPF to setting 3
        delay(200); // Wait for 200 milliseconds
    }
}

// Adds MPU6500 sensor data to the JSON document
void SensorManager::add_MPU6500_to_json(DynamicJsonDocument& transmitDoc) {
    if (!MPU6500_enabled_flag) return; // Exit if MPU6500 is disabled
    xyzFloat gValue = myMPU6500.getGValues(); // Get accelerometer values
    xyzFloat gyr = myMPU6500.getGyrValues();   // Get gyroscope values
    float tempMPU6500 = myMPU6500.getTemperature(); // Get temperature value
    float resultantG = myMPU6500.getResultantG(gValue); // Calculate resultant G-force
    transmitDoc["MPU6500"]["acc_x"] = gValue.x; // Add accelerometer X-axis data
    transmitDoc["MPU6500"]["acc_y"] = gValue.y; // Add accelerometer Y-axis data
    transmitDoc["MPU6500"]["acc_z"] = gValue.z; // Add accelerometer Z-axis data
    transmitDoc["MPU6500"]["gyr_x"] = gyr.x;    // Add gyroscope X-axis data
    transmitDoc["MPU6500"]["gyr_y"] = gyr.y;    // Add gyroscope Y-axis data
    transmitDoc["MPU6500"]["gyr_z"] = gyr.z;    // Add gyroscope Z-axis data
    transmitDoc["MPU6500"]["temp"] = tempMPU6500; // Add temperature data
    transmitDoc["MPU6500"]["g"] = resultantG;   // Add resultant G-force data
}

// Sets up the KXTJ3_1057 accelerometer
void SensorManager::setupKXTJ3_1057() {
    if (myIMU.begin(KXTJ3_1057_sample_rate, KXTJ3_1057_accelRange) != 0) { // Initialize KXTJ3_1057 with sample rate and range
        Serial.println("[KXTJ3-1057] Failed to initialize accel."); // Print error if initialization fails
        KXTJ3_1057_enabled_flag = false; // Disable KXTJ3_1057 if not initialized
    } else {
        Serial.println("[KXTJ3-1057] accel initialized."); // Confirm successful initialization
        uint8_t readData = 0;
        myIMU.readRegister(&readData, KXTJ3_WHO_AM_I); // Read WHO_AM_I register for verification
        Serial.print("[KXTJ3-1057] Who am I? 0x");
        Serial.println(readData, HEX); // Print WHO_AM_I value in hexadecimal
        myIMU.standby(false); // Disable standby mode for continuous operation
    }
}

// Adds KXTJ3_1057 accelerometer data to the JSON document
void SensorManager::add_KXTJ3_1057_to_json(DynamicJsonDocument& transmitDoc) {
    if (!KXTJ3_1057_enabled_flag) return; // Exit if KXTJ3_1057 is disabled
    transmitDoc["KXTJ3"]["acc_x"] = myIMU.axisAccel(X); // Add accelerometer X-axis data
    transmitDoc["KXTJ3"]["acc_y"] = myIMU.axisAccel(Y); // Add accelerometer Y-axis data
    transmitDoc["KXTJ3"]["acc_z"] = myIMU.axisAccel(Z); // Add accelerometer Z-axis data
}

// Sets up the LIS2MDL magnetometer
void SensorManager::setupLIS2MDL() {
    lis2mdl.enableAutoRange(true); // Enable automatic range adjustment
    if (!lis2mdl.begin()) { // Initialize LIS2MDL
        Serial.println("[LIS2MDL] Failed to initialize mag sensor."); // Print error if initialization fails
        LIS2MDL_enabled_flag = false; // Disable LIS2MDL if not initialized
        return;
    }
    Serial.println("[LIS2MDL] Mag Initialized."); // Confirm successful initialization
}

// Adds LIS2MDL magnetometer data to the JSON document
void SensorManager::add_LIS2MDL_to_json(DynamicJsonDocument& transmitDoc) {
    if (!LIS2MDL_enabled_flag) return; // Exit if LIS2MDL is disabled
    sensors_event_t event;
    lis2mdl.getEvent(&event); // Retrieve magnetometer data
    transmitDoc["LIS2MDL"]["x"] = event.magnetic.x;     // Add magnetic field X-axis data
    transmitDoc["LIS2MDL"]["y"] = event.magnetic.y;     // Add magnetic field Y-axis data
    transmitDoc["LIS2MDL"]["z"] = event.magnetic.z;     // Add magnetic field Z-axis data
    transmitDoc["LIS2MDL"]["x_raw"] = lis2mdl.raw.x;    // Add raw magnetic field X-axis data
    transmitDoc["LIS2MDL"]["y_raw"] = lis2mdl.raw.y;    // Add raw magnetic field Y-axis data
    transmitDoc["LIS2MDL"]["z_raw"] = lis2mdl.raw.z;    // Add raw magnetic field Z-axis data
}

// Sets up the HSCDTD008A humidity and temperature sensor
void SensorManager::setupHSCDTD008A() {
    hscdt.begin(); // Initialize HSCDTD008A sensor
    hscdtd_status = hscdt.initialize(); // Initialize sensor settings
    if (hscdtd_status != HSCDTD_STAT_OK) { // Check if initialization was successful
        Serial.println("[HSCDTD] Failed to initialize mag sensor."); // Print error if initialization fails
        HSCDT_enabled_flag = false; // Disable HSCDTD008A if not initialized
        return;
    }
    Serial.println("[HSCDTD] Mag Initialized."); // Confirm successful initialization
    hscdt.temperatureCompensation(); // Apply temperature compensation
    hscdt.setStateNormal();           // Set sensor to normal operating state
    hscdt.configureOutputDataRate(HSCDTD_ODR_100HZ); // Configure output data rate to 100 Hz
}

// Adds HSCDTD008A sensor data to the JSON document
void SensorManager::add_HSCDTD_to_json(DynamicJsonDocument& transmitDoc) {
    if (!HSCDT_enabled_flag) return; // Exit if HSCDTD008A is disabled
    hscdtd_status = hscdt.isDataReady(); // Check if data is ready
    if (hscdtd_status == HSCDTD_STAT_OK) { // If data is ready
        hscdt.retrieveMagData(); // Retrieve magnetometer data
        transmitDoc["HSCDTD"]["x"] = hscdt.mag.mag_x; // Add magnetometer X-axis data
        transmitDoc["HSCDTD"]["y"] = hscdt.mag.mag_y; // Add magnetometer Y-axis data
        transmitDoc["HSCDTD"]["z"] = hscdt.mag.mag_z; // Add magnetometer Z-axis data
    }
}

// Sets up the BNO055 absolute orientation sensor
void SensorManager::setupBNO055() {
    if (!bno.begin()) { // Initialize BNO055 sensor
        Serial.println("[BNO055] not detected"); // Print error if initialization fails
        BNO055_enabled_flag = false; // Disable BNO055 if not initialized
        return;
    }
    Serial.println("[BNO055] Connected"); // Confirm successful connection
    delay(1000); // Wait for 1 second
    bno.setExtCrystalUse(true); // Use external crystal for better accuracy
}

// Adds BNO055 sensor data to the JSON document
void SensorManager::add_BNO055_to_json(DynamicJsonDocument& transmitDoc) {
    if (!BNO055_enabled_flag) return; // Exit if BNO055 is disabled
    sensors_event_t quat_event, angVelocityEvent, linearAccelEvent;
    bno.getEvent(&quat_event); // Get quaternion event data
    imu::Quaternion quat = bno.getQuat(); // Get quaternion values
    transmitDoc["BNO055"]["q"]["w"] = (float)quat.w(); // Add quaternion w component
    transmitDoc["BNO055"]["q"]["x"] = (float)quat.x(); // Add quaternion x component
    transmitDoc["BNO055"]["q"]["y"] = (float)quat.y(); // Add quaternion y component
    transmitDoc["BNO055"]["q"]["z"] = (float)quat.z(); // Add quaternion z component
    bno.getEvent(&angVelocityEvent, Adafruit_BNO055::VECTOR_GYROSCOPE); // Get angular velocity data
    bno.getEvent(&linearAccelEvent, Adafruit_BNO055::VECTOR_LINEARACCEL); // Get linear acceleration data
    transmitDoc["BNO055"]["av"]["x"] = (float)angVelocityEvent.gyro.x; // Add angular velocity X-axis data
    transmitDoc["BNO055"]["av"]["y"] = (float)angVelocityEvent.gyro.y; // Add angular velocity Y-axis data
    transmitDoc["BNO055"]["av"]["z"] = (float)angVelocityEvent.gyro.z; // Add angular velocity Z-axis data
    transmitDoc["BNO055"]["la"]["x"] = (float)linearAccelEvent.acceleration.x; // Add linear acceleration X-axis data
    transmitDoc["BNO055"]["la"]["y"] = (float)linearAccelEvent.acceleration.y; // Add linear acceleration Y-axis data
    transmitDoc["BNO055"]["la"]["z"] = (float)linearAccelEvent.acceleration.z; // Add linear acceleration Z-axis data

    uint8_t sys, gyro, accel, mag = 0;
    bno.getCalibration(&sys, &gyro, &accel, &mag); // Get calibration status
    transmitDoc["BNO055"]["cal"]["sys"] = sys;      // Add system calibration status
    transmitDoc["BNO055"]["cal"]["gyro"] = gyro;    // Add gyroscope calibration status
    transmitDoc["BNO055"]["cal"]["accel"] = accel;  // Add accelerometer calibration status
    transmitDoc["BNO055"]["cal"]["mag"] = mag;      // Add magnetometer calibration status
}

// Sets up the AHT20 humidity and temperature sensor
void SensorManager::setupAHT20() {
    if (!aht.begin()) { // Initialize AHT20 sensor
        Serial.println("[AHT20] Failed to find AHT10/AHT20 chip"); // Print error if initialization fails
        AHT20_enabled_flag = false; // Disable AHT20 if not initialized
        return;
    }
    Serial.println("[AHT20] AHT10/AHT20 initialized"); // Confirm successful initialization
}

// Adds AHT20 sensor data to the JSON document
void SensorManager::add_AHT20_to_json(DynamicJsonDocument& transmitDoc) {
    if (!AHT20_enabled_flag) return; // Exit if AHT20 is disabled
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp); // Retrieve humidity and temperature data
    transmitDoc["AHT20"]["humidity"] = humidity.relative_humidity; // Add humidity data
    transmitDoc["AHT20"]["temp"] = temp.temperature;               // Add temperature data
}

// Sets up the DPS310 pressure and temperature sensor
void SensorManager::setupDPS310() {
    if (!dps.begin_I2C()) { // Initialize DPS310 sensor via I2C
        Serial.println("[DPS310] Failed to find DPS"); // Print error if initialization fails
        DPS310_enabled_flag = false; // Disable DPS310 if not initialized
        return;
    }
    Serial.println("[DPS310] DPS Initialized"); // Confirm successful initialization
    dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES); // Configure pressure sensor settings
    dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES); // Configure temperature sensor settings
}

// Adds DPS310 sensor data to the JSON document
void SensorManager::add_DPS310_to_json(DynamicJsonDocument& transmitDoc) {
    if (!DPS310_enabled_flag) return; // Exit if DPS310 is disabled
    sensors_event_t temp_event, pressure_event;
    dps.getEvents(&temp_event, &pressure_event); // Retrieve temperature and pressure data
    transmitDoc["DPS310"]["pressure"] = pressure_event.pressure; // Add pressure data
    transmitDoc["DPS310"]["temp"] = temp_event.temperature;       // Add temperature data
}

// Sets up the MS5837 pressure sensor
void SensorManager::setupMS5837() {
    if (!ms5837_sensor.init()) { // Initialize MS5837 sensor
        Serial.println("[MS5837] Failed to initialize"); // Print error if initialization fails
        MS5837_enabled_flag = false; // Disable MS5837 if not initialized
        return;
    }
    Serial.println("[MS5837] Initialized"); // Confirm successful initialization
    ms5837_sensor.setModel(MS5837::MS5837_02BA); // Set the sensor model
    ms5837_sensor.setFluidDensity(997); // Set fluid density for freshwater (997 kg/m^3, 1029 for seawater)
}

// Adds MS5837 sensor data to the JSON document
void SensorManager::add_MS5837_to_json(DynamicJsonDocument& transmitDoc) {
    if (!MS5837_enabled_flag) return; // Exit if MS5837 is disabled
    ms5837_sensor.read(); // Read sensor data
    transmitDoc["MS5837"]["pressure"] = ms5837_sensor.pressure(); // Add pressure data
    transmitDoc["MS5837"]["depth"] = ms5837_sensor.depth();         // Add depth data
    transmitDoc["MS5837"]["alt"] = ms5837_sensor.altitude();        // Add altitude data
    transmitDoc["MS5837"]["temp"] = ms5837_sensor.temperature();     // Add temperature data
}

// Converts frequency in Hz to microseconds per period
float SensorManager::hz_to_microseconds_p_period(float updateHz){
    return 1/updateHz*1000000; // Calculate microseconds per period based on Hz
}

// Processes pose-related sensors and adds their data to the JSON document
void SensorManager::processPoseSensors(DynamicJsonDocument& transmitDoc) {
    // Check if it's time to update pose sensors based on elapsed time
    if (pose_elapsed_micros >= hz_to_microseconds_p_period(pose_update_rate_hz * 2)) {
        pose_elapsed_micros -= int(hz_to_microseconds_p_period(pose_update_rate_hz * 2)); // Reset elapsed time

        // Safety check to prevent timer overflow
        if(pose_elapsed_micros > hz_to_microseconds_p_period(pose_update_rate_hz * 2)*10)
        {
            pose_elapsed_micros = 0; // Reset timer if overflow detected
        }

        // Toggle pose group to alternate sensor data addition
        if (pose_group_toggle) {
            add_LSM6DS3TR_to_json(transmitDoc); // Add LSM6DS3TR-C data
            add_MPU6500_to_json(transmitDoc);   // Add MPU6500 data
            add_HSCDTD_to_json(transmitDoc);    // Add HSCDTD008A data
            add_KXTJ3_1057_to_json(transmitDoc); // Add KXTJ3_1057 data
            add_LIS2MDL_to_json(transmitDoc);    // Add LIS2MDL data
            add_BNO055_to_json(transmitDoc);     // Add BNO055 data
        }
        message_ready_to_transmit = true; // Set the transmission flag
        pose_group_toggle = !pose_group_toggle; // Toggle the group for next cycle
    }
}

// Processes environmental sensors and adds their data to the JSON document
void SensorManager::processEnv(DynamicJsonDocument& transmitDoc) {
    // Check if it's time to update environmental sensors based on elapsed time
    if (env_elapsed_micros >= hz_to_microseconds_p_period(env_update_rate_hz)) {
        env_elapsed_micros -= int(hz_to_microseconds_p_period(env_update_rate_hz)); // Reset elapsed time

        // Safety check to prevent timer overflow
        if(env_elapsed_micros > hz_to_microseconds_p_period(env_update_rate_hz)*10)
        {
            env_elapsed_micros = 0; // Reset timer if overflow detected
        }
        add_DPS310_to_json(transmitDoc);      // Add DPS310 data
        add_AHT20_to_json(transmitDoc);       // Add AHT20 data
        add_MS5837_to_json(transmitDoc);      // Add MS5837 data
        add_Mag_Int_to_json(transmitDoc);     // Add magnetometer interrupt data
        add_leak_detected_to_json(transmitDoc); // Add leak detection status
        message_ready_to_transmit = true;      // Set the transmission flag
    }
}

// Adds magnetometer interrupt status to the JSON document
void SensorManager::add_Mag_Int_to_json(DynamicJsonDocument& transmitDoc){
    #ifdef BOARD_COMPUTE
        if (!SensorManager::_mag_switch_on) return;    // Exit if magnetometer switch is not on
        transmitDoc["mag_switch"]["on"] = SensorManager::_mag_switch_on; // Add magnetometer switch status
        SensorManager::_mag_switch_on = false;        // Reset the magnetometer switch flag
    #endif
    ;
}

// Sets up the magnetometer interrupt pin and attaches interrupts (Compute board only)
void SensorManager::setupMagIntPin() {
    #ifdef BOARD_COMPUTE
        pinMode(MAG_INT_pin, INPUT); // Set magnetometer interrupt pin as input
        attachInterrupt(digitalPinToInterrupt(MAG_INT_pin), mag_int_detected_interrupt, RISING); // Attach interrupt on rising edge
    #endif
    ;
}

// Interrupt Service Routine for magnetometer switch detection
void SensorManager::mag_int_detected_interrupt() {
    SensorManager::_mag_switch_on = true; // Set the magnetometer switch flag
}

// Resets the timers for sensor updates
void SensorManager::reset_timers(){
    env_elapsed_micros = 0;  // Reset environmental sensor timer
    pose_elapsed_micros = 0; // Reset pose sensor timer
}
