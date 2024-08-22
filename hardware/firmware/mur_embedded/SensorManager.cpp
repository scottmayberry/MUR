#include "config.h"
#include "SensorManager.h"
#include <Adafruit_Sensor.h>



#ifdef BOARD_COMPUTE
#define LEAK_DETECTOR_pin 22
HardwareSerial& gpsSerial = Serial2; //gps serial

// radio
#define GDO0_pin 15
#define GDO2_pin 14
#define RADIO_CS_pin 16
CC1101 radio = new Module(RADIO_CS_pin, GDO0_pin, RADIOLIB_NC, GDO2_pin);

//Mag switch pin
#define MAG_INT_pin 17

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can;  // Using CAN2

#elif defined(BOARD_FUSELAGE)
#define LEAK_DETECTOR_pin 14
HardwareSerial& gpsSerial = Serial4; //gps serial
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;  // Using CAN1
HardwareSerial& extSerial2 = Serial1;
#endif


// Accelerometer & Gyro
Adafruit_LSM6DS3TRC lsm6ds3trc;
bool LSM6DS3TR_enabled_flag = true;

// MPU6500
#define MPU6500_ADDR 0x68
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);
bool MPU6500_enabled_flag = true;

// Accelerometer KXTJ3_1057
#define HIGH_RESOLUTION
#define KXTJ3_1057_ADDR 0x0E
float KXTJ3_1057_sample_rate = 50;  
uint8_t KXTJ3_1057_accelRange = 2;    
bool KXTJ3_1057_enabled_flag = true;
KXTJ3 myIMU(KXTJ3_1057_ADDR);  

// Magnetometer
Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(12345);
bool LIS2MDL_enabled_flag = true;

// HSCDTD008A
HSCDTD008A hscdt;
hscdtd_status_t hscdtd_status;
bool HSCDT_enabled_flag = true;

// IMU - BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
bool BNO055_enabled_flag = true;

// GPS
bool gps_enabled_flag = true;
const char gps_header[] = "\"GPS\":";
char gps_buffer[1024];
int gps_buffer_index = 0;
int gps_text_flag_counter = 0;

// Humidity & Temp
Adafruit_AHTX0 aht;
bool AHT20_enabled_flag = true;

// Pressure & Temp
Adafruit_DPS310 dps;
bool DPS310_enabled_flag = true;

// MS5837
MS5837 ms5837_sensor;
bool MS5837_enabled_flag = true;

// External Serial
HardwareSerial& extSerial1 = Serial5;

// Define the static member variables
bool SensorManager::_leak_detected = false;
bool SensorManager::_mag_switch_on = false;

SensorManager::SensorManager() {}

void SensorManager::setupSensors() {
    setupLeakPins();
    Wire.begin();
    SPI.begin();

    delay(10000);
    setupAHT20();
    setupDPS310();
    setupKXTJ3_1057();
    setupLIS2MDL();
    setupHSCDTD008A();
    setupMPU6500();
    setupLSM6DS3TR();
    setupBNO055();
    setupMS5837();
    gpsSerial.begin(9600);
    
    #ifdef BOARD_COMPUTE
    setupMagIntPin();
    #endif

    pose_elapsed_micros = 0;
    env_elapsed_micros = 0;
}

bool SensorManager::processSensors(DynamicJsonDocument& transmitDoc) {
    processPoseSensors(transmitDoc);
    processEnv(transmitDoc);
    if(message_ready_to_transmit){
        message_ready_to_transmit = false;
        return true;
    }
    return false;
}

void SensorManager::setupLeakPins() {
    pinMode(LEAK_DETECTOR_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(LEAK_DETECTOR_pin), leak_detected_interrupt, RISING);
}

void SensorManager::leak_detected_interrupt() {
    SensorManager::_leak_detected = true;
}

void SensorManager::add_leak_detected_to_json(DynamicJsonDocument& transmitDoc) {
    if (!SensorManager::_leak_detected) return;    
    transmitDoc["leak"]["on"] = SensorManager::_leak_detected;
    SensorManager::_leak_detected = false;
}

void SensorManager::setupLSM6DS3TR() {
    if (!lsm6ds3trc.begin_I2C()) {
        Serial.println("[LSM6DS3TR] Failed to find LSM6DS3TR-C chip");
        LSM6DS3TR_enabled_flag = false;
    } else {
        Serial.println("[LSM6DS3TR] Initialized");
    }
}

void SensorManager::add_LSM6DS3TR_to_json(DynamicJsonDocument& transmitDoc) {
    if (!LSM6DS3TR_enabled_flag) return;
    sensors_event_t accel, gyro, temp;
    lsm6ds3trc.getEvent(&accel, &gyro, &temp);
    transmitDoc["LSM6DS3"]["acc_x"] = accel.acceleration.x;
    transmitDoc["LSM6DS3"]["acc_y"] = accel.acceleration.y;
    transmitDoc["LSM6DS3"]["acc_z"] = accel.acceleration.z;
    transmitDoc["LSM6DS3"]["gyr_x"] = gyro.gyro.x;
    transmitDoc["LSM6DS3"]["gyr_y"] = gyro.gyro.y;
    transmitDoc["LSM6DS3"]["gyr_z"] = gyro.gyro.z;
    transmitDoc["LSM6DS3"]["temp"] = temp.temperature;
}

void SensorManager::setupMPU6500() {
    if (!myMPU6500.init()) {
        Serial.println("[MPU6500] Failed to initialize");
        MPU6500_enabled_flag = false;
    } else {
        Serial.println("[MPU6500] Connected");
        delay(2000);
        myMPU6500.autoOffsets();
        myMPU6500.enableGyrDLPF();
        myMPU6500.setGyrDLPF(MPU6500_DLPF_3);
        myMPU6500.setSampleRateDivider(5);
        myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
        myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
        myMPU6500.enableAccDLPF(true);
        myMPU6500.setAccDLPF(MPU6500_DLPF_3);
        delay(200);
    }
}

void SensorManager::add_MPU6500_to_json(DynamicJsonDocument& transmitDoc) {
    if (!MPU6500_enabled_flag) return;
    xyzFloat gValue = myMPU6500.getGValues();
    xyzFloat gyr = myMPU6500.getGyrValues();
    float tempMPU6500 = myMPU6500.getTemperature();
    float resultantG = myMPU6500.getResultantG(gValue);
    transmitDoc["MPU6500"]["acc_x"] = gValue.x;
    transmitDoc["MPU6500"]["acc_y"] = gValue.y;
    transmitDoc["MPU6500"]["acc_z"] = gValue.z;
    transmitDoc["MPU6500"]["gyr_x"] = gyr.x;
    transmitDoc["MPU6500"]["gyr_y"] = gyr.y;
    transmitDoc["MPU6500"]["gyr_z"] = gyr.z;
    transmitDoc["MPU6500"]["temp"] = tempMPU6500;
    transmitDoc["MPU6500"]["g"] = resultantG;
}

void SensorManager::setupKXTJ3_1057() {
    if (myIMU.begin(KXTJ3_1057_sample_rate, KXTJ3_1057_accelRange) != 0) {
        Serial.println("[KXTJ3-1057] Failed to initialize accel.");
        KXTJ3_1057_enabled_flag = false;
    } else {
        Serial.println("[KXTJ3-1057] accel initialized.");
        uint8_t readData = 0;
        myIMU.readRegister(&readData, KXTJ3_WHO_AM_I);
        Serial.print("[KXTJ3-1057] Who am I? 0x");
        Serial.println(readData, HEX);
        myIMU.standby(false);
    }
}

void SensorManager::add_KXTJ3_1057_to_json(DynamicJsonDocument& transmitDoc) {
    if (!KXTJ3_1057_enabled_flag) return;
    transmitDoc["KXTJ3"]["acc_x"] = myIMU.axisAccel(X);
    transmitDoc["KXTJ3"]["acc_y"] = myIMU.axisAccel(Y);
    transmitDoc["KXTJ3"]["acc_z"] = myIMU.axisAccel(Z);
}

void SensorManager::setupLIS2MDL() {
    lis2mdl.enableAutoRange(true);
    if (!lis2mdl.begin()) {
        Serial.println("[LIS2MDL] Failed to initialize mag sensor.");
        LIS2MDL_enabled_flag = false;
        return;
    }
    Serial.println("[LIS2MDL] Mag Initialized.");
}

void SensorManager::add_LIS2MDL_to_json(DynamicJsonDocument& transmitDoc) {
    if (!LIS2MDL_enabled_flag) return;
    sensors_event_t event;
    lis2mdl.getEvent(&event);
    transmitDoc["LIS2MDL"]["x"] = event.magnetic.x;
    transmitDoc["LIS2MDL"]["y"] = event.magnetic.y;
    transmitDoc["LIS2MDL"]["z"] = event.magnetic.z;
    transmitDoc["LIS2MDL"]["x_raw"] = lis2mdl.raw.x;
    transmitDoc["LIS2MDL"]["y_raw"] = lis2mdl.raw.y;
    transmitDoc["LIS2MDL"]["z_raw"] = lis2mdl.raw.z;
}

void SensorManager::setupHSCDTD008A() {
    hscdt.begin();
    hscdtd_status = hscdt.initialize();
    if (hscdtd_status != HSCDTD_STAT_OK) {
        Serial.println("[HSCDTD] Failed to initialize mag sensor.");
        HSCDT_enabled_flag = false;
        return;
    }
    Serial.println("[HSCDTD] Mag Initialized.");
    hscdt.temperatureCompensation();
    hscdt.setStateNormal();
    hscdt.configureOutputDataRate(HSCDTD_ODR_100HZ);
}

void SensorManager::add_HSCDTD_to_json(DynamicJsonDocument& transmitDoc) {
    if (!HSCDT_enabled_flag) return;
    hscdtd_status = hscdt.isDataReady();
    if (hscdtd_status == HSCDTD_STAT_OK) {
        hscdt.retrieveMagData();
        transmitDoc["HSCDTD"]["x"] = hscdt.mag.mag_x;
        transmitDoc["HSCDTD"]["y"] = hscdt.mag.mag_y;
        transmitDoc["HSCDTD"]["z"] = hscdt.mag.mag_z;
    }
}

void SensorManager::setupBNO055() {
    if (!bno.begin()) {
        Serial.println("[BNO055] not detected");
        BNO055_enabled_flag = false;
        return;
    }
    Serial.println("[BNO055] Connected");
    delay(1000);
    bno.setExtCrystalUse(true);
}

void SensorManager::add_BNO055_to_json(DynamicJsonDocument& transmitDoc) {
    if (!BNO055_enabled_flag) return;
    sensors_event_t quat_event, angVelocityEvent, linearAccelEvent;
    bno.getEvent(&quat_event);
    imu::Quaternion quat = bno.getQuat();
    transmitDoc["BNO055"]["q"]["w"] = (float)quat.w();
    transmitDoc["BNO055"]["q"]["x"] = (float)quat.x();
    transmitDoc["BNO055"]["q"]["y"] = (float)quat.y();
    transmitDoc["BNO055"]["q"]["z"] = (float)quat.z();
    bno.getEvent(&angVelocityEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelEvent, Adafruit_BNO055::VECTOR_LINEARACCEL);
    transmitDoc["BNO055"]["av"]["x"] = (float)angVelocityEvent.gyro.x;
    transmitDoc["BNO055"]["av"]["y"] = (float)angVelocityEvent.gyro.y;
    transmitDoc["BNO055"]["av"]["z"] = (float)angVelocityEvent.gyro.z;
    transmitDoc["BNO055"]["la"]["x"] = (float)linearAccelEvent.acceleration.x;
    transmitDoc["BNO055"]["la"]["y"] = (float)linearAccelEvent.acceleration.y;
    transmitDoc["BNO055"]["la"]["z"] = (float)linearAccelEvent.acceleration.z;
    uint8_t sys, gyro, accel, mag = 0;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    transmitDoc["BNO055"]["cal"]["sys"] = sys;
    transmitDoc["BNO055"]["cal"]["gyro"] = gyro;
    transmitDoc["BNO055"]["cal"]["accel"] = accel;
    transmitDoc["BNO055"]["cal"]["mag"] = mag;
}

void SensorManager::setupAHT20() {
    if (!aht.begin()) {
        Serial.println("[AHT20] Failed to find AHT10/AHT20 chip");
        AHT20_enabled_flag = false;
        return;
    }
    Serial.println("[AHT20] AHT10/AHT20 initialized");
}

void SensorManager::add_AHT20_to_json(DynamicJsonDocument& transmitDoc) {
    if (!AHT20_enabled_flag) return;
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    transmitDoc["AHT20"]["humidity"] = humidity.relative_humidity;
    transmitDoc["AHT20"]["temp"] = temp.temperature;
}

void SensorManager::setupDPS310() {
    if (!dps.begin_I2C()) {
        Serial.println("[DPS310] Failed to find DPS");
        DPS310_enabled_flag = false;
        return;
    }
    Serial.println("[DPS310] DPS Initialized");
    dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
}

void SensorManager::add_DPS310_to_json(DynamicJsonDocument& transmitDoc) {
    if (!DPS310_enabled_flag) return;
    sensors_event_t temp_event, pressure_event;
    dps.getEvents(&temp_event, &pressure_event);
    transmitDoc["DPS310"]["pressure"] = pressure_event.pressure;
    transmitDoc["DPS310"]["temp"] = temp_event.temperature;
}

void SensorManager::setupMS5837() {
    if (!ms5837_sensor.init()) {
        Serial.println("[MS5837] Failed to initialize");
        MS5837_enabled_flag = false;
        return;
    }
    Serial.println("[MS5837] Initialized");
    ms5837_sensor.setModel(MS5837::MS5837_02BA);
    ms5837_sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

void SensorManager::add_MS5837_to_json(DynamicJsonDocument& transmitDoc) {
    if (!MS5837_enabled_flag) return;
    ms5837_sensor.read();
    transmitDoc["MS5837"]["pressure"] = ms5837_sensor.pressure();
    transmitDoc["MS5837"]["depth"] = ms5837_sensor.depth();
    transmitDoc["MS5837"]["alt"] = ms5837_sensor.altitude();
    transmitDoc["MS5837"]["temp"] = ms5837_sensor.temperature();
}

float SensorManager::hz_to_microseconds_p_period(float updateHz){
    return 1/updateHz*1000000;
}

void SensorManager::processPoseSensors(DynamicJsonDocument& transmitDoc) {
    if (pose_elapsed_micros >= hz_to_microseconds_p_period(pose_update_rate_hz * 2)) {
        pose_elapsed_micros -= int(hz_to_microseconds_p_period(pose_update_rate_hz * 2));

        //safety check to prevent overflow
        if(pose_elapsed_micros > hz_to_microseconds_p_period(pose_update_rate_hz * 2)*10)
        {
            pose_elapsed_micros = 0;
        }

        if (pose_group_toggle) {
            add_LSM6DS3TR_to_json(transmitDoc);
            add_MPU6500_to_json(transmitDoc);
            add_HSCDTD_to_json(transmitDoc);
            add_KXTJ3_1057_to_json(transmitDoc);
            add_LIS2MDL_to_json(transmitDoc);
            add_BNO055_to_json(transmitDoc);
        }
        message_ready_to_transmit = true;
        pose_group_toggle = !pose_group_toggle;
    }
}

void SensorManager::processEnv(DynamicJsonDocument& transmitDoc) {
    if (env_elapsed_micros >= hz_to_microseconds_p_period(env_update_rate_hz)) {
        env_elapsed_micros -= int(hz_to_microseconds_p_period(env_update_rate_hz));
        //safety check to prevent overflow
        if(env_elapsed_micros > hz_to_microseconds_p_period(env_update_rate_hz)*10)
        {
            env_elapsed_micros = 0;
        }
        add_DPS310_to_json(transmitDoc);
        add_AHT20_to_json(transmitDoc);
        add_MS5837_to_json(transmitDoc);
        add_Mag_Int_to_json(transmitDoc);
        add_leak_detected_to_json(transmitDoc);
        message_ready_to_transmit = true;
    }
}

void SensorManager::add_Mag_Int_to_json(DynamicJsonDocument& transmitDoc){
    #ifdef BOARD_COMPUTE
        if (!SensorManager::_mag_switch_on) return;    
        transmitDoc["mag_switch"]["on"] = SensorManager::_mag_switch_on;
        SensorManager::_mag_switch_on = false;
    #endif
    ;
}

void SensorManager::setupMagIntPin() {
    #ifdef BOARD_COMPUTE
        pinMode(MAG_INT_pin, INPUT);
        attachInterrupt(digitalPinToInterrupt(MAG_INT_pin), mag_int_detected_interrupt, RISING);
    #endif
    ;
}

void SensorManager::mag_int_detected_interrupt() {
    SensorManager::_mag_switch_on = true;
}

void SensorManager::reset_timers(){
    env_elapsed_micros = 0;
    pose_elapsed_micros = 0;
}