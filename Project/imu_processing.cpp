#include "imu_processing.h"

IMUProcessor::IMUProcessor(uint8_t device_id) : mpu(), deviceID(device_id) {}

bool IMUProcessor::begin() {
    Wire.begin();
    mpu.initialize();
    
    if (!mpu.testConnection()) {
        Serial.print("⚠️ MPU6050 (");
        Serial.print(deviceID);
        Serial.println(") connection failed!");
        return false;
    }
    Serial.print("✅ MPU6050 (");
    Serial.print(deviceID);
    Serial.println(") connected.");
    return true;
}

void IMUProcessor::update() {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    
    roll = atan2(ay, az) * 180 / PI;
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;
}

float IMUProcessor::getRoll() {
    return roll;
}

float IMUProcessor::getPitch() {
    return pitch;
}

uint8_t IMUProcessor::getDeviceID() {  // ✅ This function returns device ID
    return deviceID;
}
