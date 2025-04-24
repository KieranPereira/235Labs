#include "imu_processing.h"

IMUProcessor imu(1440); //ESP32 Upper Arm
//IMUProcessor imu(1430); //ESP32 Fore Arm
//IMUProcessor imu(1430); //ESP32 Forearm
// Change to 1410 for the second ESP32

void setup() {
    Serial.begin(115200);  // ✅ Only use Serial (no Bluetooth)
    Wire.begin();

   if (!imu.begin()) {
        while (1);  // Stop execution if IMU fails
    }
}

void loop() {
    imu.update();
    
    float roll = imu.getRoll();
    float pitch = imu.getPitch();

    Serial.print("ESP32_");
    Serial.print(imu.getDeviceID());
    Serial.print(" | Roll: ");
    Serial.print(roll);
    Serial.print(" | Pitch: ");
    Serial.println(pitch);

    delay(50);
}


