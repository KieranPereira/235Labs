/**
 * @file main.ino
 * @brief Main application for controlling the MPU6050 sensor and communicating over Bluetooth.
 *
 * This application is designed for the ESP32 platform. It performs the following tasks:
 * - Initializes a Bluetooth serial connection with the device name "Feather-BT".
 * - Sets up the MPU6050 sensor using an instance of MPU6050Controller.
 * - Transmits sensor readings (acceleration, gyroscope, temperature, and a calculated angle)
 *   as JSON data over both Bluetooth and USB Serial.
 * - Listens for incoming Bluetooth commands to control an LED:
 *     - '1' turns the LED on.
 *     - '0' turns the LED off.
 *
 * @note If the sensor initialization fails, the application halts in an infinite loop.
 */

/**
 * @file main.ino
 * @brief Main sketch for MPU‑9250 + Madgwick fusion on ESP32, streaming JSON over Bluetooth.
 */

/**
 * @file main.ino
 * @brief Initialize Bluetooth + MPU9250Controller, then loop.
 */

/**
 * @file main.ino
 * @brief Initialize Bluetooth + MPU9250Controller, then loop.
 */

#include <Arduino.h>
#include <BluetoothSerial.h>
#include "mpu9250_controller.hpp"

BluetoothSerial SerialBT;          // ESP32 BluetoothSerial instance :contentReference[oaicite:2]{index=2}
MPU9250Controller controller;      // Our controller

void setup() {
  Serial.begin(115200);
  SerialBT.begin("WROOM-ESP-BT");  // Bluetooth name :contentReference[oaicite:3]{index=3}

  // Abort if IMU init fails
  if (!controller.begin(&Serial, &SerialBT)) {
    Serial.println("MPU9250 init failed");
    while (1) delay(100);
  }
}

void loop() {
  controller.update();  // read/fuse/stream JSON
  delay(2);
}
