#include <BluetoothSerial.h>
#include "mpu6050_controller.hpp"

// Create a BluetoothSerial object.
BluetoothSerial serialBT;

// Create an instance of the MPU6050 controller.
MPU6050Controller controller;

// Variable to store incoming Bluetooth commands.
char cmd;

void setup() {
  // Start USB Serial for debugging.
  Serial.begin(115200);

  // Initialize Bluetooth with the device name "Esp32-BT".
  serialBT.begin("Esp32-BT");
  
  // Initialize the MPU6050 sensor.
  if (!controller.begin()) {
    Serial.println("Sensor initialization failed!");
    while (1) {
      delay(100);
    }
  }
}

void loop() {
  // Check if there is any Bluetooth data available.
  if (serialBT.available()) {
    cmd = serialBT.read();

    // Process commands received over Bluetooth.
    // '1' turns the LED on, and '0' turns it off.
    if (cmd == '1') {
      digitalWrite(LED_PIN, HIGH);
    } else if (cmd == '0') {
      digitalWrite(LED_PIN, LOW);
    }
  }
  
  // Update the sensor readings and output the JSON data over USB Serial.
  controller.update();

  delay(2);
}