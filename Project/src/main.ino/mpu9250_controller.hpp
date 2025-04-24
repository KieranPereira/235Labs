#ifndef MPU9250_CONTROLLER_H
#define MPU9250_CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>            // I²C :contentReference[oaicite:4]{index=4}
#include <MPU9250.h>         // hideakitai’s MPU9250 library :contentReference[oaicite:5]{index=5}
#include <ArduinoJson.h>     // JSON serialization :contentReference[oaicite:6]{index=6}
#include <BluetoothSerial.h> // ESP32 BT :contentReference[oaicite:7]{index=7}

#define I2C_SDA 21
#define I2C_SCL 22
#define LED_PIN 13
static const int UPDATE_INTERVAL_MS = 10;  // 100 Hz

class MPU9250Controller {
public:
  MPU9250Controller();
  bool begin(HardwareSerial* usb, BluetoothSerial* bt);
  void update();

private:
  HardwareSerial*  _usb;
  BluetoothSerial* _bt;
  MPU9250          mpu;       // driver + AHRS :contentReference[oaicite:8]{index=8}
  unsigned long    _lastMs;
  bool             _ledState;

  void streamJSON(float roll, float pitch, float yaw);
};

#endif // MPU9250_CONTROLLER_H
