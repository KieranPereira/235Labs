/*
   Simple MPU6050 poll – no external libraries
   ESP32 Feather pins:
     SDA = GPIO 22
     SCL = GPIO 20
   Sensor address 0x68 (AD0 LOW)
*/

#include <Wire.h>

constexpr uint8_t SDA_PIN = 22;
constexpr uint8_t SCL_PIN = 20;
constexpr uint8_t MPU_ADDR = 0x68;

// MPU6050 register addresses
constexpr uint8_t PWR_MGMT_1 = 0x68;
constexpr uint8_t ACCEL_XOUT_H = 0x3B;

void setup()
{
  Serial.begin(115200);
  while (!Serial) { }

  Wire.begin(SDA_PIN, SCL_PIN, 100000);          // 100 kHz for robustness

  // wake up the MPU6050 (clear SLEEP bit)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    Serial.println("ERROR: no ACK at 0x68 – check wiring");
    while (1) { delay(1000); }
  }

  Serial.println("MPU6050 awake – streaming raw accel/gyro…");
}

void loop()
{
  uint8_t buf[14];

  // burst‑read 14 bytes starting at ACCEL_XOUT_H
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);                   // restart
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);

  for (int i = 0; i < 14 && Wire.available(); ++i)
    buf[i] = Wire.read();

  // unpack big‑endian 16‑bit signed integers
  int16_t ax = buf[0]  << 8 | buf[1];
  int16_t ay = buf[2]  << 8 | buf[3];
  int16_t az = buf[4]  << 8 | buf[5];
  int16_t gx = buf[8]  << 8 | buf[9];
  int16_t gy = buf[10] << 8 | buf[11];
  int16_t gz = buf[12] << 8 | buf[13];

  Serial.printf("ACC (raw): %6d %6d %6d  |  GYRO (raw): %6d %6d %6d\n",
                ax, ay, az, gx, gy, gz);

  delay(500);
}
