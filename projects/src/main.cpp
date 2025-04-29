// #include <Arduino.h>
// #include <Wire.h>
// #include "MPU6050.h"
// #include <BluetoothSerial.h>

// // Bluetooth Serial Object
// BluetoothSerial SerialBT;



// // MPU addresses
// MPU6050 imuBack(0x68);  // lower-back IMU, AD0→GND
// MPU6050 imuNeck(0x69);  // neck IMU,     AD0→VCC



// // Vibration-driver pins & channels
// static constexpr int BACK_L_PWM = 12;   // back-left motor PWM (roll < -TH) (Works!)
// static constexpr int BACK_R_PWM = 32;   // back-right motor PWM (roll > +TH) (Works!)

// static constexpr int CH_BACK_L = 0;
// static constexpr int CH_BACK_R = 1;
// static constexpr int SDA_pin = 22;  // SDA pin for I2C
// static constexpr int SCL_pin = 20;  // SCL pin for I2C

// static constexpr int NECK_UP_PWM = 14;  // neck-up motor PWM (pitch > +TH) (Works!)
// static constexpr int NECK_DN_PWM = 4;  // neck-down motor PWM (pitch < -TH) (Works!)

// static constexpr int CH_NECK_UP = 2;
// static constexpr int CH_NECK_DN = 3;

// // Settings
// static constexpr float TH_ROLL    = 15.0f;   // back roll threshold
// static constexpr float TH_PITCH   = 10.0f;   // neck pitch threshold

// static constexpr int   PWM_DUTY   = 150;     // 0–255
// static constexpr int   READ_MS    = 500;     // read/update interval

// void setup() {
//     Serial.begin(115200);
//     while (!Serial) { delay(10); }
//     SerialBT.begin("ESP32_Vibro"); // Bluetooth device name
//     Serial.println("The bluetooth device is ready to pair");
//     Wire.begin(SDA_pin, SCL_pin);
//     imuBack.initialize();
//     imuNeck.initialize();
//     Serial.println("Both IMUs up, now driving motors...");

//     // set up LEDC channels (ESP32 PWM)
//     ledcSetup(CH_BACK_L, 5000, 8);
//     ledcAttachPin(BACK_L_PWM, CH_BACK_L);

//     ledcSetup(CH_BACK_R, 5000, 8);
//     ledcAttachPin(BACK_R_PWM, CH_BACK_R);

//     ledcSetup(CH_NECK_UP, 5000, 8);
//     ledcAttachPin(NECK_UP_PWM, CH_NECK_UP);

//     ledcSetup(CH_NECK_DN, 5000, 8);
//     ledcAttachPin(NECK_DN_PWM, CH_NECK_DN);
// }

// void loop() {
//     int16_t ax, ay, az, gx, gy, gz;
//     float roll, pitch;

//     // -- READ BACK IMU --
//     imuNeck.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//     roll = atan2f(ay, az) * 57.2958f;  // radians to degrees

//     // roll-based vibration
//     if (roll >  TH_ROLL) {
//         ledcWrite(CH_BACK_R, PWM_DUTY);
//         ledcWrite(CH_BACK_L, 0);
//     }
//     else if (roll < -TH_ROLL) {
//         ledcWrite(CH_BACK_L, PWM_DUTY);
//         ledcWrite(CH_BACK_R, 0);
//     }
//     else {
//         ledcWrite(CH_BACK_L, 0);
//         ledcWrite(CH_BACK_R, 0);
//     }

//     // -- READ NECK IMU --
//     imuBack.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//     pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * 57.2958f;

//     // pitch-based vibration
//     if (pitch > TH_PITCH) {
//         ledcWrite(CH_NECK_UP, PWM_DUTY);
//         ledcWrite(CH_NECK_DN, 0);
//     }
//     else if (pitch < -TH_PITCH) {
//         ledcWrite(CH_NECK_DN, PWM_DUTY);
//         ledcWrite(CH_NECK_UP, 0);
//     }
//     else {
//         ledcWrite(CH_NECK_UP, 0);
//         ledcWrite(CH_NECK_DN, 0);
//     }

//     // optional tuning output
//     Serial.printf("Lower back Roll: %+5.1f°  Upper Neck Pitch: %+5.1f°\n", roll, pitch);
//     SerialBT.printf("ROLL: %+5.1f°  PITCH: %+5.1f°\n", roll, pitch);
//     delay(READ_MS);
// }

#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include <BluetoothSerial.h>

// Bluetooth Serial Object
BluetoothSerial SerialBT;

// Single MPU-6050 instance (AD0 → GND)
MPU6050 imu(0x68);

// Vibration-driver pins & channels
static constexpr int BACK_L_PWM   = 12;  // back-left motor PWM (roll < -TH)
static constexpr int BACK_R_PWM   = 32;  // back-right motor PWM (roll > +TH)
static constexpr int NECK_UP_PWM  = 14;  // neck-up motor PWM (pitch > +TH)
static constexpr int NECK_DN_PWM  = 4;   // neck-down motor PWM (pitch < -TH)

static constexpr int CH_BACK_L    = 0;
static constexpr int CH_BACK_R    = 1;
static constexpr int CH_NECK_UP   = 2;
static constexpr int CH_NECK_DN   = 3;

// I²C pins
static constexpr int SDA_pin      = 22;
static constexpr int SCL_pin      = 21;

// Thresholds & timing
static constexpr float TH_ROLL    = 15.0f;   // roll threshold (°)
static constexpr float TH_PITCH   = 10.0f;   // pitch threshold (°)
static constexpr int   PWM_DUTY   = 150;     // 0–255
static constexpr int   READ_MS    = 500;     // ms between reads

// Onboard LED
static constexpr int LED_PIN           = 2;      // ESP32 built-in LED on GPIO2
static constexpr unsigned long LED_ON_TIME = 200;  // ms to keep LED lit
unsigned long ledTimer = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  SerialBT.begin("ESP32_Vibro_WROOM");
  Serial.println("Bluetooth ready to pair");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Wire.begin(SDA_pin, SCL_pin);
  imu.initialize();
  Serial.println("IMU initialized, driving motors...");

  // Setup ESP32 LEDC channels
  ledcSetup(CH_BACK_L, 5000, 8);
  ledcAttachPin(BACK_L_PWM, CH_BACK_L);

  ledcSetup(CH_BACK_R, 5000, 8);
  ledcAttachPin(BACK_R_PWM, CH_BACK_R);

  ledcSetup(CH_NECK_UP, 5000, 8);
  ledcAttachPin(NECK_UP_PWM, CH_NECK_UP);

  ledcSetup(CH_NECK_DN, 5000, 8);
  ledcAttachPin(NECK_DN_PWM, CH_NECK_DN);
}

void loop() {
  // --- Handle incoming Bluetooth commands ---
  while (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();
    if (cmd == "Q1" || cmd == "Q2" || cmd == "Q3" || cmd == "Q4") {
      digitalWrite(LED_PIN, HIGH);
      ledTimer = millis();
    }
  }

  // Turn off LED after timeout
  if (ledTimer && (millis() - ledTimer >= LED_ON_TIME)) {
    digitalWrite(LED_PIN, LOW);
    ledTimer = 0;
  }

  // --- Read IMU and drive motors ---
  int16_t ax, ay, az, gx, gy, gz;
  float roll, pitch;

  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Compute roll (for back motors)
  roll = atan2f(ay, az) * 57.2958f;

  if (roll >  TH_ROLL) {
    ledcWrite(CH_BACK_R, PWM_DUTY);
    ledcWrite(CH_BACK_L, 0);
  } else if (roll < -TH_ROLL) {
    ledcWrite(CH_BACK_L, PWM_DUTY);
    ledcWrite(CH_BACK_R, 0);
  } else {
    ledcWrite(CH_BACK_L, 0);
    ledcWrite(CH_BACK_R, 0);
  }

  // Compute pitch (for neck motors)
  pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * 57.2958f;

  if (pitch >  TH_PITCH) {
    ledcWrite(CH_NECK_UP, PWM_DUTY);
    ledcWrite(CH_NECK_DN, 0);
  } else if (pitch < -TH_PITCH) {
    ledcWrite(CH_NECK_DN, PWM_DUTY);
    ledcWrite(CH_NECK_UP, 0);
  } else {
    ledcWrite(CH_NECK_UP, 0);
    ledcWrite(CH_NECK_DN, 0);
  }

  // Debug output
  Serial.printf("Roll: %+5.1f°  Pitch: %+5.1f°\n", roll, pitch);
  SerialBT.printf("ROLL: %+5.1f°  PITCH: %+5.1f°\n", roll, pitch);

  delay(READ_MS);
}
