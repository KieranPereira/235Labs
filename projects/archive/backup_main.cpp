#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include <BluetoothSerial.h>

// Bluetooth Serial Object
BluetoothSerial SerialBT;

// Compile-time toggle: true = two IMUs, false = single IMU
static constexpr bool TWO_IMUS = true;

// MPU addresses
MPU6050 imuBack(0x68);  // lower-back IMU, AD0→GND
MPU6050 imuNeck(0x69);  // neck IMU,     AD0→VCC (used only if TWO_IMUS)

// Vibration-driver pins & channels
static constexpr int BACK_L_PWM   = 12;
static constexpr int BACK_R_PWM   = 32;
static constexpr int NECK_UP_PWM  = 15;
static constexpr int NECK_DN_PWM  = 4;

// PWM channels
static constexpr int CH_BACK_L    = 0;
static constexpr int CH_BACK_R    = 1;
static constexpr int CH_NECK_UP   = 2;
static constexpr int CH_NECK_DN   = 3;

// I²C pins
static constexpr int SDA_pin      = 22;
static constexpr int SCL_pin      = 20;

// Thresholds & timing
static constexpr float TH_ROLL    = 15.0f;
static constexpr float TH_PITCH   = 10.0f;
static constexpr int   PWM_DUTY   = 150;
static constexpr int   READ_MS    = 10;

// On-board LED for quadrant flash feedback
static constexpr int LED_PIN      = 13;
static constexpr int LED_FLASH_MS = 100;
unsigned long ledTimer = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    SerialBT.begin("ESP32_Vibro_real");
    Serial.println("Bluetooth ready to pair");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Wire.begin(SDA_pin, SCL_pin);
    imuBack.initialize();
    if (TWO_IMUS) {
        imuNeck.initialize();
        Serial.println("Dual IMU mode enabled");
    } else {
        Serial.println("Single IMU mode enabled");
    }

    // Setup vibration motor PWM channels
    ledcSetup(CH_BACK_L,  5000, 8);
    ledcAttachPin(BACK_L_PWM, CH_BACK_L);
    ledcSetup(CH_BACK_R,  5000, 8);
    ledcAttachPin(BACK_R_PWM, CH_BACK_R);
    ledcSetup(CH_NECK_UP, 5000, 8);
    ledcAttachPin(NECK_UP_PWM, CH_NECK_UP);
    ledcSetup(CH_NECK_DN, 5000, 8);
    ledcAttachPin(NECK_DN_PWM, CH_NECK_DN);
}

void loop() {
    // --- Listen for quadrant commands over Bluetooth ---
    if (SerialBT.available()) {
        String cmd = SerialBT.readStringUntil('\n');
        cmd.trim();
        // flash LED only on Q1-Q4
        if (cmd == "Q1" || cmd == "Q2" || cmd == "Q3" || cmd == "Q4") {
            digitalWrite(LED_PIN, HIGH);
            ledTimer = millis();
        }
        // echo back or handle further commands
        SerialBT.printf("CMD RECEIVED: %s\n", cmd.c_str());
    }
    // turn off LED after flash duration
    if (ledTimer && (millis() - ledTimer >= LED_FLASH_MS)) {
        digitalWrite(LED_PIN, LOW);
        ledTimer = 0;
    }

    // Read IMU data
    int16_t ax, ay, az, gx, gy, gz;
    float roll = 0, pitch = 0;

    if (TWO_IMUS) {
        imuBack.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        roll = atan2f(ay, az) * 57.2958f;
        imuNeck.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        pitch = atan2f(ax, sqrtf(ay*ay + az*az)) * 57.2958f;
    } else {
        imuBack.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        roll  = atan2f(ay, az) * 57.2958f;
        pitch = atan2f(ax, sqrtf(ay*ay + az*az)) * 57.2958f;
    }

    // Drive back motors based on roll
    if (roll > TH_ROLL) {
        ledcWrite(CH_BACK_R, PWM_DUTY);
        ledcWrite(CH_BACK_L, 0);
    } else if (roll < -TH_ROLL) {
        ledcWrite(CH_BACK_L, PWM_DUTY);
        ledcWrite(CH_BACK_R, 0);
    } else {
        ledcWrite(CH_BACK_L, 0);
        ledcWrite(CH_BACK_R, 0);
    }

    // Drive neck motors based on pitch
    if (pitch > TH_PITCH) {
        ledcWrite(CH_NECK_UP, PWM_DUTY);
        ledcWrite(CH_NECK_DN, 0);
    } else if (pitch < -TH_PITCH) {
        ledcWrite(CH_NECK_DN, PWM_DUTY);
        ledcWrite(CH_NECK_UP, 0);
    } else {
        ledcWrite(CH_NECK_UP, 0);
        ledcWrite(CH_NECK_DN, 0);
    }

    // Telemetry
    Serial.printf("Roll: %+5.1f°   Pitch: %+5.1f°\n", roll, pitch);
    SerialBT.printf("ROLL: %+5.1f°   PITCH: %+5.1f°\n", roll, pitch);

    delay(READ_MS);
}
