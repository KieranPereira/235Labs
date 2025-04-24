#include <Wire.h>
#include "MPU6050.h"

//—— MPU addresses ——
MPU6050 imuBack(0x68);  // lower-back IMU, AD0→GND
MPU6050 imuNeck(0x69);  // neck IMU,     AD0→VCC

//—— Vibration-driver pins & channels ——
#define BACK_L_PWM 33   // back-left motor PWM (roll < -TH)
#define BACK_R_PWM 26   // back-right motor PWM (roll > +TH)
const int CH_BACK_L = 0, CH_BACK_R = 1;

#define NECK_UP_PWM 16  // neck-up motor PWM (pitch > +TH)
#define NECK_DN_PWM 18  // neck-down motor PWM (pitch < -TH)
const int CH_NECK_UP = 2, CH_NECK_DN = 3;

//—— Settings ——
const float TH_ROLL = 15.0;   // back roll threshold
const float TH_PITCH = 10.0;  // neck pitch threshold
const int PWM_DUTY = 150;     // 0–255
const int READ_MS = 100;      // read/update interval

void setup() {
  Serial.begin(115200);
  while(!Serial);

  Wire.begin();            // SDA=21, SCL=22
  imuBack.initialize();
  imuNeck.initialize();
  Serial.println("Both IMUs up, now driving motors…");

  // attach ESP32 LEDC channels to pins:
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
  int16_t ax,ay,az,gx,gy,gz;
  float roll, pitch;

  // —— READ BACK IMU ——
  imuBack.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  roll  = atan2f(ay, az) * 57.3f;

  // motor logic for roll symmetry:
  if (roll >  TH_ROLL) {
    ledcWrite(CH_BACK_R, PWM_DUTY);  // right-side vibrate
    ledcWrite(CH_BACK_L, 0);
  }
  else if (roll < -TH_ROLL) {
    ledcWrite(CH_BACK_L, PWM_DUTY);  // left-side vibrate
    ledcWrite(CH_BACK_R, 0);
  }
  else {
    ledcWrite(CH_BACK_L, 0);
    ledcWrite(CH_BACK_R, 0);
  }

  // —— READ NECK IMU ——
  imuNeck.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  pitch = atan2f(ax, sqrtf(ay*ay + az*az)) * 57.3f;

  // motor logic for head tilt:
  if (pitch >  TH_PITCH) {
    ledcWrite(CH_NECK_UP, PWM_DUTY);
    ledcWrite(CH_NECK_DN, 0);
  }
  else if (pitch < -TH_PITCH) {
    ledcWrite(CH_NECK_DN, PWM_DUTY);
    ledcWrite(CH_NECK_UP, 0);
  }
  else {
    ledcWrite(CH_NECK_UP, 0);
    ledcWrite(CH_NECK_DN, 0);
  }

  // optional: print for tuning
  Serial.printf("ROLL: %+5.1f°  PITCH: %+5.1f°\n", roll, pitch);
  delay(READ_MS);
}
