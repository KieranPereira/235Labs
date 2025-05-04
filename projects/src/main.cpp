#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include <BluetoothSerial.h>
#include <esp_system.h>    // for ESP.restart()

// SD card includes
#include <SPI.h>
#include <SD.h>
// FreeRTOS includes
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>




// Queue handles
static QueueHandle_t sensorQueueBack;
static QueueHandle_t sensorQueueNeck;
static QueueHandle_t cmdQueue;

// Task handles
// at top, before setup()
static TaskHandle_t imuBackHandle   = NULL;
static TaskHandle_t imuNeckHandle   = NULL;


// protect shared I2C & SPI resources
static SemaphoreHandle_t i2cMutex;
static SemaphoreHandle_t sdMutex;
// Bluetooth Serial
static BluetoothSerial SerialBT;

// Compile‐time toggle: true = two IMUs, false = single IMU
static constexpr bool TWO_IMUS = true;

// Defining Wiring Addresses

// MPU addresses
static MPU6050 imuBack(0x68);  // AD0→GND
static MPU6050 imuNeck(0x69);  // AD0→VCC
// Vibration‐driver pins & PWM channels
static constexpr int BACK_L_PWM = 12;
static constexpr int BACK_R_PWM = 32;

static constexpr int NECK_UP_PWM = 15;
static constexpr int NECK_DN_PWM = 4;

static constexpr int CH_BACK_L  = 0;
static constexpr int CH_BACK_R  = 1;
static constexpr int CH_NECK_UP = 2;
static constexpr int CH_NECK_DN = 3;

// I²C pins for IMUs
static constexpr int SDA_pin = 22;
static constexpr int SCL_pin = 20;

// SPI pins for SD card
static constexpr int SD_CS   = 5;   // CS
static constexpr int SD_MOSI = 19;  // DI
static constexpr int SD_MISO = 21;  // DO
static constexpr int SD_CLK  = 27;  // SCK

// On‐board LED for quadrant flash feedback
static constexpr int LED_PIN      = 13;


// Defining Global Constants

// Thresholds & timing
static constexpr float TH_ROLL    = 15.0f;
static constexpr float TH_PITCH   = 10.0f;
static constexpr int   PWM_DUTY   = 150;
static constexpr int   READ_MS    = 500;
static constexpr int LED_FLASH_MS = 100;
unsigned long ledTimer = 0;

// Data Structures
struct SensorData 
{
  float roll;
  float pitch;
};

enum Command
{
  Q1,
  Q2,
  Q3,
  Q4,
  UNKNOWN
};

void taskIMUBack(void* pv);
void taskIMUTop(void* pv);
void taskCommand(void* pv);
void taskTelemetry(void* pv);

void setup() {
  // Serial + Bluetooth
  Serial.begin(115200);
  SerialBT.begin("ESP32_Vibro_real");
  Serial.println("Bluetooth ready to pair");

  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // IMUs
  Wire.begin(SDA_pin, SCL_pin);
  imuBack.initialize();
  if (TWO_IMUS) {
      imuNeck.initialize();
      Serial.println("Dual IMU mode enabled");
  } else {
      Serial.println("Single IMU mode enabled");
  }

  // Vibration motors PWM
  ledcSetup(CH_BACK_L,  5000, 8);
  ledcAttachPin(BACK_L_PWM, CH_BACK_L);

  ledcSetup(CH_BACK_R,  5000, 8);
  ledcAttachPin(BACK_R_PWM, CH_BACK_R);

  ledcSetup(CH_NECK_UP, 5000, 8);
  ledcAttachPin(NECK_UP_PWM, CH_NECK_UP);

  ledcSetup(CH_NECK_DN, 5000, 8);
  ledcAttachPin(NECK_DN_PWM, CH_NECK_DN);


  // Create mutexes and queues
  i2cMutex = xSemaphoreCreateMutex();
  sdMutex = xSemaphoreCreateMutex();
  sensorQueueBack = xQueueCreate(10, sizeof(SensorData));
  sensorQueueNeck = xQueueCreate(10, sizeof(SensorData));
  cmdQueue = xQueueCreate(10, sizeof(Command));


  // SD card
  SPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) 
  {
    Serial.println("❌ Card Mount Failed");
  } else 
  {
    xSemaphoreTake(sdMutex, portMAX_DELAY);
    if (SD.exists("/datalog.csv")) 
    {
      SD.remove("/datalog.csv");
    }

    File hdr = SD.open("/datalog.csv", FILE_WRITE);
    hdr.println("time,pitch lower back,roll lower back,pitch neck,roll neck");
    hdr.close();
    
    xSemaphoreGive(sdMutex);
  }

  // Create tasks
  xTaskCreate(
    taskIMUBack, // task function
    "IMU Back", // name of task
    16384,  // stack size in bytes
    NULL, // input parameter
    2, // priority
    &imuBackHandle // task handle
    );
  xTaskCreate(taskIMUTop, "IMU Neck", 16384, NULL, 2, &imuNeckHandle);
  xTaskCreate(taskCommand, "Command", 16384, NULL, 3, NULL);
  xTaskCreate(taskTelemetry, "Telemetry", 16384, NULL, 1, NULL);

}

void loop() {
// Empty loop, all tasks are handled in FreeRTOS tasks
  vTaskDelete(nullptr);
}


// ============================ Task Definitions ===========================

void taskIMUBack(void* pv) {
  SensorData data;
  uint32_t cmd;

  for (;;) {

    // check for GUI command
    if (xTaskNotifyWait(0, 0xFFFFFFFF, &cmd, 0 ) == pdTRUE) 
    {
      // check for command to set PWM duty cycle
      if (cmd ==Q3) 
      {
        ledcWrite(CH_BACK_L, 0);
        ledcWrite(CH_BACK_R, PWM_DUTY);
      } else{
        ledcWrite(CH_BACK_L, PWM_DUTY);
        ledcWrite(CH_BACK_R, 0);
      }

      // skip imu read and thresholding if command received
      vTaskDelay(pdMS_TO_TICKS(READ_MS));
      continue;
    }

    // No ovveride, normal IMU read + threshold check + motor

    // protect shared I2C bus
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
      int16_t ax, ay, az, gx, gy, gz;
      imuBack.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    xSemaphoreGive(i2cMutex);

    data.roll  = atan2f(ay, az) * 57.2958f;
    data.pitch = atan2f(ax, sqrtf(ay*ay + az*az)) * 57.2958f;
    
    // check for threshold crossing
    if      (data.roll >  TH_ROLL) ledcWrite(CH_BACK_R, PWM_DUTY), ledcWrite(CH_BACK_L, 0);
    else if (data.roll < -TH_ROLL) ledcWrite(CH_BACK_L, PWM_DUTY), ledcWrite(CH_BACK_R, 0);
    else                            ledcWrite(CH_BACK_L, 0),         ledcWrite(CH_BACK_R, 0);


    /// CHECKKKKK DO I NEED THIS??????
    xQueueSend(sensorQueueBack, &data, portMAX_DELAY);
    // vTaskDelay(pdMS_TO_TICKS(READ_MS));

  }
}

// Read the neck/top IMU and queue it every READ_MS
void taskIMUTop(void* pv) {
  SensorData data;
  uint32_t cmd;
  for (;;) {

    // check for GUI command
    if (xTaskNotifyWait(0, 0xFFFFFFFF, &cmd, 0 ) == pdTRUE) 
    {
      // check for command to set PWM duty cycle
      if (cmd ==Q1) 
      {
        ledcWrite(CH_NECK_DN, 0);
        ledcWrite(CH_NECK_UP, PWM_DUTY);
      } else{
        ledcWrite(CH_NECK_DN, PWM_DUTY);
        ledcWrite(CH_NECK_UP, 0);
      }

      // skip imu read and thresholding if command received
      vTaskDelay(pdMS_TO_TICKS(READ_MS));
      continue;
    }

    xSemaphoreTake(i2cMutex, portMAX_DELAY);
      int16_t ax, ay, az, gx, gy, gz;
      imuNeck.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    xSemaphoreGive(i2cMutex);

    data.roll  = atan2f(ay, az) * 57.2958f;
    data.pitch = atan2f(ax, sqrtf(ay*ay + az*az)) * 57.2958f;

    // drive neck motors by pitch threshold
    if      (data.pitch >  TH_PITCH) ledcWrite(CH_NECK_UP, PWM_DUTY), ledcWrite(CH_NECK_DN, 0);
    else if (data.pitch < -TH_PITCH) ledcWrite(CH_NECK_DN, PWM_DUTY), ledcWrite(CH_NECK_UP, 0);
    else                              ledcWrite(CH_NECK_UP, 0),          ledcWrite(CH_NECK_DN, 0);

    xQueueSend(sensorQueueNeck, &data, portMAX_DELAY);
    // vTaskDelay(pdMS_TO_TICKS(READ_MS));
  }
}

// Parse Serial/BT commands and toggle the LED
void taskCommand(void* pv) {
  for (;;) {
    
    if (SerialBT.available()) 
    {
      String s = SerialBT.readStringUntil('\n');
      s.trim();

      //  ─────────────── Reboot command ───────────────
      if (s == "REBOOT") {
        SerialBT.println("Rebooting now...");
        vTaskDelay(pdMS_TO_TICKS(50));    // give time to flush
        ESP.restart();                    // reset the ESP32
      }

      Command c = UNKNOWN;

      if (s=="Q1") c=Q1;
      else if (s=="Q2") c=Q2;
      else if (s=="Q3") c=Q3;
      else if (s=="Q4") c=Q4;

      // flash LED
      if (c!=UNKNOWN) 
      {
        digitalWrite(LED_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(LED_FLASH_MS));
        digitalWrite(LED_PIN, LOW);

        if (c==Q1 || c==Q2) 
        {
          xTaskNotify(imuNeckHandle, (uint32_t)c, eSetValueWithOverwrite);
        } else 
        {
          xTaskNotify(imuBackHandle, (uint32_t)c, eSetValueWithOverwrite);
        }
        xQueueSend(cmdQueue, &c, portMAX_DELAY);
      }

      
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


// Pull from sensor & command queues, print & log (with live CSV over BT)
void taskTelemetry(void* pv) {
  SensorData back, top;
  Command cmd;
  for (;;) {
    // check for back IMU
    if (xQueueReceive(sensorQueueBack, &back, 0)==pdPASS) {
      Serial.printf("Back -> Roll: %+5.1f°, Pitch: %+5.1f°\n", back.roll, back.pitch);
    }
    // check for top IMU
    if (xQueueReceive(sensorQueueNeck, &top, 0)==pdPASS) {
      Serial.printf("Top  -> Roll: %+5.1f°, Pitch: %+5.1f°\n", top.roll, top.pitch);
    }
    // check for commands
    if (xQueueReceive(cmdQueue, &cmd, 0)==pdPASS) {
      Serial.printf("CMD RECEIVED: %d\n", cmd);
      SerialBT.printf("CMD RECEIVED: %d\n", cmd);
    }

    // log both IMUs together (optional) and stream live CSV
    xSemaphoreTake(sdMutex, portMAX_DELAY);
    float t_s = millis() / 1000.0f;

      File f = SD.open("/datalog.csv", FILE_APPEND);
      if (f) {
        
        // write same CSV to SD card
        f.printf(
          "%0.3f,%.1f,%.1f,%.1f,%.1f\n",
          t_s,
          back.pitch,
          back.roll,
          top.pitch,
          top.roll
        );

        f.close();
      }

    SerialBT.printf(
      "%0.3f,%.1f,%.1f\n",
      t_s,
      back.roll,
      top.pitch
    );

    xSemaphoreGive(sdMutex);

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}