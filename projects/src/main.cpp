#include <Arduino.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <WiFi.h>
#include <esp_system.h>    // for ESP.restart()
#include <stdarg.h> 


// SD card includes
#include <SPI.h>
#include <SD.h>
// FreeRTOS includes
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>




// ============================ Queues & Mutexes ===========================

static QueueHandle_t sensorQueueBack;
static QueueHandle_t sensorQueueNeck;
static QueueHandle_t cmdQueue;

// Task handles
static TaskHandle_t imuBackHandle   = NULL;
static TaskHandle_t imuNeckHandle   = NULL;


// protect shared I2C & SPI resources
static SemaphoreHandle_t i2cMutex;  
static SemaphoreHandle_t sdMutex;

// ============================= Wifi Definitions ===========================
const char* AP_SSID     = "ESP32_IMU_AP";
const char* AP_PASSWORD = "123qweasd";
const uint16_t TCP_PORT = 3333;

WiFiServer server(TCP_PORT);
WiFiClient telemetryClient;



// ============================ IMU & PWM Setup ===========================

// Compile‐time toggle: true = two IMUs, false = single IMU
static constexpr bool TWO_IMUS = true;

// I²C pins for IMUs
static constexpr int SDA_pin = 41;
static constexpr int SCL_pin = 42;

// MPU addresses
static MPU9250_asukiaaa imuBack(0x68);  // AD0 = 0 ⇒ 0x68
static MPU9250_asukiaaa imuNeck(0x69);  // AD0→VCC

// Vibration‐driver pins & PWM channels
static constexpr int BACK_L_PWM =  4;
static constexpr int BACK_R_PWM = 38;

static constexpr int NECK_UP_PWM = 35;
static constexpr int NECK_DN_PWM = 48;

static constexpr int CH_BACK_L  = 0;
static constexpr int CH_BACK_R  = 1;
static constexpr int CH_NECK_UP = 2;
static constexpr int CH_NECK_DN = 3;

// SPI pins for SD card
static constexpr int SD_CS   = 47;   // CS
static constexpr int SD_MOSI = 33;  // DI
static constexpr int SD_MISO = 37;  // DO
static constexpr int SD_CLK  = 36;  // SCK

// On‐board LED for quadrant flash feedback
// static constexpr int LED_PIN      = 13;


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

// ============================ Sensor Processing ===========================

// ─── Filtering toggle ────────────────────────────────────────────────
// 0 = use low‑pass + Kalman   |   1 = bypass them completely
#define DISABLE_FILTERS 1

struct OneDKalman {
float R, Q;   // measurement & process covariances
float x, P;   // state estimate & error covariance

OneDKalman(float R=0.01f, float Q=1e-5f, float x0=0.0f, float P0=1.0f)
  : R(R), Q(Q), x(x0), P(P0) {}

float update(float z) {
    // Predict
    P += Q;
    // Compute gain
    float K = P / (P + R);
    // Correct
    x += K * (z - x);
    // Update covariance
    P *= (1.0f - K);
    return x;
}
};


struct LowPass {
float alpha;   // smoothing factor, 0<alpha<1
float y;       // last output

// alpha = dt / (RC + dt);  RC = 1/(2πfc)
LowPass(float alpha=0.1f, float y0=0.0f) : alpha(alpha), y(y0) {}

float filter(float x) {
    y += alpha * (x - y);
    return y;
}
};


// Back IMU filters
static OneDKalman kalmanBackRoll(0.01f, 1e-5f);
static OneDKalman kalmanBackPitch(0.01f, 1e-5f);
static LowPass  lpBackRoll(0.1f);
static LowPass  lpBackPitch(0.1f);

// Neck IMU filters
static OneDKalman kalmanNeckRoll(0.01f, 1e-5f);
static OneDKalman kalmanNeckPitch(0.01f, 1e-5f);
static LowPass  lpNeckRoll(0.1f);
static LowPass  lpNeckPitch(0.1f);

// ============================ Setup & Loop ===========================  


// ----------  Dual‑output debug helpers ----------
static void dbgWrite(const char* buf, size_t len)
{
  Serial.write((const uint8_t*)buf, len);

  if (telemetryClient && telemetryClient.connected())
    telemetryClient.write((const uint8_t*)buf, len);
}

// printf‑style
static void dbgPrintf(const char* fmt, ...)
{
  char buf[256];                       // big enough for typical log lines
  va_list ap;
  va_start(ap, fmt);
  int len = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (len > 0)                         // safety: ignore empty / err lines
    dbgWrite(buf, (size_t)len);
}

// println‑style convenience
static void dbgPrintln(const char* msg)
{
  dbgPrintf("%s\r\n", msg);
}

// Macros so you can keep one‑liners neat
#define DBG   dbgPrintf
#define DBGLN dbgPrintln




void setup() {

  Serial.begin(115200);
  while(!Serial){}

  // I2C / IMU init
  Wire.begin(SDA_pin, SCL_pin, 400000);
  yield();
  imuBack.setWire(&Wire);
  yield();
  imuBack.beginAccel();
  delay(5);
  imuBack.beginGyro();
  delay(5);
  imuBack.beginMag();
  DBGLN("✅ Back IMU initialised");
  yield();

  if (TWO_IMUS) {
    imuNeck.setWire(&Wire);
    delay(5);
    imuNeck.beginAccel();
    delay(5);
    imuNeck.beginGyro();
    delay(5);
    imuNeck.beginMag();
    DBGLN("✅ Neck IMU initialised");
    yield();
  }


  // --- Serial & WiFi ---
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  IPAddress ip = WiFi.softAPIP();
  DBG("AP started. SSID: "); DBGLN(AP_SSID);
  DBG("IP address: ");  DBG("%s\n", ip.toString().c_str());
  server.begin();
  DBG("TCP server listening on port %u\n", TCP_PORT);




  // --- LED ---
  // pinMode(LED_PIN, OUTPUT);
  // digitalWrite(LED_PIN, LOW);

  // --- RTOS primitives ---
  i2cMutex = xSemaphoreCreateMutex();
  sdMutex = xSemaphoreCreateMutex();
  sensorQueueBack = xQueueCreate(10, sizeof(SensorData));
  sensorQueueNeck = xQueueCreate(10, sizeof(SensorData));
  cmdQueue = xQueueCreate(10, sizeof(Command));

  auto initMpu = [](MPU9250_asukiaaa& mpu, const char* name) {
    mpu.setWire(&Wire);     // point driver to the I²C bus
    mpu.beginAccel();       // turn on accelerometer
    mpu.beginGyro();        // turn on gyro
    mpu.beginMag();         // comment this line out if you don’t need the compass
  
    DBG(" %s initialised\n", name);
  };
  

  initMpu(imuBack, "Back IMU");
  if (TWO_IMUS){
    initMpu(imuNeck, "Neck IMU");
    xTaskCreate(taskIMUTop, "IMU Neck", 16384, NULL, 2, &imuNeckHandle);
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

  // SD card
  SPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) 
  {
    DBGLN("❌ Card Mount Failed");
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


// --- FreeRTOS tasks ---
  xTaskCreate(
    taskIMUBack, // task function
    "IMU Back", // name of task
    16384,  // stack size in bytes
    NULL, // input parameter
    2, // priority
    &imuBackHandle // task handle
    );
  xTaskCreate(taskTelemetry, "Telemetry", 16384, NULL, 1, NULL);
    xTaskCreate(
    taskCommand,    // function
    "Command",      // name
    8192,           // stack size
    nullptr,        // parameter
    3,              // priority
    nullptr         // handle
  );
  

}

void loop() {
// Empty loop, all tasks are handled in FreeRTOS tasks
  delay(1000);
}


// ============================ Task Definitions ===========================

void taskIMUBack(void* pv) {
  SensorData data;
  uint32_t cmdVal;
  for (;;) {
    // handle override commands
    if ( xTaskNotifyWait(0, 0xFFFFFFFF, &cmdVal, 0) == pdTRUE ) {
      if (cmdVal == Q3) {
        // Q3: turn right‐back motor ON
        ledcWrite(CH_BACK_L, 0);
        ledcWrite(CH_BACK_R, PWM_DUTY);
      }
      else if (cmdVal == Q4) {
        // Q4: turn left‐back motor ON
        ledcWrite(CH_BACK_R, 0);
        ledcWrite(CH_BACK_L, PWM_DUTY);
      }
      // hold for one cycle, skip normal read
      vTaskDelay(pdMS_TO_TICKS(READ_MS));
      continue;
    }

    // No ovveride, normal IMU read + threshold check + motor

    // protect shared I2C bus
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    imuBack.accelUpdate();
    float ax = imuBack.accelX();
    float ay = imuBack.accelY();
    float az = imuBack.accelZ();
    xSemaphoreGive(i2cMutex);

    // raw:
    float rawRoll  = atan2f(-ay, -az) * 57.2958f;
    float rawPitch = atan2f(ax, sqrtf(ay*ay + az*az)) * 57.2958f;

    #if DISABLE_FILTERS
      data.roll = rawRoll;
      data.pitch = rawPitch;
    #else
      // 1) low-pass
      float smoothRoll  = lpBackRoll.filter(rawRoll);
      float smoothPitch = lpBackPitch.filter(rawPitch);

      // 2) Kalman
      data.roll  = kalmanBackRoll.update(smoothRoll);
      data.pitch = kalmanBackPitch.update(smoothPitch);
    #endif
    
    // check for threshold crossing
    if      (data.roll >  TH_ROLL) ledcWrite(CH_BACK_R, PWM_DUTY), ledcWrite(CH_BACK_L, 0);
    else if (data.roll < -TH_ROLL) ledcWrite(CH_BACK_L, PWM_DUTY), ledcWrite(CH_BACK_R, 0);
    else                            ledcWrite(CH_BACK_L, 0),         ledcWrite(CH_BACK_R, 0);


    /// CHECKKKKK DO I NEED THIS??????
    xQueueSend(sensorQueueBack, &data, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(READ_MS));

  }
}

// Read the neck/top IMU and queue it every READ_MS
void taskIMUTop(void* pv) {
  SensorData data;
  uint32_t cmdVal;
  for (;;) {
    // handle override commands
    if ( xTaskNotifyWait(0, 0xFFFFFFFF, &cmdVal, 0) == pdTRUE ) {
      if (cmdVal == Q1) {
        // Q1: tilt neck UP
        ledcWrite(CH_NECK_DN, 0);
        ledcWrite(CH_NECK_UP, PWM_DUTY);
      }
      else if (cmdVal == Q2) {
        // Q2: tilt neck DOWN
        ledcWrite(CH_NECK_UP, 0);
        ledcWrite(CH_NECK_DN, PWM_DUTY);
      }
      // hold for one cycle, skip normal read
      vTaskDelay(pdMS_TO_TICKS(READ_MS));
      continue;
    }

    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    imuNeck.accelUpdate();
    float ax = imuNeck.accelX();// already in g
    float ay = imuNeck.accelY();
    float az = imuNeck.accelZ();
    xSemaphoreGive(i2cMutex);

    float rawRoll  = atan2f(ay, az) * 57.2958f;
    float rawPitch = atan2f(ax, sqrtf(ay*ay + az*az)) * 57.2958f;

    #if DISABLE_FILTERS
    // pass raw angles straight through
      data.roll  = rawRoll;
      data.pitch = rawPitch;
    #else
      float smoothRoll  = lpNeckRoll.filter(rawRoll);
      float smoothPitch = lpNeckPitch.filter(rawPitch);

      data.roll  = kalmanNeckRoll.update(smoothRoll);
      data.pitch = kalmanNeckPitch.update(smoothPitch);
    #endif

    // drive neck motors by pitch threshold
    if      (data.pitch >  TH_PITCH) ledcWrite(CH_NECK_UP, PWM_DUTY), ledcWrite(CH_NECK_DN, 0);
    else if (data.pitch < -TH_PITCH) ledcWrite(CH_NECK_DN, PWM_DUTY), ledcWrite(CH_NECK_UP, 0);
    else                              ledcWrite(CH_NECK_UP, 0),          ledcWrite(CH_NECK_DN, 0);

    xQueueSend(sensorQueueNeck, &data, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(READ_MS));
  }
}

// Pull from sensor & command queues, print & log (with live CSV over BT)
// ==================== taskTelemetry (replace entire function) ====================
void taskTelemetry(void* pv) {
  SensorData back, top;
  float      backRoll  = NAN;   // last valid values
  float      neckPitch = NAN;
  char       buf[64];

  for (;;) {
    // pull fresh data if available
    if (xQueueReceive(sensorQueueBack, &back, 0) == pdPASS)
        backRoll = back.roll;
    if (xQueueReceive(sensorQueueNeck, &top, 0) == pdPASS)
        neckPitch = top.pitch;

    // only stream once we have both numbers
    if (!isnan(backRoll) && !isnan(neckPitch) && telemetryClient && telemetryClient.connected()) {
        float t = millis() / 1000.0f;
        int len = snprintf(buf, sizeof(buf), "%0.3f,%.1f,%.1f\n",
                           t, backRoll, neckPitch);
        telemetryClient.write((uint8_t*)buf, len);
    }

    // look for a new client if none is connected
    if (!telemetryClient || !telemetryClient.connected()) {
        WiFiClient nc = server.available();
        if (nc) telemetryClient = nc;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
// ============================ Command Processing ===========================

// Parse Serial/BT commands and toggle the LED
void taskCommand(void* pv) {
  for (;;) {
    // only proceed if a client is connected and has sent data
    if (telemetryClient && telemetryClient.connected() && telemetryClient.available()) {
      String s = telemetryClient.readStringUntil('\n');
      s.trim();

      // Reboot command
      if (s == "REBOOT") {
        telemetryClient.println("Rebooting now...");
        vTaskDelay(pdMS_TO_TICKS(50)); 
        ESP.restart();
      }

      // Map string → enum
      Command c = UNKNOWN;
      if      (s == "Q1") c = Q1;
      else if (s == "Q2") c = Q2;
      else if (s == "Q3") c = Q3;
      else if (s == "Q4") c = Q4;

      if (c != UNKNOWN) {
        // flash feedback (optional)
        vTaskDelay(pdMS_TO_TICKS(LED_FLASH_MS));

        // dispatch to correct IMU task
        // Q1/Q2 control neck motors; Q3/Q4 control back motors
        if (c == Q1 || c == Q2) {
          xTaskNotify(imuNeckHandle, (uint32_t)c, eSetValueWithOverwrite);
        } else {
          xTaskNotify(imuBackHandle, (uint32_t)c, eSetValueWithOverwrite);
        }

        // also log it for telemetry and ack back to GUI
        xQueueSend(cmdQueue, &c, portMAX_DELAY);
        telemetryClient.printf("ACK:%s\n", s.c_str());
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}