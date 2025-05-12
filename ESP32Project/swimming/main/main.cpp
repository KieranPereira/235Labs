/*********************************************************************
 *  STEP 2 – Dual‑IMU reader (back + neck)
 *  SDA = GPIO 22,  SCL = GPIO 20
 *  Back  IMU: AD0 LOW  → I²C 0x68
 *  Neck  IMU: AD0 HIGH → I²C 0x69
 *
 *  Requires idf_component.yml:
 *      dependencies:
 *        espressif/mpu6050: "^1.2.0"
 *********************************************************************/


#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/timer.h"

#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <strings.h>

#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "nvs_flash.h"

#include "esp_spp_api.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_timer.h"
#include "esp_intr_alloc.h"

#include <cmath>

extern "C" {
#include "mpu6050.h"
}



/* ───── I²C pins ──────────────────────────────────────────────── */
constexpr gpio_num_t SDA_PIN = GPIO_NUM_22;
constexpr gpio_num_t SCL_PIN = GPIO_NUM_20;
constexpr i2c_port_t I2C_PORT = I2C_NUM_0;
constexpr TickType_t SAMPLE_MS = 10;

/* ───── PWM pins ──────────────────────────────────────────────── */

constexpr int BACK_L_PWM = 12;
constexpr int BACK_R_PWM = 32;
static constexpr int NECK_UP_PWM = 15;
static constexpr int NECK_DN_PWM = 4;

constexpr ledc_channel_t CH_BACK_L = LEDC_CHANNEL_0;
constexpr ledc_channel_t CH_BACK_R = LEDC_CHANNEL_1;
constexpr ledc_channel_t CH_NECK_UP = LEDC_CHANNEL_2;
constexpr ledc_channel_t CH_NECK_DN = LEDC_CHANNEL_3;

/* ─── App parameters ──────────────────────────────────────────── */

constexpr float TH_ROLL  = 15.0f;          // deg threshold
constexpr float TH_PITCH = 10.0f;          // deg threshold
constexpr uint32_t PWM_DUTY = 150;         // 0‑255 (8‑bit)

/* ───── Bluetooth params ───────────────────────────────────────────── */

static constexpr char SPP_TAG[]         = "SPP_SERVER";
static constexpr char DEVICE_NAME[]     = "ESP32_SPP_CPP";
static constexpr char SPP_SERVER_NAME[] = "SPP_SERVER";
extern "C" void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
static uint32_t client_handle = 0;

/* ───── data structures ─────────────────────────────────────────── */
struct SensorData
  {
    float roll, 
    pitch;
  };

enum Command { UNKNOWN=0, Q1, Q2, Q3, Q4 };


/* ───── RTOS objects ────────────────────────────────────────────── */
static SemaphoreHandle_t i2cMutex;
static SemaphoreHandle_t sdMutex;
static QueueHandle_t sensorQueueBack;
static QueueHandle_t sensorQueueNeck;
static QueueHandle_t cmdLineQueue;           // holds char* lines
static QueueHandle_t sensorQueueCmd = nullptr;
static constexpr size_t MAX_CMD_LINE = 32;   // max length of one command

/* ───── MPU handles ─────────────────────────────────────────────── */
static mpu6050_handle_t imu_back = nullptr;
static mpu6050_handle_t imu_neck = nullptr;


static TaskHandle_t telemetryHandle = nullptr;
static TaskHandle_t hard_rt_task_handle = nullptr;


/* ───── SD card pins ──────────────────────────────────────────── */
static constexpr int SD_CS   = 5;
static constexpr int SD_MOSI = 19;
static constexpr int SD_MISO = 21;
static constexpr int SD_CLK  = 27;

/* ───── forward declarations ────────────────────────────────────── */
static void i2c_init(void);
static esp_err_t imu_init(void);
static void ledc_init(void);


// ============================= Hard Real-Time Definitions ===========================
// 1a) ISR, marked IRAM_ATTR so it never stalls on flash
void IRAM_ATTR timer_isr(void*){
  timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
  timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
  BaseType_t awoken = pdFALSE;
  vTaskNotifyGiveFromISR(hard_rt_task_handle, &awoken);
  if (awoken) portYIELD_FROM_ISR();
}

// 1b) One-time timer setup, called from app_main()
static void init_hardware_timer() {
    // 1) Zero out the config struct
    timer_config_t cfg = {};

    // 2) Fill in all required fields
    cfg.divider     = 80;                  // 80 MHz / 80 = 1 MHz → 1 µs tick
    cfg.counter_dir = TIMER_COUNT_UP;      // count up
    cfg.counter_en  = TIMER_PAUSE;         // start paused
    cfg.alarm_en    = TIMER_ALARM_EN;      // enable alarm
    cfg.auto_reload = TIMER_AUTORELOAD_EN;                // reload on alarm
    cfg.intr_type   = TIMER_INTR_LEVEL;    // level interrupt
    // (clk_src defaults to APB; if you need REF_TICK you can set cfg.clk_src)

    // 3) Apply the config
    timer_init(TIMER_GROUP_0, TIMER_0, &cfg);

    // 4) Reset and set the alarm value
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value  (TIMER_GROUP_0, TIMER_0, 1000);  // 1 ms

    // 5) Hook up and enable the ISR
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(
      TIMER_GROUP_0, TIMER_0, timer_isr, nullptr,
      ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1,
      nullptr
    );

    // 6) Finally kick the timer off
    timer_start(TIMER_GROUP_0, TIMER_0);

    ESP_LOGI("RT_TIMER", "Hardware timer configured at 1 kHz");
}

// ============================= Init Functions =============================

static void i2c_init()
{
    i2c_config_t cfg{};
    cfg.mode = I2C_MODE_MASTER;
    cfg.sda_io_num = SDA_PIN;
    cfg.scl_io_num = SCL_PIN;
    cfg.sda_pullup_en = cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.master.clk_speed = 400000; /* 100 kHz if long wires */
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));
}

void init_nvs() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void init_bt_controller() {
    // 1) Release BLE-only memory (we'll use BTDM)
    ESP_ERROR_CHECK(
      esp_bt_controller_mem_release(ESP_BT_MODE_BLE)
    );

    // 2) Init controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));

    // 3) *Enable* in dual mode (both Classic + BLE)

    ESP_ERROR_CHECK(
      esp_bt_controller_enable(ESP_BT_MODE_BTDM)
    );
}


void init_bluedroid() {
    ESP_ERROR_CHECK( esp_bluedroid_init() );
    ESP_ERROR_CHECK( esp_bluedroid_enable() );
}

void init_gap() {
    // set the Bluetooth “friendly” name
    ESP_ERROR_CHECK(esp_bt_gap_set_device_name(DEVICE_NAME));

    // allow others to find & connect
    ESP_ERROR_CHECK( esp_bt_gap_set_scan_mode(
        ESP_BT_CONNECTABLE,
        ESP_BT_GENERAL_DISCOVERABLE
    ) );
}

void init_spp() {
    // register your event handler
    ESP_ERROR_CHECK( esp_spp_register_callback(esp_spp_cb) );

    // default config with callback-mode
    esp_spp_cfg_t cfg = BT_SPP_DEFAULT_CONFIG();
    cfg.mode = ESP_SPP_MODE_CB;  // callback mode
    ESP_ERROR_CHECK(esp_spp_enhanced_init(&cfg));
}

static esp_err_t imu_init()
{
    /* Back IMU @0x68 ------------------------------------------------ */
    imu_back = mpu6050_create(I2C_PORT, MPU6050_I2C_ADDRESS);
    if (!imu_back || mpu6050_wake_up(imu_back) != ESP_OK)
        return ESP_FAIL;
    mpu6050_config(imu_back, ACCE_FS_4G, GYRO_FS_500DPS);

    /* Neck IMU @0x69 ------------------------------------------------ */
    imu_neck = mpu6050_create(I2C_PORT, MPU6050_I2C_ADDRESS_1);
    if (!imu_neck || mpu6050_wake_up(imu_neck) != ESP_OK) {
        ESP_LOGW("MAIN","Neck IMU not responding at 0x69 – "
                         "neck task will print errors.");
        /* still OK to run back IMU */
    } else {
        mpu6050_config(imu_neck, ACCE_FS_4G, GYRO_FS_500DPS);
    }
    return ESP_OK;
}


static void ledc_init()
{
    /* timer */
    ledc_timer_config_t timer_cfg{};
    timer_cfg.speed_mode      = LEDC_LOW_SPEED_MODE;
    timer_cfg.duty_resolution = LEDC_TIMER_8_BIT;
    timer_cfg.timer_num       = LEDC_TIMER_0;
    timer_cfg.freq_hz         = 5000; // 5 kHz
    timer_cfg.clk_cfg         = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));


    /* helper to fill the channel struct */
    auto make_ch = [&](int pin, ledc_channel_t ch) {
        ledc_channel_config_t c{};
        c.gpio_num   = pin;
        c.speed_mode = LEDC_LOW_SPEED_MODE;
        c.channel    = ch;
        c.timer_sel  = LEDC_TIMER_0;
        c.intr_type  = LEDC_INTR_DISABLE;
        c.duty       = 0;
        c.hpoint     = 0;
        return c;
    };

    /* create *named* structs so we have l‑values */
    ledc_channel_config_t chL = make_ch(BACK_L_PWM, CH_BACK_L);
    ledc_channel_config_t chR = make_ch(BACK_R_PWM, CH_BACK_R);
    ledc_channel_config_t chUp  = make_ch(NECK_UP_PWM, CH_NECK_UP);
    ledc_channel_config_t chDn  = make_ch(NECK_DN_PWM, CH_NECK_DN);

    ESP_ERROR_CHECK(ledc_channel_config(&chL));
    ESP_ERROR_CHECK(ledc_channel_config(&chR));
    ESP_ERROR_CHECK(ledc_channel_config(&chUp));
    ESP_ERROR_CHECK(ledc_channel_config(&chDn));
}

/* ---------- common math helper ---------- */
static inline void acce_to_angles(const mpu6050_acce_value_t& a,
                                  float& roll, float& pitch)
{
    roll  = atan2f( a.acce_y, a.acce_z)                     * 57.2958f;
    pitch = atan2f( a.acce_x,
                    std::sqrtf(a.acce_y*a.acce_y +
                               a.acce_z*a.acce_z) )         * 57.2958f;
}

/* ---------- Back IMU task ---------- */


// Real Time tasks
static void hard_rt_task(void* pv) {
    mpu6050_acce_value_t a;
    float roll, pitch;
    uint32_t cmd;

    for (;;) {
        // 1) Wait for the 1 kHz ISR to notify us
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 2) Handle any override command
        if (xTaskNotifyWait(0, UINT32_MAX, &cmd, 0) == pdPASS) {
            switch (cmd) {
                case Q1:
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_NECK_UP, PWM_DUTY);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_NECK_UP);
                    break;
                case Q2:
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_NECK_DN, PWM_DUTY);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_NECK_DN);
                    break;
                case Q3:
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_BACK_R, PWM_DUTY);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_BACK_R);
                    break;
                case Q4:
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_BACK_L, PWM_DUTY);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_BACK_L);
                    break;
                default:
                    break;
            }

            // turn off after pulse
            if (cmd == Q1 || cmd == Q2) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_NECK_UP, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_NECK_UP);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_NECK_DN, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_NECK_DN);
            } else {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_BACK_L, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_BACK_L);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_BACK_R, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_BACK_R);
            }

            // done for this tick—loop back and block on the next ISR
            continue;
        }

        // 3) Normal Back IMU
        xSemaphoreTake(i2cMutex, portMAX_DELAY);
          mpu6050_get_acce(imu_back, &a);
        xSemaphoreGive(i2cMutex);
        acce_to_angles(a, roll, pitch);
        SensorData datBack{roll, pitch};
        xQueueSend(sensorQueueBack, &datBack, portMAX_DELAY);

        if      (roll >  TH_ROLL) ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_BACK_R, PWM_DUTY);
        else if (roll < -TH_ROLL) ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_BACK_L, PWM_DUTY);
        else {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_BACK_L, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_BACK_R, 0);
        }
        ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_BACK_L);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_BACK_R);

        // 4) Normal Neck IMU
        xSemaphoreTake(i2cMutex, portMAX_DELAY);
          mpu6050_get_acce(imu_neck, &a);
        xSemaphoreGive(i2cMutex);
        acce_to_angles(a, roll, pitch);
        SensorData datNeck{roll, pitch};
        xQueueSend(sensorQueueNeck, &datNeck, 0);

        if      (pitch >  TH_PITCH) ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_NECK_UP, PWM_DUTY);
        else if (pitch < -TH_PITCH) ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_NECK_DN, PWM_DUTY);
        else {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_NECK_UP, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_NECK_DN, 0);
        }
        ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_NECK_UP);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_NECK_DN);

        
    }
}



/* ---------- Neck IMU task ---------- */

// ==================== taskTelemetry (replace entire function) ====================
void taskTelemetry(void* pv) {
    SensorData back, top;
    float      backRoll  = NAN;   // last valid values
    float      neckPitch = NAN;
    char       buf[64];

    for (;;) {
        ESP_LOGI("TEL", "taskTelemetry alive");
        // pull fresh data if available
        if (xQueueReceive(sensorQueueBack, &back, portMAX_DELAY) == pdPASS) {
            backRoll = back.roll;
            ESP_LOGI("TEL","Back → Roll %.1f, Pitch %.1f", back.roll, back.pitch);
        }
        if (xQueueReceive(sensorQueueNeck, &top, 0) == pdPASS) {
            neckPitch = top.pitch;
            ESP_LOGI("TEL","Neck → Roll %.1f, Pitch %.1f", top.roll, top.pitch);
        }

        // only stream once we have both numbers
        if (!std::isnan(backRoll) && !std::isnan(neckPitch)) {

            uint64_t us = esp_timer_get_time();
            float    t  = us / 1e6f;

            // ---------- SD card ----------
            // xSemaphoreTake(sdMutex, portMAX_DELAY);
            // File f = SD.open("/datalog.csv", FILE_APPEND);
            // if (f) {
            //     f.printf("%0.3f,%.1f,%.1f\n", t, backRoll, neckPitch);
            //     f.close();           // small file, close every write
            // }
            // xSemaphoreGive(sdMutex);

            // ---------- Bluetooth SPP stream ----------
            if (client_handle != 0) {
                int len = snprintf(buf, sizeof(buf),
                                   "%0.3f,%.1f,%.1f\n",
                                   t, backRoll, neckPitch);
                esp_spp_write(client_handle,
                              len,
                              (uint8_t*)buf);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void taskCommand(void* pv) {
    char buf[MAX_CMD_LINE];
    for (;;) {
        if (xQueueReceive(cmdLineQueue, buf, portMAX_DELAY) == pdPASS) {
            char* s = buf;
            while (*s == ' ') ++s;

            // REBOOT
            if (strcasecmp(s, "REBOOT") == 0) {
                const char* msg = "Rebooting now...\n";
                if (client_handle) {
                    esp_spp_write(client_handle,
                                  strlen(msg),
                                  (uint8_t*)msg);
                }
                vTaskDelay(pdMS_TO_TICKS(50));
                esp_restart();
            }

            // parse command
            Command c = UNKNOWN;           // ← fixed assignment
            if      (strcmp(s, "Q1") == 0) c = Q1;
            else if (strcmp(s, "Q2") == 0) c = Q2;
            else if (strcmp(s, "Q3") == 0) c = Q3;
            else if (strcmp(s, "Q4") == 0) c = Q4;

            if (c != UNKNOWN) {
                // blink LED
                gpio_set_level(GPIO_NUM_13, 1);
                vTaskDelay(pdMS_TO_TICKS(100));
                gpio_set_level(GPIO_NUM_13, 0);

                // notify tasks
                if (c == Q1 || c == Q2) {
                    xTaskNotify(hard_rt_task_handle,
                                (uint32_t)c,
                                eSetValueWithOverwrite);
                } else {
                    xTaskNotify(hard_rt_task_handle,
                                (uint32_t)c,
                                eSetValueWithOverwrite);
                }

                // store history if needed
                if (sensorQueueCmd) {
                    xQueueSend(sensorQueueCmd, &c, portMAX_DELAY);
                }
            }
        }
    }
}


// ============================= Bluetooth SPP callback =============================
// This function is called by the SPP stack when events occur
extern "C" void esp_spp_cb(esp_spp_cb_event_t event,
                           esp_spp_cb_param_t *param)
{

    static char lineBuf[MAX_CMD_LINE];
    static size_t idx = 0;
    switch (event) {

        case ESP_SPP_INIT_EVT:
            // SPP subsystem is ready, now start the RFCOMM server
            esp_spp_start_srv(ESP_SPP_SEC_NONE,
                              ESP_SPP_ROLE_SLAVE,
                              0,
                              SPP_SERVER_NAME);
            ESP_LOGI(SPP_TAG, "SPP initialized, starting server");
            break;

        case ESP_SPP_START_EVT:
            // Server is now listening
            ESP_LOGI(SPP_TAG, "SPP server started, name=%s", SPP_SERVER_NAME);
            break;

        case ESP_SPP_SRV_OPEN_EVT:
            // A client has connected: grab the handle
            client_handle = param->srv_open.handle;
            ESP_LOGI(SPP_TAG,
                     "Client connected! handle=%" PRIu32,
                     client_handle);
            break;

        case ESP_SPP_CLOSE_EVT:
            // Client disconnected: clear the handle
            ESP_LOGI(SPP_TAG,
                     "Client disconnected, handle=%" PRIu32,
                     param->close.handle);
            client_handle = 0;
            break;

        case ESP_SPP_DATA_IND_EVT: {
            ESP_LOGI(SPP_TAG, "SPP_DATA_IND_EVT: %d bytes", param->data_ind.len);
            for (int i = 0; i < param->data_ind.len; ++i) {
                char c = param->data_ind.data[i];
                if (c == '\r') continue;
                if (c == '\n' || idx + 1 >= MAX_CMD_LINE) {
                    lineBuf[idx] = '\0';
                    ESP_LOGI(SPP_TAG, "Complete command line: '%s'", lineBuf);
                    if (cmdLineQueue) {
                        if (xQueueSend(cmdLineQueue, lineBuf, 0) == pdPASS) {
                            ESP_LOGI(SPP_TAG, "Enqueued command line");
                        } else {
                            ESP_LOGW(SPP_TAG, "Failed to enqueue command line");
                        }
                    }
                    idx = 0;
                } else {
                    lineBuf[idx++] = c;
                }
            }
            break;
        }



        case ESP_SPP_CONG_EVT:
            // Congestion status changed (flow-control)
            if (param->cong.cong) {
                ESP_LOGW(SPP_TAG, "SPP congested on handle=%" PRIu32,
                         param->cong.handle);
            } else {
                ESP_LOGI(SPP_TAG, "SPP de-congested on handle=%" PRIu32,
                         param->cong.handle);
            }
            break;

        default:
            // handle other events here if you need them
            break;
    }
}

/* ───── app_main ───────────────────────────────────────────────── */
extern "C" void app_main() {
  init_nvs();
  init_bt_controller();
  init_bluedroid();
  init_gap();


  i2cMutex         = xSemaphoreCreateMutex();
  sdMutex          = xSemaphoreCreateMutex();
  sensorQueueBack  = xQueueCreate(20, sizeof(SensorData));
  sensorQueueNeck  = xQueueCreate(20, sizeof(SensorData));
  cmdLineQueue     = xQueueCreate(10, MAX_CMD_LINE);
  sensorQueueCmd   = xQueueCreate(10, sizeof(Command));

  init_spp();
  i2c_init();
  ledc_init();
  ESP_ERROR_CHECK( imu_init() );

  // 1) Hard real-time task on core 0
  xTaskCreatePinnedToCore(
      hard_rt_task,
      "hard_rt",
      4096,
      nullptr,
      configMAX_PRIORITIES - 1,
      &hard_rt_task_handle,
      0
  );

  init_hardware_timer();


  // 2) Telemetry + Command on core 1
  xTaskCreatePinnedToCore(taskTelemetry, "Telemetry", 4096, nullptr,
                          tskIDLE_PRIORITY+1, &telemetryHandle, 1);
  xTaskCreatePinnedToCore(taskCommand,   "Command",   4096, nullptr,
                          tskIDLE_PRIORITY+1, nullptr,            1);

  // GPIO for feedback
  gpio_reset_pin(GPIO_NUM_13);
  gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);

  ESP_LOGI("MAIN","Hard-RT and background tasks started");
}
