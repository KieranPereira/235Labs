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


#include <cmath>

extern "C" {
#include "mpu6050.h"
}

/* ───── I²C pins ──────────────────────────────────────────────── */
constexpr gpio_num_t SDA_PIN = GPIO_NUM_22;
constexpr gpio_num_t SCL_PIN = GPIO_NUM_20;
constexpr i2c_port_t I2C_PORT = I2C_NUM_0;
constexpr TickType_t SAMPLE_MS = 500;

/* ───── PWM pins ──────────────────────────────────────────────── */

constexpr int BACK_L_PWM = 12;
constexpr int BACK_R_PWM = 32;
static constexpr int NECK_UP_PWM = 15;
static constexpr int NECK_DN_PWM = 4;

constexpr ledc_channel_t CH_BACK_L = LEDC_CHANNEL_0;
constexpr ledc_channel_t CH_BACK_R = LEDC_CHANNEL_1;
constexpr ledc_channel_t CH_NECK_L = LEDC_CHANNEL_2;
constexpr ledc_channel_t CH_NECK_R = LEDC_CHANNEL_3;

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


/* ───── RTOS objects ────────────────────────────────────────────── */
static SemaphoreHandle_t i2cMutex;
static QueueHandle_t sensorQueueBack;
static QueueHandle_t sensorQueueNeck;

/* ───── MPU handles ─────────────────────────────────────────────── */
static mpu6050_handle_t imu_back = nullptr;
static mpu6050_handle_t imu_neck = nullptr;

/* ───── forward declarations ────────────────────────────────────── */
static void i2c_init(void);
static esp_err_t imu_init(void);
static void ledc_init(void);
static void taskIMUBack(void* pv);
static void taskIMUNeck(void* pv);

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
    ESP_ERROR_CHECK( esp_bt_dev_set_device_name(DEVICE_NAME) );

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
    ledc_timer_config_t t{
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = 5000,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));

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
    ledc_channel_config_t chUp  = make_ch(NECK_UP_PWM, CH_NECK_L);
    ledc_channel_config_t chDn  = make_ch(NECK_DN_PWM, CH_NECK_R);

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
static void taskIMUBack(void*)
{
    SensorData d{};
    mpu6050_acce_value_t a;

    for (;;)
    {
        if (mpu6050_get_acce(imu_back, &a) == ESP_OK) {
            acce_to_angles(a, d.roll, d.pitch);
            xQueueSend(sensorQueueBack, &d, 0);

            /* --- drive motors --- */
            if      (d.roll >  TH_ROLL) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_BACK_R, PWM_DUTY);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_BACK_R);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_BACK_L, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_BACK_L);
            }
            else if (d.roll < -TH_ROLL) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_BACK_L, PWM_DUTY);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_BACK_L);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_BACK_R, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_BACK_R);
            }
            else {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_BACK_L, 0);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_BACK_R, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_BACK_L);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_BACK_R);
            }

            ESP_LOGI("BACK","Roll %+6.1f  Pitch %+6.1f", d.roll, d.pitch);
        }
        else {
            ESP_LOGE("BACK","I2C read fail");
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));
    }
}

/* ---------- Neck IMU task ---------- */
/* ---------- Neck IMU task with thresholding ---------- */
static void taskIMUNeck(void*)
{
    if (!imu_neck) {
        ESP_LOGW("NECK","No handle – task suspended");
        vTaskDelete(nullptr);
    }

    SensorData d{};
    mpu6050_acce_value_t a;

    for (;;)
    {
        if (mpu6050_get_acce(imu_neck, &a) == ESP_OK) {

            acce_to_angles(a, d.roll, d.pitch);
            xQueueSend(sensorQueueNeck, &d, 0);

            /* ---- pitch‑based motor drive ---- */
            if      (d.pitch >  TH_PITCH) {   // tip head UP
                ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_NECK_L, PWM_DUTY);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_NECK_L);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_NECK_R, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_NECK_R);
            }
            else if (d.pitch < -TH_PITCH) {   // tip head DOWN
                ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_NECK_R, PWM_DUTY);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_NECK_R);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_NECK_L, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_NECK_L);
            }
            else {                            // within dead‑band
                ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_NECK_L, 0);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_NECK_R, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_NECK_L);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_NECK_R);
            }

            ESP_LOGI("NECK","Roll %+6.1f  Pitch %+6.1f", d.roll, d.pitch);

        } else {
            ESP_LOGE("NECK","I2C read fail");
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));
    }
}

// ==================== taskTelemetry (replace entire function) ====================
void taskTelemetry(void* pv) {
    SensorData back, top;
    float      backRoll  = NAN;   // last valid values
    float      neckPitch = NAN;
    char       buf[64];

    for (;;) {
        // pull fresh data if available
        if (xQueueReceive(sensorQueueBack, &back, 0) == pdPASS) {
            backRoll = back.roll;
        }
        if (xQueueReceive(sensorQueueNeck, &top, 0) == pdPASS) {
            neckPitch = top.pitch;
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
// ============================= Bluetooth SPP callback =============================
// This function is called by the SPP stack when events occur
extern "C" void esp_spp_cb(esp_spp_cb_event_t event,
                           esp_spp_cb_param_t *param)
{
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

        case ESP_SPP_DATA_IND_EVT:
            // Data received from the client
            ESP_LOGI(SPP_TAG,
                     "Received %d bytes on handle=%" PRIu32,
                     param->data_ind.len,
                     param->data_ind.handle);
            // Example: echo back what we got
            esp_spp_write(param->data_ind.handle,
                          param->data_ind.len,
                          param->data_ind.data);
            break;

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
extern "C" void app_main(void)
{


    init_nvs();
    init_bt_controller();
    init_bluedroid();
    init_gap();
    init_spp();


    i2cMutex        = xSemaphoreCreateMutex();
    // sdMutex         = xSemaphoreCreateMutex();
    sensorQueueBack = xQueueCreate(10, sizeof(SensorData));
    sensorQueueNeck = xQueueCreate(10, sizeof(SensorData));

    i2c_init();
    ledc_init();
    if (imu_init() != ESP_OK) {
        ESP_LOGE("MAIN","IMU init failed – check wiring.");
        return;
    }

    xTaskCreate(taskIMUBack, "IMU Back", 4096, nullptr, 2, nullptr);
    xTaskCreate(taskIMUNeck, "IMU Neck", 4096, nullptr, 2, nullptr);
    xTaskCreate(taskTelemetry, "Telemetry",    4096, nullptr, 5, nullptr);

    ESP_LOGI("MAIN","Both IMU tasks started");
}
