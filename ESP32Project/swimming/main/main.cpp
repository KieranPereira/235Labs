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
constexpr uint32_t PWM_DUTY = 150;         // 0‑255 (8‑bit)

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

/* ───── app_main ───────────────────────────────────────────────── */
extern "C" void app_main(void)
{
    i2cMutex        = xSemaphoreCreateMutex();
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

    ESP_LOGI("MAIN","Both IMU tasks started");
}

/* ───── helper implementations ─────────────────────────────────── */

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

/* ---------- LEDC for vibration motors (fixed) ---------- */
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

    ESP_ERROR_CHECK(ledc_channel_config(&chL));
    ESP_ERROR_CHECK(ledc_channel_config(&chR));
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
            ESP_LOGI("NECK","Roll %+6.1f  Pitch %+6.1f", d.roll, d.pitch);
        } else {
            ESP_LOGE("NECK","I2C read fail");
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));
    }
}
