/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"



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




extern "C" void app_main(void)
{
    printf("Hello world!\n");

    i2cMutex = xSemaphoreCreateMutex();
    sdMutex  = xSemaphoreCreateMutex();
    sensorQueueBack = xQueueCreate(10,sizeof(SensorData));
    sensorQueueNeck = xQueueCreate(10,sizeof(SensorData));
    cmdQueue        = xQueueCreate(10,sizeof(Command));

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
