#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "autonav.h"
#include "commander.h"
#include "stabilizer.h"

static const char *TAG = "AUTONAV_TEST";

static volatile uint16_t g_down_tof_mm  = 1200;
static volatile uint16_t g_front_tof_mm = 2000;
static volatile bool     g_down_ok  = true;
static volatile bool     g_front_ok = true;

bool sensorsGetDownTofMm(uint16_t* out_mm) {
    if (!out_mm) return false;
    *out_mm = g_down_tof_mm;
    return g_down_ok;
}

bool sensorsGetFrontTofMm(uint16_t* out_mm) {
    if (!out_mm) return false;
    *out_mm = g_front_tof_mm;
    return g_front_ok;
}

static void autonav_test_task(void *arg)
{
    ESP_LOGI(TAG, "Init autonav + start shape");
    autonavInit();
    autonavStartShape(1);
    autonavKickSafety();
    uint32_t start_ms = (uint32_t)(esp_timer_get_time()/1000ULL);
    uint32_t last_beat = start_ms;

    const TickType_t dt_ticks = pdMS_TO_TICKS(10);
    for (;;) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time()/1000ULL);

        if (now_ms - start_ms < 5000) {
            float t = (now_ms - start_ms) * 0.001f;
            g_down_tof_mm = (uint16_t)(1200.0f + 10.0f * sinf(2.0f * 3.14159f * 0.5f * t));
            g_front_tof_mm = 2000;
        } else if (now_ms - start_ms < 15000) {
            g_front_tof_mm = 500;
            g_down_tof_mm  = 1200;
        } else if (now_ms - start_ms < 20000) {
            g_front_tof_mm = 2000;
            g_down_tof_mm  = 1200;
        } else {
            g_front_tof_mm = 2000;
            g_down_tof_mm  = 1200;
        }

        if (now_ms - last_beat >= 1000) {
            autonavKickSafety();
            last_beat = now_ms;
        }

        autonavUpdate(now_ms);

        extern autonav_state_t autonavGetState(void);
        autonav_state_t st = autonavGetState();
        ESP_LOGI(TAG, "t=%ums  down=%umm  front=%umm  state=%d",
                 (unsigned)(now_ms - start_ms), g_down_tof_mm, g_front_tof_mm, (int)st);

        vTaskDelay(dt_ticks);
    }
}

void app_main(void)
{
    xTaskCreatePinnedToCore(autonav_test_task, "autonav_test", 4096, NULL, 5, NULL, 1);
}
