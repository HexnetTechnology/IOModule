#include "hexnet_disp_pwr.h"
#include "hexnet_io_map.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DISP_PWR";
static volatile bool s_reset_busy;

void hexnet_disp_pwr_set(bool on)
{
    const int lvl = on ? HEXNET_IO_DISP_5V_EN_ACTIVE : !HEXNET_IO_DISP_5V_EN_ACTIVE;
    gpio_set_level(HEXNET_IO_DISP_5V_EN_GPIO, lvl);
}

void hexnet_disp_pwr_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << HEXNET_IO_DISP_5V_EN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d config FAIL %s", (int)HEXNET_IO_DISP_5V_EN_GPIO, esp_err_to_name(err));
        return;
    }
    gpio_hold_dis(HEXNET_IO_DISP_5V_EN_GPIO);
    hexnet_disp_pwr_set(true);
    ESP_LOGW(TAG, "Display 5V EN GPIO%d = ON (default)", (int)HEXNET_IO_DISP_5V_EN_GPIO);
}

static void hard_reset_task(void *arg)
{
    unsigned wait_ms = (unsigned)(uintptr_t)arg;
    if (wait_ms < 500) {
        wait_ms = HEXNET_IO_DISP_HARD_RESET_MS;
    }
    ESP_LOGW(TAG, "Hard reset: 5V OFF %u ms", wait_ms);
    hexnet_disp_pwr_set(false);
    vTaskDelay(pdMS_TO_TICKS(wait_ms));
    hexnet_disp_pwr_set(true);
    ESP_LOGW(TAG, "Hard reset: 5V ON");
    s_reset_busy = false;
    vTaskDelete(NULL);
}

void hexnet_disp_pwr_hard_reset(unsigned wait_ms)
{
    if (s_reset_busy) {
        ESP_LOGW(TAG, "Hard reset already running");
        return;
    }
    s_reset_busy = true;
    if (xTaskCreate(hard_reset_task, "disp_rst", 2048, (void *)(uintptr_t)wait_ms, 5, NULL) != pdPASS) {
        s_reset_busy = false;
        ESP_LOGE(TAG, "hard reset task FAIL");
    }
}
