#include "hexnet_app.h"

#include "adc_monitor.h"
#include "hexnet_log.h"
#define LOG_LOCAL_LEVEL HEXNET_LOG_LEVEL_OTHER
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hexnet_bluetooth.h"
#include "hexnet_canbus.h"
#include "hexnet_dim_pwm.h"
#include "hexnet_motor.h"
#include "hexnet_mqtt.h"
#include "hexnet_rgb_strip.h"
#include "hexnet_wifi.h"
#include "hexnet_sensors.h"
#include "hexnet_shift_register.h"
#include "hexnet_wifi_portal.h"

static const char *TAG = "HEXNET_APP";

#define HEXNET_ONLINE_WAIT_MS              60000
#define HEXNET_BLE_SETTLE_AFTER_ONLINE_MS  3000
#define HEXNET_BLE_MIN_INTERNAL_HEAP       45000u
#define HEXNET_MQTT_TASK_PRIO              6
#define HEXNET_BLE_TASK_PRIO               3

/* MQTT + BLE birlikte (hexnet_mqtt.h HEXNET_MQTT_ENABLE=1) */
#define HEXNET_BLE_ENABLE                  1

static void ble_bridge_task(void *arg)
{
    (void)arg;
    while (1) {
        /* Full cJSON bridge PSRAM yokken OOM — lean push yeterli */
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

static void ble_start_task(void *arg)
{
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(HEXNET_BLE_SETTLE_AFTER_ONLINE_MS));

#if HEXNET_MQTT_ENABLE
    for (int i = 0; i < 30; ++i) {
        if (hexnet_mqtt_is_connected() && hexnet_mqtt_has_telemetry_ok() &&
            heap_caps_get_free_size(MALLOC_CAP_INTERNAL) >= HEXNET_BLE_MIN_INTERNAL_HEAP) {
            break;
        }
        ESP_LOGI(TAG, "BLE bekle mqtt=%d tel=%d iheap=%lu",
                 hexnet_mqtt_is_connected() ? 1 : 0,
                 hexnet_mqtt_has_telemetry_ok() ? 1 : 0,
                 (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    if (!hexnet_mqtt_is_connected() ||
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL) < HEXNET_BLE_MIN_INTERNAL_HEAP) {
        ESP_LOGW(TAG, "BLE acilmiyor — MQTT/heap yetersiz (iheap=%lu)",
                 (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        vTaskDelete(NULL);
        return;
    }
#else
    ESP_LOGI(TAG, "BLE-only: MQTT yok, hemen baslat iheap=%lu",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
#endif

    ESP_LOGI(TAG, "ADIM3: BLE basliyor iheap=%lu",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    ble_init();
    xTaskCreate(ble_bridge_task, "ble_bridge", 4096, NULL, HEXNET_BLE_TASK_PRIO, NULL);
    vTaskDelete(NULL);
}

void hexnet_app_start_ble(void)
{
    static bool s_started;
    if (s_started) {
        return;
    }
    s_started = true;
    xTaskCreate(ble_start_task, "ble_start", 4096, NULL, HEXNET_BLE_TASK_PRIO, NULL);
}

static bool wait_platform_online(uint32_t timeout_ms)
{
    const TickType_t t0 = xTaskGetTickCount();
    const TickType_t lim = pdMS_TO_TICKS(timeout_ms);
    while ((xTaskGetTickCount() - t0) < lim) {
        const bool wifi = hexnet_wifi_is_connected();
        const bool mqtt = hexnet_mqtt_is_connected();
        const bool tel = hexnet_mqtt_has_telemetry_ok();
        if (wifi && mqtt && tel) {
            ESP_LOGI(TAG, "PLATFORM ONLINE (wifi+mqtt+telemetry) heap=%lu",
                     (unsigned long)esp_get_free_heap_size());
            return true;
        }
        ESP_LOGI(TAG, "online bekleniyor wifi=%d mqtt=%d tel=%d heap=%lu",
                 wifi ? 1 : 0, mqtt ? 1 : 0, tel ? 1 : 0,
                 (unsigned long)esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    ESP_LOGW(TAG, "PLATFORM ONLINE timeout — devam (wifi=%d mqtt=%d tel=%d)",
             hexnet_wifi_is_connected() ? 1 : 0,
             hexnet_mqtt_is_connected() ? 1 : 0,
             hexnet_mqtt_has_telemetry_ok() ? 1 : 0);
    return false;
}

void hexnet_app_bringup_task(void *arg)
{
    (void)arg;

    /* CAN early (MQTT/BLE heap'inden once) — Display 0x100..0x110 telemetri */
    ESP_LOGE(TAG, "CAN TX+RX early | heap=%lu",
             (unsigned long)esp_get_free_heap_size());
    waveshare_twai_init();
    if (xTaskCreate(send_frames_task, "can_tx", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "can_tx create FAIL");
    }
    if (xTaskCreate(receive_frames_task, "can_rx", 3072, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "can_rx create FAIL");
    }
    if (xTaskCreate(can_watchdog_task, "can_wdg", 4096, NULL, 6, NULL) != pdPASS) {
        ESP_LOGE(TAG, "can_wdg create FAIL");
    }
    /* Motor GPIO: MQTT online beklemeden — 0x750 kacilmasin */
    if (xTaskCreate(hexnet_motor_control_task, "motor", 3072, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "motor create FAIL");
    }
    /* RS485 UART1 early — MQTT sonrasi heap ~6KB kalinca install asla basarisiz kaliyordu */
    (void)hexnet_sensor_rs485_uart_early_init();

    uint32_t delay_ms = hexnet_wifi_has_saved_credentials() ? 1500 : 12000;
    vTaskDelay(pdMS_TO_TICKS(delay_ms));

#if HEXNET_MQTT_ENABLE
    ESP_LOGI(TAG, "ADIM1: MQTT start (online-first)");
    hexnet_mqtt_start_tasks();
    (void)wait_platform_online(HEXNET_ONLINE_WAIT_MS);
#else
    ESP_LOGW(TAG, "ADIM1: MQTT KAPALI — BLE-only mod");
    (void)wait_platform_online;
    (void)HEXNET_ONLINE_WAIT_MS;
#endif

    /* ===== ADIM 2: yerel IO ===== */
    ESP_LOGI(TAG, "ADIM2: sensors/pwm/motor");
    adc_monitor_init();
    xTaskCreate(adc_monitor_task, "adc_mon", 4096, NULL, 5, NULL);
    xTaskCreate(hexnet_dim_pwm_task, "dim_pwm", 4096, NULL, 5, NULL);
    /* motor task already started early with CAN */
    /* Iklim SoT = RS485. DHT yok / baslatma — bitbang UART1 RX'i bozabiliyor. */
    xTaskCreate(hexnet_sensor_rs485_task, "rs485", 4096, NULL, 5, NULL);
    (void)hexnet_dht_task;
    xTaskCreate(hexnet_rgb_strip_task, "rgb_strip", 4096, NULL, 5, NULL);

#if HEXNET_BLE_ENABLE
    ESP_LOGI(TAG, "ADIM3: BLE aciliyor");
    hexnet_app_start_ble();
#else
    ESP_LOGW(TAG, "ADIM3: BLE KAPALI");
#endif

    ESP_LOGI(TAG, "Bring-up complete");
    (void)HEXNET_MQTT_TASK_PRIO;
    vTaskDelete(NULL);
}
