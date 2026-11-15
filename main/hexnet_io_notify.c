#include "hexnet_io_notify.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hexnet_bluetooth.h"
#include "hexnet_canbus.h"
#include "hexnet_mqtt.h"

static const char *TAG = "HEXNET_IO_SYNC";

/** depth-1 coalesce: birden fazla notify tek taramaya iner */
static QueueHandle_t s_notify_q;
static TaskHandle_t s_notify_task;

static void notify_worker_task(void *arg)
{
    (void)arg;
    uint8_t token = 0;
    while (1) {
        if (xQueueReceive(s_notify_q, &token, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        /* Kuyrukta biriken ekstra istekleri birlestir */
        while (xQueueReceive(s_notify_q, &token, 0) == pdTRUE) {
        }

        /* BLE RGB mirror = HW (MQTT rgb_set sonrasi tablet eski kalmasin) */
        hexnet_ble_mirror_rgb_from_hw();

        /* Display: anlik CAN (hafif TX) */
        hexnet_can_push_state_now();

        /* BLE: mevcut ble_push worker tuketsin */
        hexnet_ble_request_push();

        /* MQTT: telemetry task force-publish (lifecycle/lock orada) */
        hexnet_mqtt_request_telemetry_now();

        ESP_LOGI(TAG, "state queued can+ble+mqtt mask=0x%04X iheap=%u",
                 (unsigned)get_outputs(),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    }
}

void hexnet_io_notify_init(void)
{
    if (s_notify_q != NULL) {
        return;
    }
    s_notify_q = xQueueCreate(1, sizeof(uint8_t));
    if (s_notify_q == NULL) {
        ESP_LOGE(TAG, "notify queue create failed");
        return;
    }
    /* Kucuk stack — agir JSON burada yok */
    BaseType_t ok = xTaskCreate(notify_worker_task, "io_sync", 3072, NULL, 5, &s_notify_task);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "notify worker create failed");
        s_notify_task = NULL;
    } else {
        ESP_LOGI(TAG, "async fan-out worker ready");
    }
}

void hexnet_io_notify_state_changed(void)
{
    if (s_notify_q == NULL) {
        /* Init oncesi: senkron dusme (boot race) — sadece CAN */
        hexnet_can_push_state_now();
        return;
    }
    uint8_t token = 1;
    /* Doluysa zaten pending — coalesced */
    (void)xQueueSend(s_notify_q, &token, 0);
}
