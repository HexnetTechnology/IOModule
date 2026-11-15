#include "hexnet_panel_sync.h"

#include "esp_log.h"
#include "hexnet_bluetooth.h"
#include "hexnet_canbus.h"
#include "hexnet_mqtt.h"
/* request APIs: hexnet_mqtt_request_telemetry_now / hexnet_ble_request_push */
#include "nvs.h"

static const char *TAG = "PANEL_SYNC";

#define NVS_NS "hexnet"
#define NVS_SEQ_KEY "panel_seq"

#define PUSH_CAN  0x01u
#define PUSH_MQTT 0x02u
#define PUSH_BLE  0x04u

static uint16_t s_panel_seq = 1;
static uint8_t s_push_flags = 0;
static bool s_seq_loaded = false;
/** on_changed icindeki anlik MQTT/BLE publish boyunca true */
static bool s_changed_publish = false;
/** Display'e 0x118/119 isim burst'u henuz bitmedi */
static bool s_labels_pending = false;
/** Degisiklik sonrasi HB ile tekrar gonder (CAN kaybi toleransi). */
static uint8_t s_labels_retries_left = 0;

static void seq_load(void)
{
    if (s_seq_loaded) {
        return;
    }
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        uint16_t v = 0;
        if (nvs_get_u16(h, NVS_SEQ_KEY, &v) == ESP_OK && v != 0) {
            s_panel_seq = v;
        }
        nvs_close(h);
    }
    s_seq_loaded = true;
}

static void seq_save(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    (void)nvs_set_u16(h, NVS_SEQ_KEY, s_panel_seq);
    (void)nvs_commit(h);
    nvs_close(h);
}

uint16_t hexnet_panel_sync_get_seq(void)
{
    seq_load();
    return s_panel_seq;
}

void hexnet_panel_sync_on_changed(void)
{
    seq_load();
    s_panel_seq++;
    if (s_panel_seq == 0) {
        s_panel_seq = 1;
    }
    seq_save();
    s_push_flags = (uint8_t)(PUSH_CAN | PUSH_MQTT | PUSH_BLE);
    s_changed_publish = true;
    s_labels_pending = true;
    /* anlik + 2 heartbeat yeniden burst (Display RX kaybi) */
    s_labels_retries_left = 3;

    ESP_LOGI(TAG, "panel changed -> seq=%u push CAN|MQTT|BLE labels_pending=1 retries=%u",
             (unsigned)s_panel_seq, (unsigned)s_labels_retries_left);

    /* Aninda CAN profile; MQTT/BLE async worker (lifecycle-safe) */
    hexnet_can_publish_io_profile();
    hexnet_panel_sync_can_sent();

    /* Platform MQTT: channels (isim/ikon) sticky — force telemetri paketinde gelsin.
     * (is_changed_publish bayragi async publish'ten once dusuyordu; sticky sart.) */
    hexnet_mqtt_request_channels();
    hexnet_mqtt_request_telemetry_now();
    s_push_flags = (uint8_t)(s_push_flags & (uint8_t)~PUSH_MQTT);

    /* Lean notify channels tasimaz — tablete ikon/isim icin full paket bayragi */
    hexnet_ble_request_channels();
    hexnet_ble_request_push();
    s_push_flags = (uint8_t)(s_push_flags & (uint8_t)~PUSH_BLE);

    s_changed_publish = false;
}

bool hexnet_panel_sync_labels_pending(void)
{
    return s_labels_pending;
}

void hexnet_panel_sync_labels_sent(void)
{
    if (s_labels_retries_left > 0) {
        s_labels_retries_left--;
    }
    if (s_labels_retries_left == 0) {
        s_labels_pending = false;
        ESP_LOGI(TAG, "labels burst complete (no more retries)");
    } else {
        /* pending kalsin — sonraki 0x110 HB tekrar 0x118/119 gonderir */
        ESP_LOGI(TAG, "labels burst ok; retries_left=%u (HB re-TX)",
                 (unsigned)s_labels_retries_left);
    }
}

void hexnet_panel_sync_request_can_labels(void)
{
    seq_load();
    s_labels_pending = true;
    /* Display RX kaybi: anlik + HB yeniden burst */
    s_labels_retries_left = 3;
    s_push_flags = (uint8_t)(s_push_flags | PUSH_CAN);
    ESP_LOGW(TAG, "Display label-sync-req → CAN 0x118/119 + BLE/MQTT channels retries=%u",
             (unsigned)s_labels_retries_left);
    hexnet_can_publish_io_profile();
    hexnet_panel_sync_can_sent();
    /* Tablet/platform da ayni SoT'u alsin — lean telemetride channels yoktu */
    hexnet_mqtt_request_channels();
    hexnet_mqtt_request_telemetry_now();
    hexnet_ble_request_channels();
    hexnet_ble_request_push();
}

bool hexnet_panel_sync_is_changed_publish(void)
{
    return s_changed_publish;
}

bool hexnet_panel_sync_can_should_send(bool heartbeat_due)
{
    /* NVS yok — sadece RAM flag / heartbeat */
    if (s_push_flags & PUSH_CAN) {
        return true;
    }
    return heartbeat_due;
}

void hexnet_panel_sync_can_sent(void)
{
    s_push_flags = (uint8_t)(s_push_flags & (uint8_t)~PUSH_CAN);
}
