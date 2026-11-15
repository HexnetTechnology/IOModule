#include "hexnet_log.h"
/* CAN RX debug: anlamli/anlamsiz frame ayirir (HEXNET_CAN_RX log tag) */
#define HEXNET_CAN_RX_DEBUG 1
#if HEXNET_CAN_RX_DEBUG
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#else
#define LOG_LOCAL_LEVEL HEXNET_LOG_LEVEL_OTHER
#endif
#include "hexnet_canbus.h" // Include the Waveshare TWAI port library
#include "hexnet_io_profile.h"
#include "hexnet_io_labels.h"
#include "hexnet_disp_pwr.h"
#include "hexnet_motor.h"
#include "hexnet_shift_register.h"
#include "hexnet_bluetooth.h"
#include "hexnet_io_notify.h"
#include "hexnet_wifi.h"
#include "hexnet_mqtt.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "inttypes.h"
#include <time.h>
static bool driver_installed = false; // Driver installation status
unsigned long previousMillis = 0;     // Will store last time a message was sent
static uint32_t error_recovery_count = 0; // Error recovery counter
static uint32_t last_successful_tx = 0;   // Last successful transmission timestamp

// Forward declarations
static void check_and_recover_from_errors(void);
static void monitor_can_health(void);

// TWAI timing configuration for 50 Kbits
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
// TWAI filter configuration to accept all messages
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
// General configuration for TWAI
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NO_ACK);
#define TAG "CAN_BUS"
#define CAN_DBG_TAG "HEXNET_CAN_RX"

// CAN frame IDs (debug + TX task)
#define FRAME_1_ID 0x100
#define FRAME_PROFILE_ID 0x110 /* IO flash io_profile SoT → Display */
#define FRAME_TIME_ID 0x11A    /* NTP unix + yerel HH:MM:SS → Display (1/dk) */
#define FRAME_2_ID 0x200
#define FRAME_2B_ID 0x210 /* dimmers 4..7 (v0.60 8-dim) */
#define FRAME_3_ID 0x300
#define PROFILE_CAN_HEARTBEAT_MS 5000
#define TIME_CAN_PERIOD_MS 60000
#define CAN_TIME_FLAG_WIFI 0x01u
#define CAN_TIME_FLAG_NTP  0x02u

#include "hexnet_panel_sync.h"

#if HEXNET_CAN_RX_DEBUG
typedef struct {
    uint32_t rx_total;
    uint32_t rx_relay_720;
    uint32_t rx_relay_720_valid;
    uint32_t rx_relay_720_invalid;
    uint32_t rx_dim_730;
    uint32_t rx_rgb_740;
    uint32_t rx_motor_750;
    uint32_t rx_own_tx_echo;
    uint32_t rx_unknown;
    uint32_t rx_rtr;
    uint32_t rx_dlc_bad;
    uint64_t stats_last_ms;
} can_rx_debug_stats_t;

static can_rx_debug_stats_t s_can_rx_dbg;

static void can_debug_format_data(const twai_message_t *msg, char *out, size_t out_len)
{
    if (!msg || !out || out_len == 0) {
        return;
    }
    size_t pos = 0;
    for (int i = 0; i < msg->data_length_code && i < 8; ++i) {
        if (pos + 4 >= out_len) {
            break;
        }
        pos += (size_t)snprintf(out + pos, out_len - pos, "%02X%s", msg->data[i], (i + 1 < msg->data_length_code) ? " " : "");
    }
    if (pos == 0) {
        (void)snprintf(out, out_len, "(empty)");
    }
}

static bool can_debug_is_own_tx_id(uint32_t id)
{
    return id == FRAME_1_ID || id == FRAME_PROFILE_ID || id == FRAME_TIME_ID ||
           id == FRAME_2_ID || id == FRAME_2B_ID || id == FRAME_3_ID ||
           id == 0x111 || id == 0x112 || id == 0x118 || id == 0x119;
}

static bool can_debug_is_handler_id(uint32_t id)
{
    return id == 0x720 || id == 0x730 || id == 0x740 || id == 0x750 ||
           id == 0x761 || id == 0x762 || id == 0x763 || id == 0x764 || id == 0x770;
}

static void can_debug_print_stats_if_due(void)
{
    const uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000LL);
    if (s_can_rx_dbg.stats_last_ms != 0 && (now_ms - s_can_rx_dbg.stats_last_ms) < 30000ULL) {
        return;
    }
    s_can_rx_dbg.stats_last_ms = now_ms;
    ESP_LOGI(
        CAN_DBG_TAG,
        "ozet 30s: total=%" PRIu32 " relay720=%" PRIu32 " (ok=%" PRIu32 " HATA=%" PRIu32 ") dim730=%" PRIu32
        " rgb740=%" PRIu32 " motor750=%" PRIu32 " tx_echo=%" PRIu32 " bilinmeyen=%" PRIu32 " rtr=%" PRIu32
        " dlc_hata=%" PRIu32,
        s_can_rx_dbg.rx_total,
        s_can_rx_dbg.rx_relay_720,
        s_can_rx_dbg.rx_relay_720_valid,
        s_can_rx_dbg.rx_relay_720_invalid,
        s_can_rx_dbg.rx_dim_730,
        s_can_rx_dbg.rx_rgb_740,
        s_can_rx_dbg.rx_motor_750,
        s_can_rx_dbg.rx_own_tx_echo,
        s_can_rx_dbg.rx_unknown,
        s_can_rx_dbg.rx_rtr,
        s_can_rx_dbg.rx_dlc_bad);
}

static bool can_debug_inspect_rx(const twai_message_t *msg, const char *via)
{
    if (!msg) {
        return false;
    }

    s_can_rx_dbg.rx_total++;

    char data_hex[32];
    can_debug_format_data(msg, data_hex, sizeof(data_hex));

    if (msg->rtr) {
        s_can_rx_dbg.rx_rtr++;
        ESP_LOGW(
            CAN_DBG_TAG,
            "[%s] RTR id=0x%03lX dlc=%d data=%s (anlamsiz — RTR)",
            via ? via : "?",
            (unsigned long)msg->identifier,
            msg->data_length_code,
            data_hex);
        return false;
    }

    if (msg->identifier == 0x720) {
        s_can_rx_dbg.rx_relay_720++;
        if (msg->data_length_code < 2) {
            s_can_rx_dbg.rx_dlc_bad++;
            s_can_rx_dbg.rx_relay_720_invalid++;
            ESP_LOGW(
                CAN_DBG_TAG,
                "[%s] RELAY720 GECERSIZ dlc=%d (min 2) data=%s",
                via ? via : "?",
                msg->data_length_code,
                data_hex);
            return false;
        }

        const uint8_t index = msg->data[0];
        const uint8_t val = msg->data[1];
        bool invalid = false;

        if (index >= 16) {
            invalid = true;
            ESP_LOGW(
                CAN_DBG_TAG,
                "[%s] RELAY720 GECERSIZ index=%u (0-15) val=%u data=%s",
                via ? via : "?",
                (unsigned)index,
                (unsigned)val,
                data_hex);
        } else if (val > 1) {
            invalid = true;
            ESP_LOGW(
                CAN_DBG_TAG,
                "[%s] RELAY720 SUPHELI index=%u val=%u (0/1 degil) data=%s",
                via ? via : "?",
                (unsigned)index,
                (unsigned)val,
                data_hex);
        }

        if (invalid) {
            s_can_rx_dbg.rx_relay_720_invalid++;
            return false;
        }

        s_can_rx_dbg.rx_relay_720_valid++;
        ESP_LOGI(
            CAN_DBG_TAG,
            "[%s] RELAY720 OK r%u=%s mask_oncesi=0x%04X data=%s",
            via ? via : "?",
            (unsigned)index,
            val ? "ON" : "OFF",
            (unsigned)get_outputs(),
            data_hex);
        return true;
    }

    if (msg->identifier == 0x730) {
        s_can_rx_dbg.rx_dim_730++;
        if (msg->data_length_code < 2 || msg->data[0] >= 8) {
            ESP_LOGW(CAN_DBG_TAG, "[%s] DIM730 supheli dlc=%d data=%s", via ? via : "?", msg->data_length_code, data_hex);
        } else {
            ESP_LOGD(CAN_DBG_TAG, "[%s] DIM730 idx=%u val=%u", via ? via : "?", msg->data[0], msg->data[1]);
        }
        return true;
    }

    if (msg->identifier == 0x740) {
        s_can_rx_dbg.rx_rgb_740++;
        if (msg->data_length_code < 3) {
            ESP_LOGW(CAN_DBG_TAG, "[%s] RGB740 supheli dlc=%d data=%s", via ? via : "?", msg->data_length_code, data_hex);
        }
        return true;
    }

    if (msg->identifier == 0x750) {
        s_can_rx_dbg.rx_motor_750++;
        return true;
    }

    if (can_debug_is_own_tx_id(msg->identifier)) {
        s_can_rx_dbg.rx_own_tx_echo++;
        ESP_LOGW(
            CAN_DBG_TAG,
            "[%s] TX_ECHO id=0x%03lX dlc=%d data=%s (bus geri yansima?)",
            via ? via : "?",
            (unsigned long)msg->identifier,
            msg->data_length_code,
            data_hex);
        return false;
    }

    if (!can_debug_is_handler_id(msg->identifier)) {
        s_can_rx_dbg.rx_unknown++;
        ESP_LOGW(
            CAN_DBG_TAG,
            "[%s] BILINMEYEN id=0x%03lX dlc=%d data=%s",
            via ? via : "?",
            (unsigned long)msg->identifier,
            msg->data_length_code,
            data_hex);
        return false;
    }

    return true;
}
#endif /* HEXNET_CAN_RX_DEBUG */

uint8_t rgb_enable = 0; // Enable RGB output

// Frame data
uint8_t frame_1_data[8] = {0};  // First frame data (Voltage, Outputs, Inputs)
uint8_t frame_2_data[8] = {0};  // Second frame data (Analog inputs, Dimmable outputs 0..3)
uint8_t frame_2b_data[8] = {0}; // Dimmer outputs 4..7
uint8_t frame_3_data[8] = {0};  // Third frame data (RGB colors)
uint16_t simvoltage = 1350;
uint16_t simoutputs = 0;
uint16_t siminputs = 0;
int motorData = 0;
extern int motorDataUpdateCounter;
/* 255 = henuz gecerli ornek yok (Display hum>100 → Bağlı gösterme) */
int sensorTemp = 255;
int sensorHumidity = 255;

int get_motorData() {
    return motorData;
}

void set_motordata(int val) {
    motorData = val;
}
void set_sensorTemp(int val) {
    sensorTemp = val;
}
void set_sensorHumidity(int val) {
    sensorHumidity = val;
}   
int get_sensorTemp(void) {
    return sensorTemp;
}
int get_sensorHumidity(void) {
    return sensorHumidity;
}
/* 0x100 data[6] link flags — Display sag ust IO/BLE/WiFi */
#define CAN_LINK_WIFI  0x01u
#define CAN_LINK_BLE   0x02u
#define CAN_LINK_MQTT  0x04u

// Helper function to populate Frame 1 data
void populate_frame_1(uint16_t voltage, uint16_t outputs, uint16_t inputs) {
    frame_1_data[0] = (voltage >> 8) & 0xFF;  // Voltage MSB
    frame_1_data[1] = voltage & 0xFF;         // Voltage LSB
    frame_1_data[2] = (outputs >> 8) & 0xFF;  // Outputs MSB
    frame_1_data[3] = outputs & 0xFF;         // Outputs LSB
    frame_1_data[4] = (inputs >> 8) & 0xFF;   // Inputs MSB
    frame_1_data[5] = inputs & 0xFF;          // Inputs LSB
    uint8_t links = 0;
    if (hexnet_wifi_is_connected()) {
        links |= CAN_LINK_WIFI;
    }
    if (hexnet_ble_is_connected()) {
        links |= CAN_LINK_BLE;
    }
    if (hexnet_mqtt_is_connected()) {
        links |= CAN_LINK_MQTT;
    }
    frame_1_data[6] = links;
    frame_1_data[7] = 0;
}

// Helper function to populate Frame 2 data (analog + dim 0..3)
void populate_frame_2(uint8_t analog_inputs[4], uint8_t dimmable_outputs[8]) {
    for (int i = 0; i < 4; i++) {
        frame_2_data[i] = analog_inputs[i];        // Analog inputs
        frame_2_data[i + 4] = dimmable_outputs[i]; // Dimmable outputs 0..3
    }
}

static void populate_frame_2b(uint8_t dimmable_outputs[8])
{
    for (int i = 0; i < 4; i++) {
        frame_2b_data[i] = dimmable_outputs[i + 4]; /* dims 4..7 */
        frame_2b_data[i + 4] = 0;
    }
}

// Helper function to populate Frame 3 data
void populate_frame_3(uint8_t r, uint8_t g, uint8_t b) {
    frame_3_data[0] = r;  // Red
    frame_3_data[1] = g;  // Green
    frame_3_data[2] = b;  // Blue
    frame_3_data[3] = rgb_enable; // RGB enable flag
    frame_3_data[4] = sensorTemp; // Sensor Temperature
    frame_3_data[5] = sensorHumidity; // Sensor Humidity
}



// Function to send a CAN frame
void send_can_frame(uint32_t id, uint8_t *data) {
    static uint32_t s_tx_fail_log_ms;
    twai_message_t message;
    message.identifier = id;
    message.rtr = 0;  // Data frame
    message.data_length_code = 8;  // Max CAN data length is 8 bytes
    memcpy(message.data, data, 8);

    /* Bus yokken uzun block + LOGE flood heap/CPU eritir */
    esp_err_t res = twai_transmit(&message, pdMS_TO_TICKS(20));
    if (res != ESP_OK) {
        const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if ((now - s_tx_fail_log_ms) > 5000u) {
            s_tx_fail_log_ms = now;
            twai_status_info_t st;
            twai_get_status_info(&st);
            ESP_LOGW(TAG, "CAN TX fail id=0x%03X err=0x%x state=%d q_tx=%" PRIu32,
                     (unsigned int)id, res, (int)st.state, st.msgs_to_tx);
            if (st.msgs_to_tx > 0) {
                (void)twai_clear_transmit_queue();
            }
        }
        check_and_recover_from_errors();
    } else {
        last_successful_tx = esp_timer_get_time() / 1000;
    }
}

#define FRAME_RELAY_TYPE_LO 0x111 /* relay_type[0..7] */
#define FRAME_RELAY_TYPE_HI 0x112 /* relay_type[8..15] */
#define FRAME_LABEL_NAME    0x118 /* [slot][part 0..3][6 B utf8] */
#define FRAME_LABEL_ICON    0x119 /* [slot][part 0..3][6 B utf8] */

static void can_tx_paced(uint32_t id, uint8_t *data)
{
    send_can_frame(id, data);
    /* 0x118/119 burst Display RX kuyrugunu dolduruyordu */
    vTaskDelay(pdMS_TO_TICKS(25));
}

static void can_publish_str_chunks(uint32_t id, uint8_t slot, const char *s)
{
    char buf[24];
    memset(buf, 0, sizeof(buf));
    if (s) {
        strncpy(buf, s, sizeof(buf) - 1);
    }
    for (int part = 0; part < 4; ++part) {
        uint8_t data[8] = {0};
        data[0] = slot;
        data[1] = (uint8_t)part;
        memcpy(&data[2], &buf[part * 6], 6);
        can_tx_paced(id, data);
        bool more = false;
        for (int i = (part + 1) * 6; i < 24; ++i) {
            if (buf[i] != '\0') {
                more = true;
                break;
            }
        }
        if (!more) {
            break;
        }
    }
}

static void can_publish_relay_types(const hexnet_io_profile_t *p)
{
    uint8_t lo[8] = {0};
    uint8_t hi[8] = {0};
    if (!p) {
        return;
    }
    memcpy(lo, &p->relay_type[0], 8);
    memcpy(hi, &p->relay_type[8], 8);
    can_tx_paced(FRAME_RELAY_TYPE_LO, lo);
    can_tx_paced(FRAME_RELAY_TYPE_HI, hi);
    ESP_LOGI(TAG, "TX 0x111/112 relay_type[0]=%u [1]=%u", (unsigned)lo[0], (unsigned)lo[1]);
}

static volatile bool s_labels_tx_busy = false;
static volatile bool s_labels_tx_again = false;

static void can_publish_relay_labels(void)
{
    if (s_labels_tx_busy) {
        /* Cift burst (MQTT+BLE) CAN'de part kaybi yapar — tek seferde bitir, sonra tekrar */
        s_labels_tx_again = true;
        ESP_LOGW(TAG, "TX 0x118 busy — defer");
        return;
    }
    s_labels_tx_busy = true;

    const hexnet_io_labels_t *L = hexnet_io_labels_get();
    if (!L) {
        hexnet_panel_sync_labels_sent();
        s_labels_tx_busy = false;
        return;
    }

    do {
        s_labels_tx_again = false;
        ESP_LOGW(TAG, "TX 0x118/119 labels start r0='%s' ico='%s'",
                 L->relays[0].name, L->relays[0].icon);
        for (uint8_t i = 0; i < 12; ++i) {
            can_publish_str_chunks(FRAME_LABEL_NAME, i, L->relays[i].name);
            can_publish_str_chunks(FRAME_LABEL_ICON, i, L->relays[i].icon);
        }
        ESP_LOGW(TAG, "TX 0x118/119 labels done");
        if (s_labels_tx_again) {
            vTaskDelay(pdMS_TO_TICKS(100)); /* Display NVS/UI bitsin */
        }
    } while (s_labels_tx_again);

    /* Retry sayaci bir kez dus — HB ile tekrar gonderilebilir */
    hexnet_panel_sync_labels_sent();
    s_labels_tx_busy = false;
}

void hexnet_can_publish_io_profile(void)
{
    const hexnet_io_profile_t *p = hexnet_io_profile_get();
    if (!p) {
        return;
    }

    const uint16_t seq = hexnet_panel_sync_get_seq();
    const bool with_labels =
        hexnet_panel_sync_is_changed_publish() || hexnet_panel_sync_labels_pending();
    uint8_t data[8] = {0};
    data[0] = 1; /* version */
    data[1] = (uint8_t)((p->motor_enabled ? 0x01u : 0u) | (p->rgb_enabled ? 0x02u : 0u));
    data[2] = (uint8_t)(p->relay_enable_mask & 0xFFu);
    data[3] = (uint8_t)((p->relay_enable_mask >> 8) & 0xFFu);
    data[4] = p->dim_enable_mask;
    data[5] = p->sensor_enable_mask;
    data[6] = p->tank_enable_mask;
    data[7] = (uint8_t)(seq & 0xFFu);
    can_tx_paced(FRAME_PROFILE_ID, data);
    /* Tip ID her 0x110 ile — isimler panel degisince / pending */
    can_publish_relay_types(p);
    if (with_labels) {
        can_publish_relay_labels();
    }
}

uint8_t analog_inputs[4] = {0, 0, 0, 0};       // [0]=clean water%, [1]=dirty water%, [2..3]=reserved
uint8_t dimmable_outputs[8] = {0}; // 0..100 % — 8 dimmer
uint8_t r = 0, g = 255, b = 0; // Example RGB values (Red color)

void hexnet_can_push_state_now(void)
{
    /* MQTT/BLE degisince Display ~400ms periyodu beklemesin */
    populate_frame_1(simvoltage, simoutputs, siminputs);
    populate_frame_2(analog_inputs, dimmable_outputs);
    populate_frame_2b(dimmable_outputs);
    populate_frame_3(r, g, b);
    send_can_frame(FRAME_1_ID, frame_1_data);
    send_can_frame(FRAME_2_ID, frame_2_data);
    send_can_frame(FRAME_2B_ID, frame_2b_data);
    send_can_frame(FRAME_3_ID, frame_3_data);
}

/** WiFi+NTP varken Display'e yerel saat (CAN 0x11A). */
void hexnet_can_publish_time(void)
{
    if (!hexnet_wifi_is_connected()) {
        return;
    }
    const int64_t ts64 = hexnet_wifi_ntp_unix_ts_sec();
    if (ts64 < 1700000000LL) {
        return;
    }

    const time_t now = (time_t)ts64;
    struct tm local = {0};
    localtime_r(&now, &local);

    uint8_t data[8] = {0};
    const uint32_t unix32 = (uint32_t)ts64;
    data[0] = (uint8_t)((unix32 >> 24) & 0xFFu);
    data[1] = (uint8_t)((unix32 >> 16) & 0xFFu);
    data[2] = (uint8_t)((unix32 >> 8) & 0xFFu);
    data[3] = (uint8_t)(unix32 & 0xFFu);
    data[4] = (uint8_t)(CAN_TIME_FLAG_WIFI | CAN_TIME_FLAG_NTP);
    data[5] = (uint8_t)local.tm_hour;
    data[6] = (uint8_t)local.tm_min;
    data[7] = (uint8_t)local.tm_sec;
    send_can_frame(FRAME_TIME_ID, data);
    ESP_LOGI(TAG, "TX 0x11A time %02d:%02d:%02d unix=%" PRIu32,
             local.tm_hour, local.tm_min, local.tm_sec, unix32);
}

// Function to update analog inputs with sensor data
void update_analog_inputs(float dht1_temp, float dht2_temp, int waterlevel1, int waterlevel2) {
    // Convert temperature to uint8_t (clamp to 0-255 range)
    analog_inputs[2] = (uint8_t)(dht1_temp);
    analog_inputs[3] = (uint8_t)(dht2_temp);
    
}

// Function to update only water levels while preserving temperature values
void update_water_levels_only(int waterlevel1, int waterlevel2) {
    // Only update water level values, preserve existing temperature values
    analog_inputs[0] = (uint8_t)(waterlevel1 > 0 ? (waterlevel1 < 255 ? waterlevel1 : 255) : 0);
    analog_inputs[1] = (uint8_t)(waterlevel2 > 0 ? (waterlevel2 < 255 ? waterlevel2 : 255) : 0);
}

// Task to send frames every 100ms
void send_frames_task(void *arg) {
    /* uint32 tick — int64 local + ESP_LOGI arg clobber 0x110 heartbeat'i kirletiyordu */
    TickType_t last_profile_hb_tick = 0;
    TickType_t last_time_tx_tick = 0;
    const TickType_t hb_ticks = pdMS_TO_TICKS(PROFILE_CAN_HEARTBEAT_MS);
    const TickType_t time_ticks = pdMS_TO_TICKS(TIME_CAN_PERIOD_MS);

    while (1) {
        // Populate the frames with custom data
        populate_frame_1(simvoltage, simoutputs, siminputs);
        populate_frame_2(analog_inputs, dimmable_outputs);
        populate_frame_2b(dimmable_outputs);
        populate_frame_3(r, g, b);

        send_can_frame(FRAME_1_ID, frame_1_data);
        vTaskDelay(pdMS_TO_TICKS(100));

        send_can_frame(FRAME_2_ID, frame_2_data);
        vTaskDelay(pdMS_TO_TICKS(100));

        send_can_frame(FRAME_2B_ID, frame_2b_data);
        vTaskDelay(pdMS_TO_TICKS(100));

        send_can_frame(FRAME_3_ID, frame_3_data);
        vTaskDelay(pdMS_TO_TICKS(100));

        /* Hibrit: dirty veya ~5s heartbeat — CAN SoT profil (sürekli full yok) */
        const TickType_t now_tick = xTaskGetTickCount();
        const int hb = (last_profile_hb_tick == 0) ||
                       ((TickType_t)(now_tick - last_profile_hb_tick) >= hb_ticks);
        if (hb || hexnet_panel_sync_can_should_send(false)) {
            hexnet_can_publish_io_profile();
            hexnet_panel_sync_can_sent();
            last_profile_hb_tick = now_tick;
            ESP_LOGW(TAG, "TX 0x110 seq=%u mask=0x%04X",
                     (unsigned)hexnet_panel_sync_get_seq(),
                     (unsigned)(hexnet_io_profile_get() ? hexnet_io_profile_get()->relay_enable_mask : 0));
        }

        /* Saat: WiFi+NTP → Display, dakikada bir (ilk gecerli anda hemen) */
        const int time_due = (last_time_tx_tick == 0) ||
                             ((TickType_t)(now_tick - last_time_tx_tick) >= time_ticks);
        if (time_due && hexnet_wifi_is_connected() && hexnet_wifi_ntp_unix_ts_sec() >= 1700000000LL) {
            hexnet_can_publish_time();
            last_time_tx_tick = now_tick;
        }
    }
}

// Getter functions for RGB values
uint8_t get_r_value() {
    return r;
}

uint8_t get_g_value() {
    return g;
}

uint8_t get_b_value() {
    return b;
}
uint8_t get_rgb_enable() {
    return rgb_enable;
}

uint16_t get_outputs() {
    return simoutputs;
}

void set_outputs(uint16_t outputs) {
    simoutputs = outputs;
    /* Röle maskesi ≠ motor. Display 0x720 / tablet Output röleleri yalnızca SR. */
    hexnet_shift_register_update_now();
}

uint8_t get_dimmable_output(uint8_t index) {
    if (index < 8) {
        return dimmable_outputs[index];
    }
    return 0;
}

void set_dimmable_output(uint8_t index, uint8_t value) {
    if (index < 8) {
        dimmable_outputs[index] = value;
    }
}

void set_rgb_values(uint8_t red, uint8_t green, uint8_t blue, uint8_t enable) {
    r = red;
    g = green;
    b = blue;
    rgb_enable = enable;
}

uint8_t get_analog_input(uint8_t index) {
    if (index < 4) {
        return analog_inputs[index];
    }
    return 0;
}

void set_analog_input(uint8_t index, uint8_t value) {
    if (index < 4) {
        analog_inputs[index] = value;
    }
}

void get_voltage(uint16_t voltage) {
    simvoltage = voltage;
}

uint16_t get_voltage_value(void) {
    return simvoltage;
}


void handle_rx_message(twai_message_t message) {
#if HEXNET_CAN_RX_DEBUG
    const bool relay_apply_ok = can_debug_inspect_rx(&message, "handle");
#else
    const bool relay_apply_ok = true;
#endif
    // CAN frame id 0x720: (index, val) -> relay bit
    if (message.identifier == 0x720 && message.data_length_code >= 2) {
#if HEXNET_CAN_RX_DEBUG
        if (!relay_apply_ok) {
            return;
        }
#endif
        uint8_t index = message.data[0];
        uint8_t val = message.data[1];
        if (index >= 16) {
            return;
        }
        uint16_t mask = simoutputs;
        if (val) {
            mask |= (uint16_t)(1u << index);
            ESP_LOGI(EXAMPLE_TAG, "Setting output %d to ON simoutput: %d", index, mask);
        } else {
            mask &= (uint16_t)~(1u << index);
            ESP_LOGI(EXAMPLE_TAG, "Setting output %d to OFF simoutput: %d", index, mask);
        }
        set_outputs(mask);
        hexnet_outputs_ble_mirror(mask);
        hexnet_io_notify_state_changed();
        return;
    }
    // CAN frame id 0x730: (index, val) -> update dimmable_outputs[index]
    else if (message.identifier == 0x730 && message.data_length_code >= 2) {
        uint8_t index = message.data[0];
        uint8_t val = message.data[1];
        if (index < 8) {
            set_dimmable_output(index, val);
            ESP_LOGI(EXAMPLE_TAG, "Setting dimmable %d to %d", index, val);
            hexnet_io_notify_state_changed();
        }
    }
    // CAN frame id 0x740: (r, g, b) -> update RGB values
    else if (message.identifier == 0x740 && message.data_length_code >= 3) {
        r = message.data[0];
        g = message.data[1];
        b = message.data[2];
        rgb_enable = message.data[3]; // Update RGB enable flag
        hexnet_io_notify_state_changed();
    }
    else if (message.identifier == 0x750 && message.data_length_code >= 1) {
        const int cmd = (message.data[0] <= 2) ? (int)message.data[0] : 0;
        const int idx = (message.data_length_code >= 2 && message.data[1] <= 1)
                            ? (int)message.data[1]
                            : 0;
        hexnet_motor_set_cmd(idx, cmd);
        ESP_LOGI(EXAMPLE_TAG, "Motor cmd 0x750 M%d data=%u (PWM=21 M1=11/12 M2=13/14)",
                 idx + 1, (unsigned)cmd);
        hexnet_io_notify_state_changed();
    }
    /* Display → IO: baglandi, buton ikon/isim ID'lerini sor (0x118/119 cevap) */
    else if (message.identifier == 0x764) {
        ESP_LOGW(EXAMPLE_TAG, "RX 0x764 label-sync-req from Display → TX 0x118/119");
        hexnet_panel_sync_request_can_labels();
    }
    /* Display → IO: isim/ikon SoT (0x118/119 tersi; echo karismasin diye 0x762/763) */
    else if ((message.identifier == 0x762 || message.identifier == 0x763) &&
             message.data_length_code >= 8) {
        const uint8_t kind = (message.identifier == 0x762) ? 0u : 1u;
        const uint8_t slot = message.data[0];
        const uint8_t part = message.data[1];
        ESP_LOGW(EXAMPLE_TAG, "RX 0x%03X label kind=%u slot=%u part=%u",
                 (unsigned)message.identifier, (unsigned)kind, (unsigned)slot, (unsigned)part);
        if (hexnet_io_labels_can_chunk(kind, slot, part, &message.data[2])) {
            /* Name+icon cift gelir; ikon commit'te NVS+BLE worker'a defer (CAN RX bloklama) */
            if (kind == 1u) {
                hexnet_ble_request_display_label_commit();
            }
        }
    }
    /* Display → IO: panel aktif/pasif maskeleri (0x110 ile ayni payload) */
    else if (message.identifier == 0x761 && message.data_length_code >= 7) {
        const uint16_t relay_mask =
            (uint16_t)message.data[2] | ((uint16_t)message.data[3] << 8);
        const uint8_t motor = (message.data[1] & 0x01u) ? 1u : 0u;
        const uint8_t rgb = (message.data[1] & 0x02u) ? 1u : 0u;
        ESP_LOGW(EXAMPLE_TAG, "RX 0x761 profile-set mask=0x%04X", (unsigned)relay_mask);
        if (hexnet_io_profile_apply_display_masks(relay_mask,
                                                  message.data[4],
                                                  message.data[5],
                                                  message.data[6],
                                                  motor,
                                                  rgb)) {
            hexnet_can_publish_io_profile();
            hexnet_io_notify_state_changed();
        }
    }
    /* 0x770 Display 5V: data[0]=1 ON, 0=OFF, 2=hard reset (OFF 3s ON) */
    else if (message.identifier == 0x770 && message.data_length_code >= 1) {
        uint8_t cmd = message.data[0];
        if (cmd == 2) {
            ESP_LOGW(EXAMPLE_TAG, "Display hard reset via 0x770");
            hexnet_disp_pwr_hard_reset(HEXNET_IO_DISP_HARD_RESET_MS);
        } else if (cmd == 0) {
            hexnet_disp_pwr_set(false);
            ESP_LOGW(EXAMPLE_TAG, "Display 5V OFF");
        } else {
            hexnet_disp_pwr_set(true);
            ESP_LOGW(EXAMPLE_TAG, "Display 5V ON");
        }
    }
}

// Error recovery function
static void check_and_recover_from_errors() {
    twai_status_info_t status;
    twai_get_status_info(&status);
    
    /* Bus-Off → initiate_recovery; STOPPED → twai_start. */
    if (status.state == TWAI_STATE_BUS_OFF) {
        error_recovery_count++;
        ESP_LOGW(TAG, "CAN Bus-Off — initiate_recovery #%u tx_err=%" PRIu32 " rx_err=%" PRIu32,
                 (unsigned)error_recovery_count, status.tx_error_counter, status.rx_error_counter);
        esp_err_t rec = twai_initiate_recovery();
        if (rec != ESP_OK) {
            ESP_LOGE(TAG, "initiate_recovery failed: 0x%x", (unsigned)rec);
        }
        return;
    }
    if (status.state == TWAI_STATE_STOPPED && driver_installed) {
        ESP_LOGW(TAG, "CAN STOPPED — twai_start");
        esp_err_t st = twai_start();
        if (st != ESP_OK) {
            ESP_LOGE(TAG, "twai_start failed: 0x%x", (unsigned)st);
        }
    }
}

// Watchdog monitoring function
static void monitor_can_health() {
    static uint32_t s_last_warn_ms;
    uint32_t current_time = esp_timer_get_time() / 1000;

    if (last_successful_tx > 0 && (current_time - last_successful_tx) > 10000) {
        if ((current_time - s_last_warn_ms) > 5000u) {
            s_last_warn_ms = current_time;
            ESP_LOGW(TAG, "No successful CAN transmission for %"PRIu32" seconds",
                     (current_time - last_successful_tx) / 1000);
        }
        check_and_recover_from_errors();
    }
}

// Watchdog task to monitor CAN bus health
void can_watchdog_task(void *pvParameter) {
    uint32_t last_status_check = 0;
    uint32_t consecutive_failures = 0;
    
    while(1) {
        uint32_t current_time = esp_timer_get_time() / 1000;
        
        // Check CAN status every 5 seconds
        if (current_time - last_status_check >= 5000) {
            twai_status_info_t status;
            twai_get_status_info(&status);
            
            ESP_LOGI(TAG,
                     "CAN Status - State: %d, TX Err: %" PRIu32 ", RX Err: %" PRIu32
                     ", q_tx=%" PRIu32 ", q_rx=%" PRIu32 ", Recovery: %" PRIu32,
                     status.state, status.tx_error_counter, status.rx_error_counter,
                     status.msgs_to_tx, status.msgs_to_rx, error_recovery_count);
            
            // Check for problematic states
            if (status.state == TWAI_STATE_BUS_OFF) {
                ESP_LOGE(TAG, "CAN Bus-Off state detected by watchdog!");
                check_and_recover_from_errors();
                consecutive_failures++;
            } else if (status.state == TWAI_STATE_RECOVERING) {
                ESP_LOGW(TAG, "CAN in recovery state");
            } else if (status.state == TWAI_STATE_RUNNING) {
                consecutive_failures = 0; // Reset failure counter on successful state
            }
            
            // If we have too many consecutive failures, try a full restart
            if (consecutive_failures >= 3) {
                ESP_LOGE(TAG, "Too many consecutive CAN failures (%"PRIu32"), attempting full restart", consecutive_failures);
                
                // Stop and uninstall driver
                twai_stop();
                vTaskDelay(pdMS_TO_TICKS(1000));
                twai_driver_uninstall();
                vTaskDelay(pdMS_TO_TICKS(1000));
                
                // Reinitialize
                esp_err_t ret = waveshare_twai_init();
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "CAN driver fully restarted successfully");
                    consecutive_failures = 0;
                } else {
                    ESP_LOGE(TAG, "Failed to restart CAN driver: 0x%x", ret);
                }
            }
            
            last_status_check = current_time;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
    }
}

int twaiCounter = 0;
void receive_frames_task(void *pvParameter)
{
    twai_message_t message;
    
    while(1)
    {
        // Check if alert happened with timeout
        uint32_t alerts_triggered;
        esp_err_t alert_result = twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(100));
        
        if (alert_result == ESP_OK) {
            twai_status_info_t twaistatus;
            twai_get_status_info(&twaistatus);

            // Handle alerts
            if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
                ESP_LOGW(EXAMPLE_TAG,"Alert: TWAI controller has become error passive.");
                check_and_recover_from_errors();
            }
            
            if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
                ESP_LOGW(EXAMPLE_TAG,"Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
                ESP_LOGW(EXAMPLE_TAG,"Bus error count: %"PRIu32, twaistatus.bus_error_count);
                check_and_recover_from_errors();
            }

            if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
                ESP_LOGW(EXAMPLE_TAG,"Alert: The RX queue is full causing a received frame to be lost.");
                ESP_LOGW(EXAMPLE_TAG,"RX buffered: %"PRIu32, twaistatus.msgs_to_rx);
                ESP_LOGW(EXAMPLE_TAG,"RX missed: %"PRIu32, twaistatus.rx_missed_count);
                ESP_LOGW(EXAMPLE_TAG,"RX overrun %"PRIu32, twaistatus.rx_overrun_count);
                
                int cleared_count = 0;
                while (twai_receive(&message, 0) == ESP_OK) {
                    cleared_count++;
                    handle_rx_message(message);
                }
                ESP_LOGW(EXAMPLE_TAG,"Cleared %d messages from RX queue", cleared_count);
            }

            /* Queue'daki tum frame'leri isle (tek receive 0x750 kaciriyordu) */
            if (alerts_triggered & TWAI_ALERT_RX_DATA) {
                while (twai_receive(&message, 0) == ESP_OK) {
                    handle_rx_message(message);
                }
            }
        } else if (alert_result == ESP_ERR_TIMEOUT) {
            // Timeout is normal, continue monitoring
        } else {
            ESP_LOGE(EXAMPLE_TAG, "Error reading alerts: 0x%x", alert_result);
        }

#if HEXNET_CAN_RX_DEBUG
        can_debug_print_stats_if_due();
#endif
        
        // Monitor CAN health periodically
        monitor_can_health();
        
        // Proactive queue monitoring - check queue level and clear if getting full
        twai_status_info_t status;
        twai_get_status_info(&status);
        if (status.msgs_to_rx > 3) {
            ESP_LOGW(EXAMPLE_TAG, "RX queue getting full (%"PRIu32" msgs), draining", status.msgs_to_rx);
            int cleared_count = 0;
            while (twai_receive(&message, 0) == ESP_OK) {
                cleared_count++;
                handle_rx_message(message);
            }
            if (cleared_count > 0) {
                ESP_LOGW(EXAMPLE_TAG, "Drained %d RX messages", cleared_count);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); // Reduced delay for faster message processing
    }
}


// Function to send a message
static void send_message()
{
    // Configure message to transmit
    twai_message_t message;       // Message structure
    message.identifier = 0x0F6;   // Message identifier
    message.data_length_code = 8; // Data length code
    for (int i = 0; i < 8; i++)
    {
        message.data[i] = i; // Set message data
    }

    // Queue message for transmission
    esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(1000));
    if (result == ESP_OK)
    {
        ESP_LOGD(EXAMPLE_TAG, "Message queued for transmission successfully");
        last_successful_tx = esp_timer_get_time() / 1000; // Update successful TX timestamp
    }
    else
    {
        ESP_LOGE(EXAMPLE_TAG, "Failed to queue message for transmission: 0x%x", result);
        check_and_recover_from_errors();
    }
}

// Function to initialize Waveshare TWAI
esp_err_t waveshare_twai_init()
{
    esp_err_t ret;
    
    // Install TWAI driver
    ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret == ESP_OK) {
        ESP_LOGI(EXAMPLE_TAG,"Driver installed");
    } else {
        ESP_LOGE(EXAMPLE_TAG,"Failed to install driver: 0x%x", ret);
        return ret;
    }
    
    // Start TWAI driver
    ret = twai_start();
    if (ret == ESP_OK) {
        ESP_LOGI(EXAMPLE_TAG,"Driver started");
    } else {
        ESP_LOGE(EXAMPLE_TAG,"Failed to start driver: 0x%x", ret);
        twai_driver_uninstall();
        return ret;
    }

    // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | 
                               TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_FAILED | TWAI_ALERT_TX_SUCCESS;
    ret = twai_reconfigure_alerts(alerts_to_enable, NULL);
    if (ret == ESP_OK) {
        ESP_LOGI(EXAMPLE_TAG,"CAN Alerts reconfigured");
    } else {
        ESP_LOGE(EXAMPLE_TAG,"Failed to reconfigure alerts: 0x%x", ret);
        twai_stop();
        twai_driver_uninstall();
        return ret;
    }

    // Initialize monitoring variables
    last_successful_tx = esp_timer_get_time() / 1000;
    error_recovery_count = 0;
    
    // TWAI driver is now successfully installed and started
    driver_installed = true;
    ESP_LOGI(EXAMPLE_TAG,"CAN bus initialization completed successfully");
    return ESP_OK;           // Return success status
}

// Function to transmit messages
esp_err_t waveshare_twai_transmit()
{
    if (!driver_installed)
    {
        ESP_LOGE(EXAMPLE_TAG, "Driver not installed, cannot transmit");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait before retrying
        return ESP_FAIL;                 // Return failure status
    }
    
    // Check if alert happened with timeout
    uint32_t alerts_triggered;
    esp_err_t alert_result = twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(100));
    
    if (alert_result == ESP_OK) {
        twai_status_info_t twaistatus;
        twai_get_status_info(&twaistatus);

        // Handle alerts
        if (alerts_triggered & TWAI_ALERT_ERR_PASS)
        {
            ESP_LOGW(EXAMPLE_TAG, "Alert: TWAI controller has become error passive.");
            check_and_recover_from_errors();
        }
        if (alerts_triggered & TWAI_ALERT_BUS_ERROR)
        {
            ESP_LOGW(EXAMPLE_TAG, "Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
            ESP_LOGW(EXAMPLE_TAG, "Bus error count: %" PRIu32, twaistatus.bus_error_count);
            check_and_recover_from_errors();
        }
        if (alerts_triggered & TWAI_ALERT_TX_FAILED)
        {
            ESP_LOGW(EXAMPLE_TAG, "Alert: The Transmission failed.");
            ESP_LOGW(EXAMPLE_TAG, "TX buffered: %" PRIu32, twaistatus.msgs_to_tx);
            ESP_LOGW(EXAMPLE_TAG, "TX error: %" PRIu32, twaistatus.tx_error_counter);
            ESP_LOGW(EXAMPLE_TAG, "TX failed: %" PRIu32, twaistatus.tx_failed_count);
            check_and_recover_from_errors();
        }
        if (alerts_triggered & TWAI_ALERT_TX_SUCCESS)
        {
            ESP_LOGD(EXAMPLE_TAG, "Alert: The Transmission was successful.");
            ESP_LOGD(EXAMPLE_TAG, "TX buffered: %" PRIu32, twaistatus.msgs_to_tx);
            last_successful_tx = esp_timer_get_time() / 1000; // Update successful TX timestamp
        }
    } else if (alert_result == ESP_ERR_TIMEOUT) {
        // Timeout is normal, continue
    } else {
        ESP_LOGE(EXAMPLE_TAG, "Error reading alerts: 0x%x", alert_result);
        return alert_result;
    }

    // Send message
    unsigned long currentMillis = esp_timer_get_time() / 1000; // Get current time in milliseconds
    if (currentMillis - previousMillis >= TRANSMIT_RATE_MS)
    {                                   // Check if it's time to send the message
        previousMillis = currentMillis; // Update last send time
        send_message();                 // Call send message function
    }
    return ESP_OK; // Return success status
}
