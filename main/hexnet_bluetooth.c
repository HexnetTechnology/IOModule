/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "hexnet_log.h"
#define LOG_LOCAL_LEVEL HEXNET_LOG_LEVEL_BLE
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "hexnet_bluetooth.h"
#include "cJSON.h"
#include "hexnet_canbus.h"
#include "hexnet_resolve_mqtt.h"
#include "hexnet_io_profile.h"
#include "hexnet_io_labels.h"
#include "hexnet_panel_sync.h"
#include "hexnet_mqtt.h"
#include "hexnet_io_notify.h"
#include "hexnet_motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "nvs.h"


static const uint8_t spp_adv_data[] = {
    /* Flags */
    0x02,0x01,0x06,
    /* Complete List of 16-bit Service Class UUIDs */
    0x03,0x03,0xF0,0xAB,
    /* Complete Local Name in advertising */
    0x10,0x09, 'V','A','N','G','O',' ','I','O',' ','M','O','D','U','L','E'
};


char* converted_json_data2 = NULL;
// Example data to save
int numOfOutputs = 12;
int numOfDims = 8;
int numOfSensors = 5;
float batarya_volt = 0;
bool slaveConnectionStatus = true;
int panelThemeType = 0;
int panelWallpaperEnable = false;
int panelWallpaperTime = 0;
int numberOfNotifications = 0;
cJSON* notifications = NULL;
// Declare the global outputsBuffer
int outputsBuffer[16] = {0};
int sensorsBuffer[5] = {0};
int dimsBuffer[8] = {0};
int rgbBuffer[4] = {0};
int btn_index = 0;
uint8_t rgbEna = 0; // RGB LED enable variable
uint8_t fromBleConfig = 0;
int panelWallpaperEnableCounter = 1;
uint16_t outputsFromBle = 0;
int dimsDataBuffer[8] = {0};



// Define the characteristic UUID
#define CHAR_UUID_REGS_DATA 0x2A56

uint8_t spp_cmd_buff[1000];

/** Tablet Configuration+channels ~3KB; tek ATT write MTU ile sınırlı — chunk biriktir. */
#define BLE_RX_ACCUM_MAX 4096
static char s_ble_rx_accum[BLE_RX_ACCUM_MAX];
static size_t s_ble_rx_accum_len = 0;
static int64_t s_ble_rx_accum_us = 0;

static uint16_t spp_mtu_size = 500;  // Initialize to requested MTU, will be updated after negotiation
static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;
QueueHandle_t spp_uart_queue = NULL;
static QueueHandle_t cmd_cmd_queue = NULL;
static QueueHandle_t data_transmit_queue = NULL;

#ifdef SUPPORT_HEARTBEAT
static QueueHandle_t cmd_heartbeat_queue = NULL;
static uint8_t  heartbeat_s[9] = {'E','s','p','r','e','s','s','i','f'};
static bool enable_heart_ntf = false;
static uint8_t heartbeat_count_num = 0;
#endif

static bool enable_data_ntf = false;
static bool is_connected = false;
/** Sonraki BLE telemetride io.channels (SoT) gonder — CCC acilisinda / panel degisince. */
static bool s_ble_want_channels = false;
/** Display CAN label (ikon) tamamlandi — worker'da save + on_changed */
static volatile bool s_ble_display_label_commit = false;
static esp_bd_addr_t spp_remote_bda = {0x0,};

// MTU negotiation state tracking
static bool mtu_negotiation_pending = false;
static TimerHandle_t s_notify_timer = NULL;
static TimerHandle_t s_ble_resync_timer = NULL;
static volatile bool s_ble_resync_pending = false;
static volatile bool s_ble_push_request = false;
static volatile bool s_gatts_congested = false;
static void ble_resync_timer_cb(TimerHandle_t xTimer);
static void ble_request_push_from_timer(void);
static void ble_push_worker_task(void *arg);
static bool ble_parse_enqueue(const char *json);
static QueueHandle_t s_ble_parse_queue = NULL;
/* PSRAM kapaliyken cJSON/malloc basarisiz → lean JSON + ping-pong notify.
 * Tablet VanGoData: outNB/dimNB/senNB zorunlu (default yok) — lean'de de gonder. */
static char s_ble_json_live[1024];
static char s_ble_notify_a[1024];
static char s_ble_notify_b[1024];
static char *s_ble_notify_last = NULL;
/* Ikon/isim SoT: lean + io.channels (cJSON FULL OOM oluyor) */
/* lean(~350)+12 röle+8 dim isim/ikon UTF-8 — 3200 asiyordu, tablet ikon SoT gelmiyordu */
/* 8KB×2 iheap eritiyordu; channels paketi ~2KB — 4KB×2 yeterli */
#define BLE_CH_JSON_MAX 4096
static char s_ble_ch_a[BLE_CH_JSON_MAX];
static char s_ble_ch_b[BLE_CH_JSON_MAX];
static char *s_ble_ch_last = NULL;
static char *build_ble_lean_json(void);
static char *build_ble_channels_json(void);
static bool ble_queue_lean_notify(const char *json);
static bool ble_queue_channels_notify(const char *json);
static bool lean_append_int_array(char **pp, size_t *prem, const char *key, const int *vals, int count);
static bool lean_append_json_str(char **pp, size_t *prem, const char *s);

static uint16_t spp_handle_table[SPP_IDX_NB];

static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

typedef struct spp_receive_data_node{
    int32_t len;
    uint8_t * node_buff;
    struct spp_receive_data_node * next_node;
}spp_receive_data_node_t;

static spp_receive_data_node_t * temp_spp_recv_data_node_p1 = NULL;
static spp_receive_data_node_t * temp_spp_recv_data_node_p2 = NULL;

typedef struct spp_receive_data_buff{
    int32_t node_num;
    int32_t buff_size;
    spp_receive_data_node_t * first_node;
}spp_receive_data_buff_t;

static spp_receive_data_buff_t SppRecvDataBuff = {
    .node_num   = 0,
    .buff_size  = 0,
    .first_node = NULL
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static esp_err_t nvs_write_int(const char* key, int value);
static void parse_and_store_mqtt_route_ids(cJSON *json);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/*
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
/* WRITE + WRITE_NR: tablet WRITE_TYPE_DEFAULT (Write Request) + legacy Write Cmd. */
static const uint8_t char_prop_read_write =
    ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_READ;

#ifdef SUPPORT_HEARTBEAT
static const uint8_t char_prop_read_write_notify =
    ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR |
    ESP_GATT_CHAR_PROP_BIT_NOTIFY;
#endif

///SPP Service - data receive characteristic, read&write (with/without response)
static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t  spp_data_receive_val[20] = {0x00};

///SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t  spp_data_notify_val[20] = {0x00};
static const uint8_t  spp_data_notify_ccc[2] = {0x00, 0x00};

///SPP Service - command characteristic, read&write without response
static const uint16_t spp_command_uuid = ESP_GATT_UUID_SPP_COMMAND_RECEIVE;
static const uint8_t  spp_command_val[10] = {0x00};

///SPP Service - status characteristic, notify&read
static const uint16_t spp_status_uuid = ESP_GATT_UUID_SPP_COMMAND_NOTIFY;
static const uint8_t  spp_status_val[10] = {0x00};
static const uint8_t  spp_status_ccc[2] = {0x00, 0x00};

#ifdef SUPPORT_HEARTBEAT
///SPP Server - Heart beat characteristic, notify&write&read
static const uint16_t spp_heart_beat_uuid = ESP_GATT_UUID_SPP_HEARTBEAT;
static const uint8_t  spp_heart_beat_val[2] = {0x00, 0x00};
static const uint8_t  spp_heart_beat_ccc[2] = {0x00, 0x00};
#endif

///Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] =
{
    //SPP -  Service Declaration
    [SPP_IDX_SVC]                      	=
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
    sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

    //SPP -  data receive characteristic Declaration
    [SPP_IDX_SPP_DATA_RECV_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  data receive characteristic Value
    [SPP_IDX_SPP_DATA_RECV_VAL]             	=
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_DATA_MAX_LEN,sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val}},

    //SPP -  data notify characteristic Declaration
    [SPP_IDX_SPP_DATA_NOTIFY_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  data notify characteristic Value
    [SPP_IDX_SPP_DATA_NTY_VAL]   =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_notify_uuid, ESP_GATT_PERM_READ,
    SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *)spp_data_notify_val}},

    //SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_DATA_NTF_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},

    //SPP -  command characteristic Declaration
    [SPP_IDX_SPP_COMMAND_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  command characteristic Value
    [SPP_IDX_SPP_COMMAND_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_command_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_CMD_MAX_LEN,sizeof(spp_command_val), (uint8_t *)spp_command_val}},

    //SPP -  status characteristic Declaration
    [SPP_IDX_SPP_STATUS_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  status characteristic Value
    [SPP_IDX_SPP_STATUS_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_status_uuid, ESP_GATT_PERM_READ,
    SPP_STATUS_MAX_LEN,sizeof(spp_status_val), (uint8_t *)spp_status_val}},

    //SPP -  status characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_STATUS_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_status_ccc), (uint8_t *)spp_status_ccc}},

#ifdef SUPPORT_HEARTBEAT
    //SPP -  Heart beat characteristic Declaration
    [SPP_IDX_SPP_HEARTBEAT_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    //SPP -  Heart beat characteristic Value
    [SPP_IDX_SPP_HEARTBEAT_VAL]   =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_heart_beat_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(spp_heart_beat_val), sizeof(spp_heart_beat_val), (uint8_t *)spp_heart_beat_val}},

    //SPP -  Heart beat characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_HEARTBEAT_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_heart_beat_ccc}},
#endif
};

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;

    for(int i = 0; i < SPP_IDX_NB ; i++){
        if(handle == spp_handle_table[i]){
            return i;
        }
    }

    return error;
}

static bool store_wr_buffer(esp_ble_gatts_cb_param_t *p_data)
{
    temp_spp_recv_data_node_p1 = (spp_receive_data_node_t *)malloc(sizeof(spp_receive_data_node_t));

    if(temp_spp_recv_data_node_p1 == NULL){
        ESP_LOGI(GATTS_TABLE_TAG, "malloc error %s %d", __func__, __LINE__);
        return false;
    }
    if(temp_spp_recv_data_node_p2 != NULL){
        temp_spp_recv_data_node_p2->next_node = temp_spp_recv_data_node_p1;
    }
    temp_spp_recv_data_node_p1->len = p_data->write.len;
    SppRecvDataBuff.buff_size += p_data->write.len;
    temp_spp_recv_data_node_p1->next_node = NULL;
    temp_spp_recv_data_node_p1->node_buff = (uint8_t *)malloc(p_data->write.len);
    temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1;
    if (temp_spp_recv_data_node_p1->node_buff == NULL) {
        ESP_LOGI(GATTS_TABLE_TAG, "malloc error %s %d\n", __func__, __LINE__);
        temp_spp_recv_data_node_p1->len = 0;
    } else {
        memcpy(temp_spp_recv_data_node_p1->node_buff,p_data->write.value,p_data->write.len);
    }

    if(SppRecvDataBuff.node_num == 0){
        SppRecvDataBuff.first_node = temp_spp_recv_data_node_p1;
        SppRecvDataBuff.node_num++;
    }else{
        SppRecvDataBuff.node_num++;
    }

    return true;
}

static void ble_rx_accum_reset(void)
{
    s_ble_rx_accum_len = 0;
    s_ble_rx_accum[0] = '\0';
    s_ble_rx_accum_us = 0;
}

/** Brace-balanced JSON sonu (string icindeki { } yok sayilir — basit ATT chunk birlestirme). */
static int ble_rx_json_end_offset(const char *buf, size_t len)
{
    int depth = 0;
    bool in_str = false;
    bool esc = false;
    for (size_t i = 0; i < len; ++i) {
        const char c = buf[i];
        if (in_str) {
            if (esc) {
                esc = false;
            } else if (c == '\\') {
                esc = true;
            } else if (c == '"') {
                in_str = false;
            }
            continue;
        }
        if (c == '"') {
            in_str = true;
        } else if (c == '{') {
            depth++;
        } else if (c == '}') {
            if (depth > 0) {
                depth--;
            }
            if (depth == 0) {
                return (int)(i + 1);
            }
        }
    }
    return -1;
}

static void ble_rx_try_parse_accum(void)
{
    for (;;) {
        size_t start = 0;
        while (start < s_ble_rx_accum_len &&
               (s_ble_rx_accum[start] == ' ' || s_ble_rx_accum[start] == '\n' ||
                s_ble_rx_accum[start] == '\r' || s_ble_rx_accum[start] == '\t')) {
            start++;
        }
        if (start >= s_ble_rx_accum_len) {
            ble_rx_accum_reset();
            return;
        }
        if (s_ble_rx_accum[start] != '{') {
            ESP_LOGI(GATTS_TABLE_TAG, "Single write data received (%d bytes): %s",
                     (int)s_ble_rx_accum_len, s_ble_rx_accum);
            (void)ble_parse_enqueue(s_ble_rx_accum);
            ble_rx_accum_reset();
            return;
        }
        if (start > 0) {
            memmove(s_ble_rx_accum, s_ble_rx_accum + start, s_ble_rx_accum_len - start);
            s_ble_rx_accum_len -= start;
            s_ble_rx_accum[s_ble_rx_accum_len] = '\0';
        }

        const int end = ble_rx_json_end_offset(s_ble_rx_accum, s_ble_rx_accum_len);
        if (end < 0) {
            ESP_LOGI(GATTS_TABLE_TAG, "BLE RX accum %u B (waiting more)", (unsigned)s_ble_rx_accum_len);
            return;
        }

        const char saved = s_ble_rx_accum[end];
        s_ble_rx_accum[end] = '\0';
        ESP_LOGI(GATTS_TABLE_TAG, "BLE RX complete JSON (%d bytes)", end);
        /* ASLA BTC_TASK'ta cJSON/CAN — stack overflow (PanelChannels) */
        (void)ble_parse_enqueue(s_ble_rx_accum);
        s_ble_rx_accum[end] = saved;

        const size_t remain = s_ble_rx_accum_len - (size_t)end;
        if (remain == 0) {
            ble_rx_accum_reset();
            return;
        }
        memmove(s_ble_rx_accum, s_ble_rx_accum + end, remain);
        s_ble_rx_accum_len = remain;
        s_ble_rx_accum[s_ble_rx_accum_len] = '\0';
    }
}

static void ble_rx_feed_chunk(const uint8_t *data, uint16_t len)
{
    if (!data || len == 0) {
        return;
    }

    const int64_t now = esp_timer_get_time();
    if (s_ble_rx_accum_len > 0 && (now - s_ble_rx_accum_us) > 3000000) {
        ESP_LOGW(GATTS_TABLE_TAG, "BLE RX accum timeout — reset (%u B)", (unsigned)s_ble_rx_accum_len);
        ble_rx_accum_reset();
    }

    if (s_ble_rx_accum_len + len >= BLE_RX_ACCUM_MAX) {
        ESP_LOGE(GATTS_TABLE_TAG, "BLE RX accum overflow (%u+%u) — reset",
                 (unsigned)s_ble_rx_accum_len, (unsigned)len);
        ble_rx_accum_reset();
        if (len >= BLE_RX_ACCUM_MAX) {
            return;
        }
    }

    memcpy(s_ble_rx_accum + s_ble_rx_accum_len, data, len);
    s_ble_rx_accum_len += len;
    s_ble_rx_accum[s_ble_rx_accum_len] = '\0';
    s_ble_rx_accum_us = now;
    ble_rx_try_parse_accum();
}

static void process_complete_data(void)
{
    if (SppRecvDataBuff.first_node == NULL || SppRecvDataBuff.buff_size == 0) {
        return;
    }

    // Allocate buffer for complete data
    char *complete_data = malloc(SppRecvDataBuff.buff_size + 1);
    if (complete_data == NULL) {
        ESP_LOGE(GATTS_TABLE_TAG, "Failed to allocate memory for complete data");
        free_write_buffer();
        return;
    }

    // Reassemble data from all chunks
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;
    int offset = 0;
    
    while(temp_spp_recv_data_node_p1 != NULL){
        if (temp_spp_recv_data_node_p1->node_buff && temp_spp_recv_data_node_p1->len > 0) {
            memcpy(complete_data + offset, temp_spp_recv_data_node_p1->node_buff, temp_spp_recv_data_node_p1->len);
            offset += temp_spp_recv_data_node_p1->len;
        }
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p1->next_node;
    }
    
    complete_data[offset] = '\0'; // Null terminate
    
    ESP_LOGI(GATTS_TABLE_TAG, "Complete data received (%d bytes): %s", offset, complete_data);
    (void)ble_parse_enqueue(complete_data);
    free(complete_data);
    free_write_buffer();
}

static void free_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while(temp_spp_recv_data_node_p1 != NULL){
        temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1->next_node;
        if (temp_spp_recv_data_node_p1->node_buff) {
            free(temp_spp_recv_data_node_p1->node_buff);
        }
        free(temp_spp_recv_data_node_p1);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p2;
    }

    SppRecvDataBuff.node_num = 0;
    SppRecvDataBuff.buff_size = 0;
    SppRecvDataBuff.first_node = NULL;
}

//BURASSSSSSSSSSSSSSSSSIIIIIIIIIIIIIIIIIII  COOOOOOOOKKKK    ONEEEEEEEEEEMMMMLLLIIII
#ifdef SUPPORT_HEARTBEAT
void spp_heartbeat_task(void * arg)
{
    uint16_t cmd_id;

    for(;;) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if(xQueueReceive(cmd_heartbeat_queue, &cmd_id, portMAX_DELAY)) {
            while(1){
                heartbeat_count_num++;
                vTaskDelay(5000/ portTICK_PERIOD_MS);
                if((heartbeat_count_num >3)&&(is_connected)){
                    esp_ble_gap_disconnect(spp_remote_bda);
                }
                if(is_connected && enable_heart_ntf){
                    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_HEARTBEAT_VAL],sizeof(heartbeat_s), heartbeat_s, false);
                }else if(!is_connected){
                    break;
                }
            }
        }
    }
    vTaskDelete(NULL);
}
#endif


const char example_json_data_packet[] = "{\"temperature\": 25.0, \"humidity\": 50.0, \"pressure\": 1013.25}";

static bool ble_notify_send_chunk(const uint8_t *data, int len)
{
    if (!is_connected || !enable_data_ntf || data == NULL || len <= 0) {
        return false;
    }
    if (s_gatts_congested) {
        return false;
    }
    esp_err_t err = esp_ble_gatts_send_indicate(
        spp_gatts_if,
        spp_conn_id,
        spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],
        (uint16_t)len,
        (uint8_t *)data,
        false);
    if (err != ESP_OK) {
        ESP_LOGE(GATTS_TABLE_TAG, "notify send failed len=%d err=%s", len, esp_err_to_name(err));
        return false;
    }
    /* Notifications (need_confirm=false) do not raise ESP_GATTS_CONF_EVT on Bluedroid. */
    vTaskDelay(pdMS_TO_TICKS(15));
    return true;
}

// Data transmission task to handle chunking
void data_transmit_task(void *arg)
{
    char *data_to_send = NULL;
    
    for(;;) {
        if(xQueueReceive(data_transmit_queue, &data_to_send, portMAX_DELAY)) {
            if (data_to_send != NULL && is_connected && enable_data_ntf) {
                int data_len = strlen(data_to_send);
                int wait_mtu = 0;
                while (mtu_negotiation_pending && wait_mtu < 50) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                    wait_mtu++;
                }
                uint16_t effective_mtu = mtu_negotiation_pending ? 23 : spp_mtu_size;
                int max_payload = effective_mtu - 3;
                if (max_payload < 1) {
                    max_payload = 20;
                }

                if (data_len <= max_payload) {
                    ble_notify_send_chunk((const uint8_t *)data_to_send, data_len);
                } else {
                    int offset = 0;
                    while (offset < data_len && is_connected && enable_data_ntf && !s_gatts_congested) {
                        int chunk_len = data_len - offset;
                        if (chunk_len > max_payload) {
                            chunk_len = max_payload;
                        }
                        if (!ble_notify_send_chunk((const uint8_t *)(data_to_send + offset), chunk_len)) {
                            break;
                        }
                        offset += chunk_len;
                    }
                }
            }
            /* Ping-pong static buffer: free etme. Heap kopyalari free et. */
            if (data_to_send != NULL &&
                data_to_send != s_ble_notify_a &&
                data_to_send != s_ble_notify_b &&
                data_to_send != s_ble_ch_a &&
                data_to_send != s_ble_ch_b) {
                free(data_to_send);
            }
        }
    }
    vTaskDelete(NULL);
}

static bool ble_queue_lean_notify(const char *json)
{
    if (!json || data_transmit_queue == NULL) {
        return false;
    }
    const size_t n = strlen(json) + 1;
    if (n > sizeof(s_ble_notify_a)) {
        ESP_LOGW(GATTS_TABLE_TAG, "panel push lean too large len=%u", (unsigned)(n - 1));
        return false;
    }
    if (uxQueueMessagesWaiting(data_transmit_queue) > 0) {
        char *stale = NULL;
        if (xQueueReceive(data_transmit_queue, &stale, 0) == pdTRUE && stale &&
            stale != s_ble_notify_a && stale != s_ble_notify_b &&
            stale != s_ble_ch_a && stale != s_ble_ch_b) {
            free(stale);
        }
    }
    char *dst = (s_ble_notify_last == s_ble_notify_a) ? s_ble_notify_b : s_ble_notify_a;
    memcpy(dst, json, n);
    s_ble_notify_last = dst;
    if (xQueueSend(data_transmit_queue, &dst, 0) != pdTRUE) {
        ESP_LOGW(GATTS_TABLE_TAG, "panel push queue full");
        return false;
    }
    ESP_LOGI(GATTS_TABLE_TAG, "panel push queued seq=%u len=%u lean=1",
             (unsigned)hexnet_io_profile_get_panel_seq(), (unsigned)(n - 1));
    return true;
}

void bluetooth_send_periodic_callback(TimerHandle_t xTimer)
{
    (void)xTimer;
    /* Tmr Svc stack ~2KB — agir is yok, sadece bayrak */
    if (!is_connected || !enable_data_ntf) {
        return;
    }
#if HEXNET_MQTT_ENABLE
    if (!hexnet_mqtt_is_connected()) {
        return;
    }
#endif
    ble_request_push_from_timer();
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    ESP_LOGI(GATTS_TABLE_TAG, "GAP_EVT, event %d", event);

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "Advertising started successfully (%s)", SAMPLE_DEVICE_NAME);
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        /* Wi-Fi coex / stack may stop ADV — keep discoverable when idle. */
        if (!is_connected) {
            ESP_LOGW(GATTS_TABLE_TAG, "Advertising stopped while idle — restarting");
            esp_ble_gap_start_advertising(&spp_adv_params);
        }
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *) param;
    uint8_t res = 0xff;

    //ESP_LOGI(GATTS_TABLE_TAG, "event = %x",event);
    switch (event) {
    	case ESP_GATTS_REG_EVT:
    	    ESP_LOGI(GATTS_TABLE_TAG, "%s %d", __func__, __LINE__);
        	esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);

        	ESP_LOGI(GATTS_TABLE_TAG, "%s %d", __func__, __LINE__);
        	esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, sizeof(spp_adv_data));

        	ESP_LOGI(GATTS_TABLE_TAG, "%s %d", __func__, __LINE__);
        	esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
       	break;
    	case ESP_GATTS_READ_EVT:
            res = find_char_and_desr_index(p_data->read.handle);
            if(res == SPP_IDX_SPP_STATUS_VAL){
                //TODO:client read the status characteristic
            }
       	 break;
    	case ESP_GATTS_WRITE_EVT: {
    	    res = find_char_and_desr_index(p_data->write.handle);
            if(p_data->write.is_prep == false){
                //ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT : handle = %d", res);
                if(res == SPP_IDX_SPP_COMMAND_VAL){
                    // Use the buffer directly instead of malloc
                    memset(spp_cmd_buff, 0x0, sizeof(spp_cmd_buff));
                    memcpy(spp_cmd_buff, p_data->write.value, p_data->write.len);
                    ESP_LOGI(GATTS_TABLE_TAG, "Received Buffer Data: %s", (char *)spp_cmd_buff);
                    (void)ble_parse_enqueue((const char *)spp_cmd_buff);


                }else if(res == SPP_IDX_SPP_DATA_NTF_CFG){
                    if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)){
                        enable_data_ntf = true;
                        s_ble_want_channels = true; /* ilk paket: panel isimleri */
                        ESP_LOGI(GATTS_TABLE_TAG, "Data notifications enabled by client");
                        /* CCC aninda aninda push BT stack malloc baskisi yapabiliyor → 400ms sonra */
                        if (s_ble_resync_timer && !s_ble_resync_pending) {
                            s_ble_resync_pending = true;
                            xTimerStart(s_ble_resync_timer, 0);
                        } else {
                            hexnet_ble_push_telemetry_now();
                        }
                    }else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)){
                        enable_data_ntf = false;
                        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT : handle = ");
                    }
                }
#ifdef SUPPORT_HEARTBEAT
                else if(res == SPP_IDX_SPP_HEARTBEAT_CFG){
                    if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)){
                        enable_heart_ntf = true;
                    }else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)){
                        enable_heart_ntf = false;
                    }
                }else if(res == SPP_IDX_SPP_HEARTBEAT_VAL){
                    if((p_data->write.len == sizeof(heartbeat_s))&&(memcmp(heartbeat_s,p_data->write.value,sizeof(heartbeat_s)) == 0)){
                        heartbeat_count_num = 0;
                    }
                }
#endif
                else if(res == SPP_IDX_SPP_DATA_RECV_VAL){
                    /* ATT chunk biriktir — Configuration+channels MTU'dan buyuk olabilir */
                    ble_rx_feed_chunk(p_data->write.value, p_data->write.len);
                }else{
                    //TODO:
                }
            }else if((p_data->write.is_prep == true)&&(res == SPP_IDX_SPP_DATA_RECV_VAL)){
                
                ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_PREP_WRITE_EVT : handle = %d", res);
                store_wr_buffer(p_data);
            }
      	 	break;
    	}
    	case ESP_GATTS_EXEC_WRITE_EVT:{
    	    ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
    	    if(p_data->exec_write.exec_write_flag){
    	        // Process the complete reassembled data
    	        process_complete_data();
    	    }
    	    break;
    	}
    	case ESP_GATTS_MTU_EVT:
    	    spp_mtu_size = p_data->mtu.mtu;
    	    mtu_negotiation_pending = false; // MTU negotiation completed
    	    ESP_LOGI(GATTS_TABLE_TAG, "MTU size updated to: %d", spp_mtu_size);
    	    break;
    	case ESP_GATTS_CONF_EVT:
    	    break;
    	case ESP_GATTS_UNREG_EVT:
        	break;
    	case ESP_GATTS_DELETE_EVT:
        	break;
    	case ESP_GATTS_START_EVT:
        	break;
    	case ESP_GATTS_STOP_EVT:
        	break;
    	case ESP_GATTS_CONNECT_EVT:
    	    spp_conn_id = p_data->connect.conn_id;
    	    spp_gatts_if = gatts_if;
    	    is_connected = true;

            {
                char wifi_mac[24] = {0};
                if (hexnet_format_wifi_mac(wifi_mac, sizeof(wifi_mac))) {
                    ESP_LOGI(
                        GATTS_TABLE_TAG,
                        "BLE baglandi; platform MAC (Wi-Fi STA)=%s",
                        wifi_mac);
                }
            }

            ESP_LOGI(GATTS_TABLE_TAG, "Client connected, current MTU: %d", spp_mtu_size);

            // Initialize MTU negotiation state
            mtu_negotiation_pending = true;

            // Note: MTU exchange is initiated by the client, server responds to it
            // The local MTU is already set to 500 in ble_init()
            ESP_LOGI(GATTS_TABLE_TAG, "Waiting for client MTU exchange request...");

            hexnet_outputs_ble_mirror(get_outputs());

            if (s_notify_timer != NULL) {
                xTimerStop(s_notify_timer, 0);
                xTimerStart(s_notify_timer, 0);
            }

    	    memcpy(&spp_remote_bda,&p_data->connect.remote_bda,sizeof(esp_bd_addr_t));
#ifdef SUPPORT_HEARTBEAT
    	    uint16_t cmd = 0;
            xQueueSend(cmd_heartbeat_queue,&cmd,10/portTICK_PERIOD_MS);
#endif
        	break;
    	case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "Client disconnected, reason: %d", p_data->disconnect.reason);
            if (s_notify_timer != NULL) {
                xTimerStop(s_notify_timer, 0);
            }
            s_gatts_congested = false;
            spp_mtu_size = 500;  // Reset to requested MTU for next connection
            mtu_negotiation_pending = false; // Reset MTU negotiation state
    	    is_connected = false;
    	    enable_data_ntf = false;
    	    s_ble_want_channels = false;
#ifdef SUPPORT_HEARTBEAT
    	    enable_heart_ntf = false;
    	    heartbeat_count_num = 0;
#endif
    	    esp_ble_gap_start_advertising(&spp_adv_params);
    	    break;
    	case ESP_GATTS_OPEN_EVT:
    	    break;
    	case ESP_GATTS_CANCEL_OPEN_EVT:
    	    break;
    	case ESP_GATTS_CLOSE_EVT:
    	    break;
    	case ESP_GATTS_LISTEN_EVT:
    	    break;
    	case ESP_GATTS_CONGEST_EVT:
    	    s_gatts_congested = param->congest.congested;
    	    break;
    	case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
    	    ESP_LOGI(GATTS_TABLE_TAG, "The number handle =%x",param->add_attr_tab.num_handle);
    	    if (param->add_attr_tab.status != ESP_GATT_OK){
    	        ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
    	    }
    	    else if (param->add_attr_tab.num_handle != SPP_IDX_NB){
    	        ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, SPP_IDX_NB);
    	    }
    	    else {
    	        memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
    	        esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
    	    }
    	    break;
    	}
    	default:
    	    break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if; 
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d",param->reg.app_id, param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < SPP_PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == spp_profile_tab[idx].gatts_if) {
                if (spp_profile_tab[idx].gatts_cb) {
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

uint8_t get_rgb_val(uint8_t index) {
    if (index < 4) {
        return rgbBuffer[index];
    }
    return 0;  // Hatalı index
}

void hexnet_ble_mirror_rgb_from_hw(void)
{
    rgbBuffer[0] = get_r_value();
    rgbBuffer[1] = get_g_value();
    rgbBuffer[2] = get_b_value();
    rgbBuffer[3] = get_rgb_enable();
    rgbEna = (uint8_t)rgbBuffer[3];
}

uint8_t get_dim_value(int index) {
    if (index < 0 || index >= 8) {
        return 0;
    }
    return get_dimmable_output((uint8_t)index);
}

uint16_t get_outputsFromBle() {
    return outputsFromBle;
}

void hexnet_outputs_ble_mirror(uint16_t mask)
{
    outputsFromBle = mask;
}

static bool json_try_get_positive_int(cJSON *json, const char *key, int *out)
{
    if (!json || !key || !out) {
        return false;
    }
    cJSON *item = cJSON_GetObjectItem(json, key);
    if (item && cJSON_IsNumber(item) && item->valueint > 0) {
        *out = item->valueint;
        return true;
    }
    return false;
}

static void parse_and_store_mqtt_route_ids(cJSON *json)
{
    if (!json) {
        return;
    }

    int cid = 0;
    int did = 0;

    (void)json_try_get_positive_int(json, "mqttCid", &cid);
    (void)json_try_get_positive_int(json, "company_id", &cid);
    (void)json_try_get_positive_int(json, "companyId", &cid);
    (void)json_try_get_positive_int(json, "cid", &cid);

    (void)json_try_get_positive_int(json, "mqttDid", &did);
    (void)json_try_get_positive_int(json, "mqtt_route_device_id", &did);
    (void)json_try_get_positive_int(json, "device_id", &did);
    (void)json_try_get_positive_int(json, "deviceId", &did);
    (void)json_try_get_positive_int(json, "did", &did);

    cJSON *mqttObj = cJSON_GetObjectItem(json, "mqtt");
    if (mqttObj && cJSON_IsObject(mqttObj)) {
        (void)json_try_get_positive_int(mqttObj, "cid", &cid);
        (void)json_try_get_positive_int(mqttObj, "company_id", &cid);
        (void)json_try_get_positive_int(mqttObj, "companyId", &cid);
        (void)json_try_get_positive_int(mqttObj, "did", &did);
        (void)json_try_get_positive_int(mqttObj, "device_id", &did);
        (void)json_try_get_positive_int(mqttObj, "deviceId", &did);
        (void)json_try_get_positive_int(mqttObj, "mqtt_route_device_id", &did);
    }

    if (cid > 0) {
        (void)nvs_write_int("mqttCid", cid);
    }
    if (did > 0) {
        (void)nvs_write_int("mqttDid", did);
    }
    if (cid > 0 || did > 0) {
        ESP_LOGI("MQTT_ROUTE", "Stored MQTT route ids from BLE: cid=%d did=%d", cid, did);
    }
}

// Function to parse BLE data and call the appropriate parsing function
void parse_ble_data(const char* json_data) {
    cJSON* json = cJSON_Parse(json_data);

    if (json == NULL) {
        //ESP_LOGE(GATTS_TABLE_TAG, "Invalid JSON data");
        return;
    }

    parse_and_store_mqtt_route_ids(json);

    cJSON* messageType = cJSON_GetObjectItem(json, "MessageType");
    if (messageType == NULL || !cJSON_IsString(messageType)) {
        ESP_LOGE(GATTS_TABLE_TAG, "MessageType not found or invalid");
        cJSON_Delete(json);
        return;
    }

    if (strcmp(messageType->valuestring, "Read") == 0) {
        parse_read_data(json);
    } else if (strcmp(messageType->valuestring, "Write") == 0) {
        parse_write_data(json);
    } else if (strcmp(messageType->valuestring, "Configuration") == 0) {
        parse_configuration_data(json);
    } else if (strcmp(messageType->valuestring, "PanelChannels") == 0) {
        /* Sadece isim/ikon SoT — profil sayaclarina dokunma */
        if (hexnet_io_labels_apply_json(json)) {
            hexnet_panel_sync_on_changed();
            ESP_LOGW(GATTS_TABLE_TAG, "PanelChannels uygulandi → CAN labels");
        } else {
            ESP_LOGW(GATTS_TABLE_TAG, "PanelChannels: channels yok/bos");
        }
    } else if (strcmp(messageType->valuestring, "Rules") == 0) {
        parse_rules_data(json);
    } else {
        ESP_LOGE(GATTS_TABLE_TAG, "Unknown MessageType: %s", messageType->valuestring);
    }

    cJSON_Delete(json);
}

// Stub implementations for parsing helper functions
// These should be implemented based on your specific requirements
static void parse_read_data(cJSON* json) {
    ESP_LOGI(GATTS_TABLE_TAG, "parse_read_data called");
    // TODO: Implement read data parsing
}

// Function to parse Write data
void parse_write_data(cJSON* json) {
    cJSON* writeDataType = cJSON_GetObjectItem(json, "writeDataType");
    cJSON* writeNo = cJSON_GetObjectItem(json, "writeNo");
    cJSON* writeData = cJSON_GetObjectItem(json, "writeData");

    if (writeDataType && cJSON_IsString(writeDataType)) {
        ESP_LOGI("PARSE_WRITE_DATA", "writeDataType: %s", writeDataType->valuestring);
    }
    if (writeNo && cJSON_IsNumber(writeNo)) {
        ESP_LOGI("PARSE_WRITE_DATA", "writeNo: %d", writeNo->valueint);
    }

    if (writeDataType && cJSON_IsString(writeDataType)) {
        if (strcmp(writeDataType->valuestring, "Output") == 0) {
            if (!(writeNo && cJSON_IsNumber(writeNo))) {
                ESP_LOGW("PARSE_WRITE_DATA", "Output writeNo missing/invalid");
                return;
            }
            if (writeData && cJSON_IsNumber(writeData)) {
                const uint8_t output_index = (uint8_t)writeNo->valueint;
                if (output_index >= 16) {
                    ESP_LOGW("PARSE_WRITE_DATA", "Invalid output index: %u", output_index);
                    return;
                }
                /* Output/röle = Display 0x720 ile aynı: SR 0..15, profil pasifi yazımı engellemez (UI işi). */
                const int on = writeData->valueint ? 1 : 0;
                if (!hexnet_io_profile_relay_slot_enabled(output_index)) {
                    ESP_LOGW("PARSE_WRITE_DATA", "Relay slot %u profilde pasif — yine de SR (Display ile senkron)", output_index);
                }
                ESP_LOGI("PARSE_WRITE_DATA", "Output Value: %d", on);

                uint16_t mask = get_outputs();
                if (on) {
                    mask |= (uint16_t)(1U << output_index);
                } else {
                    mask &= (uint16_t)~(1U << output_index);
                }
                set_outputs(mask);
                hexnet_outputs_ble_mirror(mask);
                ESP_LOGI(EXAMPLE_TAG, "Relay %u -> %s mask=0x%04X", output_index, on ? "ON" : "OFF", mask);
                hexnet_io_notify_state_changed();
            } else {
                ESP_LOGW("PARSE_WRITE_DATA", "Output writeData missing/invalid");
            }
        } else if (strcmp(writeDataType->valuestring, "Dim") == 0 || strcmp(writeDataType->valuestring, "Dimmer") == 0) {
            if (!(writeNo && cJSON_IsNumber(writeNo))) {
                ESP_LOGW("PARSE_WRITE_DATA", "Dim writeNo missing/invalid");
                return;
            }
            if (writeData && cJSON_IsNumber(writeData)) {
                const uint8_t dim_index = (uint8_t)writeNo->valueint;
                if (dim_index >= 8) {
                    ESP_LOGW("PARSE_WRITE_DATA", "Invalid dim index: %u", dim_index);
                    return;
                }
                if (!hexnet_io_profile_dim_slot_enabled(dim_index)) {
                    ESP_LOGW("PARSE_WRITE_DATA", "Dim slot %u pasif (profil)", dim_index);
                    return;
                }
                const uint8_t dim_value = (uint8_t)((writeData->valueint < 0) ? 0 : (writeData->valueint > 100 ? 100 : writeData->valueint));
                ESP_LOGI("PARSE_WRITE_DATA", "Dim Value: %d", dim_value);
                set_dimmable_output(dim_index, dim_value);
                dimsDataBuffer[dim_index] = dim_value;
                hexnet_io_notify_state_changed();
            }
        } else if (strcmp(writeDataType->valuestring, "RGB") == 0) {
            cJSON* rgbArray = cJSON_GetObjectItem(json, "writeData");
            if (rgbArray && cJSON_IsArray(rgbArray) && cJSON_GetArraySize(rgbArray) >= 3) {
                const int arraySize = cJSON_GetArraySize(rgbArray);
                const int r = cJSON_GetArrayItem(rgbArray, 0)->valueint;
                const int g = cJSON_GetArrayItem(rgbArray, 1)->valueint;
                const int b = cJSON_GetArrayItem(rgbArray, 2)->valueint;
                const int ena = (arraySize >= 4 && cJSON_IsNumber(cJSON_GetArrayItem(rgbArray, 3)))
                    ? cJSON_GetArrayItem(rgbArray, 3)->valueint
                    : 1;

                rgbBuffer[0] = (uint8_t)((r < 0) ? 0 : (r > 255 ? 255 : r)); // Red
                rgbBuffer[1] = (uint8_t)((g < 0) ? 0 : (g > 255 ? 255 : g)); // Green
                rgbBuffer[2] = (uint8_t)((b < 0) ? 0 : (b > 255 ? 255 : b)); // Blue
                rgbEna = (uint8_t)(ena ? 1 : 0);
                rgbBuffer[3] = rgbEna; // RgbEnable
                set_rgb_values(rgbBuffer[0], rgbBuffer[1], rgbBuffer[2], rgbBuffer[3]);

                ESP_LOGI("PARSE_WRITE_DATA", "RGB Values: R=%d, G=%d, B=%d rgbEna=%d", rgbBuffer[0], rgbBuffer[1], rgbBuffer[2], rgbEna);
                hexnet_io_notify_state_changed();
            } else {
                ESP_LOGE("PARSE_WRITE_DATA", "RGB writeData must be an array of at least 3 values.");
            }
        } else if (strcmp(writeDataType->valuestring, "Motor") == 0) {
            if (!hexnet_io_profile_motor_enabled()) {
                ESP_LOGW("PARSE_WRITE_DATA", "Motor pasif (profil)");
                return;
            }
            if (writeData && cJSON_IsNumber(writeData)) {
                const int motorCmd = writeData->valueint;
                int motor_idx = 0;
                if (writeNo && cJSON_IsNumber(writeNo)) {
                    motor_idx = writeNo->valueint;
                }
                if (motor_idx < 0 || motor_idx > 1) {
                    motor_idx = 0;
                }
                hexnet_motor_set_cmd(motor_idx, (motorCmd >= 0 && motorCmd <= 2) ? motorCmd : 0);
                ESP_LOGI("PARSE_WRITE_DATA", "Motor%d command: %d", motor_idx + 1, hexnet_motor_get_cmd(motor_idx));
                hexnet_io_notify_state_changed();
            }
        } else {
            ESP_LOGI("PARSE_WRITE_DATA", "Unknown writeDataType: %s", writeDataType->valuestring);
        }
    }
}
// Function to parse Configuration data
void parse_configuration_data(cJSON* json) {
    cJSON* numOfOutputs_json = cJSON_GetObjectItem(json, "numOfOutputs");
    cJSON* outputsNameBuffer = cJSON_GetObjectItem(json, "OutputsNameBuffer");
    cJSON* numOfDims_json = cJSON_GetObjectItem(json, "numOfDims");
    cJSON* dimsNameBuffer = cJSON_GetObjectItem(json, "DimsNameBuffer");
    cJSON* numOfSensors_json = cJSON_GetObjectItem(json, "numOfSensors");
    cJSON* sensorsNameBuffer = cJSON_GetObjectItem(json, "SensorsNameBuffer");
    cJSON* rgbEnabled = cJSON_GetObjectItem(json, "RGBEnabled");
    cJSON* theme = cJSON_GetObjectItem(json, "Theme");

    int outputsBuf[16] = {0};
    int dimsBuf[8] = {0};
    int sensorsBuf[5] = {0};

    // Extract values and update global variables
    if (numOfOutputs_json && cJSON_IsNumber(numOfOutputs_json)) {
        numOfOutputs = numOfOutputs_json->valueint;
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "numOfOutputs: %d", numOfOutputs);
    }
    if (outputsNameBuffer && cJSON_IsArray(outputsNameBuffer)) {
        int size = cJSON_GetArraySize(outputsNameBuffer);
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "OutputsNameBuffer:");
        for (int i = 0; i < size && i < 16; i++) {
            cJSON* item = cJSON_GetArrayItem(outputsNameBuffer, i);
            if (cJSON_IsNumber(item)) {
                outputsBuf[i] = item->valueint;
                ESP_LOGI("PARSE_CONFIGURATION_DATA", "  %d", item->valueint);
            }
        }
    }
    if (numOfDims_json && cJSON_IsNumber(numOfDims_json)) {
        numOfDims = numOfDims_json->valueint;
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "numOfDims: %d", numOfDims);
    }
    if (dimsNameBuffer && cJSON_IsArray(dimsNameBuffer)) {
        int size = cJSON_GetArraySize(dimsNameBuffer);
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "DimsNameBuffer:");
        for (int i = 0; i < size && i < 8; i++) {
            cJSON* item = cJSON_GetArrayItem(dimsNameBuffer, i);
            if (cJSON_IsNumber(item)) {
                dimsBuf[i] = item->valueint;
                ESP_LOGI("PARSE_CONFIGURATION_DATA", "  %d", item->valueint);
            }
        }
    }
    if (numOfSensors_json && cJSON_IsNumber(numOfSensors_json)) {
        numOfSensors = numOfSensors_json->valueint;
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "numOfSensors: %d", numOfSensors);
    }
    if (sensorsNameBuffer && cJSON_IsArray(sensorsNameBuffer)) {
        int size = cJSON_GetArraySize(sensorsNameBuffer);
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "SensorsNameBuffer:");
        for (int i = 0; i < size && i < 5; i++) {
            cJSON* item = cJSON_GetArrayItem(sensorsNameBuffer, i);
            if (cJSON_IsNumber(item)) {
                sensorsBuf[i] = item->valueint;
                ESP_LOGI("PARSE_CONFIGURATION_DATA", "  %d", item->valueint);
            }
        }
    }
    if (rgbEnabled && cJSON_IsString(rgbEnabled)) {
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "RGBEnables: %s", rgbEnabled->valuestring);
    }
    if (theme && cJSON_IsString(theme)) {
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "Theme: %s", theme->valuestring);
    }
    
    (void)hexnet_io_profile_apply_json(json, HEXNET_IO_LINK_BLE);

    hexnet_io_profile_sync_legacy_globals();

    ESP_LOGI(
        "PARSE_CONFIGURATION_DATA",
        "Profil kaydedildi (BLE) - R=%d S=%d D=%d motor=%d rgb=%d",
        numOfOutputs,
        numOfSensors,
        numOfDims,
        hexnet_io_profile_motor_enabled(),
        hexnet_io_profile_rgb_enabled());
    
}

static void parse_rules_data(cJSON* json) {
    ESP_LOGI(GATTS_TABLE_TAG, "parse_rules_data called");
    // TODO: Implement rules data parsing
}




// Example function to create JSON data packet as a C string
char* create_json_data_packet(const uint16_t* regs_data, int numOfOutputs, int numOfDims, int numOfSensors, bool slaveConnectionStatus, int themeType, int numberOfNotifications, cJSON* notifications) {
    // Create a JSON object
    cJSON *json = cJSON_CreateObject();

    /* Platform eşleştirme / resolve-mqtt Wi-Fi STA MAC kullanır; BLE adresi farklı olabilir. */
    char wifi_mac[24] = {0};
    if (hexnet_format_wifi_mac(wifi_mac, sizeof(wifi_mac))) {
        cJSON_AddStringToObject(json, "mac", wifi_mac);
    } else {
        cJSON_AddStringToObject(json, "mac", "");
    }

    // Add number of outputs, dims, sensors, slave connection status, and theme type to the JSON object
    cJSON_AddStringToObject(json, "slvConn", "Yes");
    cJSON_AddNumberToObject(json, "numOfO", numOfOutputs);
    cJSON_AddNumberToObject(json, "numOfD", numOfDims);
    cJSON_AddNumberToObject(json, "numOfS", numOfSensors);
    cJSON_AddStringToObject(json, "RGBE", "yes");
    cJSON_AddNumberToObject(json, "Th", themeType);
    {
        const uint16_t raw_v = get_voltage_value();
        if (raw_v > 0) {
            batarya_volt = (float)raw_v / 100.0f;
        }
    }
    cJSON_AddNumberToObject(json, "volt", batarya_volt);


    // Add outputsBuffer to the JSON object
    cJSON *outputnames = cJSON_CreateIntArray(outputsBuffer, numOfOutputs);
    cJSON_AddItemToObject(json, "outNB", outputnames);

    // Add dimsBuffer to the JSON object
    cJSON *dimnames = cJSON_CreateIntArray(dimsBuffer, numOfDims);
    cJSON_AddItemToObject(json, "dimNB", dimnames);

    // Add sensorsBuffer to the JSON object
    cJSON *sensornames = cJSON_CreateIntArray(sensorsBuffer, 5);
    cJSON_AddItemToObject(json, "senNB", sensornames);


    /* Full 16-bit SR state — numOfO sadece isim sayısı; Röle12+ senkron için outDB kısaltılmaz. */
    int buf[16];
    const uint16_t relay_mask = get_outputs();
    for (int i = 0; i < 16; i++) {
        buf[i] = (relay_mask >> i) & 0x01;
    }
    cJSON *outputs = cJSON_CreateIntArray(buf, 16);
    cJSON_AddItemToObject(json, "outDB", outputs);

    // Fetch dimsBuffer from regs_data
    for (int i = 0; i < numOfDims; i++) {
        buf[i] = get_dimmable_output(i);
    }
    cJSON *dims = cJSON_CreateIntArray(buf, numOfDims);
    cJSON_AddItemToObject(json, "dDB", dims);


    // BLE/mobile: first 2 sensor slots are always temperature + humidity from IO module sensor channel.
    // Keep remaining sensor slots from existing analog map (water levels etc.) without changing MQTT/CAN logic.
    for (int i = 0; i < numOfSensors; i++) {
        if (i == 0) {
            int t = get_sensorTemp();
            buf[i] = (t < 0) ? 0 : (t > 255 ? 255 : t);
        } else if (i == 1) {
            int h = get_sensorHumidity();
            buf[i] = (h < 0) ? 0 : (h > 100 ? 100 : h);
        } else {
            const int analogIdx = i - 2;
            buf[i] = (analogIdx >= 0 && analogIdx < 4) ? get_analog_input((uint8_t)analogIdx) : 0;
        }
    }
    cJSON *sensors = cJSON_CreateIntArray(buf, numOfSensors);
    cJSON_AddItemToObject(json, "sDB", sensors);
    cJSON_AddNumberToObject(json, "sensorTemp", get_sensorTemp());
    cJSON_AddNumberToObject(json, "sensorHumidity", get_sensorHumidity());


    rgbBuffer[0] = get_r_value();
    rgbBuffer[1] = get_g_value();
    rgbBuffer[2] = get_b_value();
    rgbBuffer[3] = get_rgb_enable();
    rgbEna = (uint8_t)rgbBuffer[3];
    // Add rgbBuffer to the JSON object
    cJSON *rgb = cJSON_CreateIntArray(rgbBuffer, 4);
    cJSON_AddItemToObject(json, "RGBDB", rgb);

    const hexnet_io_profile_t *prof = hexnet_io_profile_get();
    if (prof) {
        cJSON_AddNumberToObject(json, "motorEn", prof->motor_enabled);
        cJSON_AddNumberToObject(json, "rgbEn", prof->rgb_enabled);
        cJSON_AddNumberToObject(json, "tankN", prof->num_tanks);
        cJSON_AddNumberToObject(json, "rMask", prof->relay_enable_mask);
        cJSON_AddNumberToObject(json, "dMask", prof->dim_enable_mask);
        cJSON_AddNumberToObject(json, "sMask", prof->sensor_enable_mask);
        cJSON_AddNumberToObject(json, "tMask", prof->tank_enable_mask);
        cJSON_AddNumberToObject(json, "ioLink", prof->link_flags);
        cJSON_AddNumberToObject(json, "panelSeq", hexnet_io_profile_get_panel_seq());
    }
    /* Canli BLE: channels yok (MTU/notify). SoT degisti veya CCC sonrasi bir kez dahil.
     * want_channels bayragini burada SILME — kuyruk basarisizsa tekrar denenebilsin. */
    const bool with_channels =
        s_ble_want_channels || hexnet_panel_sync_is_changed_publish();
    /* Async push'ta is_changed_publish erken false — channels varken tablete zorla bildir */
    if (prof) {
        cJSON_AddBoolToObject(json, "panelChanged",
                              hexnet_panel_sync_is_changed_publish() || with_channels);
    }
    hexnet_io_profile_append_to_json_ex(json, "io", with_channels);

    // Convert JSON object to string
    char *json_str = cJSON_PrintUnformatted(json);
    //ESP_LOGI("JSON_DATA_PACKET", "%s", json_str);

    // Free the JSON object
    cJSON_Delete(json);

    return json_str; // Caller is responsible for freeing the returned string
}


void get_data_json_format(const uint16_t* regs_data, int txPacketType, char** json_str)  {

    // Call create_json_data_packet function
    *json_str = create_json_data_packet(regs_data, numOfOutputs, numOfDims, numOfSensors, slaveConnectionStatus, panelThemeType, numberOfNotifications, notifications);
    // Check if notifications is not NULL before deleting
    if (notifications != NULL) {
        cJSON_Delete(notifications);
        notifications = NULL; // Set to NULL after deletion to avoid dangling pointer
    }
} 


// Function to read an integer from NVS
esp_err_t nvs_read_int(const char* key, int* value) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    int32_t temp_value;

    // Open NVS handle
    err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS_WRITE_PARAMS", "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    // Read integer from NVS
    err = nvs_get_i32(nvs_handle, key, &temp_value);
    if (err == ESP_OK) {
        *value = (int)temp_value;
    }
    nvs_close(nvs_handle);
    return err;
}

// Function to write an integer to NVS
static esp_err_t nvs_write_int(const char* key, int value) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Open NVS handle
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS_WRITE_PARAMS", "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    // Write integer to NVS
    err = nvs_set_i32(nvs_handle, key, value);
    if (err != ESP_OK) {
        ESP_LOGE("NVS_WRITE_PARAMS", "Error writing integer to NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    // Commit written value
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    return err;
}

// Function to read configuration data from NVS
void load_panel_configuration_from_nvs(int *totalOutpts, int buffer1[16], int *totalSensors, int buffer2[5], int *totalDims, int buffer3[8]) {
    int value;
    
    // Read totalOutpts from NVS
    if (nvs_read_int("numOfOutputs", &value) != ESP_OK || value < 0 || value > 16) {
        *totalOutpts = 12; // Set to default value if out of range
        ESP_LOGE("NVS_READ_PARAMS", "Error reading Total Outputs from NVS: %d", *totalOutpts);  
        save_panel_configuration_to_nvs(*totalOutpts, buffer1, *totalSensors, buffer2, *totalDims, buffer3);
    } else {
        *totalOutpts = (int)value;
    }

    // Read buffer1 from NVS
    for (int i = 0; i < 16; i++) {
        char key[16];
        snprintf(key, sizeof(key), "outBuf%d", i);
        if (nvs_read_int(key, &value) != ESP_OK || value < 1 || value > 18) {
            buffer1[i] = 1; // Set to default value if out of range
        } else {
            buffer1[i] = (int)value;
        }
    }

    // Read totalSensors from NVS
    if (nvs_read_int("numSens", &value) != ESP_OK || value < 0 || value > 5) {
        *totalSensors = 1; // Set to default value if out of range
        ESP_LOGE("NVS_READ_PARAMS", "Error reading Total Sensors from NVS: %d", *totalSensors);
        save_panel_configuration_to_nvs(*totalOutpts, buffer1, *totalSensors, buffer2, *totalDims, buffer3);
    } else {
        *totalSensors = (int)value;
    }

    // Read buffer2 from NVS
    for (int i = 0; i < 5; i++) {
        char key[16];
        snprintf(key, sizeof(key), "sensBuf%d", i);
        if (nvs_read_int(key, &value) != ESP_OK || value < 0 || value > 1) {
            buffer2[i] = 1; // Set to default value if out of range
        } else {
            buffer2[i] = (int)value;
        }
    }

    // Read totalDims from NVS
    if (nvs_read_int("numDims", &value) != ESP_OK || value < 0 || value > 8) {
        *totalDims = 8; // Set to default value if out of range
        ESP_LOGE("NVS_READ_PARAMS", "Error reading Total Dims from NVS: %d", *totalDims);
        save_panel_configuration_to_nvs(*totalOutpts, buffer1, *totalSensors, buffer2, *totalDims, buffer3);
    } else {
        *totalDims = (int)value;
    }

    // Read buffer3 from NVS
    for (int i = 0; i < 8; i++) {
        char key[16];
        snprintf(key, sizeof(key), "dimsBuf%d", i);
        if (nvs_read_int(key, &value) != ESP_OK || value < 1 || value > 8) {
            buffer3[i] = 1; // Set to default value if out of range
        } else {
            buffer3[i] = (int)value;
        }
    }
}

// Load the theme settings from NVS
void load_theme_configuration_from_nvs(int* themeType, int* wallpaperEnabled, int* wallpaperTimeIndex) {
    int value;

    // Read themeType from NVS
    if (nvs_read_int("thmTyp", &value) != ESP_OK || value < 0 || value > 1) {
        *themeType = 1; // Set to default value if out of range
    } else {
        *themeType = (int)value;
    }

    // Read wallpaperEnabled from NVS
    if (nvs_read_int("wallpEn", &value) != ESP_OK || value < 0 || value > 1) {
        *wallpaperEnabled = 1; // Set to default value if out of range
    } else {
        *wallpaperEnabled = (int)value;
    }

    // Read wallpaperTimeIndex from NVS
    if (nvs_read_int("wllTimI", &value) != ESP_OK || value < 0 || value > 600) {
        *wallpaperTimeIndex = 30; // Set to default value if out of range
    } else {
        *wallpaperTimeIndex = (int)value;
    }

    // Log the loaded configuration
    ESP_LOGI(GATTS_TABLE_TAG, "Loaded Theme Configuration: Theme=%d, WallpaperEnabled=%d, WallpaperTimeIndex=%d",
             *themeType, *wallpaperEnabled, *wallpaperTimeIndex);
}



void save_panel_configuration_to_nvs(int totalOutps, int buffer1[16], int totalSensors, int buffer2[5], int totalDims, int buffer3[8]) {
    // Ensure the values do not exceed the maximum allowed sizes
    if (totalOutps > 16) {
        totalOutps = 16;
    }
    if (totalSensors > 5) {
        totalSensors = 5;
    }
    if (totalDims > 8) {
        totalDims = 8;
    }

    // Save totalOutps to NVS
    nvs_write_int("numOfOutputs", totalOutps);
    ESP_LOGW("NVS_WRITE_PARAMS", "Total Outputs to save: %d", totalOutps);

    // Save buffer1 to NVS
    for (int i = 0; i < 16; i++) {
        if (buffer1[i] < 1 || buffer1[i] > 18) {
            buffer1[i] = 1; // Set to default value if out of range
        }
        char key[16];
        snprintf(key, sizeof(key), "outBuf%d", i);
        nvs_write_int(key, buffer1[i]);
    }

    // Save totalSensors to NVS
    nvs_write_int("numSens", totalSensors);
    ESP_LOGW("NVS_WRITE_PARAMS", "Total Sensors to save: %d", totalSensors);

    // Save buffer2 to NVS
    for (int i = 0; i < 5; i++) {
        if (buffer2[i] < 0 || buffer2[i] > 1) {
            buffer2[i] = 0; // Set to default value if out of range
        }
        char key[16];
        snprintf(key, sizeof(key), "sensBuf%d", i);
        nvs_write_int(key, buffer2[i]);
    }

    // Save totalDims to NVS
    nvs_write_int("numDims", totalDims);
    ESP_LOGW("NVS_WRITE_PARAMS", "Total Dims to save: %d", totalDims);

    // Save buffer3 to NVS
    for (int i = 0; i < 8; i++) {
        if (buffer3[i] < 0 || buffer3[i] > 8) {
            buffer3[i] = 0; // Set to default value if out of range
        }
        char key[16];
        snprintf(key, sizeof(key), "dimsBuf%d", i);
        nvs_write_int(key, buffer3[i]);        
    }
    ESP_LOGI("NVS_WRITE_PARAMS", "##### NVS Parameters Saved Successfully! #####");
}

void save_theme_configuration_to_nvs(int16_t* themeType, uint16_t* wallpaperEnabled, uint16_t* wallpaperTimeIndex){


    //Save themeType to NVS
    nvs_write_int("thmTyp", panelThemeType);

    //Save wallpaperEnabled to NVS
    nvs_write_int("wallpEn", panelWallpaperEnable);

    //Save wallpaperTimeIndex to NVS
    nvs_write_int("wllTimI", panelWallpaperTime);


}




void ble_init(void)
{
    esp_err_t ret;
    static bool s_ble_inited;

    hexnet_io_profile_sync_legacy_globals();
    load_theme_configuration_from_nvs(&panelThemeType, &panelWallpaperEnable, &panelWallpaperTime);

#if !BLE_ENB
    ESP_LOGI(GATTS_TABLE_TAG, "BLE disabled (BLE_ENB=0); panel config loaded from NVS");
    return;
#endif

    if (s_ble_inited) {
        ESP_LOGW(GATTS_TABLE_TAG, "ble_init already done — skip");
        return;
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth name=%s", __func__, SAMPLE_DEVICE_NAME);

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);

    // Create queues
    cmd_cmd_queue = xQueueCreate(10, sizeof(uint16_t));
    /* Tek slot: MQTT heap baskisini azalt */
    data_transmit_queue = xQueueCreate(1, sizeof(char*));
    /* 1.5s — MQTT 3s heartbeat ile carpismasin */
    s_notify_timer = xTimerCreate("ble_notify", pdMS_TO_TICKS(1500), pdTRUE, NULL, bluetooth_send_periodic_callback);
    s_ble_resync_timer = xTimerCreate("ble_resync", pdMS_TO_TICKS(400), pdFALSE, NULL, ble_resync_timer_cb);
    
    if (cmd_cmd_queue == NULL || data_transmit_queue == NULL || s_notify_timer == NULL) {
        ESP_LOGE(GATTS_TABLE_TAG, "Failed to create BLE notify resources");
        return;
    }

    /* Parse kuyrugu: GATT/BTC icinde cJSON/CAN yok — ble_push worker drain eder */
    s_ble_parse_queue = xQueueCreate(4, sizeof(char *));
    if (s_ble_parse_queue == NULL) {
        ESP_LOGE(GATTS_TABLE_TAG, "ble_parse queue FAIL");
    }

    /* Dusuk oncelik — telemetry_publish (prio 6) once */
    xTaskCreate(data_transmit_task, "data_transmit", 4096, NULL, 3, NULL);
    /* Push+parse worker: Tmr Svc / BTC stack overflow onler; 6K cJSON+labels icin */
    if (xTaskCreate(ble_push_worker_task, "ble_push", 6144, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(GATTS_TABLE_TAG, "ble_push worker FAIL");
        /* fallback smaller */
        xTaskCreate(ble_push_worker_task, "ble_push", 4096, NULL, 3, NULL);
    } else {
        ESP_LOGI(GATTS_TABLE_TAG, "ble_push+parse worker ready");
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    s_ble_inited = true;
    return;
}

bool get_connection_status(void)
{
    return is_connected;
}

bool hexnet_ble_is_connected(void)
{
#if BLE_ENB
    return is_connected;
#else
    return false;
#endif
}

static void ble_request_push_from_timer(void)
{
    s_ble_push_request = true;
}

void hexnet_ble_request_push(void)
{
    s_ble_push_request = true;
}

void hexnet_ble_request_channels(void)
{
    s_ble_want_channels = true;
}

void hexnet_ble_request_display_label_commit(void)
{
    s_ble_display_label_commit = true;
}

static void ble_push_worker_task(void *arg)
{
    (void)arg;
    while (1) {
        /* Once inbound JSON (PanelChannels/Configuration) — BTC disinda */
        if (s_ble_parse_queue) {
            char *msg = NULL;
            while (xQueueReceive(s_ble_parse_queue, &msg, 0) == pdTRUE && msg) {
                parse_ble_data(msg);
                free(msg);
                msg = NULL;
            }
        }
        /* Display→IO ikon: NVS + panel_sync CAN RX'de degil (TWAI blok riski) */
        if (s_ble_display_label_commit) {
            s_ble_display_label_commit = false;
            (void)hexnet_io_labels_save();
            hexnet_panel_sync_on_changed();
            hexnet_io_notify_state_changed();
            ESP_LOGW(GATTS_TABLE_TAG, "Display label commit → NVS + panel_sync");
        }
        if (s_ble_push_request) {
            s_ble_push_request = false;
            hexnet_ble_push_telemetry_now();
        }
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

static bool ble_parse_enqueue(const char *json)
{
    if (!json || !json[0]) {
        return false;
    }
    if (s_ble_parse_queue == NULL) {
        /* BTC icinde sync YAPMA — drop + log (crash onler) */
        ESP_LOGE(GATTS_TABLE_TAG, "ble_parse queue yok — drop %u B", (unsigned)strlen(json));
        return false;
    }
    const size_t n = strlen(json);
    char *copy = (char *)malloc(n + 1);
    if (!copy) {
        ESP_LOGE(GATTS_TABLE_TAG, "ble_parse malloc fail (%u)", (unsigned)n);
        return false;
    }
    memcpy(copy, json, n + 1);
    if (xQueueSend(s_ble_parse_queue, &copy, 0) != pdTRUE) {
        ESP_LOGW(GATTS_TABLE_TAG, "ble_parse queue full — drop %u B", (unsigned)n);
        free(copy);
        return false;
    }
    return true;
}

static void ble_resync_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    s_ble_resync_pending = false;
    /* ASLA Tmr Svc icinde push yapma — stack overflow (2048) */
    ble_request_push_from_timer();
}

static bool lean_append_int_array(char **pp, size_t *prem, const char *key, const int *vals, int count)
{
    if (!pp || !*pp || !prem || !key || count < 0) {
        return false;
    }
    int n = snprintf(*pp, *prem, "\"%s\":[", key);
    if (n < 0 || (size_t)n >= *prem) {
        return false;
    }
    *pp += n;
    *prem -= (size_t)n;
    for (int i = 0; i < count; i++) {
        const int v = vals ? vals[i] : 0;
        n = snprintf(*pp, *prem, "%s%d", (i ? "," : ""), v);
        if (n < 0 || (size_t)n >= *prem) {
            return false;
        }
        *pp += n;
        *prem -= (size_t)n;
    }
    n = snprintf(*pp, *prem, "],");
    if (n < 0 || (size_t)n >= *prem) {
        return false;
    }
    *pp += n;
    *prem -= (size_t)n;
    return true;
}

static bool lean_append_json_str(char **pp, size_t *prem, const char *s)
{
    if (!pp || !*pp || !prem) {
        return false;
    }
    if (!s) {
        s = "";
    }
    if (*prem < 2) {
        return false;
    }
    *(*pp)++ = '"';
    (*prem)--;
    for (const unsigned char *u = (const unsigned char *)s; *u; ++u) {
        if (*u == '"' || *u == '\\') {
            if (*prem < 2) {
                return false;
            }
            *(*pp)++ = '\\';
            (*prem)--;
        }
        if (*prem < 1) {
            return false;
        }
        *(*pp)++ = (char)*u;
        (*prem)--;
    }
    if (*prem < 1) {
        return false;
    }
    *(*pp)++ = '"';
    (*prem)--;
    return true;
}

static bool ble_queue_channels_notify(const char *json)
{
    if (!json || data_transmit_queue == NULL) {
        return false;
    }
    const size_t n = strlen(json) + 1;
    if (n > BLE_CH_JSON_MAX) {
        ESP_LOGW(GATTS_TABLE_TAG, "channels notify too large len=%u", (unsigned)(n - 1));
        return false;
    }
    if (uxQueueMessagesWaiting(data_transmit_queue) > 0) {
        char *stale = NULL;
        if (xQueueReceive(data_transmit_queue, &stale, 0) == pdTRUE && stale &&
            stale != s_ble_notify_a && stale != s_ble_notify_b &&
            stale != s_ble_ch_a && stale != s_ble_ch_b) {
            free(stale);
        }
    }
    char *dst = (s_ble_ch_last == s_ble_ch_a) ? s_ble_ch_b : s_ble_ch_a;
    memcpy(dst, json, n);
    s_ble_ch_last = dst;
    if (xQueueSend(data_transmit_queue, &dst, 0) != pdTRUE) {
        ESP_LOGW(GATTS_TABLE_TAG, "channels notify queue full");
        return false;
    }
    ESP_LOGW(GATTS_TABLE_TAG, "panel push FULL+channels queued len=%u", (unsigned)(n - 1));
    return true;
}

/** PSRAM yok / dusuk heap: cJSON kullanmadan canli BLE paketi (static buffer). */
static char *build_ble_lean_json(void)
{
    char *p = s_ble_json_live;
    size_t rem = sizeof(s_ble_json_live);
    int n;
    int out_nb[16];
    int dim_nb[8];
    int sen_nb[5];
    int out_db[16];
    int d_db[8];
    int s_db[5];
    const int n_out = (numOfOutputs > 0 && numOfOutputs <= 16) ? numOfOutputs : 0;
    const int n_out_db = 16; /* SR 16 bit — Röle12+ tablet senkron */
    const int n_dim = (numOfDims > 0 && numOfDims <= 8) ? numOfDims : 0;
    const int n_sen = (numOfSensors > 0 && numOfSensors <= 5) ? numOfSensors : 0;
    const uint16_t relay_mask = get_outputs();

    for (int i = 0; i < n_out; i++) {
        out_nb[i] = outputsBuffer[i];
    }
    for (int i = 0; i < n_out_db; i++) {
        out_db[i] = (int)((relay_mask >> i) & 0x01);
    }
    for (int i = 0; i < n_dim; i++) {
        dim_nb[i] = dimsBuffer[i];
        d_db[i] = get_dimmable_output((uint8_t)i);
    }
    for (int i = 0; i < n_sen; i++) {
        sen_nb[i] = sensorsBuffer[i];
        if (i == 0) {
            int t = get_sensorTemp();
            s_db[i] = (t < 0) ? 0 : (t > 255 ? 255 : t);
        } else if (i == 1) {
            int h = get_sensorHumidity();
            s_db[i] = (h < 0) ? 0 : (h > 100 ? 100 : h);
        } else {
            const int analogIdx = i - 2;
            s_db[i] = (analogIdx >= 0 && analogIdx < 4) ? get_analog_input((uint8_t)analogIdx) : 0;
        }
    }

    /* batarya_volt çoğu kurulumda hiç güncellenmiyor; ADC simvoltage (x100) SoT */
    {
        const uint16_t raw_v = get_voltage_value();
        if (raw_v > 0) {
            batarya_volt = (float)raw_v / 100.0f;
        }
    }
    n = snprintf(p, rem,
                 "{\"slvConn\":\"Yes\",\"numOfO\":%d,\"numOfD\":%d,\"numOfS\":%d,"
                 "\"RGBE\":\"yes\",\"Th\":%d,\"volt\":%.1f,",
                 numOfOutputs, numOfDims, numOfSensors, panelThemeType, (double)batarya_volt);
    if (n < 0 || (size_t)n >= rem) {
        return NULL;
    }
    p += n;
    rem -= (size_t)n;

    /* App VanGoData zorunlu alanlar — eksikse decode exception, tablet guncellenmez */
    if (!lean_append_int_array(&p, &rem, "outNB", out_nb, n_out) ||
        !lean_append_int_array(&p, &rem, "dimNB", dim_nb, n_dim) ||
        !lean_append_int_array(&p, &rem, "senNB", sen_nb, n_sen) ||
        !lean_append_int_array(&p, &rem, "outDB", out_db, n_out_db) ||
        !lean_append_int_array(&p, &rem, "dDB", d_db, n_dim) ||
        !lean_append_int_array(&p, &rem, "sDB", s_db, n_sen)) {
        return NULL;
    }

    {
        const hexnet_io_profile_t *prof = hexnet_io_profile_get();
        const unsigned rmask = prof ? (unsigned)prof->relay_enable_mask : 0u;
        const unsigned dmask = prof ? (unsigned)prof->dim_enable_mask : 0u;
        const unsigned smask = prof ? (unsigned)prof->sensor_enable_mask : 0u;
        const unsigned tmask = prof ? (unsigned)prof->tank_enable_mask : 0u;
        n = snprintf(p, rem,
                     "\"sensorTemp\":%d,\"sensorHumidity\":%d,\"RGBDB\":[%d,%d,%d,%d],"
                     "\"rMask\":%u,\"dMask\":%u,\"sMask\":%u,\"tMask\":%u,\"panelSeq\":%u}",
                     get_sensorTemp(), get_sensorHumidity(),
                     get_r_value(), get_g_value(), get_b_value(), get_rgb_enable(),
                     rmask, dmask, smask, tmask,
                     (unsigned)hexnet_io_profile_get_panel_seq());
    }
    if (n < 0 || (size_t)n >= rem) {
        return NULL;
    }
    return s_ble_json_live;
}

/** Firmware preset 1..18 → tablet PanelIconCatalog key (bos label ikonu yedegi). */
static const char *ble_preset_to_icon_key(uint8_t preset)
{
    switch (preset) {
    case 2:  return "klozet";
    case 3:  return "mutfak_aydinlatma";
    case 4:  return "yatak_basi";
    case 5:  return "kapi";
    case 7:  return "klima";
    case 8:  return "usb_priz";
    case 9:  return "buzdolabi";
    case 10: return "su_pompasi";
    case 11: return "priz_220v";
    case 12: return "firin";
    case 13: return "tv";
    case 14: return "dis_aydinlatma";
    case 15: return "priz_220v";
    case 16: return "elektrikli_isitici";
    case 17: return "spot_aydinlatma";
    case 18: return "okuma_lambasi";
    case 1:
    case 6:
    default: return "tavan_aydinlatma";
    }
}

static const char *ble_dim_type_to_icon_key(uint8_t type)
{
    static const char *keys[8] = {
        "tavan_aydinlatma", "banyo_aydinlatma", "mutfak_aydinlatma", "yatak_basi",
        "giris_aydinlatma", "merdiven_aydinlatma", "dis_aydinlatma", "spot_aydinlatma"
    };
    if (type < 1 || type > 8) {
        type = 1;
    }
    return keys[type - 1];
}

static bool ble_append_channel_obj(char **pp, size_t *prem, int id, bool en,
                                   const char *name, const char *icon, bool first)
{
    char *p = *pp;
    size_t rem = *prem;
    int n = snprintf(p, rem, "%s{\"id\":%d,\"enabled\":%s,\"name\":",
                     first ? "" : ",", id, en ? "true" : "false");
    if (n < 0 || (size_t)n >= rem) {
        return false;
    }
    p += n;
    rem -= (size_t)n;
    if (!lean_append_json_str(&p, &rem, name ? name : "")) {
        return false;
    }
    n = snprintf(p, rem, ",\"icon\":");
    if (n < 0 || (size_t)n >= rem) {
        return false;
    }
    p += n;
    rem -= (size_t)n;
    if (!lean_append_json_str(&p, &rem, icon ? icon : "")) {
        return false;
    }
    if (rem < 2) {
        return false;
    }
    *p++ = '}';
    rem--;
    *pp = p;
    *prem = rem;
    return true;
}

/** Lean + io.channels — cJSON FULL yerine static buffer (OOM yok). */
static char *build_ble_channels_json(void)
{
    char *dst = (s_ble_ch_last == s_ble_ch_a) ? s_ble_ch_b : s_ble_ch_a;
    char *p = dst;
    size_t rem = BLE_CH_JSON_MAX;
    const char *lean = build_ble_lean_json();
    const hexnet_io_labels_t *L;
    const hexnet_io_profile_t *prof;
    int icons_n = 0;
    if (!lean) {
        return NULL;
    }
    size_t lean_len = strlen(lean);
    if (lean_len < 2 || lean[lean_len - 1] != '}') {
        return NULL;
    }
    /* lean'in kapanis } oncesine panelChanged + io.channels ekle */
    if (lean_len - 1 >= rem) {
        return NULL;
    }
    memcpy(p, lean, lean_len - 1);
    p += lean_len - 1;
    rem -= (lean_len - 1);

    int n = snprintf(p, rem,
                     ",\"panelChanged\":true,\"io\":{\"panel_seq\":%u,\"panel_changed\":true,"
                     "\"relay_mask\":%u,\"dim_mask\":%u,\"channels\":{\"relays\":[",
                     (unsigned)hexnet_io_profile_get_panel_seq(),
                     (unsigned)(hexnet_io_profile_get() ? hexnet_io_profile_get()->relay_enable_mask : 0),
                     (unsigned)(hexnet_io_profile_get() ? hexnet_io_profile_get()->dim_enable_mask : 0));
    if (n < 0 || (size_t)n >= rem) {
        return NULL;
    }
    p += n;
    rem -= (size_t)n;

    L = hexnet_io_labels_get();
    prof = hexnet_io_profile_get();
    /* 12 slot sabit — pasifler enabled:false; ikon label veya relay_type yedegi */
    for (int i = 0; i < 12 && i < HEXNET_LABEL_RELAYS; ++i) {
        const bool en = hexnet_io_profile_relay_slot_enabled((uint8_t)i);
        const char *name = (L && L->relays[i].name[0]) ? L->relays[i].name : "";
        const char *icon = (L && L->relays[i].icon[0]) ? L->relays[i].icon : "";
        if (!icon[0] && prof) {
            uint8_t t = prof->relay_type[i];
            if (t == 0 && en) {
                t = 1;
            }
            if (t > 0) {
                icon = ble_preset_to_icon_key(t);
            }
        }
        if (icon[0]) {
            icons_n++;
        }
        if (!ble_append_channel_obj(&p, &rem, i, en, name, icon, i == 0)) {
            return NULL;
        }
    }

    n = snprintf(p, rem, "],\"dimmers\":[");
    if (n < 0 || (size_t)n >= rem) {
        return NULL;
    }
    p += n;
    rem -= (size_t)n;
    for (int i = 0; i < 8 && i < HEXNET_LABEL_DIMS; ++i) {
        const bool en = hexnet_io_profile_dim_slot_enabled((uint8_t)i);
        const char *name = (L && L->dimmers[i].name[0]) ? L->dimmers[i].name : "";
        const char *icon = (L && L->dimmers[i].icon[0]) ? L->dimmers[i].icon : "";
        if (!icon[0] && prof) {
            uint8_t t = prof->dim_type[i];
            if (t == 0 && en) {
                t = 1;
            }
            if (t > 0) {
                icon = ble_dim_type_to_icon_key(t);
            }
        }
        if (icon[0]) {
            icons_n++;
        }
        if (!ble_append_channel_obj(&p, &rem, i, en, name, icon, i == 0)) {
            return NULL;
        }
    }

    n = snprintf(p, rem, "],\"sensors\":[],\"water\":[]}}}");
    if (n < 0 || (size_t)n >= rem) {
        return NULL;
    }
    ESP_LOGI(GATTS_TABLE_TAG, "panel channels JSON icons=%d rem=%u", icons_n, (unsigned)rem);
    return dst;
}

void hexnet_ble_push_telemetry_now(void)
{
#if BLE_ENB
    if (!is_connected || !enable_data_ntf) {
        ESP_LOGI(GATTS_TABLE_TAG, "panel push skipped (ble connected=%d ntf=%d want_ch=%d)",
                 is_connected ? 1 : 0, enable_data_ntf ? 1 : 0, s_ble_want_channels ? 1 : 0);
        /* CCC sonrasi tekrar dene — ikon SoT kaybolmasin */
        if (s_ble_want_channels && s_ble_resync_timer && !s_ble_resync_pending) {
            s_ble_resync_pending = true;
            xTimerStart(s_ble_resync_timer, 0);
        }
        return;
    }
    /* Isim/ikon SoT: cJSON FULL OOM riski → static lean+channels */
    if (s_ble_want_channels) {
        static int s_ch_fail_streak;
        char *full = build_ble_channels_json();
        if (full && ble_queue_channels_notify(full)) {
            s_ble_want_channels = false;
            s_ch_fail_streak = 0;
            return;
        }
        s_ch_fail_streak++;
        ESP_LOGW(GATTS_TABLE_TAG, "panel push channels fail streak=%d — %s",
                 s_ch_fail_streak,
                 (s_ch_fail_streak >= 5) ? "fallback lean" : "retry");
        if (s_ch_fail_streak < 5) {
            if (s_ble_resync_timer && !s_ble_resync_pending) {
                s_ble_resync_pending = true;
                xTimerStart(s_ble_resync_timer, 0);
            }
            return;
        }
        /* Buffer/MTU/queue sorununda canlı telemetriyi kilitleme */
        s_ble_want_channels = false;
        s_ch_fail_streak = 0;
    }
    char *json = build_ble_lean_json();
    if (!json) {
        ESP_LOGW(GATTS_TABLE_TAG, "panel push lean build failed");
        if (s_ble_resync_timer && !s_ble_resync_pending) {
            s_ble_resync_pending = true;
            xTimerStart(s_ble_resync_timer, 0);
        }
        return;
    }
    set_converted_json_data(json);
    if (!ble_queue_lean_notify(json)) {
        if (s_ble_resync_timer && !s_ble_resync_pending) {
            s_ble_resync_pending = true;
            xTimerStart(s_ble_resync_timer, 0);
        }
    }
#endif
}


// Function to set the global variable converted_json_data2
void set_converted_json_data(char* converted_json_data) {
    if (converted_json_data2 != NULL &&
        converted_json_data2 != s_ble_json_live &&
        converted_json_data2 != converted_json_data) {
        /* cJSON_Print → PSRAM hooks; free() SPIRAM pointer heap'i bozuyordu */
        cJSON_free(converted_json_data2);
    }
    converted_json_data2 = converted_json_data;
    ESP_LOGD(GATTS_TABLE_TAG, "BLE JSON updated (%u bytes)",
             converted_json_data2 ? (unsigned)strlen(converted_json_data2) : 0U);
}

// Function to get the spp_cmd_buff
uint8_t* get_spp_cmd_buff() {
    return spp_cmd_buff;
}

// Function to reset the spp_cmd_buff
void reset_spp_cmd_buff() {
    memset(spp_cmd_buff, 0x0, sizeof(spp_cmd_buff));
}