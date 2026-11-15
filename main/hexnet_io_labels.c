#include "hexnet_io_labels.h"

#include <string.h>

#include "esp_log.h"
#include "hexnet_io_profile.h"
#include "nvs.h"

static const char *TAG = "IO_LABELS";

#define NVS_NS   "hexnet"
#define NVS_KEY  "io_labels"
#define MAGIC    0x4C4F4948u /* HIOL */

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t version;
    uint8_t reserved[3];
    hexnet_io_labels_t labels;
} hexnet_io_labels_blob_t;

static hexnet_io_labels_t s_labels;
static bool s_loaded;
/* Display 0x762/763 assembler */
static char s_can_asm[2][HEXNET_LABEL_RELAYS][HEXNET_LABEL_NAME_MAX + 1];
static uint8_t s_can_parts[2][HEXNET_LABEL_RELAYS];

static void slot_clear(hexnet_label_slot_t *s)
{
    if (!s) {
        return;
    }
    s->name[0] = '\0';
    s->icon[0] = '\0';
}

static void labels_clear(hexnet_io_labels_t *L)
{
    for (int i = 0; i < HEXNET_LABEL_RELAYS; ++i) {
        slot_clear(&L->relays[i]);
    }
    for (int i = 0; i < HEXNET_LABEL_DIMS; ++i) {
        slot_clear(&L->dimmers[i]);
    }
    for (int i = 0; i < HEXNET_LABEL_SENSORS; ++i) {
        slot_clear(&L->sensors[i]);
    }
    for (int i = 0; i < HEXNET_LABEL_WATER; ++i) {
        slot_clear(&L->water[i]);
    }
}

static void copy_cstr(char *dst, size_t dst_sz, const char *src)
{
    if (!dst || dst_sz == 0) {
        return;
    }
    if (!src) {
        dst[0] = '\0';
        return;
    }
    strncpy(dst, src, dst_sz - 1);
    dst[dst_sz - 1] = '\0';
}

static void apply_slot_obj(hexnet_label_slot_t *slot, const cJSON *item)
{
    if (!slot || !cJSON_IsObject(item)) {
        return;
    }
    const cJSON *name = cJSON_GetObjectItem(item, "name");
    const cJSON *icon = cJSON_GetObjectItem(item, "icon");
    if (cJSON_IsString(name) && name->valuestring) {
        copy_cstr(slot->name, sizeof(slot->name), name->valuestring);
    }
    if (cJSON_IsString(icon) && icon->valuestring) {
        copy_cstr(slot->icon, sizeof(slot->icon), icon->valuestring);
    }
}

static void apply_channel_array(hexnet_label_slot_t *slots, int max_slots, const cJSON *arr)
{
    if (!cJSON_IsArray(arr) || !slots) {
        return;
    }
    const int n = cJSON_GetArraySize(arr);
    for (int i = 0; i < n; ++i) {
        const cJSON *item = cJSON_GetArrayItem(arr, i);
        if (!cJSON_IsObject(item)) {
            continue;
        }
        const cJSON *id_item = cJSON_GetObjectItem(item, "id");
        int id = cJSON_IsNumber(id_item) ? id_item->valueint : i;
        if (id < 0 || id >= max_slots) {
            continue;
        }
        apply_slot_obj(&slots[id], item);
    }
}

static cJSON *slot_to_json(int id, const hexnet_label_slot_t *slot, int enabled)
{
    cJSON *o = cJSON_CreateObject();
    if (!o) {
        return NULL;
    }
    cJSON_AddNumberToObject(o, "id", id);
    cJSON_AddBoolToObject(o, "enabled", enabled != 0);
    cJSON_AddStringToObject(o, "name", slot && slot->name[0] ? slot->name : "");
    cJSON_AddStringToObject(o, "icon", slot && slot->icon[0] ? slot->icon : "");
    return o;
}

void hexnet_io_labels_init(void)
{
    labels_clear(&s_labels);
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        s_loaded = true;
        ESP_LOGW(TAG, "labels NVS yok — bos");
        return;
    }
    hexnet_io_labels_blob_t blob;
    size_t len = sizeof(blob);
    esp_err_t err = nvs_get_blob(h, NVS_KEY, &blob, &len);
    nvs_close(h);
    if (err == ESP_OK && len >= sizeof(blob) && blob.magic == MAGIC && blob.version == 1) {
        s_labels = blob.labels;
        ESP_LOGI(TAG, "labels NVS yuklendi");
    } else {
        ESP_LOGW(TAG, "labels blob yok/bozuk");
    }
    s_loaded = true;
}

const hexnet_io_labels_t *hexnet_io_labels_get(void)
{
    return &s_labels;
}

esp_err_t hexnet_io_labels_set_relay(int slot, const char *name, const char *icon)
{
    if (!s_loaded || slot < 0 || slot >= HEXNET_LABEL_RELAYS) {
        return ESP_ERR_INVALID_ARG;
    }
    hexnet_label_slot_t *slotp = &s_labels.relays[slot];
    if (name) {
        copy_cstr(slotp->name, sizeof(slotp->name), name);
    }
    if (icon) {
        copy_cstr(slotp->icon, sizeof(slotp->icon), icon);
    }
    return hexnet_io_labels_save();
}

esp_err_t hexnet_io_labels_save(void)
{
    hexnet_io_labels_blob_t blob;
    memset(&blob, 0, sizeof(blob));
    blob.magic = MAGIC;
    blob.version = 1;
    blob.labels = s_labels;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(h, NVS_KEY, &blob, sizeof(blob));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "labels flash'a yazildi (%u B)", (unsigned)sizeof(blob));
    }
    return err;
}

bool hexnet_io_labels_apply_json(const cJSON *json)
{
    if (!json || !s_loaded) {
        return false;
    }
    const cJSON *channels = cJSON_GetObjectItem(json, "channels");
    if (!cJSON_IsObject(channels)) {
        channels = cJSON_GetObjectItem(json, "panel_channels");
    }
    if (!cJSON_IsObject(channels)) {
        return false;
    }

    bool any = false;
    const cJSON *relays = cJSON_GetObjectItem(channels, "relays");
    if (cJSON_IsArray(relays)) {
        apply_channel_array(s_labels.relays, HEXNET_LABEL_RELAYS, relays);
        any = true;
    }
    const cJSON *dims = cJSON_GetObjectItem(channels, "dimmers");
    if (cJSON_IsArray(dims)) {
        apply_channel_array(s_labels.dimmers, HEXNET_LABEL_DIMS, dims);
        any = true;
    }
    const cJSON *sensors = cJSON_GetObjectItem(channels, "sensors");
    if (cJSON_IsArray(sensors)) {
        apply_channel_array(s_labels.sensors, HEXNET_LABEL_SENSORS, sensors);
        any = true;
    }
    const cJSON *water = cJSON_GetObjectItem(channels, "water");
    if (cJSON_IsArray(water)) {
        apply_channel_array(s_labels.water, HEXNET_LABEL_WATER, water);
        any = true;
    }

    if (any) {
        (void)hexnet_io_labels_save();
        ESP_LOGW(TAG, "channels uygulandi r0='%s' ico='%s'",
                 s_labels.relays[0].name, s_labels.relays[0].icon);
    } else {
        ESP_LOGW(TAG, "channels JSON yok/bos — labels degismedi");
    }
    return any;
}

bool hexnet_io_labels_can_chunk(uint8_t kind, uint8_t slot, uint8_t part, const uint8_t six[6])
{
    if (!s_loaded || !six || slot >= HEXNET_LABEL_RELAYS || kind > 1 || part > 3) {
        return false;
    }
    char *asm_buf = s_can_asm[kind][slot];
    uint8_t *parts = &s_can_parts[kind][slot];

    if (part == 0) {
        memset(asm_buf, 0, HEXNET_LABEL_NAME_MAX + 1);
        *parts = 0;
    } else if ((*parts & 0x01u) == 0) {
        ESP_LOGW(TAG, "CAN label skip orphan kind=%u slot=%u part=%u",
                 (unsigned)kind, (unsigned)slot, (unsigned)part);
        return false;
    }

    const size_t off = (size_t)part * 6u;
    if (off >= HEXNET_LABEL_NAME_MAX) {
        return false;
    }
    size_t n = 6;
    if (off + n > HEXNET_LABEL_NAME_MAX) {
        n = HEXNET_LABEL_NAME_MAX - off;
    }
    memcpy(asm_buf + off, six, n);
    asm_buf[HEXNET_LABEL_NAME_MAX] = '\0';
    *parts |= (uint8_t)(1u << part);

    bool has_nul = false;
    for (size_t i = 0; i < n; ++i) {
        if (six[i] == 0) {
            has_nul = true;
            break;
        }
    }
    if (!has_nul && part < 3) {
        return false;
    }

    const uint8_t need = (uint8_t)((1u << (part + 1)) - 1u);
    if ((*parts & need) != need) {
        return false;
    }

    hexnet_label_slot_t *slotp = &s_labels.relays[slot];
    if (kind == 0) {
        copy_cstr(slotp->name, sizeof(slotp->name), asm_buf);
    } else {
        copy_cstr(slotp->icon, sizeof(slotp->icon), asm_buf);
    }
    *parts = 0;
    /* NVS save CAN RX task'ta YAPILMAZ — twai/CAN kuyrugu bloklanir, 0x763 dusebilir.
     * Kalicilik: caller (deferred worker) hexnet_io_labels_save() cagirir. */
    ESP_LOGW(TAG, "Display label slot=%u kind=%u '%s'",
             (unsigned)slot, (unsigned)kind, kind == 0 ? slotp->name : slotp->icon);
    return true;
}

void hexnet_io_labels_append_channels(cJSON *io_obj)
{
    if (!io_obj) {
        return;
    }
    const hexnet_io_profile_t *prof = hexnet_io_profile_get();

    cJSON *channels = cJSON_CreateObject();
    cJSON *relays = cJSON_CreateArray();
    cJSON *dims = cJSON_CreateArray();
    cJSON *sensors = cJSON_CreateArray();
    cJSON *water = cJSON_CreateArray();
    if (!channels || !relays || !dims || !sensors || !water) {
        cJSON_Delete(channels);
        cJSON_Delete(relays);
        cJSON_Delete(dims);
        cJSON_Delete(sensors);
        cJSON_Delete(water);
        return;
    }

    /* Tum panel slotlari (aktif+pasif) — sadece enabled sayisi gonderilirse
     * tablette eksik id'ler pad ile enabled=true kalir (Display pasif / tablet aktif). */
    const int n_rel = 12;
    for (int i = 0; i < n_rel && i < HEXNET_LABEL_RELAYS; ++i) {
        cJSON_AddItemToArray(
            relays,
            slot_to_json(i, &s_labels.relays[i], hexnet_io_profile_relay_slot_enabled((uint8_t)i) ? 1 : 0));
    }
    const int n_dim = 8;
    for (int i = 0; i < n_dim && i < HEXNET_LABEL_DIMS; ++i) {
        cJSON_AddItemToArray(
            dims,
            slot_to_json(i, &s_labels.dimmers[i], hexnet_io_profile_dim_slot_enabled((uint8_t)i) ? 1 : 0));
    }
    for (int i = 0; i < HEXNET_LABEL_SENSORS; ++i) {
        cJSON_AddItemToArray(
            sensors,
            slot_to_json(i, &s_labels.sensors[i], hexnet_io_profile_sensor_slot_enabled((uint8_t)i) ? 1 : 0));
    }
    for (int i = 0; i < HEXNET_LABEL_WATER; ++i) {
        const int en = (prof && (prof->tank_enable_mask & (1u << i))) ? 1 : 0;
        cJSON_AddItemToArray(water, slot_to_json(i, &s_labels.water[i], en));
    }

    cJSON_AddItemToObject(channels, "relays", relays);
    cJSON_AddItemToObject(channels, "dimmers", dims);
    cJSON_AddItemToObject(channels, "sensors", sensors);
    cJSON_AddItemToObject(channels, "water", water);
    cJSON_AddItemToObject(io_obj, "channels", channels);
}
