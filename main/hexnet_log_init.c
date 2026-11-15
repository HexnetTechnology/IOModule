#include "hexnet_log.h"

#include "esp_log.h"

void hexnet_log_init(void)
{
    esp_log_level_set("*", ESP_LOG_ERROR);

#if HEXNET_LOG_WIFI
    esp_log_level_set("HEXNET_WIFI", ESP_LOG_VERBOSE);
    esp_log_level_set("WiFiPortal", ESP_LOG_INFO);
#endif

#if HEXNET_LOG_MQTT
    /* INFO: baglanti/telemetry; TX dump her 5s UART'i boguyordu */
    esp_log_level_set("HEXNET_MQTT", ESP_LOG_INFO);
    esp_log_level_set("HEXNET_MQTT_DBG", ESP_LOG_WARN);
#endif

#if HEXNET_LOG_BLE
    esp_log_level_set("HEXNET_BLE", ESP_LOG_INFO);
#endif

#if HEXNET_LOG_OTA
    esp_log_level_set("HEXNET_OTA", ESP_LOG_INFO);
    esp_log_level_set("OtaManager", ESP_LOG_INFO);
    esp_log_level_set("APP_MAIN", ESP_LOG_INFO);
#endif

    esp_log_level_set("HEXNET_IO_SYNC", ESP_LOG_INFO);
    esp_log_level_set("HEXNET_CJSON", ESP_LOG_INFO);
    esp_log_level_set("HEXNET_APP", ESP_LOG_INFO);
    esp_log_level_set("TWAI Master", ESP_LOG_WARN);
    /* TX_ECHO spam MQTT task'i geciktirip dusmeye katkiyormus gibi */
    esp_log_level_set("CAN_BUS", ESP_LOG_WARN);
    esp_log_level_set("HEXNET_CAN_RX", ESP_LOG_WARN);
    esp_log_level_set("HEXNET_SENSORS", ESP_LOG_WARN);
    esp_log_level_set("HEXNET_MOTOR", ESP_LOG_WARN);
}
