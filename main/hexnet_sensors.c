#include "hexnet_sensors.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_heap_caps.h"
#include "hexnet_log.h"
#define LOG_LOCAL_LEVEL HEXNET_LOG_LEVEL_OTHER
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hexnet_canbus.h"
#include "hexnet_io_map.h"

static const char *TAG = "HEXNET_SENSORS";

#define RS485_START_DELAY_MS    500
#define RS485_INSTALL_MIN_HEAP  4096
#define RS485_INSTALL_RETRIES   16
#define RS485_LINE_MAX          128

int hexnet_sensor_rs485_temperature = 0;
int hexnet_sensor_rs485_humidity = 0;
int hexnet_sensor_comm_status = HEXNET_SENSOR_COMM_LOST;

static int64_t s_last_rs485_rx_us;
static bool s_rs485_uart_ready;

esp_err_t hexnet_sensor_rs485_uart_early_init(void);

static esp_err_t rs485_uart_init(void)
{
    if (s_rs485_uart_ready) {
        return ESP_OK;
    }

    gpio_config_t rs485_en = {
        .pin_bit_mask = (1ULL << HEXNET_IO_RS485_EN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&rs485_en);
    gpio_set_level(HEXNET_IO_RS485_EN_GPIO, 0); /* receive mode */

    uart_config_t uart_cfg = {
        .baud_rate = HEXNET_IO_UART_RS485_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(
        HEXNET_IO_UART_RS485_PORT, HEXNET_IO_UART_RS485_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err == ESP_ERR_INVALID_STATE) {
        err = ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    err = uart_param_config(HEXNET_IO_UART_RS485_PORT, &uart_cfg);
    if (err != ESP_OK) {
        uart_driver_delete(HEXNET_IO_UART_RS485_PORT);
        return err;
    }

    err = uart_set_pin(
        HEXNET_IO_UART_RS485_PORT,
        HEXNET_IO_UART_RS485_TX_GPIO,
        HEXNET_IO_UART_RS485_RX_GPIO,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        uart_driver_delete(HEXNET_IO_UART_RS485_PORT);
        return err;
    }

    s_rs485_uart_ready = true;
    return ESP_OK;
}

esp_err_t hexnet_sensor_rs485_uart_early_init(void)
{
    esp_err_t err = rs485_uart_init();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "RS485 UART early-init OK heap=%lu",
                 (unsigned long)esp_get_free_heap_size());
    } else {
        ESP_LOGW(TAG, "RS485 UART early-init fail %s heap=%lu",
                 esp_err_to_name(err), (unsigned long)esp_get_free_heap_size());
    }
    return err;
}

/**
 * DHT kullanilmiyor — iklim SoT RS485.
 * Sembol referansi icin stub (app bring-up DHT task baslatmaz).
 */
void hexnet_dht_task(void *arg)
{
    (void)arg;
    ESP_LOGW(TAG, "DHT task kapali (iklim=RS485)");
    vTaskDelete(NULL);
}

static void rs485_try_parse_line(const char *line)
{
    int t = 0;
    int h = 0;
    /* Esnek: $TEMP,25;HUM,60#  veya  TEMP,25;HUM,60 */
    if (sscanf(line, "$TEMP,%d;HUM,%d#", &t, &h) == 2 ||
        sscanf(line, "TEMP,%d;HUM,%d", &t, &h) == 2 ||
        sscanf(line, "$TEMP,%d;HUM,%d", &t, &h) == 2 ||
        sscanf(line, "T:%d H:%d", &t, &h) == 2 ||
        sscanf(line, "T=%d,H=%d", &t, &h) == 2) {
        hexnet_sensor_rs485_temperature = t;
        hexnet_sensor_rs485_humidity = h;
        set_sensorTemp(t);
        set_sensorHumidity(h);
        s_last_rs485_rx_us = esp_timer_get_time();
        hexnet_sensor_comm_status = HEXNET_SENSOR_COMM_OK;
        static int s_log_div = 0;
        if ((++s_log_div % 10) == 1) {
            ESP_LOGE(TAG, "RS485 iklim T=%dC H=%d%%", t, h);
        }
    }
}

void hexnet_sensor_rs485_task(void *arg)
{
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(RS485_START_DELAY_MS));

    esp_err_t init_err = ESP_FAIL;
    for (int attempt = 0; attempt < RS485_INSTALL_RETRIES; attempt++) {
        const uint32_t heap = esp_get_free_heap_size();
        if (heap < RS485_INSTALL_MIN_HEAP) {
            ESP_LOGE(TAG, "RS485 bekliyor: heap=%lu", (unsigned long)heap);
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }
        init_err = rs485_uart_init();
        if (init_err == ESP_OK) {
            ESP_LOGE(TAG, "RS485 UART hazir (heap=%lu)", (unsigned long)heap);
            break;
        }
        ESP_LOGE(TAG, "RS485 UART kurulumu basarisiz (%s)", esp_err_to_name(init_err));
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    if (init_err != ESP_OK) {
        ESP_LOGE(TAG, "RS485 devre disi; UART kurulamadi");
        hexnet_sensor_comm_status = HEXNET_SENSOR_COMM_LOST;
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(60000));
        }
    }

    uint8_t *rx = malloc(HEXNET_IO_UART_RS485_BUF_SIZE);
    if (!rx) {
        ESP_LOGE(TAG, "RS485 RX malloc failed");
        vTaskDelete(NULL);
        return;
    }

    char line[RS485_LINE_MAX];
    size_t line_len = 0;
    s_last_rs485_rx_us = esp_timer_get_time();

    uint32_t rx_bytes_window = 0;
    int64_t last_rx_diag_us = esp_timer_get_time();

    while (1) {
        int len = uart_read_bytes(
            HEXNET_IO_UART_RS485_PORT, rx, HEXNET_IO_UART_RS485_BUF_SIZE - 1, pdMS_TO_TICKS(100));

        if (len > 0) {
            rx_bytes_window += (uint32_t)len;
            static int s_raw_log = 0;
            if ((++s_raw_log % 20) == 1) {
                int show = len > 48 ? 48 : len;
                char hex[160];
                size_t hp = 0;
                for (int i = 0; i < show && hp + 4 < sizeof(hex); i++) {
                    hp += (size_t)snprintf(hex + hp, sizeof(hex) - hp, "%02X ", rx[i]);
                }
                ESP_LOGE(TAG, "RS485 raw len=%d %s", len, hex);
            }
            for (int i = 0; i < len; i++) {
                char c = (char)rx[i];
                if (c == '\r' || c == '\n') {
                    if (line_len > 5) {
                        rs485_try_parse_line(line);
                    }
                    line_len = 0;
                    line[0] = '\0';
                    continue;
                }
                if (line_len + 1 >= sizeof(line)) {
                    line_len = 0;
                }
                line[line_len++] = c;
                line[line_len] = '\0';

                /* Tam paket: $ ... # */
                if (c == '#' && line_len > 5) {
                    rs485_try_parse_line(line);
                    line_len = 0;
                    line[0] = '\0';
                } else if (c == '$' && line_len > 1) {
                    /* Yeni paket basladi — onceki yarim satiri at */
                    line[0] = '$';
                    line_len = 1;
                    line[1] = '\0';
                }
            }
        }

        if ((esp_timer_get_time() - s_last_rs485_rx_us) > 10 * 1000000LL) {
            if (hexnet_sensor_comm_status != HEXNET_SENSOR_COMM_LOST) {
                ESP_LOGW(TAG, "RS485 iklim timeout — sample temizle");
            }
            hexnet_sensor_comm_status = HEXNET_SENSOR_COMM_LOST;
            set_sensorTemp(255);
            set_sensorHumidity(255);
        }

        if ((esp_timer_get_time() - last_rx_diag_us) > 15 * 1000000LL) {
            ESP_LOGE(
                TAG,
                "RS485 diag: rx_bytes=%lu comm=%s T=%d H=%d",
                (unsigned long)rx_bytes_window,
                (hexnet_sensor_comm_status == HEXNET_SENSOR_COMM_OK) ? "OK" : "LOST",
                hexnet_sensor_rs485_temperature,
                hexnet_sensor_rs485_humidity);
            rx_bytes_window = 0;
            last_rx_diag_us = esp_timer_get_time();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
