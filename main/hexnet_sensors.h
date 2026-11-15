#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HEXNET_SENSOR_COMM_OK    0
#define HEXNET_SENSOR_COMM_LOST  1

extern int hexnet_sensor_rs485_temperature;
extern int hexnet_sensor_rs485_humidity;
extern int hexnet_sensor_comm_status;

void hexnet_dht_task(void *arg);
void hexnet_sensor_rs485_task(void *arg);
/** MQTT/BLE heap yemeden once UART1 kur — yoksa RS485 sonsuza heap bekler. */
esp_err_t hexnet_sensor_rs485_uart_early_init(void);

#ifdef __cplusplus
}
#endif
