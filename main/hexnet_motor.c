#include "hexnet_motor.h"
#include "hexnet_io_profile.h"

#include "driver/gpio.h"
#include "hexnet_log.h"
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hexnet_canbus.h"
#include "hexnet_io_map.h"
#include "soc/gpio_struct.h"

static const char *TAG = "HEXNET_MOTOR";

int motorDataUpdateCounter = 0;

/** Son komuttan bu kadar ms geçince zorla dur (hold bırakılınca). */
#define MOTOR_KEEPALIVE_MS 400
#define MOTOR_COUNT 2

extern int numOfOutputs;

static int s_cmd[MOTOR_COUNT] = {0, 0};
static TickType_t s_rx_tick[MOTOR_COUNT];
static int s_last_applied[MOTOR_COUNT] = {-1, -1};
/** 0=kapalı, 1=tablet sanal Output, 2=CAN/BLE Motor / motor_set */
static uint8_t s_src[MOTOR_COUNT] = {0, 0};
/** Yalnızca tablet/MQTT motor sanal pinleri — Display röle maskesinden bağımsız. */
static uint16_t s_ble_mask = 0;

static int out_latched(gpio_num_t pin)
{
    if (pin < 32) {
        return (int)((GPIO.out >> pin) & 1u);
    }
    return (int)((GPIO.out1.val >> (pin - 32)) & 1u);
}

static bool motor_bridge_pins(int motor_index, uint8_t *fwd, uint8_t *rev)
{
    if (motor_index < 0 || motor_index >= MOTOR_COUNT || !fwd || !rev) {
        return false;
    }
    int base = numOfOutputs;
    if (base < 0) {
        base = 12;
    }
    if (base > 14) {
        base = 14;
    }
    const int f = base + motor_index * 2;
    const int r = f + 1;
    if (f < 0 || f > 14 || r > 15) {
        return false;
    }
    *fwd = (uint8_t)f;
    *rev = (uint8_t)r;
    return true;
}

bool hexnet_motor_is_virtual_relay(uint8_t output_index)
{
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        uint8_t f = 0;
        uint8_t r = 0;
        if (motor_bridge_pins(i, &f, &r) && (output_index == f || output_index == r)) {
            return true;
        }
    }
    return false;
}

uint16_t hexnet_motor_ble_ui_mask(void)
{
    return s_ble_mask;
}

static void motor_log_pins(int idx, const char *tag)
{
    if (idx == 0) {
        ESP_LOGW(TAG, "M1 %s pad EN%d=%d A%d=%d B%d=%d | outlat EN=%d A=%d B=%d",
                 tag,
                 (int)HEXNET_IO_MOTOR_EN_GPIO, gpio_get_level(HEXNET_IO_MOTOR_EN_GPIO),
                 (int)HEXNET_IO_MOTOR_A_GPIO, gpio_get_level(HEXNET_IO_MOTOR_A_GPIO),
                 (int)HEXNET_IO_MOTOR_B_GPIO, gpio_get_level(HEXNET_IO_MOTOR_B_GPIO),
                 out_latched(HEXNET_IO_MOTOR_EN_GPIO),
                 out_latched(HEXNET_IO_MOTOR_A_GPIO),
                 out_latched(HEXNET_IO_MOTOR_B_GPIO));
    } else {
        ESP_LOGW(TAG, "M2 %s pad EN%d=%d A%d=%d B%d=%d | outlat EN=%d A=%d B=%d",
                 tag,
                 (int)HEXNET_IO_MOTOR_EN_GPIO, gpio_get_level(HEXNET_IO_MOTOR_EN_GPIO),
                 (int)HEXNET_IO_MOTOR2_A_GPIO, gpio_get_level(HEXNET_IO_MOTOR2_A_GPIO),
                 (int)HEXNET_IO_MOTOR2_B_GPIO, gpio_get_level(HEXNET_IO_MOTOR2_B_GPIO),
                 out_latched(HEXNET_IO_MOTOR_EN_GPIO),
                 out_latched(HEXNET_IO_MOTOR2_A_GPIO),
                 out_latched(HEXNET_IO_MOTOR2_B_GPIO));
    }
}

static void motor_apply_hw(void)
{
    const int c1 = s_cmd[0];
    const int c2 = s_cmd[1];

    if (c1 == 1) {
        gpio_set_level(HEXNET_IO_MOTOR_A_GPIO, 1);
        gpio_set_level(HEXNET_IO_MOTOR_B_GPIO, 0);
    } else if (c1 == 2) {
        gpio_set_level(HEXNET_IO_MOTOR_A_GPIO, 0);
        gpio_set_level(HEXNET_IO_MOTOR_B_GPIO, 1);
    } else {
        gpio_set_level(HEXNET_IO_MOTOR_A_GPIO, 0);
        gpio_set_level(HEXNET_IO_MOTOR_B_GPIO, 0);
    }

    if (c2 == 1) {
        gpio_set_level(HEXNET_IO_MOTOR2_A_GPIO, 1);
        gpio_set_level(HEXNET_IO_MOTOR2_B_GPIO, 0);
    } else if (c2 == 2) {
        gpio_set_level(HEXNET_IO_MOTOR2_A_GPIO, 0);
        gpio_set_level(HEXNET_IO_MOTOR2_B_GPIO, 1);
    } else {
        gpio_set_level(HEXNET_IO_MOTOR2_A_GPIO, 0);
        gpio_set_level(HEXNET_IO_MOTOR2_B_GPIO, 0);
    }

    gpio_set_level(HEXNET_IO_MOTOR_EN_GPIO, (c1 != 0 || c2 != 0) ? 1 : 0);

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        if (s_last_applied[i] != s_cmd[i]) {
            const char *tag = (s_cmd[i] == 1) ? "forward" : (s_cmd[i] == 2) ? "backward" : "stop";
            motor_log_pins(i, tag);
            s_last_applied[i] = s_cmd[i];
        }
    }
}

static void motor_sync_from_ble_mask(void)
{
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        if (s_src[i] == 2u) {
            continue;
        }
        uint8_t f = 0;
        uint8_t r = 0;
        if (!motor_bridge_pins(i, &f, &r)) {
            continue;
        }
        const bool fwd_on = (s_ble_mask & (uint16_t)(1u << f)) != 0;
        const bool rev_on = (s_ble_mask & (uint16_t)(1u << r)) != 0;
        int cmd = 0;
        if (fwd_on && !rev_on) {
            cmd = 1;
        } else if (rev_on && !fwd_on) {
            cmd = 2;
        }
        s_cmd[i] = cmd;
        s_src[i] = (cmd != 0) ? 1u : 0u;
        if (i == 0) {
            set_motordata(cmd);
        }
    }
    motor_apply_hw();
}

void hexnet_motor_virtual_output_write(uint8_t output_index, bool on)
{
    if (!hexnet_io_profile_motor_enabled() || !hexnet_motor_is_virtual_relay(output_index)) {
        return;
    }
    if (on) {
        s_ble_mask |= (uint16_t)(1u << output_index);
    } else {
        s_ble_mask &= (uint16_t)~(1u << output_index);
    }
    ESP_LOGI(TAG, "BLE virt pin %u -> %s mask=0x%04X", (unsigned)output_index, on ? "ON" : "OFF",
             (unsigned)s_ble_mask);
    motor_sync_from_ble_mask();
}

void hexnet_motor_set_cmd(int motor_index, int cmd)
{
    if (motor_index < 0 || motor_index >= MOTOR_COUNT) {
        return;
    }
    if (cmd < 0 || cmd > 2) {
        cmd = 0;
    }
    s_cmd[motor_index] = cmd;
    s_src[motor_index] = (cmd != 0) ? 2u : 0u;
    if (motor_index == 0) {
        set_motordata(cmd);
    }
    hexnet_motor_note_rx_idx(motor_index);
    motor_apply_hw();
}

int hexnet_motor_get_cmd(int motor_index)
{
    if (motor_index < 0 || motor_index >= MOTOR_COUNT) {
        return 0;
    }
    return s_cmd[motor_index];
}

void hexnet_motor_forward(void)
{
    hexnet_motor_set_cmd(0, 1);
}

void hexnet_motor_backward(void)
{
    hexnet_motor_set_cmd(0, 2);
}

void hexnet_motor_stop(void)
{
    s_ble_mask = 0;
    hexnet_motor_set_cmd(0, 0);
    hexnet_motor_set_cmd(1, 0);
}

void hexnet_motor_note_rx_idx(int motor_index)
{
    if (motor_index < 0 || motor_index >= MOTOR_COUNT) {
        return;
    }
    s_rx_tick[motor_index] = xTaskGetTickCount();
    if (motor_index == 0) {
        motorDataUpdateCounter = 0;
    }
}

void hexnet_motor_note_rx(void)
{
    hexnet_motor_note_rx_idx(0);
}

void hexnet_motor_control_task(void *arg)
{
    (void)arg;
    const TickType_t now0 = xTaskGetTickCount();
    s_rx_tick[0] = now0;
    s_rx_tick[1] = now0;
    while (1) {
        if (!hexnet_io_profile_motor_enabled()) {
            s_cmd[0] = 0;
            s_cmd[1] = 0;
            s_src[0] = 0;
            s_src[1] = 0;
            s_ble_mask = 0;
            set_motordata(0);
            motor_apply_hw();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* Legacy M1: doğrudan set_motordata (tablet hold değilse) */
        const int legacy = get_motorData();
        if (s_src[0] != 1u && legacy != s_cmd[0]) {
            s_cmd[0] = (legacy >= 0 && legacy <= 2) ? legacy : 0;
            s_src[0] = (s_cmd[0] != 0) ? 2u : 0u;
            if (s_cmd[0] != 0) {
                hexnet_motor_note_rx_idx(0);
            }
        }

        /* Tablet sanal maske hold — Display simoutputs/röle ile karışmaz */
        for (int i = 0; i < MOTOR_COUNT; ++i) {
            if (s_src[i] == 2u) {
                continue;
            }
            uint8_t f = 0;
            uint8_t r = 0;
            if (!motor_bridge_pins(i, &f, &r)) {
                continue;
            }
            const bool fwd_on = (s_ble_mask & (uint16_t)(1u << f)) != 0;
            const bool rev_on = (s_ble_mask & (uint16_t)(1u << r)) != 0;
            int cmd = 0;
            if (fwd_on && !rev_on) {
                cmd = 1;
            } else if (rev_on && !fwd_on) {
                cmd = 2;
            }
            s_cmd[i] = cmd;
            s_src[i] = (cmd != 0) ? 1u : 0u;
            if (i == 0) {
                set_motordata(cmd);
            }
        }

        const TickType_t now = xTaskGetTickCount();
        for (int i = 0; i < MOTOR_COUNT; ++i) {
            if (s_src[i] != 2u || s_cmd[i] == 0) {
                continue;
            }
            const uint32_t idle_ms = (uint32_t)((now - s_rx_tick[i]) * portTICK_PERIOD_MS);
            if (idle_ms >= MOTOR_KEEPALIVE_MS) {
                ESP_LOGW(TAG, "M%d keepalive timeout %ums — stop", i + 1, (unsigned)idle_ms);
                s_cmd[i] = 0;
                s_src[i] = 0;
                if (i == 0) {
                    set_motordata(0);
                }
            }
        }
        motor_apply_hw();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
