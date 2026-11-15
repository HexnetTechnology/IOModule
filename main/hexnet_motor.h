#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void hexnet_motor_forward(void);
void hexnet_motor_backward(void);
void hexnet_motor_stop(void);

/** motor_index 0=M1, 1=M2; cmd 0=stop 1=fwd 2=rev */
void hexnet_motor_set_cmd(int motor_index, int cmd);
int hexnet_motor_get_cmd(int motor_index);

/** CAN/BLE Motor write — keepalive. */
void hexnet_motor_note_rx(void);
void hexnet_motor_note_rx_idx(int motor_index);

/**
 * Tablet Motor UI: lighting rölelerinden sonraki sanal FWD/REV çiftleri.
 * Motor1 = numOfO+0/+1, Motor2 = numOfO+2/+3 (0..15).
 * Bunlar SR rölesi değildir — Display 0x720 röle yazımından ayrı tutulur.
 */
bool hexnet_motor_is_virtual_relay(uint8_t output_index);
/** Tablet/MQTT sanal pin yazımı → yalnızca H-bridge (simoutputs/SR dokunulmaz). */
void hexnet_motor_virtual_output_write(uint8_t output_index, bool on);
/** BLE outDB için sanal motor bitleri (röle maskesine OR). */
uint16_t hexnet_motor_ble_ui_mask(void);

void hexnet_motor_control_task(void *arg);

#ifdef __cplusplus
}
#endif
