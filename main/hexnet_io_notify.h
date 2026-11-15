#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Röle/dim/RGB/motor HW degisti — tek SoT fan-out (async):
 * cagiran baglamda sadece kuyruga yazar; CAN/BLE/MQTT kendi worker'larinda tuketilir.
 */
void hexnet_io_notify_init(void);
void hexnet_io_notify_state_changed(void);

#ifdef __cplusplus
}
#endif
