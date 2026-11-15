#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** GPIO46 Display 5V enable — boot'ta ON. */
void hexnet_disp_pwr_init(void);

/** Anlik ac/kapa (aktif-high). */
void hexnet_disp_pwr_set(bool on);

/** Hard reset: 5V OFF → wait_ms → ON. Cagri non-blocking (ayri task). */
void hexnet_disp_pwr_hard_reset(unsigned wait_ms);

#ifdef __cplusplus
}
#endif
