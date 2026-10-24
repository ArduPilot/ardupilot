#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ardupilot_tusb_rx_cb_t)(void *arg);
typedef void (*ardupilot_tusb_line_state_cb_t)(bool dtr, bool rts, void *arg);

bool ardupilot_tusb_init(ardupilot_tusb_rx_cb_t rx_cb,
                         ardupilot_tusb_line_state_cb_t line_state_cb,
                         void *arg);
void ardupilot_tusb_deinit(void);
size_t ardupilot_tusb_read(uint8_t *buffer, size_t buffer_size);
size_t ardupilot_tusb_write(const uint8_t *buffer, size_t size);
esp_err_t ardupilot_tusb_flush(uint32_t timeout_ticks);
bool ardupilot_tusb_is_connected(void);
bool ardupilot_tusb_is_open(void);
bool ardupilot_tusb_is_initialized(void);
bool ardupilot_tusb_is_ready(void);
bool ardupilot_tusb_is_cdc_connected(void);
bool ardupilot_tusb_task_alive(void);
bool ardupilot_tusb_tx_pending(void);
size_t ardupilot_tusb_write_available(void);

#ifdef __cplusplus
}
#endif
