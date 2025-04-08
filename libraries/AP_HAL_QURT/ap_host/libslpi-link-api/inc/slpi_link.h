#ifndef SLPI_LINK_H
#define SLPI_LINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef void (*slpi_link_cb)(const uint8_t *data, uint32_t length_in_bytes);

int slpi_link_init(bool enable_debug_messages, slpi_link_cb callback, const char *library_name);
int slpi_link_get_time_offset(void);
int slpi_link_send(const uint8_t *data, uint32_t length_in_bytes);
void slpi_link_reset(void);

#ifdef __cplusplus
}
#endif

#endif // SLPI_LINK_H
