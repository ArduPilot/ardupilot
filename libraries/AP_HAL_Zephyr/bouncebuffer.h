/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Derived from AP_HAL_ChibiOS/hwdef/common/bouncebuffer.c, which carries the
 * same licence and the copyright of the ArduPilot authors.
 *
 * Zephyr port by @davidbuzz and Claude
 */
/* DMA bounce buffers: a transfer whose buffer the DMA engine cannot reach, or
 * that shares a cache line, is staged through one of these instead. */
#pragma once

#include <stdint.h>
#include <stdbool.h>

struct bouncebuffer_t {
    uint8_t *dma_buf;
    uint8_t *orig_buf;
    uint32_t size;
    bool busy;
};

#ifdef __cplusplus
extern "C" {
#endif

void bouncebuffer_init(struct bouncebuffer_t **bouncebuffer, uint32_t prealloc_bytes);
bool bouncebuffer_setup_read(struct bouncebuffer_t *bouncebuffer, uint8_t **buf, uint32_t size);
void bouncebuffer_finish_read(struct bouncebuffer_t *bouncebuffer, const uint8_t *buf, uint32_t size);
bool bouncebuffer_setup_write(struct bouncebuffer_t *bouncebuffer, const uint8_t **buf, uint32_t size);
void bouncebuffer_finish_write(struct bouncebuffer_t *bouncebuffer, const uint8_t *buf);
void bouncebuffer_abort(struct bouncebuffer_t *bouncebuffer);

/* is this buffer already in memory the DMA engine and CPU agree on? */
bool mem_is_dma_safe(const void *buf, uint32_t size);

#ifdef __cplusplus
}
#endif
