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
/* Ported from AP_HAL_ChibiOS/hwdef/common/bouncebuffer.c. */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR

#include "bouncebuffer.h"
#include <AP_HAL/Util.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>

extern const AP_HAL::HAL& hal;

/* cache line rounding, exactly as ChibiOS's (size+31)&~31 but taking the line
   size from Kconfig rather than hardcoding 31 */
#ifdef CONFIG_DCACHE_LINE_SIZE
#define AP_CACHE_LINE CONFIG_DCACHE_LINE_SIZE
#else
#define AP_CACHE_LINE 32
#endif
#define CACHE_ROUND(sz) (((sz) + (AP_CACHE_LINE - 1)) & ~(AP_CACHE_LINE - 1))

static inline void cache_invalidate(void *buf, uint32_t size)
{
#if defined(CONFIG_DCACHE) && defined(CONFIG_CACHE_MANAGEMENT)
    sys_cache_data_invd_range(buf, CACHE_ROUND(size));
#else
    (void)buf; (void)size;
#endif
}

static inline void cache_flush(const void *buf, uint32_t size)
{
#if defined(CONFIG_DCACHE) && defined(CONFIG_CACHE_MANAGEMENT)
    sys_cache_data_flush_range((void *)buf, CACHE_ROUND(size));
#else
    (void)buf; (void)size;
#endif
}

static void *dma_alloc(uint32_t size)
{
    return hal.util->malloc_type(size, AP_HAL::Util::MEM_DMA_SAFE);
}

static void dma_free(void *p, uint32_t size)
{
    hal.util->free_type(p, size, AP_HAL::Util::MEM_DMA_SAFE);
}

void bouncebuffer_init(struct bouncebuffer_t **bouncebuffer, uint32_t prealloc_bytes)
{
    *bouncebuffer = (struct bouncebuffer_t *)calloc(1, sizeof(struct bouncebuffer_t));
    if (*bouncebuffer == nullptr) {
        AP_HAL::panic("Failed to allocate bouncebuffer");
    }
    if (prealloc_bytes) {
        (*bouncebuffer)->dma_buf = (uint8_t *)dma_alloc(prealloc_bytes);
        if (!(*bouncebuffer)->dma_buf) {
            AP_HAL::panic("Failed to allocate bouncebuffer DMA buffer");
        }
        (*bouncebuffer)->size = prealloc_bytes;
    }
}

/*
  setup for reading from a device into memory, allocating a bouncebuffer if
  needed
 */
bool bouncebuffer_setup_read(struct bouncebuffer_t *bouncebuffer, uint8_t **buf, uint32_t size)
{
    /* nothing to do for an empty or absent buffer. ArduPilot issues plenty of
       half-duplex transfers with recv=nullptr/recv_len=0 (and write-only ones
       with send=nullptr), and without this the zero-size allocation below fails
       and aborts the whole transfer. */
    if (*buf == nullptr || size == 0) {
        return true;
    }
    if (!bouncebuffer || mem_is_dma_safe(*buf, size)) {
        /* invalidate so we know the state in bouncebuffer_finish_read */
        cache_invalidate(*buf, size);
        return true;
    }
    if (bouncebuffer->busy) {
        return false;
    }
    bouncebuffer->orig_buf = *buf;
    if (bouncebuffer->size < size) {
        if (bouncebuffer->size > 0) {
            dma_free(bouncebuffer->dma_buf, bouncebuffer->size);
        }
        bouncebuffer->dma_buf = (uint8_t *)dma_alloc(size);
        if (!bouncebuffer->dma_buf) {
            bouncebuffer->size = 0;
            return false;
        }
        bouncebuffer->size = size;
    }
    *buf = bouncebuffer->dma_buf;
    cache_invalidate(*buf, size);
    bouncebuffer->busy = true;
    return true;
}

/*
  finish a read operation
 */
void bouncebuffer_finish_read(struct bouncebuffer_t *bouncebuffer, const uint8_t *buf, uint32_t size)
{
    if (buf == nullptr || size == 0) {
        return;
    }
    /* FLUSH, not invalidate - ChibiOS's reasoning verbatim: an invalidate would drop
     * a dirty line the CPU has not written back yet. */
    cache_flush(buf, size);
    if (bouncebuffer && buf == bouncebuffer->dma_buf) {
        if (bouncebuffer->orig_buf) {
            memcpy(bouncebuffer->orig_buf, buf, size);
        }
        bouncebuffer->busy = false;
    }
}

/*
  setup for writing from memory to a device, allocating a bouncebuffer if needed
 */
bool bouncebuffer_setup_write(struct bouncebuffer_t *bouncebuffer, const uint8_t **buf, uint32_t size)
{
    /* nothing to do for an empty or absent buffer. ArduPilot issues plenty of
       half-duplex transfers with recv=nullptr/recv_len=0 (and write-only ones
       with send=nullptr), and without this the zero-size allocation below fails
       and aborts the whole transfer. */
    if (*buf == nullptr || size == 0) {
        return true;
    }
    if (!bouncebuffer || mem_is_dma_safe(*buf, size)) {
        /* flush pending CPU writes to memory before the DMA reads it */
        cache_flush(*buf, size);
        return true;
    }
    if (bouncebuffer->busy) {
        return false;
    }
    if (bouncebuffer->size < size) {
        if (bouncebuffer->size > 0) {
            dma_free(bouncebuffer->dma_buf, bouncebuffer->size);
        }
        bouncebuffer->dma_buf = (uint8_t *)dma_alloc(size);
        if (!bouncebuffer->dma_buf) {
            bouncebuffer->size = 0;
            return false;
        }
        bouncebuffer->size = size;
    }
    if (*buf) {
        memcpy(bouncebuffer->dma_buf, *buf, size);
    }
    *buf = bouncebuffer->dma_buf;
    cache_flush(*buf, size);
    bouncebuffer->busy = true;
    return true;
}

/*
  finish a write operation
 */
void bouncebuffer_finish_write(struct bouncebuffer_t *bouncebuffer, const uint8_t *buf)
{
    if (bouncebuffer && buf == bouncebuffer->dma_buf) {
        bouncebuffer->busy = false;
    }
}

void bouncebuffer_abort(struct bouncebuffer_t *bouncebuffer)
{
    if (bouncebuffer) {
        bouncebuffer->busy = false;
    }
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
