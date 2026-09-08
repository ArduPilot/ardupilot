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
 * Code by @davidbuzz and Claude
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* On embedded ARM off_t is 32-bit; on native 64-bit Linux hosts it is 64-bit, so
 * anything crossing that boundary must not assume a width. */
#ifdef __linux__
#undef off_t
#define off_t int32_t
/* 'unix' is a predefined macro (=1) on Linux; undef it so it can be used
 * as a variable name in embedded-style code. */
#undef unix
#endif

/*
 * On Zephyr, DMA safety and bounce-buffering are handled transparently
 * by the kernel's DMA API and SDMMC driver. All memory is DMA-safe from
 * the ArduPilot filesystem layer's perspective.
 */
static inline bool mem_is_dma_safe(const void * /*addr*/, uint32_t /*size*/, bool /*filesystem_op*/)
{
    return true;
}
