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
 */
#pragma once

#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void stm32_timer_set_input_filter(stm32_tim_t *tim, uint8_t channel, uint8_t filter_mode);
void stm32_timer_set_channel_input(stm32_tim_t *tim, uint8_t channel, uint8_t input_source);

#if CH_DBG_ENABLE_STACK_CHECK == TRUE
// print stack usage
void show_stack_usage(void);
#endif

// allocation functions in malloc.c    
size_t mem_available(void);
void *malloc_ccm(size_t size);
void *malloc_dtcm(size_t size);
void *malloc_dma(size_t size);

// UTC system clock handling    
void stm32_set_utc_usec(uint64_t time_utc_usec);
uint64_t stm32_get_utc_usec(void);

// hook for FAT timestamps    
uint32_t get_fattime(void);
    
#ifdef __cplusplus
}
#endif

