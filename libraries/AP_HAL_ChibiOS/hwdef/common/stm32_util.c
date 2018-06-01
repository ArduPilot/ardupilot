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

#include "stm32_util.h"
#include <stdint.h>
#include <stdio.h>

void stm32_timer_set_input_filter(stm32_tim_t *tim, uint8_t channel, uint8_t filter_mode)
{
    switch (channel) {
    case 0:
        tim->CCMR1 |= STM32_TIM_CCMR1_IC1F(filter_mode);
        break;
    case 1:
        tim->CCMR1 |= STM32_TIM_CCMR1_IC2F(filter_mode);
        break;
    case 2:
        tim->CCMR2 |= STM32_TIM_CCMR2_IC3F(filter_mode);
        break;
    case 3:
        tim->CCMR2 |= STM32_TIM_CCMR2_IC4F(filter_mode);
        break;
    }
}


/*
  invalidate data cache following a DMA transfer into memory.
 */
void dma_invalidate(void *buf, uint32_t size)
{
#if defined(STM32F7) && STM32_DMA_CACHE_HANDLING == TRUE
    if (((ptrdiff_t)buf) > (ptrdiff_t)0x20020000) {
        dmaBufferInvalidate(buf, size);
    }
#endif
}

/*
  flush data cache into RAM before a DMA transfer
 */
void dma_flush(const void *buf, uint32_t size)
{
#if defined(STM32F7) && STM32_DMA_CACHE_HANDLING == TRUE
    if (((ptrdiff_t)buf) > (ptrdiff_t)0x20020000) {
        dmaBufferFlush(buf, size);
    }
#endif
}

#if CH_DBG_ENABLE_STACK_CHECK == TRUE
void show_stack_usage(void)
{
  thread_t *tp;

  tp = chRegFirstThread();
  do {
      uint32_t stklimit = (uint32_t)tp->wabase;
      uint8_t *p = (uint8_t *)tp->wabase;
      while (*p == CH_DBG_STACK_FILL_VALUE) {
          p++;
      }
      uint32_t stack_left = ((uint32_t)p) - stklimit;
      printf("%s %u\n", tp->name, (unsigned)stack_left);
      tp = chRegNextThread(tp);
  } while (tp != NULL);
}
#endif
