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
