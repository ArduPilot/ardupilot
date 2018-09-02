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
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#pragma once

#include "hwdef.h"

#ifndef HAL_BOARD_INIT_HOOK_DEFINE
#define HAL_BOARD_INIT_HOOK_DEFINE
#endif

#ifndef HAL_BOARD_INIT_HOOK_CALL
#define HAL_BOARD_INIT_HOOK_CALL
#endif

// default to interrupts on port D
#ifndef HAL_GPIO_INTERRUPT_PORT
#define HAL_GPIO_INTERRUPT_PORT EXT_MODE_GPIOD
#endif

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
  HAL_BOARD_INIT_HOOK_DEFINE
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

