/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
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
 * Modified for use in AP_HAL by Andrew Tridgell and Siddharth Bharat Purohit
 */
/*
  this provides the default mcuconf.h for each board. Override values in hwdef.dat
 */
#pragma once

// include generated config
#include "hwdef.h"

#if defined(STM32F1)
#include "stm32f1_mcuconf.h"
#elif defined(STM32F4) || defined(STM32F7)
#include "stm32f47_mcuconf.h"
#elif defined(STM32H7)
#include "stm32h7_mcuconf.h"
#else
#error "Unsupported MCU"
#endif
