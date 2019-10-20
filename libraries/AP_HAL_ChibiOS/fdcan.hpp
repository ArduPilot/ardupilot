/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Pavel Kirienko
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
 * Code by Siddharth Bharat Purohit
 */

#pragma once

#include "AP_HAL_ChibiOS.h"

#if HAL_WITH_UAVCAN

#if defined(STM32H7XX)

#include <uavcan/uavcan.hpp>
#include <stdint.h>

#ifndef UAVCAN_CPP_VERSION
# error UAVCAN_CPP_VERSION
#endif

#if UAVCAN_CPP_VERSION < UAVCAN_CPP11
// #undef'ed at the end of this file
# define constexpr const
#endif

namespace ChibiOS_CAN
{
namespace fdcan
{
typedef FDCAN_GlobalTypeDef CanType;

constexpr unsigned long IDE = (0x40000000U); // Identifier Extension
constexpr unsigned long STID_MASK = (0x1FFC0000U); // Standard Identifier Mask
constexpr unsigned long EXID_MASK = (0x1FFFFFFFU); // Extended Identifier Mask
constexpr unsigned long RTR       = (0x20000000U); // Remote Transmission Request
constexpr unsigned long DLC_MASK  = (0x000F0000U); // Data Length Code

/**
 * CANx register sets
 */
CanType* const Can[UAVCAN_STM32_NUM_IFACES] = {
    reinterpret_cast<CanType*>(FDCAN1_BASE)
#if UAVCAN_STM32_NUM_IFACES > 1
    ,
    reinterpret_cast<CanType*>(FDCAN2_BASE)
#endif
};
}
}

#if UAVCAN_CPP_VERSION < UAVCAN_CPP11
# undef constexpr
#endif
#endif //defined(STM32H7XX)
#endif //HAL_WITH_UAVCAN
