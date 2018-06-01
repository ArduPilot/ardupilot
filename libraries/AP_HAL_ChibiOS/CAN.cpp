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
 * Based on stm32 can driver by Pavel Kirienko
 */

#include "CAN.h"

#if HAL_WITH_UAVCAN

#include <uavcan_stm32/../../src/internal.hpp>

using namespace ChibiOS;
using namespace uavcan_stm32;

extern const AP_HAL::HAL& hal;

namespace uavcan_stm32 {

uint64_t clock::getUtcUSecFromCanInterrupt()
{
    return AP_HAL::micros64();
}

uavcan::MonotonicTime clock::getMonotonic()
{
    return uavcan::MonotonicTime::fromUSec(AP_HAL::micros64());
}

}

bool CANManager::begin(uint32_t bitrate, uint8_t can_number)
{
    if (can_helper.init(bitrate, CanIface::OperatingMode::NormalMode, can_number) == 0) {
        bitrate_ = bitrate;
        initialized_ = true;
    }
    return initialized_;
}

bool CANManager::is_initialized()
{
    return initialized_;
}

void CANManager::initialized(bool val)
{
    initialized_ = val;
}

AP_UAVCAN *CANManager::get_UAVCAN(void)
{
    return p_uavcan;
}

void CANManager::set_UAVCAN(AP_UAVCAN *uavcan)
{
    p_uavcan = uavcan;
}

void CANManager::_timer_tick()
{
    if (!initialized_) return;

    if (p_uavcan != nullptr) {
        p_uavcan->do_cyclic();
    }
}

#endif //HAL_WITH_UAVCAN
