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

#include "CANInternal.h"
#include "CANClock.h"


extern const AP_HAL::HAL& hal;
using namespace ChibiOS;
namespace ChibiOS_CAN {

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
    return (can_helper.init(bitrate, ChibiOS_CAN::CanIface::OperatingMode::NormalMode, can_number) == 0);
}

bool CANManager::is_initialized()
{
    return initialized_;
}

void CANManager::initialized(bool val)
{
    initialized_ = val;
}

#endif //HAL_WITH_UAVCAN
