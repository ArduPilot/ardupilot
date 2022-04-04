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
 * Author: Dmitry Ponomarev
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_LIBUAVCAN_V1_DRIVERS

#include "canard.h"


class UavcanFirstTransportIface
{
public:
    UavcanFirstTransportIface() {};
    void attach_can_iface(AP_HAL::CANIface* new_can_iface);
    bool receive(CanardFrame* can_frame);
    bool send(const CanardTxQueueItem* transfer);

private:
    AP_HAL::CANIface* _can_iface = nullptr;
};

#endif // HAL_ENABLE_LIBUAVCAN_V1_DRIVERS
