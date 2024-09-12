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
 * Author: Oliver Walters / Currawong Engineering Pty Ltd
 */


#pragma once

#include <AP_CANManager/AP_CANManager.h>

#include "AP_PiccoloCAN_Device.h"
#include "piccolo_protocol/ECUPackets.h"

#define PICCOLO_CAN_ECU_ID_DEFAULT 0

#if HAL_PICCOLO_CAN_ENABLE

/*
 * Class representing an individual PiccoloCAN ECU
 */
class AP_PiccoloCAN_ECU : public AP_PiccoloCAN_Device
{
public:
    // TODO
};

#endif // HAL_PICCOLO_CAN_ENABLE
