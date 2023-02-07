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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

class AP_CANManager;
class AP_CANDriver
{
public:

    friend class AP_CANManager;

    // init method for protocol drivers, specify driver index and if filters
    // are to be enabled
    virtual void init(uint8_t driver_index, bool enable_filters) = 0;

    // link protocol drivers with interfaces by adding reference to CANIface
    virtual bool add_interface(AP_HAL::CANIface* can_iface) = 0;

};
