/*
  Battery SMBus PX4 driver
*/
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor_SMBus.h"

class AP_BattMonitor_SMBus_PX4 : public AP_BattMonitor_SMBus
{
public:
    // Constructor
    AP_BattMonitor_SMBus_PX4(AP_BattMonitor &mon, uint8_t instance, AP_BattMonitor::BattMonitor_State &mon_state);

    /// init
    void init();

    /// read - read the battery voltage and current
    void read();

private:
    int         _batt_sub;          // orb subscription description
    int         _batt_fd;           // file descriptor
    bool        _capacity_updated;  // capacity info read
};
