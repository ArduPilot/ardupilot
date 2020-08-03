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

/*
  Simulated i2c buses and devices
*/

#pragma once

#include <AP_Param/AP_Param.h>

#include "stdint.h"

namespace SITL {

class I2C {
public:
    I2C() {}

    // update i2c state
    void update(const class Aircraft &aircraft);

    int ioctl(uint8_t ioctl_type, void *data);

    // the following must be identical to AP_HAL_SITL/I2CDevice.h
#define I2C_M_RD 1
#define I2C_RDWR 0
    struct i2c_msg {
        uint8_t addr;
        uint8_t flags;
        uint8_t *buf;
        uint16_t len; // FIXME: what type should this be?
    };
    struct i2c_rdwr_ioctl_data {
        i2c_msg *msgs;
        uint8_t nmsgs;
    };
    // end "the following"

private:
    int ioctl_rdwr(i2c_rdwr_ioctl_data *data);

};

}
