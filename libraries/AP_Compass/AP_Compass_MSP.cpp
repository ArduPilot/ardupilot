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

#include <AP_HAL/AP_HAL.h>
#include "AP_Compass_MSP.h"

#if HAL_MSP_COMPASS_ENABLED

AP_Compass_MSP::AP_Compass_MSP(uint8_t _msp_instance)
{
    msp_instance = _msp_instance;

    auto devid = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_MSP, 0, _msp_instance, 0);
    register_compass(devid, instance);

    set_dev_id(instance, devid);
    set_external(instance, true);
    set_rotation(instance, ROTATION_NONE);
}

void AP_Compass_MSP::handle_msp(const MSP::msp_compass_data_message_t &pkt)
{
    if (pkt.instance != msp_instance) {
        return;
    }
    Vector3f field(pkt.magX, pkt.magY, pkt.magZ);
    accumulate_sample(field, instance);
}

void AP_Compass_MSP::read(void)
{
    drain_accumulated_samples(instance);
}

#endif // HAL_MSP_COMPASS_ENABLED

