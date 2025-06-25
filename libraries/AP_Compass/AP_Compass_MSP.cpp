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

#include "AP_Compass_MSP.h"

#if AP_COMPASS_MSP_ENABLED

#include <AP_HAL/AP_HAL.h>

AP_Compass_Backend *AP_Compass_MSP::probe(uint8_t _msp_instance)
{
    auto *ret = NEW_NOTHROW AP_Compass_MSP(_msp_instance);
    if (ret == nullptr) {
        return nullptr;
    }
    if (!ret->init()) {
        delete ret;
        return nullptr;
    }
    return ret;
}

bool AP_Compass_MSP::init()
{
    auto devid = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_MSP, 0, msp_instance, 0);
    if (!register_compass(devid)) {
        return false;
    }

    set_external(true);

    return true;
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
    drain_accumulated_samples();
}

#endif // AP_COMPASS_MSP_ENABLED

