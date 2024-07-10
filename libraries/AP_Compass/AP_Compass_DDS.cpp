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
#include "AP_Compass_DDS.h"

#if AP_COMPASS_DDS_ENABLED

AP_Compass_DDS::AP_Compass_DDS()
{
    // _instance is not set until compass is registered
    auto dev_id = AP_HAL::Device::make_bus_id(
                      AP_HAL::Device::BUS_TYPE_DDS, 0, 0/*_instance*/, DEVTYPE_DDS);

    register_compass(dev_id, _instance);
    set_dev_id(_instance, dev_id);
    set_external(_instance, true);
}

void AP_Compass_DDS::handle_external(
    const AP_ExternalAHRS::mag_data_message_t &pkt)
{
    //! @todo(srmainwaring) pkt does not have instance to cf this->_instance
    Vector3f field = pkt.field;
    accumulate_sample(field, _instance);
}

void AP_Compass_DDS::read(void)
{
    drain_accumulated_samples(_instance);
}

#endif // AP_COMPASS_DDS_ENABLED
