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

//! @todo(srmainwaring) remove debug info
#include <GCS_MAVLink/GCS.h>

AP_Compass_DDS::AP_Compass_DDS()
{
    //! @todo(srmainwaring) ordering awkward here, baro registers first,
    // then uses the instance to generate the devid.
    auto devid = AP_HAL::Device::make_bus_id(
        AP_HAL::Device::BUS_TYPE_DDS, 0, 0/*_instance*/, DEVTYPE_DDS);

    register_compass(devid, _instance);
    set_dev_id(_instance, devid);
    set_external(_instance, true);

    //! @todo(srmainwaring) remove debug info
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Compass_DDS: instance: %d, devid: %d",
        _instance, devid);
}

void AP_Compass_DDS::handle_external(
    const AP_ExternalAHRS::mag_data_message_t &pkt)
{
    Vector3f field = pkt.field;
    accumulate_sample(field, _instance);

    // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Compass_DDS: field: (%f, %f, %f)",
    //     field.x, field.y, field.z);
}

void AP_Compass_DDS::read(void)
{
    drain_accumulated_samples(_instance);
}

#endif // AP_COMPASS_DDS_ENABLED
