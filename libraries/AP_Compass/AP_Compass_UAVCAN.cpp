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

#if HAL_WITH_UAVCAN

#include "AP_Compass_UAVCAN.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

#define debug_mag_uavcan(level, fmt, args...) do { if ((level) <= AP_BoardConfig::get_can_debug()) { hal.console->printf(fmt, ##args); }} while (0)

// There is limitation to use only one UAVCAN magnetometer now.

/*
  constructor - registers instance at top Compass driver
 */
AP_Compass_UAVCAN::AP_Compass_UAVCAN(Compass &compass):
    AP_Compass_Backend(compass)
{
    if (hal.can_mgr != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr->get_UAVCAN();
        if (ap_uavcan != nullptr) {
            // Give time to receive some packets from CAN if baro sensor is present
            // This way it will get calibrated correctly
            _instance = register_compass();
            hal.scheduler->delay(1000);
            uint8_t listener_channel = ap_uavcan->register_mag_listener(this, 1);

            struct DeviceStructure {
                uint8_t bus_type : 3;
                uint8_t bus: 5;
                uint8_t address;
                uint8_t devtype;
            };
            union DeviceId {
                struct DeviceStructure devid_s;
                uint32_t devid;
            };
            union DeviceId d;

            d.devid_s.bus_type = 3;
            d.devid_s.bus = 0;
            d.devid_s.address = listener_channel;
            d.devid_s.devtype = 0;

            set_dev_id(_instance, d.devid);
            set_external(_instance, true);

            _sum.zero();
            _count = 0;

            accumulate();

            debug_mag_uavcan(2, "AP_Compass_UAVCAN loaded\n\r");
        }
    }

    _mag_baro = hal.util->new_semaphore();
}

AP_Compass_UAVCAN::~AP_Compass_UAVCAN()
{
    if (hal.can_mgr != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr->get_UAVCAN();
        if (ap_uavcan != nullptr) {
            ap_uavcan->remove_mag_listener(this);

            debug_mag_uavcan(2, "AP_Compass_UAVCAN destructed\n\r");
        }
    }
}

void AP_Compass_UAVCAN::read(void)
{
    // avoid division by zero if we haven't received any mag reports
    if (_count == 0) {
        return;
    }

    if (_mag_baro->take(0)) {
        _sum /= _count;

        publish_filtered_field(_sum, _instance);

        _sum.zero();
        _count = 0;
        _mag_baro->give();
    }
}

void AP_Compass_UAVCAN::handle_mag_msg(Vector3f &mag)
{
    Vector3f raw_field = mag * 1000.0;

    // rotate raw_field from sensor frame to body frame
    rotate_field(raw_field, _instance);

    _last_timestamp = AP_HAL::micros64();
    // publish raw_field (uncorrected point sample) for calibration use
    publish_raw_field(raw_field, (uint32_t) _last_timestamp, _instance);

    // correct raw_field for known errors
    correct_field(raw_field, _instance);

    if (_mag_baro->take(0)) {
        // accumulate into averaging filter
        _sum += raw_field;
        _count++;
        _mag_baro->give();
    }
}

#endif
