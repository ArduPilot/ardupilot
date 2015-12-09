/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#if CONFIG_HAL_BOARD == HAL_BOARD_QURT
#include "AP_Compass_QURT.h"
#include <AP_InertialSensor/AP_InertialSensor_QURT.h>

extern const AP_HAL::HAL& hal;

// Public Methods //////////////////////////////////////////////////////////////

// constructor
AP_Compass_QURT::AP_Compass_QURT(Compass &compass):
    AP_Compass_Backend(compass)
{
}

// detect the sensor
AP_Compass_Backend *AP_Compass_QURT::detect(Compass &compass)
{
    AP_Compass_QURT *sensor = new AP_Compass_QURT(compass);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->init()) {
        delete sensor;
        return NULL;
    }
    return sensor;
}

bool AP_Compass_QURT::init(void)
{
    instance = register_compass();
    // publish a zero as a hack
    publish_filtered_field(Vector3f(), instance);
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Compass_QURT::timer_update, void));
    return true;
}

void AP_Compass_QURT::read(void)
{
    // avoid division by zero if we haven't received any mag reports
    if (count != 0) {
        sum /= count;
        publish_filtered_field(sum, instance);
        sum.zero();
        count = 0;
    }
}

void AP_Compass_QURT::timer_update(void)
{
    // cope the data
    struct mpu9x50_data data;
    if (mpu9250_mag_buffer == nullptr || !mpu9250_mag_buffer->pop(data)) {
        return;
    }

    last_timestamp = data.timestamp;
    
    Vector3f raw_field(data.mag_raw[0],
                       data.mag_raw[1],
                       -data.mag_raw[2]);
    
    // rotate raw_field from sensor frame to body frame
    rotate_field(raw_field, instance);

    // publish raw_field (uncorrected point sample) for calibration use
    publish_raw_field(raw_field, data.timestamp, instance);

    // correct raw_field for known errors
    correct_field(raw_field, instance);

    // publish raw_field (corrected point sample) for EKF use
    publish_unfiltered_field(raw_field, data.timestamp, instance);

    // accumulate into averaging filter
    sum += raw_field;
    count++;
}

#endif // CONFIG_HAL_BOARD
