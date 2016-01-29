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

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_QFLIGHT

#include "AP_Compass_qflight.h"
#include <AP_HAL_Linux/qflight/qflight_util.h>
#include <AP_HAL_Linux/qflight/qflight_dsp.h>
#include "AP_Compass_qflight.h"
#include <unistd.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_Compass_QFLIGHT::AP_Compass_QFLIGHT(Compass &compass):
    AP_Compass_Backend(compass)
{
}

// detect the sensor
AP_Compass_Backend *AP_Compass_QFLIGHT::detect(Compass &compass)
{
    AP_Compass_QFLIGHT *sensor = new AP_Compass_QFLIGHT(compass);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->init()) {
        delete sensor;
        return NULL;
    }
    return sensor;
}

bool AP_Compass_QFLIGHT::init(void)
{
    instance = register_compass();
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Compass_QFLIGHT::timer_update, void));
    // give time for at least one sample
    hal.scheduler->delay(100);
    return true;
}

void AP_Compass_QFLIGHT::read(void)
{
    if (count > 0) {
        hal.scheduler->suspend_timer_procs();
        publish_filtered_field(sum/count, instance);
        sum.zero();
        count = 0;
        hal.scheduler->resume_timer_procs();
    }
}

void AP_Compass_QFLIGHT::timer_update(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - last_check_ms < 25) {
        return;
    }
    last_check_ms = now;

    if (magbuf == nullptr) {
        magbuf = QFLIGHT_RPC_ALLOCATE(DSPBuffer::MAG);
        if (magbuf == nullptr) {
            AP_HAL::panic("unable to allocate MAG buffer");
        }
    }
    int ret = qflight_get_mag_data((uint8_t *)magbuf, sizeof(*magbuf));
    if (ret != 0) {
        return;
    }
    uint64_t time_us = AP_HAL::micros64();
    for (uint16_t i=0; i<magbuf->num_samples; i++) {
        DSPBuffer::MAG::BUF &b = magbuf->buf[i];
        
        // get raw_field - sensor frame, uncorrected
        Vector3f raw_field(b.mag_raw[0], b.mag_raw[1], -b.mag_raw[2]);

        // rotate raw_field from sensor frame to body frame
        rotate_field(raw_field, instance);

        // publish raw_field (uncorrected point sample) for calibration use
        publish_raw_field(raw_field, time_us, instance);

        // correct raw_field for known errors
        correct_field(raw_field, instance);

        // publish raw_field (corrected point sample) for EKF use
        publish_unfiltered_field(raw_field, time_us, instance);

        // accumulate into averaging filter
        sum += raw_field;
        count++;
    }
}

#endif // CONFIG_HAL_BOARD_SUBTYPE

