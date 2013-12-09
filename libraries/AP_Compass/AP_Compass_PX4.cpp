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

/*
 *       AP_Compass_PX4.cpp - Arduino Library for PX4 magnetometer
 *
 */


#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "AP_Compass_PX4.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <stdio.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;


// Public Methods //////////////////////////////////////////////////////////////

bool AP_Compass_PX4::init(void)
{
	_mag_fd[0] = open(MAG_DEVICE_PATH, O_RDONLY);
	if (_mag_fd[0] < 0) {
        hal.console->printf("Unable to open " MAG_DEVICE_PATH "\n");
        return false;
	}

	_mag_fd[1] = open(MAG_DEVICE_PATH "1", O_RDONLY);
	if (_mag_fd[1] >= 0) {
        _num_instances = 2;
	} else {
        _num_instances = 1;
    }

    for (uint8_t i=0; i<_num_instances; i++) {
        // average over up to 20 samples
        if (ioctl(_mag_fd[i], SENSORIOCSQUEUEDEPTH, 20) != 0) {
            hal.console->printf("Failed to setup compass queue\n");
            return false;                
        }

        // remember if the compass is external
        _is_external[i] = (ioctl(_mag_fd[i], MAGIOCGEXTERNAL, 0) > 0);
        if (_is_external[0]) {
            hal.console->printf("Using external compass[%u]\n", (unsigned)i);
        }
        _count[0] = 0;
        _sum[i].zero();
        _healthy[i] = false;
    }

    // give the driver a chance to run, and gather one sample
    hal.scheduler->delay(40);
    accumulate();
    if (_count[0] == 0) {
        hal.console->printf("Failed initial compass accumulate\n");        
    }
    return true;
}

bool AP_Compass_PX4::read(void)
{
    // try to accumulate one more sample, so we have the latest data
    accumulate();

    // consider the compass healthy if we got a reading in the last 0.2s
    for (uint8_t i=0; i<_num_instances; i++) {
        _healthy[i] = (hrt_absolute_time() - _last_timestamp[i] < 200000);
    }

    for (uint8_t i=0; i<_num_instances; i++) {
        _sum[i] /= _count[i];
        _sum[i] *= 1000;

        // apply default board orientation for this compass type. This is
        // a noop on most boards
        _sum[i].rotate(MAG_BOARD_ORIENTATION);

        // override any user setting of COMPASS_EXTERNAL 
        _external.set(_is_external[0]);

        if (_is_external[i]) {
            // add user selectable orientation
            _sum[i].rotate((enum Rotation)_orientation.get());
        } else {
            // add in board orientation from AHRS
            _sum[i].rotate(_board_orientation);
        }

        _sum[i] += _offset[i].get();

        // apply motor compensation
        if (_motor_comp_type != AP_COMPASS_MOT_COMP_DISABLED && _thr_or_curr != 0.0f) {
            _motor_offset[i] = _motor_compensation[i].get() * _thr_or_curr;
            _sum[i] += _motor_offset[i];
        } else {
            _motor_offset[i].zero();
        }
    
        _field[i] = _sum[i];
    
        _sum[i].zero();
        _count[i] = 0;
    }

    last_update = _last_timestamp[0];
    
    return _healthy[0];
}

void AP_Compass_PX4::accumulate(void)
{
    struct mag_report mag_report;
    for (uint8_t i=0; i<_num_instances; i++) {
        while (::read(_mag_fd[i], &mag_report, sizeof(mag_report)) == sizeof(mag_report) &&
               mag_report.timestamp != _last_timestamp[i]) {
            _sum[i] += Vector3f(mag_report.x, mag_report.y, mag_report.z);
            _count[i]++;
            _last_timestamp[i] = mag_report.timestamp;
        }
    }
}

uint8_t AP_Compass_PX4::_get_primary(void) const
{
    for (uint8_t i=0; i<_num_instances; i++) {
        if (_healthy[i]) return i;
    }    
    return 0;
}

#endif // CONFIG_HAL_BOARD
