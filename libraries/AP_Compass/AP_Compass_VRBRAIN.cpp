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
 *       AP_Compass_VRBRAIN.cpp - Arduino Library for VRBRAIN magnetometer
 *
 */


#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include "AP_Compass_VRBRAIN.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_device.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <stdio.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;


// Public Methods //////////////////////////////////////////////////////////////

bool AP_Compass_VRBRAIN::init(void)
{
	_mag_fd[0] = open(MAG_DEVICE_PATH, O_RDONLY);
    _mag_fd[1] = open(MAG_DEVICE_PATH "1", O_RDONLY);
    _num_instances = 0;
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if (_mag_fd[i] >= 0) {
            _num_instances = i+1;
        }
    }
    if (_num_instances == 0) {
        hal.console->printf("Unable to open " MAG_DEVICE_PATH "\n");
        return false;
	}

    for (uint8_t i=0; i<_num_instances; i++) {
        // get device id
        _dev_id[i] = ioctl(_mag_fd[i], DEVIOCGDEVICEID, 0);

        // average over up to 20 samples
        if (ioctl(_mag_fd[i], SENSORIOCSQUEUEDEPTH, 20) != 0) {
            hal.console->printf("Failed to setup compass queue\n");
            return false;                
        }

        // remember if the compass is external
        _external[i] = (ioctl(_mag_fd[i], MAGIOCGEXTERNAL, 0) > 0);
#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V45)
		//deal with situations where user has cut internal mag on VRBRAIN 4.5 
		//and uses only one external mag attached to the internal I2C bus
        bool external_tmp = _external[i];
        if (!_external[i].load()) {
            _external[i] = external_tmp;
        }
#endif
        if (_external[i]) {
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

bool AP_Compass_VRBRAIN::read(void)
{
    // try to accumulate one more sample, so we have the latest data
    accumulate();

    // consider the compass healthy if we got a reading in the last 0.2s
    for (uint8_t i=0; i<_num_instances; i++) {
        _healthy[i] = (hal.scheduler->micros64() - _last_timestamp[i] < 200000);
    }

    for (uint8_t i=0; i<_num_instances; i++) {
        // avoid division by zero if we haven't received any mag reports
        if (_count[i] == 0) continue;

        _sum[i] /= _count[i];
        _sum[i] *= 1000;

        // apply default board orientation for this compass type. This is
        // a noop on most boards
        _sum[i].rotate(MAG_BOARD_ORIENTATION);

        // override any user setting of COMPASS_EXTERNAL 
        //_external.set(_is_external[0]);

        if (_external[i]) {
            // add user selectable orientation
            _sum[i].rotate((enum Rotation)_orientation[i].get());
        } else {
            // add in board orientation from AHRS
            _sum[i].rotate(_board_orientation);
        }
    
        _field[i] = _sum[i];
        apply_corrections(_field[i],i);
    
        _sum[i].zero();
        _count[i] = 0;
    }

    last_update = _last_timestamp[get_primary()];
    
    return _healthy[get_primary()];
}

void AP_Compass_VRBRAIN::accumulate(void)
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

uint8_t AP_Compass_VRBRAIN::get_primary(void) const
{
    if (_primary < _num_instances && _healthy[_primary] && use_for_yaw(_primary)) {
        return _primary;
    }
    for (uint8_t i=0; i<_num_instances; i++) {
        if (_healthy[i] && (use_for_yaw(i))) return i;
    }    
    return 0;
}

#endif // CONFIG_HAL_BOARD
