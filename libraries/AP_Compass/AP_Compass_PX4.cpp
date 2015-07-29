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


#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include "AP_Compass_PX4.h"

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

// constructor
AP_Compass_PX4::AP_Compass_PX4(Compass &compass):
    AP_Compass_Backend(compass),
    _num_sensors(0)
{
}

// detect the sensor
AP_Compass_Backend *AP_Compass_PX4::detect(Compass &compass)
{
    AP_Compass_PX4 *sensor = new AP_Compass_PX4(compass);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->init()) {
        delete sensor;
        return NULL;
    }
    return sensor;
}

bool AP_Compass_PX4::init(void)
{
	_mag_fd[0] = open(MAG_BASE_DEVICE_PATH"0", O_RDONLY);
	_mag_fd[1] = open(MAG_BASE_DEVICE_PATH"1", O_RDONLY);
	_mag_fd[2] = open(MAG_BASE_DEVICE_PATH"2", O_RDONLY);

    _num_sensors = 0;
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if (_mag_fd[i] >= 0) {
            _num_sensors = i+1;
        }
    }    
	if (_num_sensors == 0) {
        hal.console->printf("Unable to open " MAG_BASE_DEVICE_PATH"0" "\n");
        return false;
	}

    for (uint8_t i=0; i<_num_sensors; i++) {
        _instance[i] = register_compass();

        // get device id
        set_dev_id(_instance[i], ioctl(_mag_fd[i], DEVIOCGDEVICEID, 0));

        // average over up to 20 samples
        if (ioctl(_mag_fd[i], SENSORIOCSQUEUEDEPTH, 20) != 0) {
            hal.console->printf("Failed to setup compass queue\n");
            return false;                
        }

        // remember if the compass is external
        set_external(_instance[i], ioctl(_mag_fd[i], MAGIOCGEXTERNAL, 0) > 0);
        _count[i] = 0;
        _sum[i].zero();
    }

    // give the driver a chance to run, and gather one sample
    hal.scheduler->delay(40);
    accumulate();
    if (_count[0] == 0) {
        hal.console->printf("Failed initial compass accumulate\n");        
    }

    return true;
}

void AP_Compass_PX4::read(void)
{
    // try to accumulate one more sample, so we have the latest data
    accumulate();

    for (uint8_t i=0; i<_num_sensors; i++) {
        // avoid division by zero if we haven't received any mag reports
        if (_count[i] == 0) continue;

        _sum[i] /= _count[i];
        _sum[i] *= 1000;

        publish_field(_sum[i], _instance[i]);
    
        _sum[i].zero();
        _count[i] = 0;
    }
}

void AP_Compass_PX4::accumulate(void)
{
    struct mag_report mag_report;
    for (uint8_t i=0; i<_num_sensors; i++) {
        while (::read(_mag_fd[i], &mag_report, sizeof(mag_report)) == sizeof(mag_report) &&
               mag_report.timestamp != _last_timestamp[i]) {
            _sum[i] += Vector3f(mag_report.x, mag_report.y, mag_report.z);
            _count[i]++;
            _last_timestamp[i] = mag_report.timestamp;
        }
    }
}

#endif // CONFIG_HAL_BOARD
