// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "AP_RangeFinder_PX4.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_range_finder.h>
#include <drivers/drv_hrt.h>
#include <stdio.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;

uint8_t AP_RangeFinder_PX4::num_px4_instances = 0;

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_PX4::AP_RangeFinder_PX4(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state) :
	AP_RangeFinder_Backend(_ranger, instance, _state),
    _last_max_distance_cm(-1),
    _last_min_distance_cm(-1)
{
    _fd = open_driver();

    // consider this path used up
    num_px4_instances++;

	if (_fd == -1) {
        hal.console->printf("Unable to open PX4 rangefinder %u\n", num_px4_instances);
        state.healthy = false;
        return;
	}

    // average over up to 20 samples
    if (ioctl(_fd, SENSORIOCSQUEUEDEPTH, 20) != 0) {
        hal.console->printf("Failed to setup range finder queue\n");
        state.healthy = false;
        return;
    }

    state.healthy = true;
}

/* 
   close the file descriptor
*/
AP_RangeFinder_PX4::~AP_RangeFinder_PX4()
{
    if (_fd != -1) {
        close(_fd);
    }
}

/* 
   open the PX4 driver, returning the file descriptor
*/
int AP_RangeFinder_PX4::open_driver(void)
{
    // work out the device path based on how many PX4 drivers we have loaded
    char path[] = RANGE_FINDER_DEVICE_PATH "n";
    if (num_px4_instances == 0) {
        path[strlen(path)-1] = 0;
    } else {
        path[strlen(path)-1] = '1' + (num_px4_instances-1);
    }
    return open(path, O_RDONLY);
}

/* 
   see if the PX4 driver is available
*/
bool AP_RangeFinder_PX4::detect(RangeFinder &_ranger, uint8_t instance)
{
    int fd = open_driver();
    if (fd == -1) {
        return false;
    }
    close(fd);
    return true;
}

void AP_RangeFinder_PX4::update(void)
{
    if (_fd == -1) {
        state.healthy = false;
        return;
    }

    struct range_finder_report range_report;
    float sum = 0;
    uint16_t count = 0;

    if (_last_max_distance_cm != ranger._max_distance_cm[state.instance] ||
        _last_min_distance_cm != ranger._min_distance_cm[state.instance]) {
        float max_distance = ranger._max_distance_cm[state.instance]*0.01f;
        float min_distance = ranger._min_distance_cm[state.instance]*0.01f;
        if (ioctl(_fd, RANGEFINDERIOCSETMAXIUMDISTANCE, (unsigned long)&max_distance) == 0 &&
            ioctl(_fd, RANGEFINDERIOCSETMINIUMDISTANCE, (unsigned long)&min_distance) == 0) {
            _last_max_distance_cm = ranger._max_distance_cm[state.instance];
            _last_min_distance_cm = ranger._min_distance_cm[state.instance];
        }
    }


    while (::read(_fd, &range_report, sizeof(range_report)) == sizeof(range_report) &&
           range_report.timestamp != _last_timestamp) {
            // Only take valid readings
            if (range_report.valid == 1) {
                sum += range_report.distance;
                count++;
                _last_timestamp = range_report.timestamp;
            }
    }

    // consider the range finder healthy if we got a reading in the last 0.2s
    state.healthy = (hal.scheduler->micros64() - _last_timestamp < 200000);

    if (count != 0) {
        state.distance_cm = sum / count * 100.0f;
    }
}

#endif // CONFIG_HAL_BOARD
