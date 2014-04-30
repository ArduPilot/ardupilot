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

// Constructor ////////////////////////////////////////////////////////
AP_RangeFinder_PX4::AP_RangeFinder_PX4(FilterInt16 *filter) :
    RangeFinder(NULL, filter),
    _num_instances(0){ }

bool AP_RangeFinder_PX4::init(void)
{
	_range_fd[0] = open(RANGE_FINDER_DEVICE_PATH, O_RDONLY);
	if (_range_fd[0] < 0) {
        hal.console->printf("Unable to open " RANGE_FINDER_DEVICE_PATH "\n");
        return false;
	}

	_range_fd[1] = open(RANGE_FINDER_DEVICE_PATH "1", O_RDONLY);
	if (_range_fd[1] >= 0) {
        _num_instances = 2;
	} else {
        _num_instances = 1;
    }

    for (uint8_t i=0; i<_num_instances; i++) {
        // average over up to 20 samples
        if (ioctl(_range_fd[i], SENSORIOCSQUEUEDEPTH, 20) != 0) {
            hal.console->printf("Failed to setup range finder queue\n");
            return false;                
        }

        _count[0] = 0;
        _sum[i] = 0;
        _healthy[i] = false;
    }

    // give the driver a chance to run, and gather one sample
    hal.scheduler->delay(40);
    accumulate();
    if (_count[0] == 0) {
        hal.console->printf("Failed initial range finder accumulate\n");        
    }
    return true;
}

bool AP_RangeFinder_PX4::take_reading(void)
{
    // try to accumulate one more sample, so we have the latest data
    accumulate();

    // consider the compass healthy if we got a reading in the last 0.2s
    for (uint8_t i=0; i<_num_instances; i++) {
        _healthy[i] = (hrt_absolute_time() - _last_timestamp[i] < 200000);
    }

    for (uint8_t i=0; i<_num_instances; i++) {
        // avoid division by zero if we haven't received any range reports
        if (_count[i] == 0) continue;

        _sum[i] /= _count[i];
        _sum[i] *= 100.00f;
    
        _distance[i] = _mode_filter->apply(_sum[i]);
    
        _sum[i] = 0;
        _count[i] = 0;
    }
    
    return _healthy[_get_primary()];
}

void AP_RangeFinder_PX4::accumulate(void)
{
    struct range_finder_report range_report;
    for (uint8_t i=0; i<_num_instances; i++) {
        while (::read(_range_fd[i], &range_report, sizeof(range_report)) == sizeof(range_report) &&
               range_report.timestamp != _last_timestamp[i]) {
            
            // Only take valid readings
            if (range_report.valid == 1) {
               _sum[i] += range_report.distance;
               _count[i]++;
               _last_timestamp[i] = range_report.timestamp;
            }
        }
    }
}

uint8_t AP_RangeFinder_PX4::_get_primary(void) const
{
    for (uint8_t i=0; i<_num_instances; i++) {
        if (_healthy[i]) return i;
    }    
    return 0;
}

int16_t AP_RangeFinder_PX4::read()
{
    take_reading();
    return _distance[_get_primary()];
}

#endif // CONFIG_HAL_BOARD
