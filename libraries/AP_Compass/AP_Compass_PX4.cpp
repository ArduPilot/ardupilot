/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *       AP_Compass_PX4.cpp - Arduino Library for PX4 magnetometer
 *
 *       This library is free software; you can redistribute it and / or
 *       modify it under the terms of the GNU Lesser General Public
 *       License as published by the Free Software Foundation; either
 *       version 2.1 of the License, or (at your option) any later version.
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

int AP_Compass_PX4::_mag_fd = -1;
Vector3f AP_Compass_PX4::_sum;
uint32_t AP_Compass_PX4::_count = 0;
uint32_t AP_Compass_PX4::_last_timer = 0;
uint64_t AP_Compass_PX4::_last_timestamp = 0;


// Public Methods //////////////////////////////////////////////////////////////

bool AP_Compass_PX4::init(void)
{
	_mag_fd = open(MAG_DEVICE_PATH, O_RDONLY);
	if (_mag_fd < 0) {
        hal.console->printf("Unable to open " MAG_DEVICE_PATH);
        return false;
	}

	/* set the mag internal poll rate to at least 150Hz */
	ioctl(_mag_fd, MAGIOCSSAMPLERATE, 150);

	/* set the driver to poll at 150Hz */
	ioctl(_mag_fd, SENSORIOCSPOLLRATE, 150);

    // average over up to 10 samples
    ioctl(_mag_fd, SENSORIOCSQUEUEDEPTH, 10);

    healthy = false;
    _count = 0;
    _sum.zero();

    hal.scheduler->register_timer_process(_compass_timer);

    // give the timer a chance to run, and gather one sample
    hal.scheduler->delay(40);

    return true;
}

bool AP_Compass_PX4::read(void)
{
    hal.scheduler->suspend_timer_procs();

    // try to accumulate one more sample, so we have the latest data
    _accumulate();

    // consider the compass healthy if we got a reading in the last 0.2s
    healthy = (hrt_absolute_time() - _last_timestamp < 200000);
    if (!healthy || _count == 0) {
        hal.scheduler->resume_timer_procs();
        return healthy;
    }

    _sum /= _count;
    _sum *= 1000;

    // apply default board orientation for this compass type. This is
    // a noop on most boards
    _sum.rotate(MAG_BOARD_ORIENTATION);

    // add user selectable orientation
    _sum.rotate((enum Rotation)_orientation.get());

    // and add in AHRS_ORIENTATION setting
    _sum.rotate(_board_orientation);
    _sum += _offset.get();

    // apply motor compensation
    if (_motor_comp_type != AP_COMPASS_MOT_COMP_DISABLED && _thr_or_curr != 0.0f) {
        _motor_offset = _motor_compensation.get() * _thr_or_curr;
        _sum += _motor_offset;
    } else {
        _motor_offset.x = 0;
        _motor_offset.y = 0;
        _motor_offset.z = 0;
    }
    
    mag_x = _sum.x;
    mag_y = _sum.y;
    mag_z = _sum.z;
    
    _sum.zero();
    _count = 0;

    hal.scheduler->resume_timer_procs();
    
    last_update = _last_timestamp;
    
    return true;
}

void AP_Compass_PX4::_accumulate(void)
{
    struct mag_report mag_report;
    while (::read(_mag_fd, &mag_report, sizeof(mag_report)) == sizeof(mag_report) &&
           mag_report.timestamp != _last_timestamp) {
        _sum += Vector3f(mag_report.x, mag_report.y, mag_report.z);
        _count++;
        _last_timestamp = mag_report.timestamp;
    }
}

void AP_Compass_PX4::accumulate(void)
{
    // let the timer do the work
}

void AP_Compass_PX4::_compass_timer(uint32_t now)
{
    // try to accumulate samples at 100Hz
    if (now - _last_timer < 10000) {
        return;
    }
    _last_timer = hal.scheduler->micros();
    _accumulate();
}

#endif // CONFIG_HAL_BOARD
