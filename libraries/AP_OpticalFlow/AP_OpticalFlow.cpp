/*
 *       ADC.cpp - Analog Digital Converter Base Class for Ardupilot Mega
 *       Code by James Goppert. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 */

#include "AP_OpticalFlow.h"

#define FORTYFIVE_DEGREES 0.78539816f

// pointer to the last instantiated optical flow sensor.  Will be turned into
// a table if we ever add support for more than one sensor
AP_OpticalFlow* AP_OpticalFlow::_sensor = NULL;
// number of times we have been called by 1khz timer process.
// We use this to throttle read down to 20hz
uint8_t AP_OpticalFlow::_num_calls;

bool AP_OpticalFlow::init()
{
    _orientation = ROTATION_NONE;
    update_conversion_factors();
    return true;      // just return true by default
}

// set_orientation - Rotation vector to transform sensor readings to the body
// frame.
void AP_OpticalFlow::set_orientation(enum Rotation rotation)
{
    _orientation = rotation;
}

// parent method called at 1khz by periodic process
// this is slowed down to 20hz and each instance's update function is called
// (only one instance is supported at the moment)
void AP_OpticalFlow::read(uint32_t now)
{
    _num_calls++;

    if( _num_calls >= AP_OPTICALFLOW_NUM_CALLS_FOR_20HZ ) {
        _num_calls = 0;
        // call to update all attached sensors
        if( _sensor != NULL ) {
            _sensor->update(now);
        }
    }
};

// read value from the sensor.  Should be overridden by derived class
void AP_OpticalFlow::update(uint32_t now){ }

// reads a value from the sensor (will be sensor specific)
uint8_t AP_OpticalFlow::read_register(uint8_t address){ return 0; }

// writes a value to one of the sensor's register (will be sensor specific)
void AP_OpticalFlow::write_register(uint8_t address, uint8_t value) {}

// rotate raw values to arrive at final x,y,dx and dy values
void AP_OpticalFlow::apply_orientation_matrix()
{
    Vector3f rot_vector;
    rot_vector(raw_dx, raw_dy, 0);

    // next rotate dx and dy
    rot_vector.rotate(_orientation);

    dx = rot_vector.x;
    dy = rot_vector.y;

    // add rotated values to totals (perhaps this is pointless as we need
    // to take into account yaw, roll, pitch)
    x += dx;
    y += dy;
}

// updates conversion factors that are dependent upon field_of_view
void AP_OpticalFlow::update_conversion_factors()
{
    // multiply this number by altitude and pixel change to get horizontal
    // move (in same units as altitude)
    conv_factor = ((1.0f / (float)(num_pixels * scaler))
                   * 2.0f * tanf(field_of_view / 2.0f));
    // 0.00615
    radians_to_pixels = (num_pixels * scaler) / field_of_view;
    // 162.99
}

// updates internal lon and lat with estimation based on optical flow
void AP_OpticalFlow::update_position(float roll, float pitch,
        float sin_yaw, float cos_yaw, float altitude)
{
    float diff_roll     = roll  - _last_roll;
    float diff_pitch    = pitch - _last_pitch;

    // only update position if surface quality is good and angle is not
    // over 45 degrees
    if( surface_quality >= 10 && fabsf(roll) <= FORTYFIVE_DEGREES
     && fabsf(pitch) <= FORTYFIVE_DEGREES ) {
	altitude = max(altitude, 0);
        // calculate expected x,y diff due to roll and pitch change
        exp_change_x = diff_roll * radians_to_pixels;
        exp_change_y = -diff_pitch * radians_to_pixels;

        // real estimated raw change from mouse
        change_x = dx - exp_change_x;
        change_y = dy - exp_change_y;

        float avg_altitude = (altitude + _last_altitude)*0.5f;

        // convert raw change to horizontal movement in cm
        // perhaps this altitude should actually be the distance to the
        // ground?  i.e. if we are very rolled over it should be longer?
        x_cm = -change_x * avg_altitude * conv_factor;
        // for example if you are leaned over at 45 deg the ground will
        // appear farther away and motion from opt flow sensor will be less
        y_cm = -change_y * avg_altitude * conv_factor;

        // convert x/y movements into lon/lat movement
        vlon = x_cm * cos_yaw + y_cm * sin_yaw;
        vlat = y_cm * cos_yaw - x_cm * sin_yaw;
    }

    _last_altitude = altitude;
    _last_roll = roll;
    _last_pitch = pitch;
}
