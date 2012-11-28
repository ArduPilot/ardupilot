#ifndef __AP_OPTICALFLOW_H__
#define __AP_OPTICALFLOW_H__

/*
 *       AP_OpticalFlow.cpp - OpticalFlow Base Class for Ardupilot Mega
 *       Code by Randy Mackay. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *       Methods:
 *               init() : initializate sensor and library.
 *               read   : reads latest value from OpticalFlow and
 *                        stores values in x,y, surface_quality parameter
 *               read_register()  : reads a value from the sensor (will be
 *                                  sensor specific)
 *               write_register() : writes a value to one of the sensor's
 *                                  register (will be sensor specific)
 */

#include <AP_Math.h>

// timer process runs at 1khz.  100 iterations = 10hz
#define AP_OPTICALFLOW_NUM_CALLS_FOR_10HZ     100
// timer process runs at 1khz.  50 iterations = 20hz
#define AP_OPTICALFLOW_NUM_CALLS_FOR_20HZ     50
// timer process runs at 1khz.  20 iterations = 50hz
#define AP_OPTICALFLOW_NUM_CALLS_FOR_50HZ     20

class AP_OpticalFlow
{
public:
    // raw sensor change in x and y position (i.e. unrotated)
    int      raw_dx, raw_dy;
    // image quality (below 15 you really can't trust the x,y values returned)
    int      surface_quality;
    // total x,y position
    int      x,y;
    // rotated change in x and y position
    int      dx,dy;
    // position as offsets from original position
    float    vlon, vlat;
    // millis() time of last update
    uint32_t last_update;
    // field of view in Radians
    float    field_of_view;
    // number returned from sensor when moved one pixel
    float    scaler;
    // number of pixels of resolution in the sensor
    int      num_pixels;
    // temp - delete me!
    float    exp_change_x, exp_change_y;
    float    change_x, change_y;
    float    x_cm, y_cm;

    AP_OpticalFlow() {
        _sensor = this;
    };
    ~AP_OpticalFlow() {
        _sensor = NULL;
    };
    virtual bool init(); 
    // parameter controls whether I2C/SPI interface is initialised
    // (set to false if other devices are on the I2C/SPI bus and have already
    // initialised the interface)
    virtual uint8_t read_register(uint8_t address);
    virtual void    write_register(uint8_t address, uint8_t value);
    // Rotation vector to transform sensor readings to the body frame.
    virtual void    set_orientation(enum Rotation rotation);
    // sets field of view of sensor
    virtual void    set_field_of_view(const float fov) { 
        field_of_view = fov; update_conversion_factors();
    };
    // called by timer process to read sensor data from all attached sensors
    static void     read(uint32_t now);
    // read latest values from sensor and fill in x,y and totals.
    virtual void    update(uint32_t now);
    // updates internal lon and lat with estimation based on optical flow
    virtual void    update_position(float roll,
            float pitch, float cos_yaw_x, float sin_yaw_y, float altitude);

protected:
    // pointer to the last instantiated optical flow sensor.  Will be turned
    // into a table if we ever add support for more than one sensor
    static AP_OpticalFlow *  _sensor;
    enum Rotation            _orientation;
    // multiply this number by altitude and pixel change to get horizontal
    // move (in same units as altitude)
    float conv_factor;
    float radians_to_pixels;
    float _last_roll;
    float _last_pitch;
    float _last_altitude;
    // rotate raw values to arrive at final x,y,dx and dy values
    virtual void apply_orientation_matrix();
    virtual void update_conversion_factors();

private:
    // number of times we have been called by 1khz timer process.
    // We use this to throttle read down to 20hz
    static uint8_t _num_calls;
};

#include "AP_OpticalFlow_ADNS3080.h"

#endif
