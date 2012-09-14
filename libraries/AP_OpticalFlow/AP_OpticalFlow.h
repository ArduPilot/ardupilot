#ifndef AP_OPTICALFLOW_H
#define AP_OPTICALFLOW_H

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
 *               init()           : initializate sensor and library.
 *               read             : reads latest value from OpticalFlow and stores values in x,y, surface_quality parameter
 *               read_register()  : reads a value from the sensor (will be sensor specific)
 *               write_register() : writes a value to one of the sensor's register (will be sensor specific)
 */

#include <FastSerial.h>
#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_PeriodicProcess.h>

#define AP_OPTICALFLOW_NUM_CALLS_FOR_10HZ     100         // timer process runs at 1khz.  100 iterations = 10hz
#define AP_OPTICALFLOW_NUM_CALLS_FOR_20HZ     50          // timer process runs at 1khz.  50 iterations = 20hz
#define AP_OPTICALFLOW_NUM_CALLS_FOR_50HZ     20          // timer process runs at 1khz.  20 iterations = 50hz

class AP_OpticalFlow
{
public:
    int                     raw_dx, raw_dy; // raw sensor change in x and y position (i.e. unrotated)
    int                     surface_quality; // image quality (below 15 you really can't trust the x,y values returned)
    int                     x,y; // total x,y position
    int                     dx,dy; // rotated change in x and y position
    float                   vlon, vlat; // position as offsets from original position
    unsigned long           last_update; // millis() time of last update
    float                   field_of_view; // field of view in Radians
    float                   scaler; // number returned from sensor when moved one pixel
    int                     num_pixels; // number of pixels of resolution in the sensor
    // temp variables - delete me!
    float                   exp_change_x, exp_change_y;
    float                   change_x, change_y;
    float                   x_cm, y_cm;

    AP_OpticalFlow() {
        _sensor = this;
    };
    ~AP_OpticalFlow() {
        _sensor = NULL;
    };
    virtual bool                    init(bool initCommAPI, AP_PeriodicProcess *scheduler); // parameter controls whether I2C/SPI interface is initialised (set to false if other devices are on the I2C/SPI bus and have already initialised the interface)
    virtual byte                    read_register(byte address);
    virtual void                    write_register(byte address, byte value);
    virtual void                    set_orientation(enum Rotation rotation); // Rotation vector to transform sensor readings to the body frame.
    virtual void                    set_field_of_view(const float fov) { field_of_view = fov; update_conversion_factors(); };   // sets field of view of sensor
    static void                     read(uint32_t now);   // called by timer process to read sensor data from all attached sensors
    virtual void                    update(uint32_t now); // read latest values from sensor and fill in x,y and totals.
    virtual void                    update_position(float roll, float pitch, float cos_yaw_x, float sin_yaw_y, float altitude); // updates internal lon and lat with estimation based on optical flow

protected:
    static AP_OpticalFlow *         _sensor; // pointer to the last instantiated optical flow sensor.  Will be turned into a table if we ever add support for more than one sensor
    enum Rotation                   _orientation;
    float                           conv_factor; // multiply this number by altitude and pixel change to get horizontal move (in same units as altitude)
    float                           radians_to_pixels;
    float                           _last_roll, _last_pitch, _last_altitude;
    virtual void                    apply_orientation_matrix(); // rotate raw values to arrive at final x,y,dx and dy values
    virtual void                    update_conversion_factors();

private:
    static uint8_t                         _num_calls; // number of times we have been called by 1khz timer process.  We use this to throttle read down to 20hz
};

#include "AP_OpticalFlow_ADNS3080.h"

#endif
