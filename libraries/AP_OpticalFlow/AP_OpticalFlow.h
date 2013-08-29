#ifndef __AP_OPTICALFLOW_H__
#define __AP_OPTICALFLOW_H__
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
 *       AP_OpticalFlow.cpp - OpticalFlow Base Class for Ardupilot Mega
 *       Code by Randy Mackay. DIYDrones.com
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

#define AP_OPTICALFLOW_NUM_CALLS_FOR_10HZ     100       // timer process runs at 1khz.  100 iterations = 10hz
#define AP_OPTICALFLOW_NUM_CALLS_FOR_20HZ     50        // timer process runs at 1khz.  50 iterations = 20hz
#define AP_OPTICALFLOW_NUM_CALLS_FOR_50HZ     20        // timer process runs at 1khz.  20 iterations = 50hz

class AP_OpticalFlow
{
public:
    // constructor
    AP_OpticalFlow() {
        _sensor = this;
    };
    ~AP_OpticalFlow() {
        _sensor = NULL;
    };

    virtual bool init(); 

    virtual uint8_t read_register(uint8_t address);
    virtual void    write_register(uint8_t address, uint8_t value);

    // Rotation vector to transform sensor readings to the body frame.
    virtual void    set_orientation(enum Rotation rotation);

    // sets field of view of sensor
    virtual void    set_field_of_view(const float fov) { field_of_view = fov; update_conversion_factors(); };

    // called by timer process to read sensor data from all attached sensors
    static void     read(uint32_t now);

    // read latest values from sensor and fill in x,y and totals.
    virtual void    update(uint32_t now);

    // updates internal lon and lat with estimation based on optical flow
    virtual void    update_position(float roll, float pitch, float sin_yaw, float cos_yaw, float altitude);

    // public variables
    int16_t  raw_dx;            // raw sensor change in x and y position (i.e. unrotated)
    int16_t  raw_dy;            // raw sensor change in x and y position (i.e. unrotated)
    uint8_t  surface_quality;   // image quality (below 15 you really can't trust the x,y values returned)
    int16_t  x,y;               // total x,y position
    int16_t  dx,dy;             // rotated change in x and y position
    float    vlon, vlat;        // position as offsets from original position
    uint32_t last_update;       // millis() time of last update
    float    field_of_view;     // field of view in Radians
    float    scaler;            // number returned from sensor when moved one pixel
    int16_t  num_pixels;        // number of pixels of resolution in the sensor

    // public variables for reporting purposes
    float    exp_change_x, exp_change_y;    // expected change in x, y coordinates
    float    change_x, change_y;            // actual change in x, y coordinates
    float    x_cm, y_cm;                    // x,y position in cm

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
