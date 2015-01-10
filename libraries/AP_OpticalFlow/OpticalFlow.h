#ifndef __OpticalFlow_H__
#define __OpticalFlow_H__
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
 *       OpticalFlow.h - OpticalFlow Base Class for Ardupilot
 *       Code by Randy Mackay. DIYDrones.com
 */

#include <AP_HAL.h>
#include <AP_Math.h>

class OpticalFlow_backend;

class OpticalFlow
{
    friend class OpticalFlow_backend;

public:
    // constructor
    OpticalFlow(void);

    // init - initialise sensor
    void init(void);

    // enabled - returns true if optical flow is enabled
    bool enabled() const { return _enabled; }

    // healthy - return true if the sensor is healthy
    bool healthy() const { return backend != NULL && _flags.healthy; }

    // read latest values from sensor and fill in x,y and totals.
    void update(void);

    // quality - returns the surface quality as a measure from 0 ~ 255
    uint8_t quality() const { return _state.surface_quality; }

    // raw - returns the raw movement from the sensor
    const Vector2f& flowRate() const { return _state.flowRate; }

    // velocity - returns the velocity in m/s
    const Vector2f& bodyRate() const { return _state.bodyRate; }

    // device_id - returns device id
    uint8_t device_id() const { return _state.device_id; }

    // last_update() - returns system time of last sensor update
    uint32_t last_update() const { return _last_update_ms; }

    // parameter var info table
    static const struct AP_Param::GroupInfo var_info[];

    struct OpticalFlow_state {
        uint8_t device_id;          // device id
        uint8_t  surface_quality;   // image quality (below TBD you can't trust the dx,dy values returned)
        Vector2f flowRate;          // optical flow angular rate in rad/sec measured about the X and Y body axis. A RH rotation about a sensor axis produces a positive rate.
        Vector2f bodyRate;          // body inertial angular rate in rad/sec measured about the X and Y body axis. A RH rotation about a sensor axis produces a positive rate.
    };

    // support for HIL/SITL
    void setHIL(const struct OpticalFlow_state &state);

private:
    OpticalFlow_backend *backend;

    struct AP_OpticalFlow_Flags {
        uint8_t healthy     : 1;    // true if sensor is healthy
    } _flags;

    // parameters
    AP_Int8  _enabled;              // enabled/disabled flag
    AP_Int16 _flowScalerX;          // X axis flow scale factor correction - parts per thousand
    AP_Int16 _flowScalerY;          // Y axis flow scale factor correction - parts per thousand


    // state filled in by backend
    struct OpticalFlow_state _state;

    uint32_t _last_update_ms;        // millis() time of last update
};

#include "OpticalFlow_backend.h"
#include "AP_OpticalFlow_HIL.h"
#include "AP_OpticalFlow_PX4.h"

#endif
