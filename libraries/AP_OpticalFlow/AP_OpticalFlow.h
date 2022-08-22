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
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_OPTICALFLOW_ENABLED
#define AP_OPTICALFLOW_ENABLED 1
#endif

#ifndef HAL_MSP_OPTICALFLOW_ENABLED
#define HAL_MSP_OPTICALFLOW_ENABLED (AP_OPTICALFLOW_ENABLED && (HAL_MSP_ENABLED && !HAL_MINIMIZE_FEATURES))
#endif

#ifndef AP_OPTICALFLOW_SITL_ENABLED
#define AP_OPTICALFLOW_SITL_ENABLED AP_SIM_ENABLED
#endif

#if AP_OPTICALFLOW_ENABLED

/*
 * AP_OpticalFlow.h - OpticalFlow Base Class for ArduPilot
 */

#include <AP_MSP/msp.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_OpticalFlow_Calibrator.h"

class OpticalFlow_backend;

class AP_OpticalFlow
{
    friend class OpticalFlow_backend;

public:
    AP_OpticalFlow();

    CLASS_NO_COPY(AP_OpticalFlow);

    // get singleton instance
    static AP_OpticalFlow *get_singleton() {
        return _singleton;
    }

    enum class Type {
        NONE = 0,
        PX4FLOW = 1,
        PIXART = 2,
        BEBOP = 3,
        CXOF = 4,
        MAVLINK = 5,
        UAVCAN = 6,
        MSP = 7,
        UPFLOW = 8,
        SITL = 10,
    };

    // init - initialise sensor
    void init(uint32_t log_bit);

    // enabled - returns true if optical flow is enabled
    bool enabled() const { return _type != Type::NONE; }

    // healthy - return true if the sensor is healthy
    bool healthy() const { return backend != nullptr && _flags.healthy; }

    // read latest values from sensor and fill in x,y and totals.
    void update(void);

    // handle optical flow mavlink messages
    void handle_msg(const mavlink_message_t &msg);

#if HAL_MSP_OPTICALFLOW_ENABLED
    // handle optical flow msp messages
    void handle_msp(const MSP::msp_opflow_data_message_t &pkt);
#endif

    // quality - returns the surface quality as a measure from 0 ~ 255
    uint8_t quality() const { return _state.surface_quality; }

    // raw - returns the raw movement from the sensor
    const Vector2f& flowRate() const { return _state.flowRate; }

    // velocity - returns the velocity in m/s
    const Vector2f& bodyRate() const { return _state.bodyRate; }

    // last_update() - returns system time of last sensor update
    uint32_t last_update() const { return _last_update_ms; }

    struct OpticalFlow_state {
        uint8_t  surface_quality;   // image quality (below TBD you can't trust the dx,dy values returned)
        Vector2f flowRate;          // optical flow angular rate in rad/sec measured about the X and Y body axis. A RH rotation about a sensor axis produces a positive rate.
        Vector2f bodyRate;          // body inertial angular rate in rad/sec measured about the X and Y body axis. A RH rotation about a sensor axis produces a positive rate.
    };

    // return a 3D vector defining the position offset of the sensors focal point in metres relative to the body frame origin
    const Vector3f &get_pos_offset(void) const {
        return _pos_offset;
    }

    // start or stop calibration
    void start_calibration();
    void stop_calibration();

    // parameter var info table
    static const struct AP_Param::GroupInfo var_info[];

private:

    static AP_OpticalFlow *_singleton;

    OpticalFlow_backend *backend;

    struct AP_OpticalFlow_Flags {
        uint8_t healthy     : 1;    // true if sensor is healthy
    } _flags;

    // parameters
    AP_Enum<Type>  _type;           // user configurable sensor type
    AP_Int16 _flowScalerX;          // X axis flow scale factor correction - parts per thousand
    AP_Int16 _flowScalerY;          // Y axis flow scale factor correction - parts per thousand
    AP_Int16 _yawAngle_cd;          // yaw angle of sensor X axis with respect to vehicle X axis - centi degrees
    AP_Vector3f _pos_offset;        // position offset of the flow sensor in the body frame
    AP_Int8  _address;              // address on the bus (allows selecting between 8 possible I2C addresses for px4flow)

    // method called by backend to update frontend state:
    void update_state(const OpticalFlow_state &state);

    // state filled in by backend
    struct OpticalFlow_state _state;

    uint32_t _last_update_ms;        // millis() time of last update

    void Log_Write_Optflow();
    uint32_t _log_bit = -1;     // bitmask bit which indicates if we should log.  -1 means we always log

    // calibrator
    AP_OpticalFlow_Calibrator *_calibrator;

};

namespace AP {
    AP_OpticalFlow *opticalflow();
}

#include "AP_OpticalFlow_Backend.h"

#endif // AP_OPTICALFLOW_ENABLED
