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

#include "AP_OpticalFlow_config.h"

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
#if AP_OPTICALFLOW_PX4FLOW_ENABLED
        PX4FLOW = 1,
#endif  // AP_OPTICALFLOW_PX4FLOW_ENABLED
#if AP_OPTICALFLOW_PIXART_ENABLED
        PIXART = 2,
#endif  // AP_OPTICALFLOW_PIXART_ENABLED
#if AP_OPTICALFLOW_ONBOARD_ENABLED
        BEBOP = 3,
#endif  // AP_OPTICALFLOW_ONBOARD_ENABLED
#if AP_OPTICALFLOW_CXOF_ENABLED
        CXOF = 4,
#endif  // AP_OPTICALFLOW_CXOF_ENABLED
#if AP_OPTICALFLOW_MAV_ENABLED
        MAVLINK = 5,
#endif  // AP_OPTICALFLOW_MAV_ENABLED
#if AP_OPTICALFLOW_HEREFLOW_ENABLED
        UAVCAN = 6,
#endif  // AP_OPTICALFLOW_HEREFLOW_ENABLED
#if HAL_MSP_OPTICALFLOW_ENABLED
        MSP = 7,
#endif  // HAL_MSP_OPTICALFLOW_ENABLED
#if AP_OPTICALFLOW_UPFLOW_ENABLED
        UPFLOW = 8,
#endif  // AP_OPTICALFLOW_UPFLOW_ENABLED
#if AP_OPTICALFLOW_SITL_ENABLED
        SITL = 10,
#endif  // AP_OPTICALFLOW_SITL_ENABLED
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

    // flowRate - returns the raw movement from the sensor in rad/s
    const Vector2f& flowRate() const { return _state.flowRate; }

    // bodyRate - returns the IMU-adjusted movement in rad/s
    const Vector2f& bodyRate() const { return _state.bodyRate; }

    // last_update() - returns system time of last sensor update
    uint32_t last_update() const { return _last_update_ms; }

    // get_height_override() - returns the user-specified height of sensor above ground
    float get_height_override() const { return _height_override; }

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
        bool healthy;               // true if sensor is healthy
    } _flags;

    // parameters
    AP_Enum<Type>  _type;           // user configurable sensor type
    AP_Int16 _flowScalerX;          // X axis flow scale factor correction - parts per thousand
    AP_Int16 _flowScalerY;          // Y axis flow scale factor correction - parts per thousand
    AP_Int16 _yawAngle_cd;          // yaw angle of sensor X axis with respect to vehicle X axis - centi degrees
    AP_Vector3f _pos_offset;        // position offset of the flow sensor in the body frame
    AP_Int8  _address;              // address on the bus (allows selecting between 8 possible I2C addresses for px4flow)
    AP_Float  _height_override;              // height of the sensor above the ground. Only used in rover
    AP_Int16 _options;              // options parameter

    // method called by backend to update frontend state:
    void update_state(const OpticalFlow_state &state);

    // state filled in by backend
    struct OpticalFlow_state _state;

    uint32_t _last_update_ms;        // millis() time of last update

    void Log_Write_Optflow();
    uint32_t _log_bit = -1;     // bitmask bit which indicates if we should log.  -1 means we always log

#if AP_OPTICALFLOW_CALIBRATOR_ENABLED
    // calibrator
    AP_OpticalFlow_Calibrator *_calibrator;
#endif
};

namespace AP {
    AP_OpticalFlow *opticalflow();
}

#include "AP_OpticalFlow_Backend.h"

#endif // AP_OPTICALFLOW_ENABLED
