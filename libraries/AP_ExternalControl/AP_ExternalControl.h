/*
  external control library for MAVLink, DDS and scripting
 */

#pragma once

#include "AP_ExternalControl_config.h"

#if AP_EXTERNAL_CONTROL_ENABLED

#include <AP_Arming/AP_Arming.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_Camera/AP_Camera.h>
#include <AP_Common/Location.h>

class AP_ExternalControl
{
public:

    AP_ExternalControl();

    /*
        Sets the target airspeed.
    */
    virtual bool set_airspeed(const float airspeed) WARN_IF_UNUSED {
        return false;
    }
    /*
      Set linear velocity and yaw rate. Pass NaN for yaw_rate_rads to not control yaw.
      Velocity is in earth frame, NED [m/s].
      Yaw is in earth frame, NED [rad/s].
     */
    virtual bool set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity, float yaw_rate_rads) WARN_IF_UNUSED {
        return false;
    }

    /*
        Sets the target global position with standard guided mode behavior.
    */
    virtual bool set_global_position(const Location& loc) WARN_IF_UNUSED {
        return false;
    }

    /*
        Arm the vehicle
    */
    virtual bool arm(AP_Arming::Method method, bool do_arming_checks) WARN_IF_UNUSED;

    /*
        Disarm the vehicle
    */
    virtual bool disarm(AP_Arming::Method method, bool do_disarm_checks) WARN_IF_UNUSED;

    /*
        Start image capture
    */
    virtual bool start_image_capture(uint8_t camera_id, uint32_t interval_msec, uint16_t total_images) WARN_IF_UNUSED;

    /*
        Stop image capture
    */
    virtual bool stop_image_capture(uint8_t camera_id) WARN_IF_UNUSED;

    /*
        Get total images captured since start of image capture
    */
    virtual uint16_t total_image_cap(uint8_t camera_id) WARN_IF_UNUSED;

    /*
        Get current image sequence number
    */
    virtual uint16_t image_current_seq(uint8_t camera_id) WARN_IF_UNUSED;

    static AP_ExternalControl *get_singleton(void) WARN_IF_UNUSED {
        return singleton;
    }
protected:
    ~AP_ExternalControl() {}

private:
    static AP_ExternalControl *singleton;
};


namespace AP
{
AP_ExternalControl *externalcontrol();
};

#endif // AP_EXTERNAL_CONTROL_ENABLED
