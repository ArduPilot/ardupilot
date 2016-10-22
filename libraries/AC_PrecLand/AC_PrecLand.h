#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <stdint.h>
#include "PosVelEKF.h"
#include <AP_Buffer/AP_Buffer.h>

// declare backend classes
class AC_PrecLand_Backend;
class AC_PrecLand_Companion;
class AC_PrecLand_IRLock;

class AC_PrecLand
{
    // declare backends as friends
    friend class AC_PrecLand_Backend;
    friend class AC_PrecLand_Companion;
    friend class AC_PrecLand_IRLock;

public:

    // precision landing behaviours (held in PRECLAND_ENABLED parameter)
    enum PrecLandBehaviour {
        PRECLAND_BEHAVIOUR_DISABLED,
        PRECLAND_BEHAVIOR_ALWAYSLAND,
        PRECLAND_BEHAVIOR_CAUTIOUS
    };

    // types of precision landing (used for PRECLAND_TYPE parameter)
    enum PrecLandType {
        PRECLAND_TYPE_NONE = 0,
        PRECLAND_TYPE_COMPANION,
        PRECLAND_TYPE_IRLOCK
    };

    // constructor
    AC_PrecLand(const AP_AHRS& ahrs, const AP_InertialNav& inav);

    // perform any required initialisation of landing controllers
    void init();

    // returns true if precision landing is healthy
    bool healthy() const { return _backend_state.healthy; }

    // returns time of last update
    uint32_t last_update_ms() const { return _last_update_ms; }

    // give chance to driver to get updates from sensor
    void update(float rangefinder_alt_cm, bool rangefinder_alt_valid);

    // returns target position relative to origin
    bool get_target_position_cm(Vector2f& ret) const;

    // returns target position relative to vehicle
    bool get_target_position_relative_cm(Vector2f& ret) const;

    // returns target velocity relative to vehicle
    bool get_target_velocity_relative_cms(Vector2f& ret) const;

    // returns true when the landing target has been detected
    bool target_acquired() const;

    // process a LANDING_TARGET mavlink message
    void handle_msg(mavlink_message_t* msg);

    // accessors for logging
    bool enabled() const { return _enabled; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:
    // returns enabled parameter as an behaviour
    enum PrecLandBehaviour get_behaviour() const { return (enum PrecLandBehaviour)(_enabled.get()); }

    // references to inertial nav and ahrs libraries
    const AP_AHRS&              _ahrs;
    const AP_InertialNav&       _inav;

    // parameters
    AP_Int8                     _enabled;           // enabled/disabled and behaviour
    AP_Int8                     _type;              // precision landing controller type
    AP_Float                    _yaw_align;         // Yaw angle from body x-axis to sensor x-axis.
    AP_Float                    _land_ofs_cm_x;     // Desired landing position of the camera forward of the target in vehicle body frame
    AP_Float                    _land_ofs_cm_y;     // Desired landing position of the camera right of the target in vehicle body frame

    uint32_t                    _last_update_ms;      // epoch time in millisecond when update is called
    uint32_t                    _last_backend_los_meas_ms;

    PosVelEKF                   _ekf_x, _ekf_y;
    uint32_t                    _outlier_reject_count;
    
    AP_Buffer<Matrix3f,8>       _attitude_history;

    // backend state
    struct precland_state {
        bool    healthy;
    } _backend_state;
    AC_PrecLand_Backend         *_backend;  // pointers to backend precision landing driver
};
