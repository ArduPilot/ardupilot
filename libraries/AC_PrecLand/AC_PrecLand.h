/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <stdint.h>

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
    bool healthy() { return _backend_state.healthy; }

    // returns time of last update
    uint32_t last_update_ms() { return _last_update_ms; }

    // give chance to driver to get updates from sensor
    void update(float alt_above_terrain_cm);

    // returns 3D vector of earth-frame position adjustments to target
    Vector3f get_target_shift(const Vector3f& orig_target);

    // returns target position relative to origin
    bool get_target_position(Vector3f& ret);

    // returns target position relative to vehicle
    bool get_target_position_relative(Vector3f& ret);

    // returns target velocity relative to vehicle
    bool get_target_velocity_relative(Vector3f& ret);

    // returns true when the landing target has been detected
    bool target_acquired();

    // process a LANDING_TARGET mavlink message
    void handle_msg(mavlink_message_t* msg);

    // accessors for logging
    bool enabled() const { return _enabled; }
    const Vector2f& last_bf_angle_to_target() const { return _angle_to_target; }
    const Vector2f& last_ef_angle_to_target() const { return _ef_angle_to_target; }
    const Vector3f& last_target_pos_offset() const { return _target_pos_rel; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    // converts sensor's body-frame angles to earth-frame angles and position estimate
    //  angles stored in _angle_to_target
    //  earth-frame angles stored in _ef_angle_to_target
    //  position estimate is stored in _target_pos
    void calc_angles_and_pos(const Vector3f& target_vec_unit_body, float alt_above_terrain_cm);

    // returns enabled parameter as an behaviour
    enum PrecLandBehaviour get_behaviour() const { return (enum PrecLandBehaviour)(_enabled.get()); }

    // references to inertial nav and ahrs libraries
    const AP_AHRS&              _ahrs;
    const AP_InertialNav&       _inav;

    // parameters
    AP_Int8                     _enabled;           // enabled/disabled and behaviour
    AP_Int8                     _type;              // precision landing controller type

    uint32_t                    _last_update_ms;      // epoch time in millisecond when update is called
    uint32_t                    _last_backend_los_meas_ms;

    // output from sensor (stored for logging)
    Vector2f                    _angle_to_target;   // last raw sensor angle to target
    Vector2f                    _ef_angle_to_target;// last earth-frame angle to target

    // estimator output
    Vector3f                    _target_pos_rel;    // estimate target position relative to vehicle in NEU cm
    Vector3f                    _target_pos;        // estimate target position in NEU cm

    // backend state
    struct precland_state {
        bool    healthy;
    } _backend_state;
    AC_PrecLand_Backend         *_backend;  // pointers to backend precision landing driver
};
