/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __AC_PRECLAND_H__
#define __AC_PRECLAND_H__

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_PI_2D.h>
#include <AP_InertialNav/AP_InertialNav.h>

// definitions
#define AC_PRECLAND_SPEED_XY_DEFAULT            100.0f  // maximum horizontal speed
#define PRECLAND_P                              2.0f    // velocity controller P gain default
#define PRECLAND_I                              1.0f    // velocity controller I gain default
#define PRECLAND_IMAX                         500.0f    // velocity controller IMAX default
#define PRECLAND_FILT_HZ                        5.0f    // velocity controller filter hz
#define PRECLAND_UPDATE_TIME                    0.02f   // precland runs at 50hz

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

    // Constructor
    AC_PrecLand(const AP_AHRS& ahrs, const AP_InertialNav& inav, float dt);

    // init - perform any required initialisation of landing controllers
    void init();

    // healthy - returns true if precision landing is healthy
    bool healthy() { return _backend_state.healthy; }

    // update - give chance to driver to get updates from sensor
    void update(float alt_above_terrain_cm);

    // get_target_shift - returns 3D vector of earth-frame position adjustments to target
    Vector3f get_target_shift(const Vector3f& orig_target);

    // handle_msg - Process a LANDING_TARGET mavlink message
    void handle_msg(mavlink_message_t* msg);

    // accessors for logging
    bool enabled() const { return _enabled; }
    const Vector2f& last_bf_angle_to_target() const { return _angle_to_target; }
    const Vector2f& last_ef_angle_to_target() const { return _ef_angle_to_target; }
    const Vector3f& last_target_pos_offset() const { return _target_pos_offset; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    // calc_angles_and_pos - converts sensor's body-frame angles to earth-frame angles and position estimate
    //  angles stored in _angle_to_target
    //  earth-frame angles stored in _ef_angle_to_target
    //  position estimate is stored in _target_pos
    void calc_angles_and_pos(float alt_above_terrain_cm);

    // get_behaviour - returns enabled parameter as an behaviour
    enum PrecLandBehaviour get_behaviour() const { return (enum PrecLandBehaviour)(_enabled.get()); }

    // references to inertial nav and ahrs libraries
    const AP_AHRS&              _ahrs;
    const AP_InertialNav&       _inav;
    AC_PI_2D                    _pi_vel_xy;         // horizontal velocity PI controller

    // parameters
    AP_Int8                     _enabled;           // enabled/disabled and behaviour
    AP_Int8                     _type;              // precision landing controller type
    AP_Float                    _speed_xy;          // maximum horizontal speed in cm/s

    // internal variables
    float                       _dt;                // time difference (in seconds) between calls from the main program

    // output from sensor (stored for logging)
    Vector2f                    _angle_to_target;   // last raw sensor angle to target
    Vector2f                    _ef_angle_to_target;// last earth-frame angle to target

    // output from controller
    bool                        _have_estimate;     // true if we have a recent estimated position offset
    Vector3f                    _target_pos_offset; // estimate target position offset from vehicle in earth-frame

    // backend state
    struct precland_state {
        bool    healthy;
    } _backend_state;
    AC_PrecLand_Backend         *_backend;  // pointers to backend precision landing driver
};
#endif	// __AC_PRECLAND_H__
