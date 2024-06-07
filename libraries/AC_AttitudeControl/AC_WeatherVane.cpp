/*
 * Aircraft Weathervane options common to vtol plane and copters
 */
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include "AC_WeatherVane.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    #define WVANE_PARAM_ENABLED 1
    #define WVANE_PARAM_SPD_MAX_DEFAULT 0
    #define WVANE_PARAM_VELZ_MAX_DEFAULT 0
    #define WVANE_PARAM_GAIN_DEFAULT 0
#else
    #define WVANE_PARAM_ENABLED 0
    #define WVANE_PARAM_SPD_MAX_DEFAULT 2
    #define WVANE_PARAM_VELZ_MAX_DEFAULT 1
    #define WVANE_PARAM_GAIN_DEFAULT 1
#endif


const AP_Param::GroupInfo AC_WeatherVane::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable
    // @Description: Enable weather vaning.  When active, the aircraft will automatically yaw into wind when in a VTOL position controlled mode. Pilot yaw commands override the weathervaning action.
    // @Values: -1:Only use during takeoffs or landing see weathervane takeoff and land override parameters,0:Disabled,1:Nose into wind,2:Nose or tail into wind,3:Side into wind,4:tail into wind
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AC_WeatherVane, _direction, WVANE_PARAM_ENABLED, AP_PARAM_FLAG_ENABLE),

    // @Param: GAIN
    // @DisplayName: Weathervaning gain
    // @Description: This converts the target roll/pitch angle of the aircraft into the correcting (into wind) yaw rate. e.g. Gain = 2, roll = 30 deg, pitch = 0 deg, yaw rate = 60 deg/s.
    // @Range: 0.5 4
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("GAIN", 2, AC_WeatherVane, _gain, WVANE_PARAM_GAIN_DEFAULT),

    // @Param: ANG_MIN
    // @DisplayName: Weathervaning min angle
    // @Description: The minimum target roll/pitch angle before active weathervaning will start.  This provides a dead zone that is particularly useful for poorly trimmed quadplanes.
    // @Units: deg
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("ANG_MIN", 3, AC_WeatherVane, _min_dz_ang_deg, 1.0),

    // @Param: HGT_MIN
    // @DisplayName: Weathervaning min height
    // @Description: Above this height weathervaning is permitted.  If a range finder is fitted or if terrain is enabled, this parameter sets height AGL.  Otherwise, this parameter sets height above home.  Set zero to ignore minimum height requirement to activate weathervaning.
    // @Description{Plane}: Above this height weathervaning is permitted.  If RNGFND_LANDING is enabled or terrain is enabled then this parameter sets height AGL. Otherwise this parameter sets height above home.  Set zero to ignore minimum height requirement to activate weathervaning
    // @Units: m
    // @Range: 0 50
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("HGT_MIN", 4, AC_WeatherVane, _min_height, 0.0),

    // @Param: SPD_MAX
    // @DisplayName: Weathervaning max ground speed
    // @Description: Below this ground speed weathervaning is permitted. Set to 0 to ignore this condition when checking if vehicle should weathervane.
    // @Units: m/s
    // @Range: 0 50
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("SPD_MAX", 5, AC_WeatherVane, _max_vel_xy, WVANE_PARAM_SPD_MAX_DEFAULT),

    // @Param: VELZ_MAX
    // @DisplayName: Weathervaning max vertical speed
    // @Description: The maximum climb or descent speed that the vehicle will still attempt to weathervane. Set to 0 to ignore this condition to get the aircraft to weathervane at any climb/descent rate.  This is particularly useful for aircraft with low disc loading that struggle with yaw control in decent.
    // @Units: m/s
    // @Range: 0 5
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("VELZ_MAX", 6, AC_WeatherVane, _max_vel_z, WVANE_PARAM_VELZ_MAX_DEFAULT),

    // @Param: TAKEOFF
    // @DisplayName: Takeoff override
    // @Description: Override the weather vaning behaviour when in takeoffs
    // @Values: -1:No override,0:Disabled,1:Nose into wind,2:Nose or tail into wind,3:Side into wind,4:tail into wind
    // @User: Standard
    AP_GROUPINFO("TAKEOFF", 7, AC_WeatherVane, _takeoff_direction, -1),

    // @Param: LAND
    // @DisplayName: Landing override
    // @Description: Override the weather vaning behaviour when in landing
    // @Values: -1:No override,0:Disabled,1:Nose into wind,2:Nose or tail into wind,3:Side into wind,4:tail into wind
    // @User: Standard
    AP_GROUPINFO("LAND", 8, AC_WeatherVane, _landing_direction, -1),

    // @Param: OPTIONS
    // @DisplayName: Weathervaning options
    // @Description: Options impacting weathervaning behaviour
    // @Bitmask: 0:Use pitch when nose or tail-in for faster weathervaning
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 9, AC_WeatherVane, _options, 0),
    
    AP_GROUPEND
};


// Constructor
AC_WeatherVane::AC_WeatherVane(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool AC_WeatherVane::get_yaw_out(float &yaw_output, const int16_t pilot_yaw, const float hgt, const float roll_cdeg, const float pitch_cdeg, const bool is_takeoff, const bool is_landing)
{
    Direction dir = (Direction)_direction.get();
    if ((dir == Direction::OFF) || !allowed || (pilot_yaw != 0) || !is_positive(_gain)) {
        // parameter disabled, or 0 gain
        // disabled temporarily
        // dont't override pilot
        reset();
        return false;
    }

    // override direction when in takeoff for landing
    if (is_takeoff && (_takeoff_direction >= 0)) {
        dir = (Direction)_takeoff_direction.get();
    }
    if (is_landing && (_landing_direction >= 0)) {
        dir = (Direction)_landing_direction.get();
    }
    if (dir == Direction::OFF || (dir == Direction::TAKEOFF_OR_LAND_ONLY)) {
        // Disabled for takeoff or landing
        // Disabled if in flight and dir = -1
        reset();
        return false;
    }

    // Check if we are above the minimum height to weather vane
    if (is_positive(_min_height) && (hgt <= _min_height)) {
        reset();
        return false;
    }

    // Check if we meet the velocity thresholds to allow weathervaning
    if (is_positive(_max_vel_xy) || is_positive(_max_vel_z)) {
        Vector3f vel_ned;
        if (!AP::ahrs().get_velocity_NED(vel_ned) || // need speed estimate
                (is_positive(_max_vel_xy) && (vel_ned.xy().length_squared() > (_max_vel_xy*_max_vel_xy))) || // check xy speed
                (is_positive(_max_vel_z) && (fabsf(vel_ned.z) > _max_vel_z))) { // check z speed
            reset();
            return false;
        }
    }

    const uint32_t now = AP_HAL::millis();
    if (now - last_check_ms > 250) {
        // not run this function or reset recently
        reset();
    }
    last_check_ms = now;

    /*
      Use a 2 second buffer to ensure weathervaning occurs once the vehicle has
      clearly achieved an acceptable condition.
    */
    if (first_activate_ms == 0) {
        first_activate_ms = now;
    }
    if (now - first_activate_ms < 2000) {
        return false;
    }

    const float deadzone_cdeg = _min_dz_ang_deg*100.0;
    float output = 0.0;
    const char* dir_string = "";

    // should we enable pitch input for nose-in and tail-in?
    const bool pitch_enable = (uint8_t(_options.get()) & uint8_t(Options::PITCH_ENABLE)) != 0;

    switch (dir) {
        case Direction::OFF:
        case Direction::TAKEOFF_OR_LAND_ONLY:
            reset();
            return false;

        case Direction::NOSE_IN:
            if (pitch_enable && is_positive(pitch_cdeg - deadzone_cdeg)) {
                output = fabsf(roll_cdeg) + (pitch_cdeg - deadzone_cdeg);
            } else {
                output = MAX(fabsf(roll_cdeg) - deadzone_cdeg, 0.0);
            }
            if (is_negative(roll_cdeg)) {
                output *= -1.0;
            }
            dir_string = "nose in";
            break;

        case Direction::NOSE_OR_TAIL_IN:
            output = MAX(fabsf(roll_cdeg) - deadzone_cdeg, 0.0);
            if (is_negative(roll_cdeg) != is_positive(pitch_cdeg)) {
                output *= -1.0;
            }
            dir_string = "nose or tail in";
            break;

        case Direction::SIDE_IN:
            output = MAX(fabsf(pitch_cdeg) - deadzone_cdeg, 0.0);
            if (is_positive(pitch_cdeg) != is_positive(roll_cdeg)) {
                output *= -1.0;
            }
            dir_string = "side in";
            break;

        case Direction::TAIL_IN:
            if (pitch_enable && is_negative(pitch_cdeg + deadzone_cdeg)) {
                output = fabsf(roll_cdeg) - (pitch_cdeg + deadzone_cdeg);
            } else {
                output = MAX(fabsf(roll_cdeg) - deadzone_cdeg, 0.0);
            }
            if (is_positive(roll_cdeg)) {
                output *= -1.0;
            }
            dir_string = "tail in";
            break;
    }

    if (!active_msg_sent) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Weathervane Active: %s", dir_string);
        (void)dir_string;  // in case GCS is disabled
        active_msg_sent = true;
    }

    // Slew output and apply gain
    last_output = 0.98 * last_output + 0.02 * output * _gain;
    yaw_output = last_output;
    return true;
}

// Reset the weathervane controller
void AC_WeatherVane::reset(void)
{
    last_output = 0;
    active_msg_sent = false;
    first_activate_ms = 0;
    last_check_ms = AP_HAL::millis();
}

