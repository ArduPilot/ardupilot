/*
 * Aircraft Weathervane options common to vtol plane and copters
 */
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#if !(HAL_MINIMIZE_FEATURES && APM_BUILD_TYPE(APM_BUILD_ArduCopter))

#include <AP_HAL/AP_HAL.h>
#include "AC_WeatherVane.h"
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Terrain/AP_Terrain.h>

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    #define WVANE_PARAM_ENABLED 1
#else
    #define WVANE_PARAM_ENABLED 0
#endif


const AP_Param::GroupInfo AC_WeatherVane::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable
    // @Description{Copter}: Enable weather vaning.  When active, and the appropriate _OPTIONS bit is set for auto, or guided, the aircraft will yaw into wind once the vehicle is in a condition that meets that set by the WVANE parameters and there has been no pilot input for 3 seconds.
    // @Description{Plane}: Enable weather vaning.  When active, the aircraft will automatically yaw into wind when in a VTOL position controlled mode. Pilot yaw commands overide the weathervaning action.
    // @Values: 0:Disabled,1:Nose into wind,2:Nose or tail into wind,3:Side into wind
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AC_WeatherVane, _direction, WVANE_PARAM_ENABLED, AP_PARAM_FLAG_ENABLE),

    // @Param: GAIN
    // @DisplayName: Weathervaning gain
    // @Description: This converts the target roll/pitch angle of the aircraft into the correcting (into wind) yaw rate. e.g. Gain = 2, roll = 30 deg, yaw rate = 60 deg/s.
    // @Range: 0 4
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("GAIN", 2, AC_WeatherVane, _gain, 0.5),

    // @Param: ANG_MIN
    // @DisplayName: Weathervaning min angle
    // @Description: The minimum target roll/pitch angle before active weathervaning will start.  This provides a dead zone that is particularly useful for poorly trimmed quadplanes.
    // @Units: deg
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("ANG_MIN", 3, AC_WeatherVane, _min_dz_ang_deg, 1),

    // @Param: HGT_MIN
    // @DisplayName: Weathervaning min height
    // @Description: Above this height weathervaning is permitted.  If terrain is enabled, this parameter sets height AGL.  If terrain is not enabled, this parameter sets height above home.  Set zero to ignore height requirement.
    // @Units: m
    // @Range: 0 50
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("HGT_MIN", 4, AC_WeatherVane, _min_height, 2),

    // @Param: VXY_MAX
    // @DisplayName: Weathervaning max ground speed
    // @Description: Below this hoizontal velocity weathervaning is permitted.  Based on ground speed.  Set to 0 to ignore this condition when checking if vehicle should weathervane.
    // @Units: m/s
    // @Range: 0 50
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("VXY_MAX", 5, AC_WeatherVane, _max_vel_xy, 2.0f),

    // @Param: VZ_MAX
    // @DisplayName: Weathervaning max vertical speed
    // @Description: The maximum climb or descent speed that the vehicle will still attempt to weathervane.  Set to 0 to ignore this condition to get the aircraft to weathervane at any climb/descent rate.
    // @Units: m/s
    // @Range: 0 5
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("VZ_MAX", 6, AC_WeatherVane, _max_vel_z, 1.0f),

    AP_GROUPEND
};


// Constructor
AC_WeatherVane::AC_WeatherVane(const AP_InertialNav& inav) :
    _inav(inav)
{
    reset();
    AP_Param::setup_object_defaults(this, var_info);
}

float AC_WeatherVane::get_yaw_rate_cds(const int16_t roll_cdeg, const int16_t pitch_cdeg)
{
    if (!active_msg_sent) {
        gcs().send_text(MAV_SEVERITY_INFO, "Weathervane Active");
        active_msg_sent = true;
    }

    float output = 0.0f;
    switch (get_direction()) {
        case Direction::OFF:
            return 0.0f;

        case Direction::NOSE_OR_TAIL_IN:
            output = roll_cdeg * _gain;

            if (pitch_cdeg > 0) {
                output *= -1.0f;
            }
            break;

        case Direction::NOSE_IN:
            output = (fabsf(roll_cdeg) + MAX(pitch_cdeg,0.0f)) * _gain;

            // Yaw in the direction of the lowest 'wing'
            if (roll_cdeg < 0) {
                output *= -1.0f;
            }
            break;

        case Direction::SIDE_IN:
            output = pitch_cdeg * _gain;

            if (roll_cdeg < 0) {
                output *= -1.0f;
            }
            break;

    }

    // Don't activatively weather vane if less than deadzone angle
    float angle = get_direction() == Direction::SIDE_IN ? pitch_cdeg : roll_cdeg;
    if (fabsf(angle) < _min_dz_ang_deg*100.0f && !(pitch_cdeg > _min_dz_ang_deg*100.0f && get_direction() == Direction::NOSE_IN)) {
        output = 0.0f;
    }

    // Force the controller to relax.  This can be called when landing.
    if (should_relax) {
        output = 0.0f;
        // Always reset should relax. Maintain relaxed condition by persistant calls to relax()
        should_relax = false;
    }

    // Slew output
    last_output = 0.98f * last_output + 0.02f * output;

    return last_output;
}

// Called on an interupt to reset the weathervane controller
void AC_WeatherVane::reset(void)
{
    last_output = 0;
    active_msg_sent = false;
    should_relax = false;
    first_activate_ms = 0;
}

bool AC_WeatherVane::should_weathervane(const int16_t pilot_yaw, const int16_t roll_cdeg, const int16_t pitch_cdeg)
{
    // Check enabled
    if (get_direction() == Direction::OFF){
        reset();
        return false;
    }

    const uint32_t now = AP_HAL::millis();

    // Don't fight pilot inputs
    if (pilot_yaw != 0) {
        last_pilot_input_ms = now;
        reset();
        return false;
    }

    // Only allow weather vaning if no input from pilot in last 3 seconds
    if (now - last_pilot_input_ms < 3000) {
        reset();
        return false;
    }

    // Check if we are above the minimum height to weather vane
    if (below_min_height()) {
        reset();
        return false;
    }

    // Check if we meet the horizontal velocity thresholds to allow weathervaning
    if ((_max_vel_xy.get() > 0) && (_inav.get_speed_xy()*0.01f) > _max_vel_xy.get()) {
        reset();
        return false;
    }

    // Check if we meet the vertical velocity thresholds to allow weathervaning
    if ((_max_vel_z.get() > 0) && fabsf(_inav.get_velocity_z())*0.01f > _max_vel_z.get()) {
        reset();
        return false;
    }

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

    // If we got this far then we should allow weathervaning
    return true;
}

bool AC_WeatherVane::below_min_height(void)
{
    // Check min height is set
    if (_min_height.get() <= 0) {
        return false;
    }

#if AP_TERRAIN_AVAILABLE
    AP_Terrain* terrain = AP_Terrain::get_singleton();
    if (terrain != nullptr) {
        float terr_height = 0.0f;
        if (terrain->enabled() && terrain->height_above_terrain(terr_height, true) && terr_height >= (float)_min_height.get()) {
            return false;
        }
    }
#endif

    if (_inav.get_altitude() >= (float)_min_height.get()) {
        return false;
    }

    return true;
}
#endif
