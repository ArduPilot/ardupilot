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

#include "AC_Avoidance_config.h"

#if AP_AVOIDANCE_ENABLED

#include "AC_Avoid.h"
#include <AP_AHRS/AP_AHRS.h>     // AHRS library
#include <AC_Fence/AC_Fence.h>         // Failsafe fence library
#include <AP_Proximity/AP_Proximity.h>
#include <AP_Beacon/AP_Beacon.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <stdio.h>

#if !APM_BUILD_TYPE(APM_BUILD_ArduPlane)

#if APM_BUILD_TYPE(APM_BUILD_Rover)
 # define AP_AVOID_BEHAVE_DEFAULT AC_Avoid::BehaviourType::BEHAVIOR_STOP
#else
 # define AP_AVOID_BEHAVE_DEFAULT AC_Avoid::BehaviourType::BEHAVIOR_SLIDE
#endif

#if APM_BUILD_COPTER_OR_HELI
    # define AP_AVOID_ENABLE_Z          1
#endif

const AP_Param::GroupInfo AC_Avoid::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Avoidance control enable/disable
    // @Description: Enabled/disable avoidance input sources
    // @Bitmask: 0:UseFence,1:UseProximitySensor,2:UseBeaconFence
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1,  AC_Avoid, _enabled, AC_AVOID_DEFAULT, AP_PARAM_FLAG_ENABLE),

    // @Param{Copter}: ANGLE_MAX
    // @DisplayName: Avoidance max lean angle in non-GPS flight modes
    // @Description: Max lean angle used to avoid obstacles while in non-GPS modes
    // @Units: cdeg
    // @Increment: 10
    // @Range: 0 4500
    // @User: Standard
    AP_GROUPINFO_FRAME("ANGLE_MAX", 2,  AC_Avoid, _angle_max_cd, 1000, AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_HELI | AP_PARAM_FRAME_TRICOPTER),

    // @Param{Copter}: DIST_MAX
    // @DisplayName: Avoidance distance maximum in non-GPS flight modes
    // @Description: Distance from object at which obstacle avoidance will begin in non-GPS modes
    // @Units: m
    // @Range: 1 30
    // @User: Standard
    AP_GROUPINFO_FRAME("DIST_MAX", 3,  AC_Avoid, _dist_max_m, AC_AVOID_NONGPS_DIST_MAX_DEFAULT, AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_HELI | AP_PARAM_FRAME_TRICOPTER),

    // @Param: MARGIN
    // @DisplayName: Avoidance distance margin in GPS modes
    // @Description: Vehicle will attempt to stay at least this distance (in meters) from objects while in GPS modes
    // @Units: m
    // @Range: 1 10
    // @User: Standard
    AP_GROUPINFO("MARGIN", 4, AC_Avoid, _margin_m, 2.0f),

    // @Param{Copter, Rover}: BEHAVE
    // @DisplayName: Avoidance behaviour
    // @Description: Avoidance behaviour (slide or stop)
    // @Values: 0:Slide,1:Stop
    // @User: Standard
    AP_GROUPINFO_FRAME("BEHAVE", 5, AC_Avoid, _behavior, AP_AVOID_BEHAVE_DEFAULT, AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_HELI | AP_PARAM_FRAME_TRICOPTER | AP_PARAM_FRAME_ROVER),

    // @Param: BACKUP_SPD
    // @DisplayName: Avoidance maximum horizontal backup speed
    // @Description: Maximum speed that will be used to back away from obstacles horizontally in position control modes (m/s). Set zero to disable horizontal backup.
    // @Units: m/s
    // @Range: 0 2
    // @User: Standard
    AP_GROUPINFO("BACKUP_SPD", 6, AC_Avoid, _backup_speed_max_ne_ms, 0.75f),

    // @Param{Copter}: ALT_MIN
    // @DisplayName: Avoidance minimum altitude
    // @Description: Minimum altitude above which proximity based avoidance will start working. This requires a valid downward facing rangefinder reading to work. Set zero to disable
    // @Units: m
    // @Range: 0 6
    // @User: Standard
    AP_GROUPINFO_FRAME("ALT_MIN", 7, AC_Avoid, _alt_min_m, 0.0f, AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_HELI | AP_PARAM_FRAME_TRICOPTER),

    // @Param: ACCEL_MAX
    // @DisplayName: Avoidance maximum acceleration
    // @Description: Maximum acceleration with which obstacles will be avoided with. Set zero to disable acceleration limits
    // @Units: m/s/s
    // @Range: 0 9
    // @User: Standard
    AP_GROUPINFO("ACCEL_MAX", 8, AC_Avoid, _accel_max_mss, 3.0f),

    // @Param: BACKUP_DZ
    // @DisplayName: Avoidance deadzone between stopping and backing away from obstacle
    // @Description: Distance beyond AVOID_MARGIN parameter, after which vehicle will backaway from obstacles. Increase this parameter if you see vehicle going back and forth in front of obstacle.
    // @Units: m
    // @Range: 0 2
    // @User: Standard
    AP_GROUPINFO("BACKUP_DZ", 9, AC_Avoid, _backup_deadzone_m, 0.10f),

    // @Param: BACKZ_SPD
    // @DisplayName: Avoidance maximum vertical backup speed
    // @Description: Maximum speed that will be used to back away from obstacles vertically in height control modes (m/s). Set zero to disable vertical backup.
    // @Units: m/s
    // @Range: 0 2
    // @User: Standard
    AP_GROUPINFO("BACKZ_SPD", 10, AC_Avoid, _backup_speed_max_u_ms, 0.75),

    AP_GROUPEND
};

/// Constructor
AC_Avoid::AC_Avoid()
{
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

/*
* This method limits velocity and calculates backaway velocity from various supported fences
* Also limits vertical velocity using adjust_velocity_z method
*/
void AC_Avoid::adjust_velocity_fence(float kP, float accel_cmss, Vector3f &desired_vel_neu_cms, Vector3f &backup_vel_neu_cms, float kP_z, float accel_z_cmss, float dt)
{   
    // Only horizontal component needed for most fences, since fences are 2D
    Vector2f desired_velocity_ne_cms{desired_vel_neu_cms.x, desired_vel_neu_cms.y};

#if AP_FENCE_ENABLED || AP_BEACON_ENABLED
    // limit acceleration
    const float accel_limited_cmss = MIN(accel_cmss, AC_AVOID_ACCEL_CMSS_MAX);
#endif

    // maximum component of desired  backup velocity in each quadrant 
    Vector2f quad_1_back_vel_ne_cms, quad_2_back_vel_ne_cms, quad_3_back_vel_ne_cms, quad_4_back_vel_ne_cms;

#if AP_FENCE_ENABLED
    if ((_enabled & AC_AVOID_STOP_AT_FENCE) > 0) {
        // Store velocity needed to back away from fence
        Vector2f backup_vel_fence_ne_cms;

        adjust_velocity_circle_fence(kP, accel_limited_cmss, desired_velocity_ne_cms, backup_vel_fence_ne_cms, dt);
        find_max_quadrant_velocity(backup_vel_fence_ne_cms, quad_1_back_vel_ne_cms, quad_2_back_vel_ne_cms, quad_3_back_vel_ne_cms, quad_4_back_vel_ne_cms);
        
        // backup_vel_fence_ne_cms is set to zero after each fence in case the velocity is unset from previous methods
        backup_vel_fence_ne_cms.zero();
        adjust_velocity_inclusion_and_exclusion_polygons(kP, accel_limited_cmss, desired_velocity_ne_cms, backup_vel_fence_ne_cms, dt);
        find_max_quadrant_velocity(backup_vel_fence_ne_cms, quad_1_back_vel_ne_cms, quad_2_back_vel_ne_cms, quad_3_back_vel_ne_cms, quad_4_back_vel_ne_cms);
        
        backup_vel_fence_ne_cms.zero();
        adjust_velocity_inclusion_circles(kP, accel_limited_cmss, desired_velocity_ne_cms, backup_vel_fence_ne_cms, dt);
        find_max_quadrant_velocity(backup_vel_fence_ne_cms, quad_1_back_vel_ne_cms, quad_2_back_vel_ne_cms, quad_3_back_vel_ne_cms, quad_4_back_vel_ne_cms);
        
        backup_vel_fence_ne_cms.zero();
        adjust_velocity_exclusion_circles(kP, accel_limited_cmss, desired_velocity_ne_cms, backup_vel_fence_ne_cms, dt);
        find_max_quadrant_velocity(backup_vel_fence_ne_cms, quad_1_back_vel_ne_cms, quad_2_back_vel_ne_cms, quad_3_back_vel_ne_cms, quad_4_back_vel_ne_cms);
    }
#endif // AP_FENCE_ENABLED

#if AP_BEACON_ENABLED
    if ((_enabled & AC_AVOID_STOP_AT_BEACON_FENCE) > 0) {
        // Store velocity needed to back away from beacon fence
        Vector2f backup_vel_beacon_ne_cms;
        adjust_velocity_beacon_fence(kP, accel_limited_cmss, desired_velocity_ne_cms, backup_vel_beacon_ne_cms, dt);
        find_max_quadrant_velocity(backup_vel_beacon_ne_cms, quad_1_back_vel_ne_cms, quad_2_back_vel_ne_cms, quad_3_back_vel_ne_cms, quad_4_back_vel_ne_cms);
    }
#endif // AP_BEACON_ENABLED

    // check for vertical fence
    float desired_velocity_z_cms = desired_vel_neu_cms.z;
    float desired_backup_vel_u_cms = 0.0f;
    adjust_velocity_z(kP_z, accel_z_cmss, desired_velocity_z_cms, desired_backup_vel_u_cms, dt);

    // Desired backup velocity is sum of maximum velocity component in each quadrant 
    const Vector2f desired_backup_vel_ne_cms = quad_1_back_vel_ne_cms + quad_2_back_vel_ne_cms + quad_3_back_vel_ne_cms + quad_4_back_vel_ne_cms;
    backup_vel_neu_cms = Vector3f{desired_backup_vel_ne_cms.x, desired_backup_vel_ne_cms.y, desired_backup_vel_u_cms};
    desired_vel_neu_cms = Vector3f{desired_velocity_ne_cms.x, desired_velocity_ne_cms.y, desired_velocity_z_cms};
}

/*
* Adjusts the desired velocity so that the vehicle can stop
* before the fence/object.
* kP, accel_cmss are for the horizontal axis
* kP_z, accel_z_cmss are for vertical axis
*/
void AC_Avoid::adjust_velocity(Vector3f &desired_vel_neu_cms, bool &backing_up, float kP, float accel_cmss, float kP_z, float accel_z_cmss, float dt)
{
    // exit immediately if disabled
    if (_enabled == AC_AVOID_DISABLED) {
        return;
    }

    // make a copy of input velocity, because desired_vel_neu_cms might be changed
    const Vector3f desired_vel_original_neu_cms = desired_vel_neu_cms;

    // limit acceleration
    const float accel_limited_cmss = MIN(accel_cmss, AC_AVOID_ACCEL_CMSS_MAX);

    // maximum component of horizontal desired  backup velocity in each quadrant 
    Vector2f quad_1_back_vel_ne_cms, quad_2_back_vel_ne_cms, quad_3_back_vel_ne_cms, quad_4_back_vel_ne_cms;
    float back_vel_up_cms = 0.0f;
    float back_vel_down_cms = 0.0f;
    
    // Avoidance in response to proximity sensor
    if (proximity_avoidance_enabled() && _proximity_alt_enabled) {
        // Store velocity needed to back away from physical obstacles
        Vector3f backup_vel_proximity_neu_cms;
        adjust_velocity_proximity(kP, accel_limited_cmss, desired_vel_neu_cms, backup_vel_proximity_neu_cms, kP_z,accel_z_cmss, dt);
        find_max_quadrant_velocity_3D(backup_vel_proximity_neu_cms, quad_1_back_vel_ne_cms, quad_2_back_vel_ne_cms, quad_3_back_vel_ne_cms, quad_4_back_vel_ne_cms, back_vel_up_cms, back_vel_down_cms);
    }
    
    // Avoidance in response to various fences 
    Vector3f backup_vel_fence_neu_cms;
    adjust_velocity_fence(kP, accel_cmss, desired_vel_neu_cms, backup_vel_fence_neu_cms, kP_z, accel_z_cmss, dt);
    find_max_quadrant_velocity_3D(backup_vel_fence_neu_cms , quad_1_back_vel_ne_cms, quad_2_back_vel_ne_cms, quad_3_back_vel_ne_cms, quad_4_back_vel_ne_cms, back_vel_up_cms, back_vel_down_cms);
    
    // Desired backup velocity is sum of maximum velocity component in each quadrant
    const Vector2f desired_backup_vel_ne_cms = quad_1_back_vel_ne_cms + quad_2_back_vel_ne_cms + quad_3_back_vel_ne_cms + quad_4_back_vel_ne_cms;
    const float desired_backup_vel_u_cms = back_vel_down_cms + back_vel_up_cms;
    Vector3f desired_backup_vel_neu_cm{desired_backup_vel_ne_cms.x, desired_backup_vel_ne_cms.y, desired_backup_vel_u_cms};

    const float backup_speed_max_ne_cms = _backup_speed_max_ne_ms * 100.0;
    if (!desired_backup_vel_neu_cm.xy().is_zero() && is_positive(backup_speed_max_ne_cms)) {
        backing_up = true;
        // Constrain horizontal backing away speed
        desired_backup_vel_neu_cm.xy().limit_length(backup_speed_max_ne_cms);

        // let user take control if they are backing away at a greater speed than what we have calculated
        // this has to be done for x,y,z separately. For eg, user is doing fine in "x" direction but might need backing up in "y".
        if (!is_zero(desired_backup_vel_neu_cm.x)) {
            if (is_positive(desired_backup_vel_neu_cm.x)) {
                desired_vel_neu_cms.x = MAX(desired_vel_neu_cms.x, desired_backup_vel_neu_cm.x);
            } else {
                desired_vel_neu_cms.x = MIN(desired_vel_neu_cms.x, desired_backup_vel_neu_cm.x);
            }
        }
        if (!is_zero(desired_backup_vel_neu_cm.y)) {
            if (is_positive(desired_backup_vel_neu_cm.y)) {
                desired_vel_neu_cms.y = MAX(desired_vel_neu_cms.y, desired_backup_vel_neu_cm.y);
            } else {
                desired_vel_neu_cms.y = MIN(desired_vel_neu_cms.y, desired_backup_vel_neu_cm.y);
            }
        }
    }

    const float backup_speed_max_u_cms = _backup_speed_max_u_ms * 100.0;
    if (!is_zero(desired_backup_vel_neu_cm.z) && is_positive(backup_speed_max_u_cms)) {
        backing_up = true;

        // Constrain vertical backing away speed
        desired_backup_vel_neu_cm.z = constrain_float(desired_backup_vel_neu_cm.z, -backup_speed_max_u_cms, backup_speed_max_u_cms);

        if (!is_zero(desired_backup_vel_neu_cm.z)) {
            if (is_positive(desired_backup_vel_neu_cm.z)) {
                desired_vel_neu_cms.z = MAX(desired_vel_neu_cms.z, desired_backup_vel_neu_cm.z);
            } else {
                desired_vel_neu_cms.z = MIN(desired_vel_neu_cms.z, desired_backup_vel_neu_cm.z);
            }
        }
    }

    // limit acceleration
    limit_accel_NEU_cm(desired_vel_original_neu_cms, desired_vel_neu_cms, dt);

    if (desired_vel_original_neu_cms != desired_vel_neu_cms) {
        _last_limit_time = AP_HAL::millis();
    }

#if HAL_LOGGING_ENABLED
    if (limits_active()) {
        // log at not more than 10hz (adjust_velocity method can be potentially called at 400hz!)
        uint32_t now = AP_HAL::millis();
        if ((now - _last_log_ms) > 100) {
            _last_log_ms = now;
            Write_SimpleAvoidance(true, desired_vel_original_neu_cms, desired_vel_neu_cms, backing_up);
        }
    } else {
        // avoidance isn't active anymore
        // log once so that it registers in logs
        if (_last_log_ms) {
            Write_SimpleAvoidance(false, desired_vel_original_neu_cms, desired_vel_neu_cms, backing_up);
            // this makes sure logging won't run again till it is active
            _last_log_ms = 0;
        }
    }
#endif
}

/*
* Limit acceleration so that change of velocity output by avoidance library is controlled
* This helps reduce jerks and sudden movements in the vehicle
*/
void AC_Avoid::limit_accel_NEU_cm(const Vector3f &original_vel_neu_cms, Vector3f &modified_vel_neu_cms, float dt)
{
    if (original_vel_neu_cms == modified_vel_neu_cms || is_zero(_accel_max_mss) || !is_positive(dt)) {
        // we can't limit accel if any of these conditions are true
        return;
    }

    if (AP_HAL::millis() - _last_limit_time > AC_AVOID_ACCEL_TIMEOUT_MS) {
        // reset this velocity because its been a long time since avoidance was active
        _prev_avoid_vel_neu_cms = original_vel_neu_cms;
    }

    // acceleration demanded by avoidance
    const Vector3f accel_neu_cmss = (modified_vel_neu_cms - _prev_avoid_vel_neu_cms)/dt;

    // max accel in cm
    const float accel_max_cmss = _accel_max_mss * 100.0f;

    if (accel_neu_cmss.length() > accel_max_cmss) {
        // pull back on the acceleration
        const Vector3f accel_direction_neu = accel_neu_cmss.normalized();
        modified_vel_neu_cms = (accel_direction_neu * accel_max_cmss) * dt + _prev_avoid_vel_neu_cms;
    }

    _prev_avoid_vel_neu_cms = modified_vel_neu_cms;
    return;
}

// This method is used in most Rover modes and not in Copter
// adjust desired horizontal speed so that the vehicle stops before the fence or object
// accel (maximum acceleration/deceleration) is in m/s/s
// heading is in radians
// speed is in m/s
// kP should be zero for linear response, non-zero for non-linear response
void AC_Avoid::adjust_speed(float kP, float accel_mss, float heading_rad, float &speed_ms, float dt)
{
    // convert heading and speed into velocity vector
    Vector3f vel_neu_cms{
        cosf(heading_rad) * speed_ms * 100.0f,
        sinf(heading_rad) * speed_ms * 100.0f,
        0.0f
    };

    bool backing_up  = false;
    adjust_velocity(vel_neu_cms, backing_up, kP, accel_mss * 100.0f, 0, 0, dt);
    const Vector2f vel_ne_cms{vel_neu_cms.x, vel_neu_cms.y};

    if (backing_up) {
        // back up
        if (fabsf(wrap_180(degrees(vel_ne_cms.angle())) - degrees(heading_rad)) > 90.0f) {
            // Big difference between the direction of velocity vector and actual heading therefore we need to reverse the direction
            speed_ms = -vel_ne_cms.length() * 0.01f;
        } else {
            speed_ms = vel_ne_cms.length() * 0.01f;
        }
        return;
    }

    // No need to back up so adjust speed towards zero if needed
    if (is_negative(speed_ms)) {
        speed_ms = -vel_ne_cms.length() * 0.01f;
    } else {
        speed_ms = vel_ne_cms.length() * 0.01f;
    }
}

// adjust vertical climb rate so vehicle does not break the vertical fence
void AC_Avoid::adjust_velocity_z(float kP, float accel_cmss, float& climb_rate_cms, float dt) {
    float backup_speed_cms = 0.0f;
    adjust_velocity_z(kP, accel_cmss, climb_rate_cms, backup_speed_cms, dt);
    if (!is_zero(backup_speed_cms)) {
        if (is_negative(backup_speed_cms)) {
            climb_rate_cms = MIN(climb_rate_cms, backup_speed_cms);
        } else {
            climb_rate_cms = MAX(climb_rate_cms, backup_speed_cms);
        }
    }
}

// adjust vertical climb rate so vehicle does not break the vertical fence
void AC_Avoid::adjust_velocity_z(float kP, float accel_cmss, float& climb_rate_cms, float& backup_speed_cms, float dt)
{
#ifdef AP_AVOID_ENABLE_Z

    // exit immediately if disabled
    if (_enabled == AC_AVOID_DISABLED) {
        return;
    }
    
    // do not adjust climb_rate if level
    if (is_zero(climb_rate_cms)) {
        return;
    }

    const AP_AHRS &_ahrs = AP::ahrs();
    // limit acceleration
    const float accel_limited_cmss = MIN(accel_cmss, AC_AVOID_ACCEL_CMSS_MAX);

    bool limit_min_alt = false;
    bool limit_max_alt = false;
    float max_alt_diff_m = 0.0f; // distance from altitude limit to vehicle in metres (positive means vehicle is below limit)
    float min_alt_diff_m = 0.0f;
#if AP_FENCE_ENABLED
    // calculate distance below fence
    AC_Fence *fence = AP::fence();
    if ((_enabled & AC_AVOID_STOP_AT_FENCE) > 0 && fence) {
        // calculate distance from vehicle to safe altitude
        float veh_alt_m;
        _ahrs.get_relative_position_D_home(veh_alt_m);
        if ((fence->get_enabled_fences() & AC_FENCE_TYPE_ALT_MIN) > 0) {
            // fence.get_safe_alt_max_m() is UP, veh_alt_m is DOWN:
            min_alt_diff_m = -(fence->get_safe_alt_min_m() + veh_alt_m);
            limit_min_alt = true;
        }
        if ((fence->get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) > 0) {
            // fence.get_safe_alt_max_m() is UP, veh_alt_m is DOWN:
            max_alt_diff_m = fence->get_safe_alt_max_m() + veh_alt_m;
            limit_max_alt = true;
        }
    }
#endif

    // calculate distance to (e.g.) optical flow altitude limit
    // AHRS values are always in metres
    float alt_limit_m;
    float curr_alt_m;
    if (_ahrs.get_hgt_ctrl_limit(alt_limit_m) &&
        _ahrs.get_relative_position_D_origin_float(curr_alt_m)) {
        // alt_limit_m is UP, curr_alt_m is DOWN:
        const float ctrl_alt_diff_m = alt_limit_m + curr_alt_m;
        if (!limit_max_alt || ctrl_alt_diff_m < max_alt_diff_m) {
            max_alt_diff_m = ctrl_alt_diff_m;
            limit_max_alt = true;
        }
    }

#if HAL_PROXIMITY_ENABLED
    // get distance from proximity sensor
    float proximity_alt_diff_m;
    AP_Proximity *proximity = AP::proximity();
    if (proximity && proximity_avoidance_enabled() && proximity->get_upward_distance(proximity_alt_diff_m)) {
        proximity_alt_diff_m -= _margin_m;
        if (!limit_max_alt || proximity_alt_diff_m < max_alt_diff_m) {
            max_alt_diff_m = proximity_alt_diff_m;
            limit_max_alt = true;
        }
    }
#endif

    // limit climb rate
    if (limit_max_alt || limit_min_alt) {
        const float max_back_spd_cms = _backup_speed_max_u_ms * 100.0;
        // do not allow climbing if we've breached the safe altitude
        if (max_alt_diff_m <= 0.0f && limit_max_alt) {
            climb_rate_cms = MIN(climb_rate_cms, 0.0f);
            // also calculate backup speed that will get us back to safe altitude
            if (is_positive(max_back_spd_cms)) {
                backup_speed_cms = -1*(get_max_speed(kP, accel_limited_cmss, -max_alt_diff_m * 100.0f, dt));

                // Constrain to max backup speed
                backup_speed_cms = MAX(backup_speed_cms, -max_back_spd_cms);
            }
            return;
        // do not allow descending if we've breached the safe altitude
        } else if (min_alt_diff_m <= 0.0f && limit_min_alt) {
            climb_rate_cms =  MAX(climb_rate_cms, 0.0f);
            // also calculate backup speed that will get us back to safe altitude
            if (is_positive(max_back_spd_cms)) {
                backup_speed_cms = get_max_speed(kP, accel_limited_cmss, -min_alt_diff_m * 100.0f, dt);

                // Constrain to max backup speed
                backup_speed_cms = MIN(backup_speed_cms, max_back_spd_cms);
            }
            return;
        }

        // limit climb rate
        if (limit_max_alt) {
            const float max_alt_max_speed_cms = get_max_speed(kP, accel_limited_cmss, max_alt_diff_m * 100.0f, dt);
            climb_rate_cms = MIN(max_alt_max_speed_cms, climb_rate_cms);
        }

        if (limit_min_alt) {
            const float max_alt_min_speed = get_max_speed(kP, accel_limited_cmss, min_alt_diff_m * 100.0f, dt);
            climb_rate_cms = MAX(-max_alt_min_speed, climb_rate_cms);
        }
    }
#endif
}

// adjust roll-pitch to push vehicle away from objects
// roll and pitch value are in radians
// veh_angle_max_rad is the user defined maximum lean angle for the vehicle in radians
void AC_Avoid::adjust_roll_pitch_rad(float &roll_rad, float &pitch_rad, float veh_angle_max_rad) const
{
    // exit immediately if proximity based avoidance is disabled
    if (!proximity_avoidance_enabled()) {
        return;
    }

    // exit immediately if angle max is zero
    if (_angle_max_cd <= 0.0f || veh_angle_max_rad <= 0.0f) {
        return;
    }

    float roll_positive_norm = 0.0f;    // maximum positive roll value
    float roll_negative_norm = 0.0f;    // minimum negative roll value
    float pitch_positive_norm = 0.0f;   // maximum positive pitch value
    float pitch_negative_norm = 0.0f;   // minimum negative pitch value

    // get maximum positive and negative roll and pitch percentages from proximity sensor
    get_proximity_roll_pitch_norm(roll_positive_norm, roll_negative_norm, pitch_positive_norm, pitch_negative_norm);

    // add maximum positive and negative percentages together for roll and pitch, convert to radians
    Vector2f rp_out_rad((roll_positive_norm + roll_negative_norm) * radians(45.0), (pitch_positive_norm + pitch_negative_norm) * radians(45.0));

    // apply avoidance angular limits
    // the object avoidance lean angle is never more than 75% of the total angle-limit to allow the pilot to override
    const float angle_limit_rad = constrain_float(cd_to_rad(_angle_max_cd), 0.0f, veh_angle_max_rad * AC_AVOID_ANGLE_MAX_PERCENT);
    float vec_length_rad = rp_out_rad.length();
    if (vec_length_rad > angle_limit_rad) {
        rp_out_rad *= (angle_limit_rad / vec_length_rad);
    }

    // add passed in roll, pitch angles
    rp_out_rad.x += roll_rad;
    rp_out_rad.y += pitch_rad;

    // apply total angular limits
    vec_length_rad = rp_out_rad.length();
    if (vec_length_rad > veh_angle_max_rad) {
        rp_out_rad *= (veh_angle_max_rad / vec_length_rad);
    }

    // return adjusted roll, pitch
    roll_rad = rp_out_rad.x;
    pitch_rad = rp_out_rad.y;
}

/*
 * Note: This method is used to limit velocity horizontally only 
 * Limits the component of desired_vel in the direction of the unit vector
 * limit_direction_ne to be at most the maximum speed permitted by the limit_distance.
 *
 * The function is unit-agnostic — accel, desired_vel_ne, and limit_direction_ne
 * must all use the same base unit (e.g. m, cm) for correct scaling.
 *
 * Uses velocity adjustment idea from Randy's second email on this thread:
 * https://groups.google.com/forum/#!searchin/drones-discuss/obstacle/drones-discuss/QwUXz__WuqY/qo3G8iTLSJAJ
 */
void AC_Avoid::limit_velocity_NE(float kP, float accel, Vector2f &desired_vel_ne, const Vector2f& limit_direction_ne, float limit_distance, float dt) const
{
    const float max_speed = get_max_speed(kP, accel, limit_distance, dt);
    // project onto limit direction
    const float speed = desired_vel_ne * limit_direction_ne;
    if (speed > max_speed) {
        // subtract difference between desired speed and maximum acceptable speed
        desired_vel_ne += limit_direction_ne * (max_speed - speed);
    }
}

/*
 * Note: This method is used to limit velocity horizontally and vertically given a 3D desired velocity vector 
 * Limits the component of desired_vel_neu in the direction of the obstacle_vector_neu based on the passed value of "margin"
 *
 * The function is unit-agnostic — accel, desired_vel_neu, obstacle_vector_neu, margin, and accel_u
 * must all use the same base unit (e.g. m, cm) for correct scaling.
 */
void AC_Avoid::limit_velocity_NEU(float kP, float accel, Vector3f &desired_vel_neu, const Vector3f& obstacle_vector_neu, float margin, float kP_z, float accel_u, float dt) const
{  
    if (desired_vel_neu.is_zero()) {
        // nothing to limit
        return;
    }
    // create a margin length vector in the direction of desired_vel_cms
    // this will create larger margin towards the direction vehicle is travelling in
    const Vector3f margin_vector_neu = desired_vel_neu.normalized() * margin;
    const Vector2f limit_direction_ne{obstacle_vector_neu.x, obstacle_vector_neu.y};
    
    if (!limit_direction_ne.is_zero()) {
        const float distance_from_fence_xy = MAX((limit_direction_ne.length() - Vector2f{margin_vector_neu.x, margin_vector_neu.y}.length()), 0.0f);
        Vector2f velocity_ne{desired_vel_neu.x, desired_vel_neu.y};
        limit_velocity_NE(kP, accel, velocity_ne, limit_direction_ne.normalized(), distance_from_fence_xy, dt);
        desired_vel_neu.x = velocity_ne.x;
        desired_vel_neu.y = velocity_ne.y;
    }
    
    if (is_zero(desired_vel_neu.z) || is_zero(obstacle_vector_neu.z)) {
        // nothing to limit vertically if desired_vel_cms.z is zero
        // if obstacle_vector_neu.z is zero then the obstacle is probably horizontally located, and we can move vertically
        return;
    }

    if (is_positive(desired_vel_neu.z) != is_positive(obstacle_vector_neu.z)) {
        // why limit velocity vertically when we are going the opposite direction
        return;
    }
    
    // to check if Z velocity changes
    const float velocity_original_u = desired_vel_neu.z;
    const float speed_u = fabsf(desired_vel_neu.z);

    // obstacle_vector_neu.z and margin_vector_neu.z should be in same direction as checked above
    const float dist_u = MAX(fabsf(obstacle_vector_neu.z) - fabsf(margin_vector_neu.z), 0.0f); 
    if (is_zero(dist_u)) {
        // eliminate any vertical velocity 
        desired_vel_neu.z = 0.0f;
    } else {
        const float max_z_speed = get_max_speed(kP_z, accel_u, dist_u, dt);
        desired_vel_neu.z = MIN(max_z_speed, speed_u);
    }

    // make sure the direction of the Z velocity did not change
    // we are only limiting speed here, not changing directions 
    // check if original z velocity is positive or negative
    if (is_negative(velocity_original_u)) {
        desired_vel_neu.z = desired_vel_neu.z * -1.0f;
    }
}

/*
 * Compute the back away horizontal velocity required to avoid breaching margin
 * INPUT: This method requires the breach in margin distance (back_distance_cm), direction towards the breach (limit_direction)
 *        It then calculates the desired backup velocity and passes it on to "find_max_quadrant_velocity" method to distribute the velocity vectors into respective quadrants
 * OUTPUT: The method then outputs four velocities (quad1/2/3/4_back_vel_cms), which correspond to the maximum horizontal desired backup velocity in each quadrant
 */
void AC_Avoid::calc_backup_velocity_2D(float kP, float accel_cmss, Vector2f &quad1_back_vel_cms, Vector2f &quad2_back_vel_cms, Vector2f &quad3_back_vel_cms, Vector2f &quad4_back_vel_cms, float back_distance_cm, Vector2f limit_direction, float dt) const
{      
    if (limit_direction.is_zero()) {
        // protect against divide by zero
        return; 
    }
    // speed required to move away the exact distance that we have breached the margin with 
    const float back_speed_cms = get_max_speed(kP, 0.4f * accel_cmss, fabsf(back_distance_cm), dt);
    
    // direction to the obstacle
    limit_direction.normalize();

    // move in the opposite direction with the required speed
    Vector2f back_direction_vel_cms = limit_direction * (-back_speed_cms);
    // divide the vector into quadrants, find maximum velocity component in each quadrant 
    find_max_quadrant_velocity(back_direction_vel_cms, quad1_back_vel_cms, quad2_back_vel_cms, quad3_back_vel_cms, quad4_back_vel_cms);
}

/*
* Compute the back away velocity required to avoid breaching margin, including vertical component
* min_z_vel is <= 0, and stores the greatest velocity in the downwards direction
* max_z_vel is >= 0, and stores the greatest velocity in the upwards direction
* eventually max_z_vel + min_z_vel will give the final desired Z backaway velocity
*/
void AC_Avoid::calc_backup_velocity_3D(float kP, float accel_cmss, Vector2f &quad1_back_vel_cms, Vector2f &quad2_back_vel_cms, Vector2f &quad3_back_vel_cms, Vector2f &quad4_back_vel_cms, 
                                        float back_distance_cms, Vector3f limit_direction_neu, float kp_z, float accel_z_cmss, float back_distance_u_cm, float& min_vel_u_cms, float& max_vel_u_cms, float dt) const
{   
    // backup horizontally 
    if (is_positive(back_distance_cms)) {
        Vector2f limit_direction_ne{limit_direction_neu.x, limit_direction_neu.y};
        calc_backup_velocity_2D(kP, accel_cmss, quad1_back_vel_cms, quad2_back_vel_cms, quad3_back_vel_cms, quad4_back_vel_cms, back_distance_cms, limit_direction_ne, dt);
    }

    // backup vertically 
    if (!is_zero(back_distance_u_cm)) {
        float back_speed_z_cms = get_max_speed(kp_z, 0.4f * accel_z_cmss, fabsf(back_distance_u_cm), dt);
        // Down is positive
        if (is_positive(back_distance_u_cm)) {
            back_speed_z_cms *= -1.0f;
        } 

        // store the z backup speed into min or max z if possible
        if (back_speed_z_cms < min_vel_u_cms) {
            min_vel_u_cms = back_speed_z_cms;  
        }
        if (back_speed_z_cms > max_vel_u_cms) {
            max_vel_u_cms = back_speed_z_cms;
        }
    }
}

/*
 * Calculate maximum velocity vector that can be formed in each quadrant 
 * This method takes the desired backup velocity, and four other velocities corresponding to each quadrant
 * The desired velocity is then fit into one of the 4 quadrant velocities as per the sign of its components
 * This ensures that if we have multiple backup velocities, we can get the maximum of all of those velocities in each quadrant
*/
void AC_Avoid::find_max_quadrant_velocity(Vector2f &desired_vel, Vector2f &quad1_vel, Vector2f &quad2_vel, Vector2f &quad3_vel, Vector2f &quad4_vel)  const
{   
    if (desired_vel.is_zero()) {
        return;
    }
     // first quadrant: +ve x, +ve y direction
    if (is_positive(desired_vel.x) && is_positive(desired_vel.y)) {
        quad1_vel = Vector2f{MAX(quad1_vel.x, desired_vel.x), MAX(quad1_vel.y,desired_vel.y)};
    }
    // second quadrant: -ve x, +ve y direction
    if (is_negative(desired_vel.x) && is_positive(desired_vel.y)) {
        quad2_vel = Vector2f{MIN(quad2_vel.x, desired_vel.x), MAX(quad2_vel.y,desired_vel.y)};
    }
    // third quadrant: -ve x, -ve y direction
    if (is_negative(desired_vel.x) && is_negative(desired_vel.y)) {
        quad3_vel = Vector2f{MIN(quad3_vel.x, desired_vel.x), MIN(quad3_vel.y,desired_vel.y)};
    }
    // fourth quadrant: +ve x, -ve y direction
    if (is_positive(desired_vel.x) && is_negative(desired_vel.y)) {
        quad4_vel = Vector2f{MAX(quad4_vel.x, desired_vel.x), MIN(quad4_vel.y,desired_vel.y)};
    }
}

/*
Calculate maximum velocity vector that can be formed in each quadrant and separately store max & min of vertical components
*/
void AC_Avoid::find_max_quadrant_velocity_3D(Vector3f &desired_vel, Vector2f &quad1_vel, Vector2f &quad2_vel, Vector2f &quad3_vel, Vector2f &quad4_vel, float &max_z_vel, float &min_z_vel) const
{   
    // split into horizontal and vertical components 
    Vector2f velocity_ne{desired_vel.x, desired_vel.y};
    find_max_quadrant_velocity(velocity_ne, quad1_vel, quad2_vel, quad3_vel, quad4_vel);
    
    // store maximum and minimum of z 
    if (is_positive(desired_vel.z) && (desired_vel.z > max_z_vel)) {
        max_z_vel = desired_vel.z;
    }
    if (is_negative(desired_vel.z) && (desired_vel.z < min_z_vel)) {
        min_z_vel = desired_vel.z;
    }
}

/*
 * Computes the speed such that the stopping distance
 * of the vehicle will be exactly the input distance.
 */
float AC_Avoid::get_max_speed(float kP, float accel, float distance, float dt) const
{
    if (is_zero(kP)) {
        return safe_sqrt(2.0f * distance * accel);
    } else {
        return sqrt_controller(distance, kP, accel, dt);
    }
}

#if AP_FENCE_ENABLED

/*
 * Adjusts the desired velocity for the circular fence.
 */
void AC_Avoid::adjust_velocity_circle_fence(float kP, float accel_cmss, Vector2f &desired_vel_ne_cms, Vector2f &backup_vel_ne_cms, float dt)
{
    AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return;
    }

    AC_Fence &_fence = *fence;

    // exit if circular fence is not enabled
    if ((_fence.get_enabled_fences() & AC_FENCE_TYPE_CIRCLE) == 0) {
        return;
    }

    // exit if the circular fence has already been breached
    if ((_fence.get_breaches() & AC_FENCE_TYPE_CIRCLE) != 0) {
        return;
    }

    // get desired speed
    const float desired_speed_cms = desired_vel_ne_cms.length();
    if (is_zero(desired_speed_cms)) {
        // no avoidance necessary when desired speed is zero
        return;
    }

    const AP_AHRS &_ahrs = AP::ahrs();

    // get position as a 2D offset from ahrs home
    Vector2f position_ne_cm;
    if (!_ahrs.get_relative_position_NE_home(position_ne_cm)) {
        // we have no idea where we are....
        return;
    }
    position_ne_cm *= 100.0f; // m -> cm

    // get the fence radius in cm
    const float fence_radius_cm = _fence.get_radius_m() * 100.0f;
    // get the margin to the fence in cm
    const float margin_cm = _fence.get_margin_ne_m() * 100.0f;

    if (margin_cm > fence_radius_cm) {
        return;
    }

    // get vehicle distance from home
    const float dist_from_home_cm = position_ne_cm.length();
    if (dist_from_home_cm > fence_radius_cm) {
        // outside of circular fence, no velocity adjustments
        return;
    }
    const float distance_to_boundary_cm = fence_radius_cm - dist_from_home_cm;

    // for backing away
    Vector2f quad_1_back_vel_ne_cms, quad_2_back_vel_ne_cms, quad_3_back_vel_ne_cms, quad_4_back_vel_ne_cms;
    
    // back away if vehicle has breached margin
    if (is_negative(distance_to_boundary_cm - margin_cm)) {     
        calc_backup_velocity_2D(kP, accel_cmss, quad_1_back_vel_ne_cms, quad_2_back_vel_ne_cms, quad_3_back_vel_ne_cms, quad_4_back_vel_ne_cms, margin_cm - distance_to_boundary_cm, position_ne_cm, dt);
    }
    // desired backup velocity is sum of maximum velocity component in each quadrant 
    backup_vel_ne_cms = quad_1_back_vel_ne_cms + quad_2_back_vel_ne_cms + quad_3_back_vel_ne_cms + quad_4_back_vel_ne_cms;

    // vehicle is inside the circular fence
    switch (_behavior) {
    case BEHAVIOR_SLIDE: {
        // implement sliding behaviour
        const Vector2f stopping_point_ne_cm = position_ne_cm + desired_vel_ne_cms * (get_stopping_distance(kP, accel_cmss, desired_speed_cms) / desired_speed_cms);
        const float stopping_point_dist_from_home_ne_cm = stopping_point_ne_cm.length();
        if (stopping_point_dist_from_home_ne_cm <= fence_radius_cm - margin_cm) {
            // stopping before before fence so no need to adjust
            return;
        }
        // unsafe desired velocity - will not be able to stop before reaching margin from fence
        // Project stopping point radially onto fence boundary
        // Adjusted velocity will point towards this projected point at a safe speed
        const Vector2f target_offset_ne_cm = stopping_point_ne_cm * ((fence_radius_cm - margin_cm) / stopping_point_dist_from_home_ne_cm);
        const Vector2f target_direction_ne_cm = target_offset_ne_cm - position_ne_cm;
        const float distance_to_target_cm = target_direction_ne_cm.length();
        if (is_positive(distance_to_target_cm)) {
            const float max_speed_cms = get_max_speed(kP, accel_cmss, distance_to_target_cm, dt);
            desired_vel_ne_cms = target_direction_ne_cm * (MIN(desired_speed_cms,max_speed_cms) / distance_to_target_cm);
        }
      break;
    } 
  
    case (BEHAVIOR_STOP): {
        // implement stopping behaviour
        // calculate stopping point plus a margin so we look forward far enough to intersect with circular fence
        const Vector2f stopping_point_plus_margin_ne_cm = position_ne_cm + desired_vel_ne_cms*((2.0f + margin_cm + get_stopping_distance(kP, accel_cmss, desired_speed_cms))/desired_speed_cms);
        const float stopping_point_plus_margin_dist_from_home_cm = stopping_point_plus_margin_ne_cm.length();
        if (dist_from_home_cm >= fence_radius_cm - margin_cm) {
            // vehicle has already breached margin around fence
            // if stopping point is even further from home (i.e. in wrong direction) then adjust speed to zero
            // otherwise user is backing away from fence so do not apply limits
            if (stopping_point_plus_margin_dist_from_home_cm >= dist_from_home_cm) {
                desired_vel_ne_cms.zero();
            }
        } else {
            // shorten vector without adjusting its direction
            Vector2f intersection_ne_cm;
            if (Vector2f::circle_segment_intersection(position_ne_cm, stopping_point_plus_margin_ne_cm, Vector2f(0.0f,0.0f), fence_radius_cm - margin_cm, intersection_ne_cm)) {
                const float distance_to_target_cm = (intersection_ne_cm - position_ne_cm).length();
                const float max_speed_cms = get_max_speed(kP, accel_cmss, distance_to_target_cm, dt);
                if (max_speed_cms < desired_speed_cms) {
                    desired_vel_ne_cms *= MAX(max_speed_cms, 0.0f) / desired_speed_cms;
                }
            }
        }
        break;
    }
    }
}

/*
 * Adjusts the desired velocity for the exclusion polygons
 */
void AC_Avoid::adjust_velocity_inclusion_and_exclusion_polygons(float kP, float accel_cmss, Vector2f &desired_vel_ne_cms, Vector2f &backup_vel_ne_cms, float dt)
{
    const AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return;
    }

    // exit if polygon fences are not enabled
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_POLYGON) == 0) {
        return;
    }

    // for backing away
    Vector2f quad_1_back_vel_ne_cms, quad_2_back_vel_ne_cms, quad_3_back_vel_ne_cms, quad_4_back_vel_ne_cms;

    // iterate through inclusion polygons
    const uint8_t num_inclusion_polygons = fence->polyfence().get_inclusion_polygon_count();
    for (uint8_t i = 0; i < num_inclusion_polygons; i++) {
        uint16_t num_points;
        const Vector2f* boundary = fence->polyfence().get_inclusion_polygon(i, num_points);
        Vector2f backup_vel_inc_ne_cms;
        // adjust velocity
        adjust_velocity_polygon(kP, accel_cmss, desired_vel_ne_cms, backup_vel_inc_ne_cms, boundary, num_points, fence->get_margin_ne_m(), dt, true);
        find_max_quadrant_velocity(backup_vel_inc_ne_cms, quad_1_back_vel_ne_cms, quad_2_back_vel_ne_cms, quad_3_back_vel_ne_cms, quad_4_back_vel_ne_cms);
    }

    // iterate through exclusion polygons
    const uint8_t num_exclusion_polygons = fence->polyfence().get_exclusion_polygon_count();
    for (uint8_t i = 0; i < num_exclusion_polygons; i++) {
        uint16_t num_points;
        const Vector2f* boundary = fence->polyfence().get_exclusion_polygon(i, num_points);
        Vector2f backup_vel_exc_ne_cms;
        // adjust velocity
        adjust_velocity_polygon(kP, accel_cmss, desired_vel_ne_cms, backup_vel_exc_ne_cms, boundary, num_points, fence->get_margin_ne_m(), dt, false);
        find_max_quadrant_velocity(backup_vel_exc_ne_cms, quad_1_back_vel_ne_cms, quad_2_back_vel_ne_cms, quad_3_back_vel_ne_cms, quad_4_back_vel_ne_cms);
    }
    // desired backup velocity is sum of maximum velocity component in each quadrant 
    backup_vel_ne_cms = quad_1_back_vel_ne_cms + quad_2_back_vel_ne_cms + quad_3_back_vel_ne_cms + quad_4_back_vel_ne_cms;
}

/*
 * Adjusts the desired velocity for the inclusion circles
 */
void AC_Avoid::adjust_velocity_inclusion_circles(float kP, float accel_cmss, Vector2f &desired_vel_ne_cms, Vector2f &backup_vel_ne_cms, float dt)
{
    const AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return;
    }

    // return immediately if no inclusion circles
    const uint8_t num_circles = fence->polyfence().get_inclusion_circle_count();
    if (num_circles == 0) {
        return;
    }

    // exit if polygon fences are not enabled
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_POLYGON) == 0) {
        return;
    }

    // get vehicle position
    Vector2f position_ne_cm;
    if (!AP::ahrs().get_relative_position_NE_origin_float(position_ne_cm)) {
        // do not limit velocity if we don't have a position estimate
        return;
    }
    position_ne_cm = position_ne_cm * 100.0f;  // m to cm

    // get the margin to the fence in cm
    const float margin_cm = fence->get_margin_ne_m() * 100.0f;

    // get desired speed
    const float desired_speed_cms = desired_vel_ne_cms.length();

    // get stopping distance as an offset from the vehicle
    Vector2f stopping_offset_ne_cm;
    if (!is_zero(desired_speed_cms)) {
        switch (_behavior) {
            case BEHAVIOR_SLIDE:
                stopping_offset_ne_cm = desired_vel_ne_cms * (get_stopping_distance(kP, accel_cmss, desired_speed_cms) / desired_speed_cms);
                break;
            case BEHAVIOR_STOP:
                // calculate stopping point plus a margin so we look forward far enough to intersect with circular fence
                stopping_offset_ne_cm = desired_vel_ne_cms * ((2.0f + margin_cm + get_stopping_distance(kP, accel_cmss, desired_speed_cms)) / desired_speed_cms);
                break;
        }
    }

    // for backing away
    Vector2f quad_1_back_vel_ne_cms, quad_2_back_vel_ne_cms, quad_3_back_vel_ne_cms, quad_4_back_vel_ne_cms;

    // iterate through inclusion circles
    for (uint8_t i = 0; i < num_circles; i++) {
        Vector2f center_pos_ne_cm;
        float radius_m;
        if (fence->polyfence().get_inclusion_circle(i, center_pos_ne_cm, radius_m)) {
            // get position relative to circle's center
            const Vector2f position_rel_ne_cm = (position_ne_cm - center_pos_ne_cm);

            // if we are outside this circle do not limit velocity for this circle
            const float dist_sq_cm = position_rel_ne_cm.length_squared();
            const float radius_cm = (radius_m * 100.0f);
            if (dist_sq_cm > sq(radius_cm)) {
                continue;
            }

            const float radius_with_margin_cm = radius_cm - margin_cm;
            if (is_negative(radius_with_margin_cm)) {
                return;
            }
            
            const float margin_breach_cm = radius_with_margin_cm - safe_sqrt(dist_sq_cm);
            // back away if vehicle has breached margin
            if (is_negative(margin_breach_cm)) {
                calc_backup_velocity_2D(kP, accel_cmss, quad_1_back_vel_ne_cms, quad_2_back_vel_ne_cms, quad_3_back_vel_ne_cms, quad_4_back_vel_ne_cms, margin_breach_cm, position_rel_ne_cm, dt);
            }
            if (is_zero(desired_speed_cms)) {
                // no avoidance necessary when desired speed is zero
                continue;
            }

            switch (_behavior) {
                case BEHAVIOR_SLIDE: {
                    // implement sliding behaviour
                    const Vector2f stopping_point_ne_cm = position_rel_ne_cm + stopping_offset_ne_cm;
                    const float stopping_point_dist_cm = stopping_point_ne_cm.length();
                    if (is_zero(stopping_point_dist_cm) || (stopping_point_dist_cm <= (radius_cm - margin_cm))) {
                        // stopping before before fence so no need to adjust for this circle
                        continue;
                    }
                    // unsafe desired velocity - will not be able to stop before reaching margin from fence
                    // project stopping point radially onto fence boundary
                    // adjusted velocity will point towards this projected point at a safe speed
                    const Vector2f target_offset_ne_cm = stopping_point_ne_cm * ((radius_cm - margin_cm) / stopping_point_dist_cm);
                    const Vector2f target_direction_ne_cm = target_offset_ne_cm - position_rel_ne_cm;
                    const float distance_to_target_cm = target_direction_ne_cm.length();
                    if (is_positive(distance_to_target_cm)) {
                        const float max_speed_cms = get_max_speed(kP, accel_cmss, distance_to_target_cm, dt);
                        desired_vel_ne_cms = target_direction_ne_cm * (MIN(desired_speed_cms, max_speed_cms) / distance_to_target_cm);
                    }
                }
                break;
                case BEHAVIOR_STOP: {
                    // implement stopping behaviour
                    const Vector2f stopping_point_plus_margin_ne_cm = position_rel_ne_cm + stopping_offset_ne_cm;
                    const float dist_cm = safe_sqrt(dist_sq_cm);
                    if (dist_cm >= radius_cm - margin_cm) {
                        // vehicle has already breached margin around fence
                        // if stopping point is even further from center (i.e. in wrong direction) then adjust speed to zero
                        // otherwise user is backing away from fence so do not apply limits
                        if (stopping_point_plus_margin_ne_cm.length() >= dist_cm) {
                            desired_vel_ne_cms.zero();
                            // desired backup velocity is sum of maximum velocity component in each quadrant 
                            backup_vel_ne_cms = quad_1_back_vel_ne_cms + quad_2_back_vel_ne_cms + quad_3_back_vel_ne_cms + quad_4_back_vel_ne_cms;
                            return;
                        }
                    } else {
                        // shorten vector without adjusting its direction
                        Vector2f intersection_ne_cm;
                        if (Vector2f::circle_segment_intersection(position_rel_ne_cm, stopping_point_plus_margin_ne_cm, Vector2f(0.0f,0.0f), radius_cm - margin_cm, intersection_ne_cm)) {
                            const float distance_to_target_cm = (intersection_ne_cm - position_rel_ne_cm).length();
                            const float max_speed_cms = get_max_speed(kP, accel_cmss, distance_to_target_cm, dt);
                            if (max_speed_cms < desired_speed_cms) {
                                desired_vel_ne_cms *= MAX(max_speed_cms, 0.0f) / desired_speed_cms;
                            }
                        }
                    }
                }
                break;
            }
        }
    }
    // desired backup velocity is sum of maximum velocity component in each quadrant 
    backup_vel_ne_cms = quad_1_back_vel_ne_cms + quad_2_back_vel_ne_cms + quad_3_back_vel_ne_cms + quad_4_back_vel_ne_cms;
}

/*
 * Adjusts the desired velocity for the exclusion circles
 */
void AC_Avoid::adjust_velocity_exclusion_circles(float kP, float accel_cmss, Vector2f &desired_vel_ne_cms, Vector2f &backup_vel_ne_cms, float dt)
{
    const AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return;
    }

    // return immediately if no inclusion circles
    const uint8_t num_circles = fence->polyfence().get_exclusion_circle_count();
    if (num_circles == 0) {
        return;
    }

    // exit if polygon fences are not enabled
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_POLYGON) == 0) {
        return;
    }

    // get vehicle position
    Vector2f position_ne_cm;
    if (!AP::ahrs().get_relative_position_NE_origin_float(position_ne_cm)) {
        // do not limit velocity if we don't have a position estimate
        return;
    }
    position_ne_cm = position_ne_cm * 100.0f;  // m to cm

    // get the margin to the fence in cm
    const float margin_cm = fence->get_margin_ne_m() * 100.0f;

    // for backing away
    Vector2f quad_1_back_vel_ne_cms, quad_2_back_vel_ne_cms, quad_3_back_vel_ne_cms, quad_4_back_vel_ne_cms;

    // get desired speed
    const float desired_speed_cms = desired_vel_ne_cms.length();
    
    // calculate stopping distance as an offset from the vehicle (only used for BEHAVIOR_STOP)
    // add a margin so we look forward far enough to intersect with circular fence
    Vector2f stopping_offset_ne_cm;
    if (!is_zero(desired_speed_cms)) {
        if ((AC_Avoid::BehaviourType)_behavior.get() == BEHAVIOR_STOP) {
            stopping_offset_ne_cm = desired_vel_ne_cms * ((2.0f + margin_cm + get_stopping_distance(kP, accel_cmss, desired_speed_cms)) / desired_speed_cms);
        }
    }
    // iterate through exclusion circles
    for (uint8_t i = 0; i < num_circles; i++) {
        Vector2f center_pos_ne_cm;
        float radius_m;
        if (fence->polyfence().get_exclusion_circle(i, center_pos_ne_cm, radius_m)) {
            // get position relative to circle's center
            const Vector2f position_rel_ne_cm = (position_ne_cm - center_pos_ne_cm);

            // if we are inside this circle do not limit velocity for this circle
            const float dist_sq_cm = position_rel_ne_cm.length_squared();
            const float radius_cm = (radius_m * 100.0f);
            if (radius_cm < margin_cm) {
                return;
            }
            if (dist_sq_cm < sq(radius_cm)) {
                continue;
            }
        
            const Vector2f vector_to_center_ne_cm = center_pos_ne_cm - position_ne_cm;
            const float dist_to_boundary_cm = vector_to_center_ne_cm.length() - radius_cm;
            // back away if vehicle has breached margin
            if (is_negative(dist_to_boundary_cm - margin_cm)) {
                calc_backup_velocity_2D(kP, accel_cmss, quad_1_back_vel_ne_cms, quad_2_back_vel_ne_cms, quad_3_back_vel_ne_cms, quad_4_back_vel_ne_cms, margin_cm - dist_to_boundary_cm, vector_to_center_ne_cm, dt);
            }
            if (is_zero(desired_speed_cms)) {
                // no avoidance necessary when desired speed is zero
                continue;
            }

            switch (_behavior) {
                case BEHAVIOR_SLIDE: {
                    // vector from current position to circle's center
                    Vector2f limit_direction_ne_cm = vector_to_center_ne_cm;
                    if (limit_direction_ne_cm.is_zero()) {
                        // vehicle is exactly on circle center so do not limit velocity
                        continue;
                    }
                    // calculate distance to edge of circle
                    const float limit_distance_cm = limit_direction_ne_cm.length() - radius_cm;
                    if (!is_positive(limit_distance_cm)) {
                        // vehicle is within circle so do not limit velocity
                        continue;
                    }
                    // vehicle is outside the circle, adjust velocity to stay outside
                    limit_direction_ne_cm.normalize();
                    limit_velocity_NE(kP, accel_cmss, desired_vel_ne_cms, limit_direction_ne_cm, MAX(limit_distance_cm - margin_cm, 0.0f), dt);
                }
                break;
                case BEHAVIOR_STOP: {
                    // implement stopping behaviour
                    const Vector2f stopping_point_plus_margin_ne_cm = position_rel_ne_cm + stopping_offset_ne_cm;
                    const float dist_cm = safe_sqrt(dist_sq_cm);
                    if (dist_cm < radius_cm + margin_cm) {
                        // vehicle has already breached margin around fence
                        // if stopping point is closer to center (i.e. in wrong direction) then adjust speed to zero
                        // otherwise user is backing away from fence so do not apply limits
                        if (stopping_point_plus_margin_ne_cm.length() <= dist_cm) {
                            desired_vel_ne_cms.zero();
                            backup_vel_ne_cms = quad_1_back_vel_ne_cms + quad_2_back_vel_ne_cms + quad_3_back_vel_ne_cms + quad_4_back_vel_ne_cms;
                            return;
                        }
                    } else {
                        // shorten vector without adjusting its direction
                        Vector2f intersection_ne_cm;
                        if (Vector2f::circle_segment_intersection(position_rel_ne_cm, stopping_point_plus_margin_ne_cm, Vector2f(0.0f,0.0f), radius_cm + margin_cm, intersection_ne_cm)) {
                            const float distance_to_target_cm = (intersection_ne_cm - position_rel_ne_cm).length();
                            const float max_speed_cms = get_max_speed(kP, accel_cmss, distance_to_target_cm, dt);
                            if (max_speed_cms < desired_speed_cms) {
                                desired_vel_ne_cms *= MAX(max_speed_cms, 0.0f) / desired_speed_cms;
                            }
                        }
                    }
                }
                break;
            }
        }
    }
    // desired backup velocity is sum of maximum velocity component in each quadrant 
    backup_vel_ne_cms = quad_1_back_vel_ne_cms + quad_2_back_vel_ne_cms + quad_3_back_vel_ne_cms + quad_4_back_vel_ne_cms;
}
#endif // AP_FENCE_ENABLED

#if AP_BEACON_ENABLED
/*
 * Adjusts the desired velocity for the beacon fence.
 */
void AC_Avoid::adjust_velocity_beacon_fence(float kP, float accel_cmss, Vector2f &desired_vel_ne_cms, Vector2f &backup_vel_ne_cms, float dt)
{
    AP_Beacon *_beacon = AP::beacon();

    // exit if the beacon is not present
    if (_beacon == nullptr) {
        return;
    }

    // get boundary from beacons
    uint16_t num_points = 0;
    const Vector2f* boundary = _beacon->get_boundary_points(num_points);
    if ((boundary == nullptr) || (num_points == 0)) {
        return;
    }

    // adjust velocity using beacon
    float margin_m = 0;
#if AP_FENCE_ENABLED
    if (AP::fence()) {
        margin_m = AP::fence()->get_margin_ne_m();
    }
#endif
    adjust_velocity_polygon(kP, accel_cmss, desired_vel_ne_cms, backup_vel_ne_cms, boundary, num_points, margin_m, dt, true);
}
#endif  // AP_BEACON_ENABLED

/*
 * Adjusts the desired velocity based on output from the proximity sensor
 */
void AC_Avoid::adjust_velocity_proximity(float kP, float accel_cmss, Vector3f &desired_vel_neu_cms, Vector3f &backup_vel_neu_cms, float kP_z, float accel_u_cmss, float dt)
{
#if HAL_PROXIMITY_ENABLED
    // exit immediately if proximity sensor is not present
    AP_Proximity *proximity = AP::proximity();
    if (!proximity) {
        return;
    }

    AP_Proximity &_proximity = *proximity;
    // get total number of obstacles
    const uint8_t obstacle_num = _proximity.get_obstacle_count();
    if (obstacle_num == 0) {
        // no obstacles
        return;
    }
 
    const AP_AHRS &_ahrs = AP::ahrs();
    
    // for backing away
    Vector2f quad_1_back_vel_ne_cms, quad_2_back_vel_ne_cms, quad_3_back_vel_ne_cms, quad_4_back_vel_ne_cms;
    float max_back_vel_u_cms = 0.0f;
    float min_back_vel_u_cms = 0.0f; 

    // rotate velocity vector from earth frame to body-frame since obstacles are in body-frame
    const Vector2f desired_vel_body_ne_cms = _ahrs.earth_to_body2D(Vector2f{desired_vel_neu_cms.x, desired_vel_neu_cms.y});
    
    // safe_vel_ne_cms will be adjusted to stay away from Proximity Obstacles
    Vector3f safe_vel_neu_cms = Vector3f{desired_vel_body_ne_cms.x, desired_vel_body_ne_cms.y, desired_vel_neu_cms.z};
    const Vector3f safe_vel_orig_neu_cms = safe_vel_neu_cms;

    // calc margin in cm
    const float margin_cm = MAX(_margin_m * 100.0f, 0.0f);
    Vector3f stopping_point_plus_margin_neu_cm;
    if (!desired_vel_neu_cms.is_zero()) {
        // only used for "stop mode". Pre-calculating the stopping point here makes sure we do not need to repeat the calculations under iterations.
        const float speed_cms = safe_vel_neu_cms.length();
        stopping_point_plus_margin_neu_cm = safe_vel_neu_cms * ((2.0f + margin_cm + get_stopping_distance(kP, accel_cmss, speed_cms)) / speed_cms);
    }

    for (uint8_t i = 0; i<obstacle_num; i++) {
        // get obstacle from proximity library
        Vector3f vector_to_obstacle_neu;
        if (!_proximity.get_obstacle(i, vector_to_obstacle_neu)) {
            // this one is not valid
            continue;
        }

        const float dist_to_boundary_cm = vector_to_obstacle_neu.length();
        if (is_zero(dist_to_boundary_cm)) {
            continue;
        }

        // back away if vehicle has breached margin
        if (is_negative(dist_to_boundary_cm - margin_cm)) {
            const float breach_dist_cm = margin_cm - dist_to_boundary_cm;
            // add a deadzone so that the vehicle doesn't backup and go forward again and again
            const float deadzone_cm = MAX(0.0f, _backup_deadzone_m) * 100.0f;
            if (breach_dist_cm > deadzone_cm) {
                // this vector will help us decide how much we have to back away horizontally and vertically
                const Vector3f margin_vector_neu_cm = vector_to_obstacle_neu.normalized() * breach_dist_cm;
                const float xy_back_dist = margin_vector_neu_cm.xy().length();
                const float z_back_dist = margin_vector_neu_cm.z;
                calc_backup_velocity_3D(kP, accel_cmss, quad_1_back_vel_ne_cms, quad_2_back_vel_ne_cms, quad_3_back_vel_ne_cms, quad_4_back_vel_ne_cms, xy_back_dist, vector_to_obstacle_neu, kP_z, accel_u_cmss, z_back_dist, min_back_vel_u_cms, max_back_vel_u_cms, dt);
            }
        }

        if (desired_vel_neu_cms.is_zero()) {
            // cannot limit velocity if there is nothing to limit
            // backing up (if needed) has already been done
            continue;
        }

        switch (_behavior) {
        case BEHAVIOR_SLIDE: {
            Vector3f limit_direction_neu{vector_to_obstacle_neu};
            // distance to closest point
            const float limit_distance_cm = limit_direction_neu.length();
            if (is_zero(limit_distance_cm)) {
                // We are exactly on the edge, this should ideally never be possible
                // i.e. do not adjust velocity.
                continue;
            }
            // Adjust velocity to not violate margin.
            limit_velocity_NEU(kP, accel_cmss, safe_vel_neu_cms, limit_direction_neu, margin_cm, kP_z, accel_u_cmss, dt);
        
            break;
        }

        case BEHAVIOR_STOP: {
            // vector from current position to obstacle
            Vector3f limit_direction_neu;
            // find closest point with line segment
            // also see if the vehicle will "roughly" intersect the boundary with the projected stopping point
            const bool intersect = _proximity.closest_point_from_segment_to_obstacle(i, Vector3f{}, stopping_point_plus_margin_neu_cm, limit_direction_neu);
            if (intersect) {
                // the vehicle is intersecting the plane formed by the boundary
                // distance to the closest point from the stopping point
                float limit_distance_cm = limit_direction_neu.length();
                if (is_zero(limit_distance_cm)) {
                    // We are exactly on the edge, this should ideally never be possible
                    // i.e. do not adjust velocity.
                    return;
                }
                if (limit_distance_cm <= margin_cm) {
                    // we are within the margin so stop vehicle
                    safe_vel_neu_cms.zero();
                } else {
                    // vehicle inside the given edge, adjust velocity to not violate this edge
                    limit_velocity_NEU(kP, accel_cmss, safe_vel_neu_cms, limit_direction_neu, margin_cm, kP_z, accel_u_cmss, dt);
                }

                break;
            }
        }
        }
    }

    // desired backup velocity is sum of maximum velocity component in each quadrant 
    const Vector2f desired_back_vel_cms_xy = quad_1_back_vel_ne_cms + quad_2_back_vel_ne_cms + quad_3_back_vel_ne_cms + quad_4_back_vel_ne_cms;
    const float desired_back_vel_cms_z = max_back_vel_u_cms + min_back_vel_u_cms;

    if (safe_vel_neu_cms == safe_vel_orig_neu_cms && desired_back_vel_cms_xy.is_zero() && is_zero(desired_back_vel_cms_z)) {
        // proximity avoidance did nothing, no point in doing the calculations below. Return early
        backup_vel_neu_cms.zero();
        return;
    }

    // set modified desired velocity vector and back away velocity vector
    // vectors were in body-frame, rotate resulting vector back to earth-frame
    const Vector2f safe_vel_ne_cms = _ahrs.body_to_earth2D(Vector2f{safe_vel_neu_cms.x, safe_vel_neu_cms.y});
    desired_vel_neu_cms = Vector3f{safe_vel_ne_cms.x, safe_vel_ne_cms.y, safe_vel_neu_cms.z};
    const Vector2f backup_vel_ne_cms = _ahrs.body_to_earth2D(desired_back_vel_cms_xy);
    backup_vel_neu_cms = Vector3f{backup_vel_ne_cms.x, backup_vel_ne_cms.y, desired_back_vel_cms_z};
#endif // HAL_PROXIMITY_ENABLED
}

/*
 * Adjusts the desired velocity for the polygon fence.
 */
void AC_Avoid::adjust_velocity_polygon(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel_ne_cms, const Vector2f* boundary, uint16_t num_points, float margin, float dt, bool stay_inside)
{
    // exit if there are no points
    if (boundary == nullptr || num_points == 0) {
        return;
    }

    const AP_AHRS &_ahrs = AP::ahrs();

    // do not adjust velocity if vehicle is outside the polygon fence
    Vector2f position_ne_cm;
    if (!_ahrs.get_relative_position_NE_origin_float(position_ne_cm)) {
        // boundary is in earth frame but we have no idea
        // where we are
        return;
    }
    position_ne_cm = position_ne_cm * 100.0f;  // m to cm


    // return if we have already breached polygon
    const bool inside_polygon = !Polygon_outside(position_ne_cm, boundary, num_points);
    if (inside_polygon != stay_inside) {
        return;
    }

    // Safe_vel will be adjusted to remain within fence.
    // We need a separate vector in case adjustment fails,
    // e.g. if we are exactly on the boundary.
    Vector2f safe_vel_ne_cms(desired_vel_cms);
    Vector2f desired_back_vel_cms;

    // calc margin in cm
    const float margin_cm = MAX(margin * 100.0f, 0.0f);

    // for stopping
    const float speed = safe_vel_ne_cms.length();
    Vector2f stopping_point_plus_margin_ne_cm; 
    if (!desired_vel_cms.is_zero()) {
        stopping_point_plus_margin_ne_cm = position_ne_cm + safe_vel_ne_cms*((2.0f + margin_cm + get_stopping_distance(kP, accel_cmss, speed))/speed);
    }

    // for backing away
    Vector2f quad_1_back_vel_ne_cms, quad_2_back_vel_ne_cms, quad_3_back_vel_ne_cms, quad_4_back_vel_ne_cms;
   
    for (uint16_t i=0; i<num_points; i++) {
        uint16_t j = i+1;
        if (j >= num_points) {
            j = 0;
        }
        // end points of current edge
        Vector2f start = boundary[j];
        Vector2f end = boundary[i];
        Vector2f vector_to_boundary_ne_cm = Vector2f::closest_point(position_ne_cm, start, end) - position_ne_cm;
        // back away if vehicle has breached margin
        if (is_negative(vector_to_boundary_ne_cm.length() - margin_cm)) {
            calc_backup_velocity_2D(kP, accel_cmss, quad_1_back_vel_ne_cms, quad_2_back_vel_ne_cms, quad_3_back_vel_ne_cms, quad_4_back_vel_ne_cms, margin_cm - vector_to_boundary_ne_cm.length(), vector_to_boundary_ne_cm, dt);
        }
        
        // exit immediately if no desired velocity
        if (desired_vel_cms.is_zero()) {
            continue;
        }

        switch (_behavior) {
        case (BEHAVIOR_SLIDE): {
            // vector from current position to closest point on current edge
            Vector2f limit_direction_ne_cm = vector_to_boundary_ne_cm;
            // distance to closest point
            const float limit_distance_cm = limit_direction_ne_cm.length();
            if (is_zero(limit_distance_cm)) {
                // We are exactly on the edge - treat this as a fence breach.
                // i.e. do not adjust velocity.
                return;
            }
            // We are strictly inside the given edge.
            // Adjust velocity to not violate this edge.
            limit_direction_ne_cm /= limit_distance_cm;
            limit_velocity_NE(kP, accel_cmss, safe_vel_ne_cms, limit_direction_ne_cm, MAX(limit_distance_cm - margin_cm, 0.0f), dt);
            break;
        } 
        
        case (BEHAVIOR_STOP): {
            // find intersection_ne_cm with line segment
            Vector2f intersection_ne_cm;
            if (Vector2f::segment_intersection(position_ne_cm, stopping_point_plus_margin_ne_cm, start, end, intersection_ne_cm)) {
                // vector from current position to point on current edge
                Vector2f limit_direction_ne_cm = intersection_ne_cm - position_ne_cm;
                const float limit_distance_cm = limit_direction_ne_cm.length();
                if (is_zero(limit_distance_cm)) {
                    // We are exactly on the edge - treat this as a fence breach.
                    // i.e. do not adjust velocity.
                    return;
                }
                if (limit_distance_cm <= margin_cm) {
                    // we are within the margin so stop vehicle
                    safe_vel_ne_cms.zero();
                } else {
                    // vehicle inside the given edge, adjust velocity to not violate this edge
                    limit_direction_ne_cm /= limit_distance_cm;
                    limit_velocity_NE(kP, accel_cmss, safe_vel_ne_cms, limit_direction_ne_cm, MAX(limit_distance_cm - margin_cm, 0.0f), dt);
                }
            }
        break;
        }
        }
    }
    // desired backup velocity is sum of maximum velocity component in each quadrant 
    desired_back_vel_cms = quad_1_back_vel_ne_cms + quad_2_back_vel_ne_cms + quad_3_back_vel_ne_cms + quad_4_back_vel_ne_cms;

    // set modified desired velocity vector or back away velocity vector
    desired_vel_cms = safe_vel_ne_cms;
    backup_vel_ne_cms = desired_back_vel_cms;
}

/*
 * Computes distance required to stop, given current speed.
 *
 * Implementation copied from AC_PosControl.
 */
float AC_Avoid::get_stopping_distance(float kP, float accel_cmss, float speed_cms) const
{
    // avoid divide by zero by using current position if the velocity is below 10cm/s, kP is very low or acceleration is zero
    if (accel_cmss <= 0.0f || is_zero(speed_cms)) {
        return 0.0f;
    }

    // handle linear deceleration
    if (kP <= 0.0f) {
        return 0.5f * sq(speed_cms) / accel_cmss;
    }

    // calculate distance within which we can stop
    // accel_cmss/kP is the point at which velocity switches from linear to sqrt
    if (speed_cms < accel_cmss/kP) {
        return speed_cms/kP;
    } else {
        // accel_cmss/(2.0f*kP*kP) is the distance at which we switch from linear to sqrt response
        return accel_cmss/(2.0f*kP*kP) + (speed_cms*speed_cms)/(2.0f*accel_cmss);
    }
}

// convert distance (in meters) to a lean percentage (in 0~1 range) for use in manual flight modes
float AC_Avoid::distance_m_to_lean_norm(float dist_m) const
{
    // ignore objects beyond DIST_MAX
    if (dist_m < 0.0f || dist_m >= _dist_max_m || _dist_max_m <= 0.0f) {
        return 0.0f;
    }
    // inverted but linear response
    return 1.0f - (dist_m / _dist_max_m);
}

// returns the maximum positive and negative roll and pitch percentages (in -1 ~ +1 range) based on the proximity sensor
void AC_Avoid::get_proximity_roll_pitch_norm(float &roll_positive_norm, float &roll_negative_norm, float &pitch_positive_norm, float &pitch_negative_norm) const
{
#if HAL_PROXIMITY_ENABLED
    AP_Proximity *proximity = AP::proximity();
    if (proximity == nullptr) {
        return;
    }
    AP_Proximity &_proximity = *proximity;
    const uint8_t obj_count = _proximity.get_object_count();
    // if no objects return
    if (obj_count == 0) {
        return;
    }

    // calculate maximum roll, pitch values from objects
    for (uint8_t i=0; i<obj_count; i++) {
        float ang_deg, dist_m;
        if (_proximity.get_object_angle_and_distance(i, ang_deg, dist_m)) {
            if (dist_m < _dist_max_m) {
                // convert distance to lean angle (in 0 to 1 range)
                const float lean_norm = distance_m_to_lean_norm(dist_m);
                // convert angle to roll and pitch lean percentages
                const float angle_rad = radians(ang_deg);
                const float roll_norm = -sinf(angle_rad) * lean_norm;
                const float pitch_norm = cosf(angle_rad) * lean_norm;
                // update roll, pitch maximums
                if (roll_norm > 0.0f) {
                    roll_positive_norm = MAX(roll_positive_norm, roll_norm);
                } else if (roll_norm < 0.0f) {
                    roll_negative_norm = MIN(roll_negative_norm, roll_norm);
                }
                if (pitch_norm > 0.0f) {
                    pitch_positive_norm = MAX(pitch_positive_norm, pitch_norm);
                } else if (pitch_norm < 0.0f) {
                    pitch_negative_norm = MIN(pitch_negative_norm, pitch_norm);
                }
            }
        }
    }
#endif // HAL_PROXIMITY_ENABLED
}

// singleton instance
AC_Avoid *AC_Avoid::_singleton;

namespace AP {

AC_Avoid *ac_avoid()
{
    return AC_Avoid::get_singleton();
}

}

#endif // !APM_BUILD_Arduplane

#endif  // AP_AVOIDANCE_ENABLED
