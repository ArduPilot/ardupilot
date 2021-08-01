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

#include "AC_Avoid.h"
#include <AP_AHRS/AP_AHRS.h>     // AHRS library
#include <AC_Fence/AC_Fence.h>         // Failsafe fence library
#include <AP_Proximity/AP_Proximity.h>
#include <AP_Beacon/AP_Beacon.h>
#include <AP_Logger/AP_Logger.h>
#include <stdio.h>

#if APM_BUILD_TYPE(APM_BUILD_Rover)
 # define AP_AVOID_BEHAVE_DEFAULT AC_Avoid::BehaviourType::BEHAVIOR_STOP
#else
 # define AP_AVOID_BEHAVE_DEFAULT AC_Avoid::BehaviourType::BEHAVIOR_SLIDE
#endif

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
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
    AP_GROUPINFO_FRAME("ANGLE_MAX", 2,  AC_Avoid, _angle_max, 1000, AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_HELI | AP_PARAM_FRAME_TRICOPTER),

    // @Param{Copter}: DIST_MAX
    // @DisplayName: Avoidance distance maximum in non-GPS flight modes
    // @Description: Distance from object at which obstacle avoidance will begin in non-GPS modes
    // @Units: m
    // @Range: 1 30
    // @User: Standard
    AP_GROUPINFO_FRAME("DIST_MAX", 3,  AC_Avoid, _dist_max, AC_AVOID_NONGPS_DIST_MAX_DEFAULT, AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_HELI | AP_PARAM_FRAME_TRICOPTER),

    // @Param: MARGIN
    // @DisplayName: Avoidance distance margin in GPS modes
    // @Description: Vehicle will attempt to stay at least this distance (in meters) from objects while in GPS modes
    // @Units: m
    // @Range: 1 10
    // @User: Standard
    AP_GROUPINFO("MARGIN", 4, AC_Avoid, _margin, 2.0f),

    // @Param{Copter}: BEHAVE
    // @DisplayName: Avoidance behaviour
    // @Description: Avoidance behaviour (slide or stop)
    // @Values: 0:Slide,1:Stop
    // @User: Standard
    AP_GROUPINFO_FRAME("BEHAVE", 5, AC_Avoid, _behavior, AP_AVOID_BEHAVE_DEFAULT, AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_HELI | AP_PARAM_FRAME_TRICOPTER),

    // @Param: BACKUP_SPD
    // @DisplayName: Avoidance maximum backup speed
    // @Description: Maximum speed that will be used to back away from obstacles in GPS modes (m/s). Set zero to disable
    // @Units: m/s
    // @Range: 0 2
    // @User: Standard
    AP_GROUPINFO("BACKUP_SPD", 6, AC_Avoid, _backup_speed_max, 0.75f),

    // @Param{Copter}: ALT_MIN
    // @DisplayName: Avoidance minimum altitude
    // @Description: Minimum altitude above which proximity based avoidance will start working. This requires a valid downward facing rangefinder reading to work. Set zero to disable
    // @Units: m
    // @Range: 0 6
    // @User: Standard
    AP_GROUPINFO_FRAME("ALT_MIN", 7, AC_Avoid, _alt_min, 0.0f, AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_HELI | AP_PARAM_FRAME_TRICOPTER),

    // @Param: ACCEL_MAX
    // @DisplayName: Avoidance maximum acceleration
    // @Description: Maximum acceleration with which obstacles will be avoided with. Set zero to disable acceleration limits
    // @Units: m/s/s
    // @Range: 0 9
    // @User: Standard
    AP_GROUPINFO("ACCEL_MAX", 8, AC_Avoid, _accel_max, 3.0f),

    // @Param: BACKUP_DZ
    // @DisplayName: Avoidance deadzone between stopping and backing away from obstacle
    // @Description: Distance beyond AVOID_MARGIN parameter, after which vehicle will backaway from obstacles. Increase this parameter if you see vehicle going back and forth in front of obstacle.
    // @Units: m
    // @Range: 0 2
    // @User: Standard
    AP_GROUPINFO("BACKUP_DZ", 9, AC_Avoid, _backup_deadzone, 0.10f),

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
void AC_Avoid::adjust_velocity_fence(float kP, float accel_cmss, Vector3f &desired_vel_cms, Vector3f &backup_vel, float kP_z, float accel_cmss_z, float dt)
{   
    // Only horizontal component needed for most fences, since fences are 2D
    Vector2f desired_velocity_xy_cms{desired_vel_cms.x, desired_vel_cms.y};
    
    // limit acceleration
    const float accel_cmss_limited = MIN(accel_cmss, AC_AVOID_ACCEL_CMSS_MAX);

    // maximum component of desired  backup velocity in each quadrant 
    Vector2f quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel;

    if ((_enabled & AC_AVOID_STOP_AT_FENCE) > 0) {
        // Store velocity needed to back away from fence
        Vector2f backup_vel_fence;

        adjust_velocity_circle_fence(kP, accel_cmss_limited, desired_velocity_xy_cms, backup_vel_fence, dt);
        find_max_quadrant_velocity(backup_vel_fence, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel);
        
        // backup_vel_fence is set to zero after each fence incase the velocity is unset from previous methods
        backup_vel_fence.zero();
        adjust_velocity_inclusion_and_exclusion_polygons(kP, accel_cmss_limited, desired_velocity_xy_cms, backup_vel_fence, dt);
        find_max_quadrant_velocity(backup_vel_fence, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel);
        
        backup_vel_fence.zero();
        adjust_velocity_inclusion_circles(kP, accel_cmss_limited, desired_velocity_xy_cms, backup_vel_fence, dt);
        find_max_quadrant_velocity(backup_vel_fence, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel);
        
        backup_vel_fence.zero();
        adjust_velocity_exclusion_circles(kP, accel_cmss_limited, desired_velocity_xy_cms, backup_vel_fence, dt);
        find_max_quadrant_velocity(backup_vel_fence, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel);
    }

    if ((_enabled & AC_AVOID_STOP_AT_BEACON_FENCE) > 0) {
        // Store velocity needed to back away from beacon fence
        Vector2f backup_vel_beacon;
        adjust_velocity_beacon_fence(kP, accel_cmss_limited, desired_velocity_xy_cms, backup_vel_beacon, dt);
        find_max_quadrant_velocity(backup_vel_beacon, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel);
    }

    // check for vertical fence
    float desired_velocity_z_cms = desired_vel_cms.z;
    float desired_backup_vel_z = 0.0f;
    adjust_velocity_z(kP_z, accel_cmss_z, desired_velocity_z_cms, desired_backup_vel_z, dt);

    // Desired backup velocity is sum of maximum velocity component in each quadrant 
    const Vector2f desired_backup_vel_xy = quad_1_back_vel + quad_2_back_vel + quad_3_back_vel + quad_4_back_vel;
    backup_vel = Vector3f{desired_backup_vel_xy.x, desired_backup_vel_xy.y, desired_backup_vel_z};
    desired_vel_cms = Vector3f{desired_velocity_xy_cms.x, desired_velocity_xy_cms.y, desired_velocity_z_cms};
}

/*
* Adjusts the desired velocity so that the vehicle can stop
* before the fence/object.
* kP, accel_cmss are for the horizontal axis
* kP_z, accel_cmss_z are for vertical axis
*/
void AC_Avoid::adjust_velocity(Vector3f &desired_vel_cms, bool &backing_up, float kP, float accel_cmss, float kP_z, float accel_cmss_z, float dt)
{
    // exit immediately if disabled
    if (_enabled == AC_AVOID_DISABLED) {
        return;
    }

    // make a copy of input velocity, because desired_vel_cms might be changed
    const Vector3f desired_vel_cms_original = desired_vel_cms;

    // limit acceleration
    const float accel_cmss_limited = MIN(accel_cmss, AC_AVOID_ACCEL_CMSS_MAX);

    // maximum component of horizontal desired  backup velocity in each quadrant 
    Vector2f quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel;
    float back_vel_up = 0.0f;
    float back_vel_down = 0.0f;
    
    // Avoidance in response to proximity sensor
    if (proximity_avoidance_enabled() && _proximity_alt_enabled) {
        // Store velocity needed to back away from physical obstacles
        Vector3f backup_vel_proximity;
        adjust_velocity_proximity(kP, accel_cmss_limited, desired_vel_cms, backup_vel_proximity, kP_z,accel_cmss_z, dt);
        find_max_quadrant_velocity_3D(backup_vel_proximity, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel, back_vel_up, back_vel_down);
    }
    
    // Avoidance in response to various fences 
    Vector3f backup_vel_fence;
    adjust_velocity_fence(kP, accel_cmss, desired_vel_cms, backup_vel_fence, kP_z, accel_cmss_z, dt);
    find_max_quadrant_velocity_3D(backup_vel_fence , quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel, back_vel_up, back_vel_down);
    
    // Desired backup velocity is sum of maximum velocity component in each quadrant
    const Vector2f desired_backup_vel_xy = quad_1_back_vel + quad_2_back_vel + quad_3_back_vel + quad_4_back_vel;
    const float desired_backup_vel_z = back_vel_down + back_vel_up;
    Vector3f desired_backup_vel{desired_backup_vel_xy.x, desired_backup_vel_xy.y, desired_backup_vel_z};

    const float max_back_spd_cms = _backup_speed_max * 100.0f;
    if (!desired_backup_vel.is_zero() && is_positive(max_back_spd_cms)) {
        backing_up = true;
        // Constrain backing away speed
        if (desired_backup_vel.length() > max_back_spd_cms) {
            desired_backup_vel = desired_backup_vel.normalized() * max_back_spd_cms;
        }
    
        // let user take control if they are backing away at a greater speed than what we have calculated
        // this has to be done for x,y,z seperately. For eg, user is doing fine in "x" direction but might need backing up in "y".
        if (!is_zero(desired_backup_vel.x)) {
            if (is_positive(desired_backup_vel.x)) {
                desired_vel_cms.x = MAX(desired_vel_cms.x, desired_backup_vel.x);
            } else {
                desired_vel_cms.x = MIN(desired_vel_cms.x, desired_backup_vel.x);
            }
        }
        if (!is_zero(desired_backup_vel.y)) {
            if (is_positive(desired_backup_vel.y)) {
                desired_vel_cms.y = MAX(desired_vel_cms.y, desired_backup_vel.y);
            } else {
                desired_vel_cms.y = MIN(desired_vel_cms.y, desired_backup_vel.y);
            }
        }
        if (!is_zero(desired_backup_vel.z)) {
            if (is_positive(desired_backup_vel.z)) {
                desired_vel_cms.z = MAX(desired_vel_cms.z, desired_backup_vel.z);
            } else {
                desired_vel_cms.z = MIN(desired_vel_cms.z, desired_backup_vel.z);
            }
        }
    }
    // limit acceleration
    limit_accel(desired_vel_cms_original, desired_vel_cms, dt);

    if (desired_vel_cms_original != desired_vel_cms) {
        _last_limit_time = AP_HAL::millis();
    }

    if (limits_active()) {
        // log at not more than 10hz (adjust_velocity method can be potentially called at 400hz!)
        uint32_t now = AP_HAL::millis();
        if ((now - _last_log_ms) > 100) {
            _last_log_ms = now;
            Write_SimpleAvoidance(true, desired_vel_cms_original, desired_vel_cms, backing_up);
        }
    } else {
        // avoidance isn't active anymore
        // log once so that it registers in logs
        if (_last_log_ms) {
            Write_SimpleAvoidance(false, desired_vel_cms_original, desired_vel_cms, backing_up);
            // this makes sure logging won't run again till it is active
            _last_log_ms = 0;
        }
    }
}

/*
* Limit acceleration so that change of velocity output by avoidance library is controlled
* This helps reduce jerks and sudden movements in the vehicle
*/
void AC_Avoid::limit_accel(const Vector3f &original_vel, Vector3f &modified_vel, float dt)
{
    if (original_vel == modified_vel || is_zero(_accel_max) || !is_positive(dt)) {
        // we can't limit accel if any of these conditions are true
        return;
    }

    if (AP_HAL::millis() - _last_limit_time > AC_AVOID_ACCEL_TIMEOUT_MS) {
        // reset this velocity because its been a long time since avoidance was active
        _prev_avoid_vel = original_vel;
    }

    // acceleration demanded by avoidance
    const Vector3f accel = (modified_vel - _prev_avoid_vel)/dt;

    // max accel in cm
    const float max_accel_cm = _accel_max * 100.0f;

    if (accel.length() > max_accel_cm) {
        // pull back on the acceleration
        const Vector3f accel_direction = accel.normalized();
        modified_vel = (accel_direction * max_accel_cm) * dt + _prev_avoid_vel;
    }

    _prev_avoid_vel = modified_vel;
    return;
}

// This method is used in most Rover modes and not in Copter
// adjust desired horizontal speed so that the vehicle stops before the fence or object
// accel (maximum acceleration/deceleration) is in m/s/s
// heading is in radians
// speed is in m/s
// kP should be zero for linear response, non-zero for non-linear response
void AC_Avoid::adjust_speed(float kP, float accel, float heading, float &speed, float dt)
{
    // convert heading and speed into velocity vector
    Vector3f vel{
        cosf(heading) * speed * 100.0f,
        sinf(heading) * speed * 100.0f,
        0.0f
    };

    bool backing_up  = false;
    adjust_velocity(vel, backing_up, kP, accel * 100.0f, 0, 0, dt);
    const Vector2f vel_xy{vel.x, vel.y};

    if (backing_up) {
        // back up
        if (fabsf(wrap_180(degrees(vel_xy.angle())) - degrees(heading)) > 90.0f) {
            // Big difference between the direction of velocity vector and actual heading therefore we need to reverse the direction
            speed = -vel_xy.length() * 0.01f;
        } else {
            speed = vel_xy.length() * 0.01f;
        }
        return;
    }

    // No need to back up so adjust speed towards zero if needed
    if (is_negative(speed)) {
        speed = -vel_xy.length() * 0.01f;
    } else {
        speed = vel_xy.length() * 0.01f;
    }
}

// adjust vertical climb rate so vehicle does not break the vertical fence
void AC_Avoid::adjust_velocity_z(float kP, float accel_cmss, float& climb_rate_cms, float& backup_speed, float dt)
{
#ifdef AP_AVOID_ENABLE_Z

    // exit immediately if disabled
    if (_enabled == AC_AVOID_DISABLED) {
        return;
    }
    
    // do not adjust climb_rate if level or descending
    if (climb_rate_cms <= 0.0f) {
        return;
    }

    // limit acceleration
    const float accel_cmss_limited = MIN(accel_cmss, AC_AVOID_ACCEL_CMSS_MAX);

    bool limit_alt = false;
    float alt_diff = 0.0f;   // distance from altitude limit to vehicle in metres (positive means vehicle is below limit)

    const AP_AHRS &_ahrs = AP::ahrs();

    // calculate distance below fence
    AC_Fence *fence = AP::fence();
    if ((_enabled & AC_AVOID_STOP_AT_FENCE) > 0 && fence && (fence->get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) > 0) {
        // calculate distance from vehicle to safe altitude
        float veh_alt;
        _ahrs.get_relative_position_D_home(veh_alt);
        // _fence.get_safe_alt_max() is UP, veh_alt is DOWN:
        alt_diff = fence->get_safe_alt_max() + veh_alt;
        limit_alt = true;
    }

    // calculate distance to (e.g.) optical flow altitude limit
    // AHRS values are always in metres
    float alt_limit;
    float curr_alt;
    if (_ahrs.get_hgt_ctrl_limit(alt_limit) &&
        _ahrs.get_relative_position_D_origin(curr_alt)) {
        // alt_limit is UP, curr_alt is DOWN:
        const float ctrl_alt_diff = alt_limit + curr_alt;
        if (!limit_alt || ctrl_alt_diff < alt_diff) {
            alt_diff = ctrl_alt_diff;
            limit_alt = true;
        }
    }

#if HAL_PROXIMITY_ENABLED
    // get distance from proximity sensor
    float proximity_alt_diff;
    AP_Proximity *proximity = AP::proximity();
    if (proximity && proximity_avoidance_enabled() && proximity->get_upward_distance(proximity_alt_diff)) {
        proximity_alt_diff -= _margin;
        if (!limit_alt || proximity_alt_diff < alt_diff) {
            alt_diff = proximity_alt_diff;
            limit_alt = true;
        }
    }
#endif

    // limit climb rate
    if (limit_alt) {
        // do not allow climbing if we've breached the safe altitude
        if (alt_diff <= 0.0f) {
            climb_rate_cms = MIN(climb_rate_cms, 0.0f);
            // also calculate backup speed that will get us back to safe altitude
            backup_speed = -1*(get_max_speed(kP, accel_cmss_limited, -alt_diff*100.0f, dt));
            return;
        }

        // limit climb rate
        const float max_speed = get_max_speed(kP, accel_cmss_limited, alt_diff*100.0f, dt);
        climb_rate_cms = MIN(max_speed, climb_rate_cms);
    }
# endif
}

// adjust roll-pitch to push vehicle away from objects
// roll and pitch value are in centi-degrees
void AC_Avoid::adjust_roll_pitch(float &roll, float &pitch, float veh_angle_max)
{
    // exit immediately if proximity based avoidance is disabled
    if (!proximity_avoidance_enabled()) {
        return;
    }

    // exit immediately if angle max is zero
    if (_angle_max <= 0.0f || veh_angle_max <= 0.0f) {
        return;
    }

    float roll_positive = 0.0f;    // maximum positive roll value
    float roll_negative = 0.0f;    // minimum negative roll value
    float pitch_positive = 0.0f;   // maximum positive pitch value
    float pitch_negative = 0.0f;   // minimum negative pitch value

    // get maximum positive and negative roll and pitch percentages from proximity sensor
    get_proximity_roll_pitch_pct(roll_positive, roll_negative, pitch_positive, pitch_negative);

    // add maximum positive and negative percentages together for roll and pitch, convert to centi-degrees
    Vector2f rp_out((roll_positive + roll_negative) * 4500.0f, (pitch_positive + pitch_negative) * 4500.0f);

    // apply avoidance angular limits
    // the object avoidance lean angle is never more than 75% of the total angle-limit to allow the pilot to override
    const float angle_limit = constrain_float(_angle_max, 0.0f, veh_angle_max * AC_AVOID_ANGLE_MAX_PERCENT);
    float vec_len = rp_out.length();
    if (vec_len > angle_limit) {
        rp_out *= (angle_limit / vec_len);
    }

    // add passed in roll, pitch angles
    rp_out.x += roll;
    rp_out.y += pitch;

    // apply total angular limits
    vec_len = rp_out.length();
    if (vec_len > veh_angle_max) {
        rp_out *= (veh_angle_max / vec_len);
    }

    // return adjusted roll, pitch
    roll = rp_out.x;
    pitch = rp_out.y;
}

/*
 * Note: This method is used to limit velocity horizontally only 
 * Limits the component of desired_vel_cms in the direction of the unit vector
 * limit_direction to be at most the maximum speed permitted by the limit_distance_cm.
 *
 * Uses velocity adjustment idea from Randy's second email on this thread:
 * https://groups.google.com/forum/#!searchin/drones-discuss/obstacle/drones-discuss/QwUXz__WuqY/qo3G8iTLSJAJ
 */
void AC_Avoid::limit_velocity_2D(float kP, float accel_cmss, Vector2f &desired_vel_cms, const Vector2f& limit_direction, float limit_distance_cm, float dt)
{
    const float max_speed = get_max_speed(kP, accel_cmss, limit_distance_cm, dt);
    // project onto limit direction
    const float speed = desired_vel_cms * limit_direction;
    if (speed > max_speed) {
        // subtract difference between desired speed and maximum acceptable speed
        desired_vel_cms += limit_direction*(max_speed - speed);
    }
}

/*
 * Note: This method is used to limit velocity horizontally and vertically given a 3D desired velocity vector 
 * Limits the component of desired_vel_cms in the direction of the obstacle_vector based on the passed value of "margin"
 */
void AC_Avoid::limit_velocity_3D(float kP, float accel_cmss, Vector3f &desired_vel_cms, const Vector3f& obstacle_vector, float margin_cm, float kP_z, float accel_cmss_z, float dt)
{  
    if (desired_vel_cms.is_zero()) {
        // nothing to limit
        return;
    }
    // create a margin_cm length vector in the direction of desired_vel_cms
    // this will create larger margin towards the direction vehicle is traveling in
    const Vector3f margin_vector = desired_vel_cms.normalized() * margin_cm;
    const Vector2f limit_direction_xy{obstacle_vector.x, obstacle_vector.y};
    
    if (!limit_direction_xy.is_zero()) {
        const float distance_from_fence_xy = MAX((limit_direction_xy.length() - Vector2f{margin_vector.x, margin_vector.y}.length()), 0.0f);
        Vector2f velocity_xy{desired_vel_cms.x, desired_vel_cms.y};
        limit_velocity_2D(kP, accel_cmss, velocity_xy, limit_direction_xy.normalized(), distance_from_fence_xy, dt);
        desired_vel_cms.x = velocity_xy.x;
        desired_vel_cms.y = velocity_xy.y;
    }
    
    if (is_zero(desired_vel_cms.z) || is_zero(obstacle_vector.z)) {
        // nothing to limit vertically if desired_vel_cms.z is zero
        // if obstacle_vector.z is zero then the obstacle is probably horizontally located, and we can move vertically
        return;
    }

    if (is_positive(desired_vel_cms.z) != is_positive(obstacle_vector.z)) {
        // why limit velocity vertically when we are going the opposite direction
        return;
    }
    
    // to check if Z velocity changes
    const float velocity_z_original = desired_vel_cms.z;
    const float z_speed = fabsf(desired_vel_cms.z);

    // obstacle_vector.z and margin_vector.z should be in same direction as checked above
    const float dist_z = MAX(fabsf(obstacle_vector.z) - fabsf(margin_vector.z), 0.0f); 
    if (is_zero(dist_z)) {
        // eliminate any vertical velocity 
        desired_vel_cms.z = 0.0f;
    } else {
        const float max_z_speed = get_max_speed(kP_z, accel_cmss_z, dist_z, dt);
        desired_vel_cms.z = MIN(max_z_speed, z_speed);
    }

    // make sure the direction of the Z velocity did not change
    // we are only limiting speed here, not changing directions 
    // check if original z velocity is positive or negative
    if (is_negative(velocity_z_original)) {
        desired_vel_cms.z = desired_vel_cms.z * -1.0f;
    }
}

/*
 * Compute the back away horizontal velocity required to avoid breaching margin
 * INPUT: This method requires the breach in margin distance (back_distance_cm), direction towards the breach (limit_direction)
 *        It then calculates the desired backup velocity and passes it on to "find_max_quadrant_velocity" method to distribute the velocity vectors into respective quadrants
 * OUTPUT: The method then outputs four velocities (quad1/2/3/4_back_vel_cms), which correspond to the maximum horizontal desired backup velocity in each quadrant
 */
void AC_Avoid::calc_backup_velocity_2D(float kP, float accel_cmss, Vector2f &quad1_back_vel_cms, Vector2f &quad2_back_vel_cms, Vector2f &quad3_back_vel_cms, Vector2f &quad4_back_vel_cms, float back_distance_cm, Vector2f limit_direction, float dt)
{      
    if (limit_direction.is_zero()) {
        // protect against divide by zero
        return; 
    }
    // speed required to move away the exact distance that we have breached the margin with 
    const float back_speed = get_max_speed(kP, 0.4f * accel_cmss, fabsf(back_distance_cm), dt);
    
    // direction to the obstacle
    limit_direction.normalize();

    // move in the opposite direction with the required speed
    Vector2f back_direction_vel = limit_direction * (-back_speed);
    // divide the vector into quadrants, find maximum velocity component in each quadrant 
    find_max_quadrant_velocity(back_direction_vel, quad1_back_vel_cms, quad2_back_vel_cms, quad3_back_vel_cms, quad4_back_vel_cms);
}

/*
* Compute the back away velocity required to avoid breaching margin, including vertical component
* min_z_vel is <= 0, and stores the greatest velocity in the downwards direction
* max_z_vel is >= 0, and stores the greatest velocity in the upwards direction
* eventually max_z_vel + min_z_vel will give the final desired Z backaway velocity
*/
void AC_Avoid::calc_backup_velocity_3D(float kP, float accel_cmss, Vector2f &quad1_back_vel_cms, Vector2f &quad2_back_vel_cms, Vector2f &quad3_back_vel_cms, Vector2f &quad4_back_vel_cms, float back_distance_cms, Vector3f limit_direction, float kp_z, float accel_cmss_z, float back_distance_z, float& min_z_vel, float& max_z_vel, float dt)
{   
    // backup horizontally 
    if (is_positive(back_distance_cms)) {
        Vector2f limit_direction_2d{limit_direction.x, limit_direction.y};
        calc_backup_velocity_2D(kP, accel_cmss, quad1_back_vel_cms, quad2_back_vel_cms, quad3_back_vel_cms, quad4_back_vel_cms, back_distance_cms, limit_direction_2d, dt);
    }

    // backup vertically 
    if (!is_zero(back_distance_z)) {
        float back_speed_z = get_max_speed(kp_z, 0.4f * accel_cmss_z, fabsf(back_distance_z), dt);
        // Down is positive
        if (is_positive(back_distance_z)) {
            back_speed_z *= -1.0f;
        } 

        // store the z backup speed into min or max z if possible
        if (back_speed_z < min_z_vel) {
            min_z_vel = back_speed_z;  
        }
        if (back_speed_z > max_z_vel) {
            max_z_vel = back_speed_z;
        }
    }
}

/*
 * Calculate maximum velocity vector that can be formed in each quadrant 
 * This method takes the desired backup velocity, and four other velocities corresponding to each quadrant
 * The desired velocity is then fit into one of the 4 quadrant velocities as per the sign of its components
 * This ensures that if we have multiple backup velocities, we can get the maximum of all of those velocities in each quadrant
*/
void AC_Avoid::find_max_quadrant_velocity(Vector2f &desired_vel, Vector2f &quad1_vel, Vector2f &quad2_vel, Vector2f &quad3_vel, Vector2f &quad4_vel) 
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
void AC_Avoid::find_max_quadrant_velocity_3D(Vector3f &desired_vel, Vector2f &quad1_vel, Vector2f &quad2_vel, Vector2f &quad3_vel, Vector2f &quad4_vel, float &max_z_vel, float &min_z_vel)
{   
    // split into horizontal and vertical components 
    Vector2f velocity_xy{desired_vel.x, desired_vel.y};
    find_max_quadrant_velocity(velocity_xy, quad1_vel, quad2_vel, quad3_vel, quad4_vel);
    
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
float AC_Avoid::get_max_speed(float kP, float accel_cmss, float distance_cm, float dt) const
{
    if (is_zero(kP)) {
        return safe_sqrt(2.0f * distance_cm * accel_cmss);
    } else {
        return sqrt_controller(distance_cm, kP, accel_cmss, dt);
    }
}

/*
 * Adjusts the desired velocity for the circular fence.
 */
void AC_Avoid::adjust_velocity_circle_fence(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, float dt)
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
    const float desired_speed = desired_vel_cms.length();
    if (is_zero(desired_speed)) {
        // no avoidance necessary when desired speed is zero
        return;
    }

    const AP_AHRS &_ahrs = AP::ahrs();

    // get position as a 2D offset from ahrs home
    Vector2f position_xy;
    if (!_ahrs.get_relative_position_NE_home(position_xy)) {
        // we have no idea where we are....
        return;
    }
    position_xy *= 100.0f; // m -> cm

    // get the fence radius in cm
    const float fence_radius = _fence.get_radius() * 100.0f;
    // get the margin to the fence in cm
    const float margin_cm = _fence.get_margin() * 100.0f;

    if (margin_cm > fence_radius) {
        return;
    }

    // get vehicle distance from home
    const float dist_from_home = position_xy.length();
    if (dist_from_home > fence_radius) {
        // outside of circular fence, no velocity adjustments
        return;
    }
    const float distance_to_boundary = fence_radius - dist_from_home;

    // for backing away
    Vector2f quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel;
    
    // back away if vehicle has breached margin
    if (is_negative(distance_to_boundary - margin_cm)) {     
        calc_backup_velocity_2D(kP, accel_cmss, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel, margin_cm - distance_to_boundary, position_xy, dt);
    }
    // desired backup velocity is sum of maximum velocity component in each quadrant 
    backup_vel = quad_1_back_vel + quad_2_back_vel + quad_3_back_vel + quad_4_back_vel;

    // vehicle is inside the circular fence
    switch (_behavior) {
    case BEHAVIOR_SLIDE: {
        // implement sliding behaviour
        const Vector2f stopping_point = position_xy + desired_vel_cms*(get_stopping_distance(kP, accel_cmss, desired_speed)/desired_speed);
        const float stopping_point_dist_from_home = stopping_point.length();
        if (stopping_point_dist_from_home <= fence_radius - margin_cm) {
            // stopping before before fence so no need to adjust
            return;
        }
        // unsafe desired velocity - will not be able to stop before reaching margin from fence
        // Project stopping point radially onto fence boundary
        // Adjusted velocity will point towards this projected point at a safe speed
        const Vector2f target_offset = stopping_point * ((fence_radius - margin_cm) / stopping_point_dist_from_home);
        const Vector2f target_direction = target_offset - position_xy;
        const float distance_to_target = target_direction.length();
        if (is_positive(distance_to_target)) {
            const float max_speed = get_max_speed(kP, accel_cmss, distance_to_target, dt);
            desired_vel_cms = target_direction * (MIN(desired_speed,max_speed) / distance_to_target);
        }
      break;
    } 
  
    case (BEHAVIOR_STOP): {
        // implement stopping behaviour
        // calculate stopping point plus a margin so we look forward far enough to intersect with circular fence
        const Vector2f stopping_point_plus_margin = position_xy + desired_vel_cms*((2.0f + margin_cm + get_stopping_distance(kP, accel_cmss, desired_speed))/desired_speed);
        const float stopping_point_plus_margin_dist_from_home = stopping_point_plus_margin.length();
        if (dist_from_home >= fence_radius - margin_cm) {
            // vehicle has already breached margin around fence
            // if stopping point is even further from home (i.e. in wrong direction) then adjust speed to zero
            // otherwise user is backing away from fence so do not apply limits
            if (stopping_point_plus_margin_dist_from_home >= dist_from_home) {
                desired_vel_cms.zero();
            }
        } else {
            // shorten vector without adjusting its direction
            Vector2f intersection;
            if (Vector2f::circle_segment_intersection(position_xy, stopping_point_plus_margin, Vector2f(0.0f,0.0f), fence_radius - margin_cm, intersection)) {
                const float distance_to_target = (intersection - position_xy).length();
                const float max_speed = get_max_speed(kP, accel_cmss, distance_to_target, dt);
                if (max_speed < desired_speed) {
                    desired_vel_cms *= MAX(max_speed, 0.0f) / desired_speed;
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
void AC_Avoid::adjust_velocity_inclusion_and_exclusion_polygons(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, float dt)
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
    Vector2f quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel;

    // iterate through inclusion polygons
    const uint8_t num_inclusion_polygons = fence->polyfence().get_inclusion_polygon_count();
    for (uint8_t i = 0; i < num_inclusion_polygons; i++) {
        uint16_t num_points;
        const Vector2f* boundary = fence->polyfence().get_inclusion_polygon(i, num_points);
        Vector2f backup_vel_inc;
        // adjust velocity
        adjust_velocity_polygon(kP, accel_cmss, desired_vel_cms, backup_vel_inc, boundary, num_points, fence->get_margin(), dt, true);
        find_max_quadrant_velocity(backup_vel_inc, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel);
    }

    // iterate through exclusion polygons
    const uint8_t num_exclusion_polygons = fence->polyfence().get_exclusion_polygon_count();
    for (uint8_t i = 0; i < num_exclusion_polygons; i++) {
        uint16_t num_points;
        const Vector2f* boundary = fence->polyfence().get_exclusion_polygon(i, num_points);
        Vector2f backup_vel_exc;
        // adjust velocity
        adjust_velocity_polygon(kP, accel_cmss, desired_vel_cms, backup_vel_exc, boundary, num_points, fence->get_margin(), dt, false);
        find_max_quadrant_velocity(backup_vel_exc, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel);
    }
    // desired backup velocity is sum of maximum velocity component in each quadrant 
    backup_vel = quad_1_back_vel + quad_2_back_vel + quad_3_back_vel + quad_4_back_vel;
}

/*
 * Adjusts the desired velocity for the inclusion circles
 */
void AC_Avoid::adjust_velocity_inclusion_circles(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, float dt)
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
    Vector2f position_NE;
    if (!AP::ahrs().get_relative_position_NE_origin(position_NE)) {
        // do not limit velocity if we don't have a position estimate
        return;
    }
    position_NE = position_NE * 100.0f;  // m to cm

    // get the margin to the fence in cm
    const float margin_cm = fence->get_margin() * 100.0f;

    // get desired speed
    const float desired_speed = desired_vel_cms.length();

    // get stopping distance as an offset from the vehicle
    Vector2f stopping_offset;
    if (!is_zero(desired_speed)) {
        switch (_behavior) {
            case BEHAVIOR_SLIDE:
                stopping_offset = desired_vel_cms*(get_stopping_distance(kP, accel_cmss, desired_speed)/desired_speed);
                break;
            case BEHAVIOR_STOP:
                // calculate stopping point plus a margin so we look forward far enough to intersect with circular fence
                stopping_offset = desired_vel_cms*((2.0f + margin_cm + get_stopping_distance(kP, accel_cmss, desired_speed))/desired_speed);
                break;
        }
    }

    // for backing away
    Vector2f quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel;

    // iterate through inclusion circles
    for (uint8_t i = 0; i < num_circles; i++) {
        Vector2f center_pos_cm;
        float radius;
        if (fence->polyfence().get_inclusion_circle(i, center_pos_cm, radius)) {
            // get position relative to circle's center
            const Vector2f position_NE_rel = (position_NE - center_pos_cm);

            // if we are outside this circle do not limit velocity for this circle
            const float dist_sq_cm = position_NE_rel.length_squared();
            const float radius_cm = (radius * 100.0f);
            if (dist_sq_cm > sq(radius_cm)) {
                continue;
            }

            const float radius_with_margin = radius_cm - margin_cm;
            if (is_negative(radius_with_margin)) {
                return;
            }
            
            const float margin_breach = radius_with_margin - safe_sqrt(dist_sq_cm);
            // back away if vehicle has breached margin
            if (is_negative(margin_breach)) {
                calc_backup_velocity_2D(kP, accel_cmss, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel, margin_breach, position_NE_rel, dt);
            }
            if (is_zero(desired_speed)) {
                // no avoidance necessary when desired speed is zero
                continue;
            }

            switch (_behavior) {
                case BEHAVIOR_SLIDE: {
                    // implement sliding behaviour
                    const Vector2f stopping_point = position_NE_rel + stopping_offset;
                    const float stopping_point_dist = stopping_point.length();
                    if (is_zero(stopping_point_dist) || (stopping_point_dist <= (radius_cm - margin_cm))) {
                        // stopping before before fence so no need to adjust for this circle
                        continue;
                    }
                    // unsafe desired velocity - will not be able to stop before reaching margin from fence
                    // project stopping point radially onto fence boundary
                    // adjusted velocity will point towards this projected point at a safe speed
                    const Vector2f target_offset = stopping_point * ((radius_cm - margin_cm) / stopping_point_dist);
                    const Vector2f target_direction = target_offset - position_NE_rel;
                    const float distance_to_target = target_direction.length();
                    if (is_positive(distance_to_target)) {
                        const float max_speed = get_max_speed(kP, accel_cmss, distance_to_target, dt);
                        desired_vel_cms = target_direction * (MIN(desired_speed,max_speed) / distance_to_target);
                    }
                }
                break;
                case BEHAVIOR_STOP: {
                    // implement stopping behaviour
                    const Vector2f stopping_point_plus_margin = position_NE_rel + stopping_offset;
                    const float dist_cm = safe_sqrt(dist_sq_cm);
                    if (dist_cm >= radius_cm - margin_cm) {
                        // vehicle has already breached margin around fence
                        // if stopping point is even further from center (i.e. in wrong direction) then adjust speed to zero
                        // otherwise user is backing away from fence so do not apply limits
                        if (stopping_point_plus_margin.length() >= dist_cm) {
                            desired_vel_cms.zero();
                            // desired backup velocity is sum of maximum velocity component in each quadrant 
                            backup_vel = quad_1_back_vel + quad_2_back_vel + quad_3_back_vel + quad_4_back_vel;
                            return;
                        }
                    } else {
                        // shorten vector without adjusting its direction
                        Vector2f intersection;
                        if (Vector2f::circle_segment_intersection(position_NE_rel, stopping_point_plus_margin, Vector2f(0.0f,0.0f), radius_cm - margin_cm, intersection)) {
                            const float distance_to_target = (intersection - position_NE_rel).length();
                            const float max_speed = get_max_speed(kP, accel_cmss, distance_to_target, dt);
                            if (max_speed < desired_speed) {
                                desired_vel_cms *= MAX(max_speed, 0.0f) / desired_speed;
                            }
                        }
                    }
                }
                break;
            }
        }
    }
    // desired backup velocity is sum of maximum velocity component in each quadrant 
    backup_vel = quad_1_back_vel + quad_2_back_vel + quad_3_back_vel + quad_4_back_vel;
}

/*
 * Adjusts the desired velocity for the exclusion circles
 */
void AC_Avoid::adjust_velocity_exclusion_circles(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, float dt)
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
    Vector2f position_NE;
    if (!AP::ahrs().get_relative_position_NE_origin(position_NE)) {
        // do not limit velocity if we don't have a position estimate
        return;
    }
    position_NE = position_NE * 100.0f;  // m to cm

    // get the margin to the fence in cm
    const float margin_cm = fence->get_margin() * 100.0f;

    // for backing away
    Vector2f quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel;

    // get desired speed
    const float desired_speed = desired_vel_cms.length();
    
    // calculate stopping distance as an offset from the vehicle (only used for BEHAVIOR_STOP)
    // add a margin so we look forward far enough to intersect with circular fence
    Vector2f stopping_offset;
    if (!is_zero(desired_speed)) {
        if ((AC_Avoid::BehaviourType)_behavior.get() == BEHAVIOR_STOP) {
            stopping_offset = desired_vel_cms*((2.0f + margin_cm + get_stopping_distance(kP, accel_cmss, desired_speed))/desired_speed);
        }
    }
    // iterate through exclusion circles
    for (uint8_t i = 0; i < num_circles; i++) {
        Vector2f center_pos_cm;
        float radius;
        if (fence->polyfence().get_exclusion_circle(i, center_pos_cm, radius)) {
            // get position relative to circle's center
            const Vector2f position_NE_rel = (position_NE - center_pos_cm);

            // if we are inside this circle do not limit velocity for this circle
            const float dist_sq_cm = position_NE_rel.length_squared();
            const float radius_cm = (radius * 100.0f);
            if (radius_cm < margin_cm) {
                return;
            }
            if (dist_sq_cm < sq(radius_cm)) {
                continue;
            }
        
            const Vector2f vector_to_center = center_pos_cm - position_NE;
            const float dist_to_boundary = vector_to_center.length() - radius_cm;
            // back away if vehicle has breached margin
            if (is_negative(dist_to_boundary - margin_cm)) {
                calc_backup_velocity_2D(kP, accel_cmss, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel, margin_cm - dist_to_boundary, vector_to_center, dt);
            }
            if (is_zero(desired_speed)) {
                // no avoidance necessary when desired speed is zero
                continue;
            }

            switch (_behavior) {
                case BEHAVIOR_SLIDE: {
                    // vector from current position to circle's center
                    Vector2f limit_direction = vector_to_center;
                    if (limit_direction.is_zero()) {
                        // vehicle is exactly on circle center so do not limit velocity
                        continue;
                    }
                    // calculate distance to edge of circle
                    const float limit_distance_cm = limit_direction.length() - radius_cm;
                    if (!is_positive(limit_distance_cm)) {
                        // vehicle is within circle so do not limit velocity
                        continue;
                    }
                    // vehicle is outside the circle, adjust velocity to stay outside
                    limit_direction.normalize();
                    limit_velocity_2D(kP, accel_cmss, desired_vel_cms, limit_direction, MAX(limit_distance_cm - margin_cm, 0.0f), dt);
                }
                break;
                case BEHAVIOR_STOP: {
                    // implement stopping behaviour
                    const Vector2f stopping_point_plus_margin = position_NE_rel + stopping_offset;
                    const float dist_cm = safe_sqrt(dist_sq_cm);
                    if (dist_cm < radius_cm + margin_cm) {
                        // vehicle has already breached margin around fence
                        // if stopping point is closer to center (i.e. in wrong direction) then adjust speed to zero
                        // otherwise user is backing away from fence so do not apply limits
                        if (stopping_point_plus_margin.length() <= dist_cm) {
                            desired_vel_cms.zero();
                            backup_vel = quad_1_back_vel + quad_2_back_vel + quad_3_back_vel + quad_4_back_vel;
                            return;
                        }
                    } else {
                        // shorten vector without adjusting its direction
                        Vector2f intersection;
                        if (Vector2f::circle_segment_intersection(position_NE_rel, stopping_point_plus_margin, Vector2f(0.0f,0.0f), radius_cm + margin_cm, intersection)) {
                            const float distance_to_target = (intersection - position_NE_rel).length();
                            const float max_speed = get_max_speed(kP, accel_cmss, distance_to_target, dt);
                            if (max_speed < desired_speed) {
                                desired_vel_cms *= MAX(max_speed, 0.0f) / desired_speed;
                            }
                        }
                    }
                }
                break;
            }
        }
    }
    // desired backup velocity is sum of maximum velocity component in each quadrant 
    backup_vel = quad_1_back_vel + quad_2_back_vel + quad_3_back_vel + quad_4_back_vel;
}

/*
 * Adjusts the desired velocity for the beacon fence.
 */
void AC_Avoid::adjust_velocity_beacon_fence(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, float dt)
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
    float margin = 0;
    if (AP::fence()) {
        margin = AP::fence()->get_margin();
    }
    adjust_velocity_polygon(kP, accel_cmss, desired_vel_cms, backup_vel, boundary, num_points, margin, dt, true);
}

/*
 * Adjusts the desired velocity based on output from the proximity sensor
 */
void AC_Avoid::adjust_velocity_proximity(float kP, float accel_cmss, Vector3f &desired_vel_cms, Vector3f &backup_vel, float kP_z, float accel_cmss_z, float dt)
{
#if HAL_PROXIMITY_ENABLED
    // exit immediately if proximity sensor is not present
    AP_Proximity *proximity = AP::proximity();
    if (!proximity) {
        return;
    }

    AP_Proximity &_proximity = *proximity;
    // check for status of the sensor
    if (_proximity.get_status() != AP_Proximity::Status::Good) {
        return;
    }
    // get total number of obstacles
    const uint8_t obstacle_num = _proximity.get_obstacle_count();
    if (obstacle_num == 0) {
        // no obstacles
        return;
    }
 
    const AP_AHRS &_ahrs = AP::ahrs();
    
    // for backing away
    Vector2f quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel;
    float max_back_vel_z = 0.0f;
    float min_back_vel_z = 0.0f; 

    // rotate velocity vector from earth frame to body-frame since obstacles are in body-frame
    const Vector2f desired_vel_body_cms = _ahrs.earth_to_body2D(Vector2f{desired_vel_cms.x, desired_vel_cms.y});
    
    // safe_vel will be adjusted to stay away from Proximity Obstacles
    Vector3f safe_vel = Vector3f{desired_vel_body_cms.x, desired_vel_body_cms.y, desired_vel_cms.z};
    const Vector3f safe_vel_orig = safe_vel;

    // calc margin in cm
    const float margin_cm = MAX(_margin * 100.0f, 0.0f);
    Vector3f stopping_point_plus_margin;
    if (!desired_vel_cms.is_zero()) {
        // only used for "stop mode". Pre-calculating the stopping point here makes sure we do not need to repeat the calculations under iterations.
        const float speed = safe_vel.length();
        stopping_point_plus_margin = safe_vel * ((2.0f + margin_cm + get_stopping_distance(kP, accel_cmss, speed))/speed);
    }

    for (uint8_t i = 0; i<obstacle_num; i++) {
        // get obstacle from proximity library
        Vector3f vector_to_obstacle;
        if (!_proximity.get_obstacle(i, vector_to_obstacle)) {
            // this one is not valid
            continue;
        }

        const float dist_to_boundary = vector_to_obstacle.length();
        if (is_zero(dist_to_boundary)) {
            continue;
        }

        // back away if vehicle has breached margin
        if (is_negative(dist_to_boundary - margin_cm)) {
            const float breach_dist = margin_cm - dist_to_boundary;
            // add a deadzone so that the vehicle doesn't backup and go forward again and again
            const float deadzone = MAX(0.0f, _backup_deadzone) * 100.0f;
            if (breach_dist > deadzone) {
                // this vector will help us decide how much we have to back away horizontally and vertically
                const Vector3f margin_vector = vector_to_obstacle.normalized() * breach_dist;
                const float xy_back_dist = norm(margin_vector.x, margin_vector.y);
                const float z_back_dist = margin_vector.z;
                calc_backup_velocity_3D(kP, accel_cmss, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel, xy_back_dist, vector_to_obstacle, kP_z, accel_cmss_z, z_back_dist, min_back_vel_z, max_back_vel_z, dt);
            }
        }

        if (desired_vel_cms.is_zero()) {
            // cannot limit velocity if there is nothing to limit
            // backing up (if needed) has already been done
            continue;
        }

        switch (_behavior) {
        case BEHAVIOR_SLIDE: {
            Vector3f limit_direction{vector_to_obstacle};
            // distance to closest point
            const float limit_distance_cm = limit_direction.length();
            if (is_zero(limit_distance_cm)) {
                // We are exactly on the edge, this should ideally never be possible
                // i.e. do not adjust velocity.
                continue;
            }
            // Adjust velocity to not violate margin.
            limit_velocity_3D(kP, accel_cmss, safe_vel, limit_direction, margin_cm, kP_z, accel_cmss_z, dt);
        
            break;
        }

        case BEHAVIOR_STOP: {
            // vector from current position to obstacle
            Vector3f limit_direction;
            // find closest point with line segment
            // also see if the vehicle will "roughly" intersect the boundary with the projected stopping point
            const bool intersect = _proximity.closest_point_from_segment_to_obstacle(i, Vector3f{}, stopping_point_plus_margin, limit_direction);
            if (intersect) {
                // the vehicle is intersecting the plane formed by the boundary
                // distance to the closest point from the stopping point
                float limit_distance_cm = limit_direction.length();
                if (is_zero(limit_distance_cm)) {
                    // We are exactly on the edge, this should ideally never be possible
                    // i.e. do not adjust velocity.
                    return;
                }
                if (limit_distance_cm <= margin_cm) {
                    // we are within the margin so stop vehicle
                    safe_vel.zero();
                } else {
                    // vehicle inside the given edge, adjust velocity to not violate this edge
                    limit_velocity_3D(kP, accel_cmss, safe_vel, limit_direction, margin_cm, kP_z, accel_cmss_z, dt);
                }

                break;
            }
        }
        }
    }

    // desired backup velocity is sum of maximum velocity component in each quadrant 
    const Vector2f desired_back_vel_cms_xy = quad_1_back_vel + quad_2_back_vel + quad_3_back_vel + quad_4_back_vel;
    const float desired_back_vel_cms_z = max_back_vel_z + min_back_vel_z;

    if (safe_vel == safe_vel_orig && desired_back_vel_cms_xy.is_zero() && is_zero(desired_back_vel_cms_z)) {
        // proximity avoidance did nothing, no point in doing the calculations below. Return early
        backup_vel.zero();
        return;
    }

    // set modified desired velocity vector and back away velocity vector
    // vectors were in body-frame, rotate resulting vector back to earth-frame
    const Vector2f safe_vel_2d = _ahrs.body_to_earth2D(Vector2f{safe_vel.x, safe_vel.y});
    desired_vel_cms = Vector3f{safe_vel_2d.x, safe_vel_2d.y, safe_vel.z};
    const Vector2f backup_vel_xy = _ahrs.body_to_earth2D(desired_back_vel_cms_xy);
    backup_vel = Vector3f{backup_vel_xy.x, backup_vel_xy.y, desired_back_vel_cms_z};
#endif // HAL_PROXIMITY_ENABLED
}

/*
 * Adjusts the desired velocity for the polygon fence.
 */
void AC_Avoid::adjust_velocity_polygon(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, const Vector2f* boundary, uint16_t num_points, float margin, float dt, bool stay_inside)
{
    // exit if there are no points
    if (boundary == nullptr || num_points == 0) {
        return;
    }

    const AP_AHRS &_ahrs = AP::ahrs();

    // do not adjust velocity if vehicle is outside the polygon fence
    Vector2f position_xy;
    if (!_ahrs.get_relative_position_NE_origin(position_xy)) {
        // boundary is in earth frame but we have no idea
        // where we are
        return;
    }
    position_xy = position_xy * 100.0f;  // m to cm


    // return if we have already breached polygon
    const bool inside_polygon = !Polygon_outside(position_xy, boundary, num_points);
    if (inside_polygon != stay_inside) {
        return;
    }

    // Safe_vel will be adjusted to remain within fence.
    // We need a separate vector in case adjustment fails,
    // e.g. if we are exactly on the boundary.
    Vector2f safe_vel(desired_vel_cms);
    Vector2f desired_back_vel_cms;

    // calc margin in cm
    const float margin_cm = MAX(margin * 100.0f, 0.0f);

    // for stopping
    const float speed = safe_vel.length();
    Vector2f stopping_point_plus_margin; 
    if (!desired_vel_cms.is_zero()) {
        stopping_point_plus_margin = position_xy + safe_vel*((2.0f + margin_cm + get_stopping_distance(kP, accel_cmss, speed))/speed);
    }

    // for backing away
    Vector2f quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel;
   
    for (uint16_t i=0; i<num_points; i++) {
        uint16_t j = i+1;
        if (j >= num_points) {
            j = 0;
        }
        // end points of current edge
        Vector2f start = boundary[j];
        Vector2f end = boundary[i];
        Vector2f vector_to_boundary = Vector2f::closest_point(position_xy, start, end) - position_xy;
        // back away if vehicle has breached margin
        if (is_negative(vector_to_boundary.length() - margin_cm)) {
            calc_backup_velocity_2D(kP, accel_cmss, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel, margin_cm-vector_to_boundary.length(), vector_to_boundary, dt);
        }
        
        // exit immediately if no desired velocity
        if (desired_vel_cms.is_zero()) {
            continue;
        }

        switch (_behavior) {
        case (BEHAVIOR_SLIDE): {
            // vector from current position to closest point on current edge
            Vector2f limit_direction = vector_to_boundary;
            // distance to closest point
            const float limit_distance_cm = limit_direction.length();
            if (is_zero(limit_distance_cm)) {
                // We are exactly on the edge - treat this as a fence breach.
                // i.e. do not adjust velocity.
                return;
            }
            // We are strictly inside the given edge.
            // Adjust velocity to not violate this edge.
            limit_direction /= limit_distance_cm;
            limit_velocity_2D(kP, accel_cmss, safe_vel, limit_direction, MAX(limit_distance_cm - margin_cm, 0.0f), dt);
            break;
        } 
        
        case (BEHAVIOR_STOP): {
            // find intersection with line segment
            Vector2f intersection;
            if (Vector2f::segment_intersection(position_xy, stopping_point_plus_margin, start, end, intersection)) {
                // vector from current position to point on current edge
                Vector2f limit_direction = intersection - position_xy;
                const float limit_distance_cm = limit_direction.length();
                if (is_zero(limit_distance_cm)) {
                    // We are exactly on the edge - treat this as a fence breach.
                    // i.e. do not adjust velocity.
                    return;
                }
                if (limit_distance_cm <= margin_cm) {
                    // we are within the margin so stop vehicle
                    safe_vel.zero();
                } else {
                    // vehicle inside the given edge, adjust velocity to not violate this edge
                    limit_direction /= limit_distance_cm;
                    limit_velocity_2D(kP, accel_cmss, safe_vel, limit_direction, MAX(limit_distance_cm - margin_cm, 0.0f), dt);
                }
            }
        break;
        }
        }
    }
    // desired backup velocity is sum of maximum velocity component in each quadrant 
    desired_back_vel_cms = quad_1_back_vel + quad_2_back_vel + quad_3_back_vel + quad_4_back_vel;

    // set modified desired velocity vector or back away velocity vector
    desired_vel_cms = safe_vel;
    backup_vel = desired_back_vel_cms;
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
float AC_Avoid::distance_to_lean_pct(float dist_m)
{
    // ignore objects beyond DIST_MAX
    if (dist_m < 0.0f || dist_m >= _dist_max || _dist_max <= 0.0f) {
        return 0.0f;
    }
    // inverted but linear response
    return 1.0f - (dist_m / _dist_max);
}

// returns the maximum positive and negative roll and pitch percentages (in -1 ~ +1 range) based on the proximity sensor
void AC_Avoid::get_proximity_roll_pitch_pct(float &roll_positive, float &roll_negative, float &pitch_positive, float &pitch_negative)
{
#if HAL_PROXIMITY_ENABLED
    AP_Proximity *proximity = AP::proximity();
    if (proximity == nullptr) {
        return;
    }
    AP_Proximity &_proximity = *proximity;

    // exit immediately if proximity sensor is not present
    if (_proximity.get_status() != AP_Proximity::Status::Good) {
        return;
    }

    const uint8_t obj_count = _proximity.get_object_count();

    // if no objects return
    if (obj_count == 0) {
        return;
    }

    // calculate maximum roll, pitch values from objects
    for (uint8_t i=0; i<obj_count; i++) {
        float ang_deg, dist_m;
        if (_proximity.get_object_angle_and_distance(i, ang_deg, dist_m)) {
            if (dist_m < _dist_max) {
                // convert distance to lean angle (in 0 to 1 range)
                const float lean_pct = distance_to_lean_pct(dist_m);
                // convert angle to roll and pitch lean percentages
                const float angle_rad = radians(ang_deg);
                const float roll_pct = -sinf(angle_rad) * lean_pct;
                const float pitch_pct = cosf(angle_rad) * lean_pct;
                // update roll, pitch maximums
                if (roll_pct > 0.0f) {
                    roll_positive = MAX(roll_positive, roll_pct);
                } else if (roll_pct < 0.0f) {
                    roll_negative = MIN(roll_negative, roll_pct);
                }
                if (pitch_pct > 0.0f) {
                    pitch_positive = MAX(pitch_positive, pitch_pct);
                } else if (pitch_pct < 0.0f) {
                    pitch_negative = MIN(pitch_negative, pitch_pct);
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
