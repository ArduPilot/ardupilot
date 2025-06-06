#include "Copter.h"

#if MODE_FAILSAFE_COMPASS_ENABLED

// failsafe_compass_init - initialise failsafe compass mode
bool ModeFailsafeCompass::init(bool ignore_checks)
{
    // initialize vertical maximum speeds and acceleration
    pos_control->set_max_speed_accel_U_cm(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_U_cmss(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise altitude controller
    if (!pos_control->is_active_U()) {
        pos_control->init_U_controller();
    }

    // set initial target heading to configured failsafe heading
    _target_heading_deg = g2.fs_compass_heading;
    
    // convert heading to radians
    float target_heading_rad = radians(_target_heading_deg);
    
    // calculate target velocity vector from heading and default speed
    _target_vel_x = FAILSAFE_COMPASS_SPEED_CMS * sinf(target_heading_rad);
    _target_vel_y = FAILSAFE_COMPASS_SPEED_CMS * cosf(target_heading_rad);

    return true;
}

// failsafe_compass_run - runs the failsafe compass mode
// should be called at 100hz or more
void ModeFailsafeCompass::run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // Get target altitude from RTL altitude parameter
    float target_alt_cm = g.rtl_altitude * 100.0f;
    
    // Get current altitude (cm above home)
    float current_alt_cm = pos_control->get_pos_target_U_cm();
    
    // Calculate climb rate based on altitude difference
    float alt_diff_cm = target_alt_cm - current_alt_cm;
    float target_climb_rate = 0.0f;
    
    if (alt_diff_cm > FAILSAFE_COMPASS_ALT_TOLERANCE_CM) {
        // Need to climb
        target_climb_rate = g.pilot_speed_up;
    } else if (alt_diff_cm < -FAILSAFE_COMPASS_ALT_TOLERANCE_CM) {
        // Need to descend
        target_climb_rate = -get_pilot_speed_dn();
    }
    
    // Send the commanded climb rate to the position controller
    pos_control->set_pos_target_U_from_climb_rate_cm(target_climb_rate);

    // Convert target heading to radians
    float target_heading_rad = radians(_target_heading_deg);
    
    // Calculate target velocity vector from heading and default speed
    float target_vel_x = FAILSAFE_COMPASS_SPEED_CMS * sinf(target_heading_rad);
    float target_vel_y = FAILSAFE_COMPASS_SPEED_CMS * cosf(target_heading_rad);
    
    // Get current velocity from AHRS
    Vector3f vel_ned;
    if (!ahrs.get_velocity_NED(vel_ned)) {
        // If we can't get velocity, use zero
        vel_ned.zero();
    }
    // Convert from m/s to cm/s and extract horizontal components
    Vector2f curr_vel(vel_ned.x * 100.0f, vel_ned.y * 100.0f);
    
    // Calculate velocity error
    float vel_error_x = target_vel_x - curr_vel.x;
    float vel_error_y = target_vel_y - curr_vel.y;
    
    // Convert velocity error to lean angles using simple P controller
    float target_roll_cd = -vel_error_y * FAILSAFE_COMPASS_VEL_P_GAIN;
    float target_pitch_cd = vel_error_x * FAILSAFE_COMPASS_VEL_P_GAIN;
    
    // Constrain lean angles to fixed 10 degrees for POC
    target_roll_cd = constrain_float(target_roll_cd, -FAILSAFE_COMPASS_PITCH_DEG * 100, FAILSAFE_COMPASS_PITCH_DEG * 100);
    target_pitch_cd = constrain_float(target_pitch_cd, -FAILSAFE_COMPASS_PITCH_DEG * 100, FAILSAFE_COMPASS_PITCH_DEG * 100);

    // Calculate target yaw from target heading
    float target_yaw_cd = target_heading_rad * RAD_TO_DEG * 100.0f;
    
    // Call attitude controller
    attitude_control->input_euler_angle_roll_pitch_yaw_cd(target_roll_cd, target_pitch_cd, target_yaw_cd, true);
    
    // Update altitude controller
    pos_control->update_U_controller();
}

float ModeFailsafeCompass::wp_distance_m() const
{
    return 0.0f;
}

int32_t ModeFailsafeCompass::wp_bearing() const
{
    return _target_heading_deg;
}

#endif  // MODE_FAILSAFE_COMPASS_ENABLED