// ArduSub transect flight mode
// Sub follows a line according to current crosstrack error supplied by XTE NMEA sentence
// Requires GPS providing crosstrack error
// Jacob Walser August 2016

#include "Sub.h"
#include "version.h"
#include "GCS_Mavlink.h"

#if TRANSECT_ENABLED == ENABLED

namespace {
static uint32_t last_transect_message_ms = 0;

float des_velx = 0; // inav earth-frame desired velocity +/- = north/south
float des_vely = 0; // inav earth-frame desired velocity +/- = east/west
float des_velf = 0; // pilot body-frame desired velocity +/- = forward/backward
float des_velr = 0; // pilot body-frame desired velocity +/- = right/left

// Heading PID controller update interval
uint32_t last_pid_ms = 0;
uint8_t pid_dt = 1000/20;
}

// Initialize transect controller
bool Sub::transect_init(bool ignore_checks)
{
    // fail to initialise transect mode if no GPS lock
    if (!position_ok() && !ignore_checks) {
        return false;
    }

    pos_control.init_xy_controller();

    // set speed and acceleration from wpnav's speed and acceleration
    pos_control.set_speed_xy(wp_nav.get_speed_xy());
    pos_control.set_accel_xy(wp_nav.get_wp_acceleration());
    pos_control.set_jerk_xy_to_default();

    const Vector3f& curr_pos = inertial_nav.get_position();
    const Vector3f& curr_vel = inertial_nav.get_velocity();

    // set target position and velocity to current position and velocity
    pos_control.set_xy_target(curr_pos.x, curr_pos.y);
    pos_control.set_desired_velocity_xy(curr_vel.x, curr_vel.y);

    // initialize vertical speeds and acceleration
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    last_pilot_heading = ahrs.yaw_sensor;
    des_velf = 0;
    des_velr = 0;
    des_velx = 0;
    des_vely = 0;

    return true;
}

// should be called at 100hz or more
void Sub::transect_run()
{
    uint32_t tnow = millis();

    // convert inertial nav earth-frame velocities to body-frame
    const Vector3f& vel = inertial_nav.get_velocity();
    float vel_fw = vel.x*ahrs.cos_yaw() + vel.y*ahrs.sin_yaw();
    float vel_right = -vel.x*ahrs.sin_yaw() + vel.y*ahrs.cos_yaw();

    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors.armed() || !ap.auto_armed || !motors.get_interlock()) {

        //reset targets
        des_velf = 0;
        des_velr = 0;
        des_velx = 0;
        des_vely = 0;
        last_pilot_heading = ahrs.yaw_sensor;

        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);

        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(motors.get_throttle_hover());
        pos_control.set_pos_target(inertial_nav.get_position());
        pos_control.set_desired_velocity(Vector3f(0,0,0));
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // get pilot's desired yaw rate in centidegrees per second
    //float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    //int16_t xtrack_error = constrain_int16(-gps.crosstrack_error(), -4500, 4500);
    int16_t xtrack_error = -channel_lateral->get_control_in() / 10;
    double target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

    int16_t pilot_lateral = channel_lateral->get_control_in();
    int16_t pilot_forward = channel_forward->get_control_in();

    float lateral_out = 0;
    float forward_out = 0;

    // Pilot adjusts desired velocities to maintain during transect
    if (pilot_lateral > 1000 || pilot_lateral < -1000 || pilot_forward > 1000 || pilot_forward < -1000) {

        des_velf += pilot_forward * 0.0001;
        des_velr += pilot_lateral * 0.0001;

        // desired forward and right speeds in body-frame
        des_velf = constrain_float(des_velf, -25.0, 25.0);
        des_velr = constrain_float(des_velr, -25.0, 25.0);
    }

    // rotate pilot desired velocities to earth-frame

    // forward velocity only (maintain zero lateral velocity)
    des_vely = des_velf * ahrs.sin_yaw(); // +East / -West
    des_velx = des_velf * ahrs.cos_yaw(); // +North / -South

    // lateral velocity only (maintain zero forward velocity)
    //  des_vely = des_velr * ahrs.cos_yaw(); // +East / -West
    //  des_velx = -des_velr * ahrs.sin_yaw(); // +North / -South

    //combined forward/lateral velocities
    //  des_vely = des_velf * ahrs.sin_yaw() + des_velr * ahrs.cos_yaw(); // +East / -West
    //  des_velx = des_velf * ahrs.cos_yaw() - des_velr * ahrs.sin_yaw(); // +North / -South

    // set target position and velocity to current position and velocity
    pos_control.set_desired_velocity_xy(des_velx, des_vely);

    // run position controller
    pos_control.update_xy_controller(AC_PosControl::XY_MODE_POS_AND_VEL_FF, ekfNavVelGainScaler, false);

    // get pos_control forward and lateral outputs from wp_nav pitch and roll (from copter code)
    float poscontrol_lateral = pos_control.get_roll(); //
    float poscontrol_forward = -pos_control.get_pitch(); // output is reversed

    // constrain target forward/lateral values
    poscontrol_lateral = constrain_int16(poscontrol_lateral, -aparm.angle_max, aparm.angle_max);
    poscontrol_forward = constrain_int16(poscontrol_forward, -aparm.angle_max, aparm.angle_max);

    lateral_out = poscontrol_lateral/(float)aparm.angle_max;
    forward_out = poscontrol_forward/(float)aparm.angle_max;

    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    int32_t error_heading = 0;
    if (!is_zero(target_yaw_rate)) {
        // Allow pilot to set approximate heading to maintain during transect
        last_pilot_heading = ahrs.yaw_sensor;
        last_pilot_yaw_input_ms = tnow;

    } else {
        // Set target heading after we have slowed to a stop after pilot input
        if (tnow < last_pilot_yaw_input_ms + 250) {
            target_yaw_rate = 0;
            last_pilot_heading = ahrs.yaw_sensor;
        } else {
            // We are holding the current heading
            error_heading = last_pilot_heading - ahrs.yaw_sensor;

            // Wrap error 0~360 degrees
            if (error_heading > 18000) {
                error_heading = error_heading - 36000;
            } else if (error_heading < -18000) {
                error_heading = error_heading + 36000;
            }

            // Adjust heading to correct for crosstrack error
            target_yaw_rate = g.pid_heading_control.get_pid() + g.pid_crosstrack_control.get_pid();
        }
    }

    // Update crosstrack and heading controllers
    if (tnow > last_pid_ms + pid_dt) {
        last_pid_ms = tnow;
        g.pid_heading_control.set_input_filter_all(error_heading);
        //g.pid_crosstrack_control.set_input_filter_all(-channel_lateral->get_control_in());
        g.pid_crosstrack_control.set_input_filter_all(xtrack_error);
    }

    // update attitude controller targets
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // adjust climb rate using rangefinder
    if (rangefinder_alt_ok()) {
        // if rangefinder is ok, use surface tracking
        target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
    }

    // call z axis position controller
    if (ap.at_bottom) {
        pos_control.relax_alt_hold_controllers(0.0); // clear velocity and position targets, and integrator
        pos_control.set_alt_target(inertial_nav.get_altitude() + 10.0f); // set target to 10 cm above bottom
    } else {
        if (inertial_nav.get_altitude() < g.surface_depth) { // pilot allowed to move up or down freely
            pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        } else if (target_climb_rate < 0) { // pilot allowed to move only down freely
            if (pos_control.get_vel_target_z() > 0) {
                pos_control.relax_alt_hold_controllers(0); // reset target velocity and acceleration
            }
            pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        } else if (pos_control.get_alt_target() > g.surface_depth) { // hold depth at surface level.
            pos_control.set_alt_target(g.surface_depth);
        }
    }

    pos_control.update_z_controller();

    if (tnow > last_transect_message_ms + 200) {
        last_transect_message_ms = tnow;
        mavlink_msg_command_long_send(
            (mavlink_channel_t)0, //channel
            0, //target system
            0, //target component
            47, //command id
            0, //confirmation
            des_velf,//1
            des_velr,
            vel_fw,
            vel_right,
            forward_out,
            lateral_out,
            poscontrol_forward
        );
        //gcs_send_text_fmt(MAV_SEVERITY_INFO, "%ld, %ld, %ld, %f, %d", error_heading, ahrs.yaw_sensor, last_pilot_heading, target_yaw_rate, channel_lateral->get_control_in());
        //gcs_send_text_fmt(MAV_SEVERITY_INFO, "%f, %f", g.pid_heading_control.get_pid(), g.pid_crosstrack_control.get_pid());
        //gcs_send_text_fmt(MAV_SEVERITY_INFO, "%f, %ld, %ld, %f, %d", vel_fw, ahrs.yaw_sensor, last_pilot_heading, des_velf, gps.crosstrack_error());
    }
}
#endif
