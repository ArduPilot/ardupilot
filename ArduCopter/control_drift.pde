/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_drift.pde - init and run calls for drift flight mode
 */

#ifndef DRIFT_SPEEDGAIN
 # define DRIFT_SPEEDGAIN 14.0
#endif

// drift_init - initialise drift controller
static bool drift_init(bool ignore_checks)
{
    if (GPS_ok() || ignore_checks) {
        return true;
    }else{
        return false;
    }
}

// drift_run - runs the drift controller
// should be called at 100hz or more
static void drift_run()
{
    static float breaker = 0.0;
    int16_t target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;

    // if not armed or landed, set throttle to zero and exit immediately
    if(!motors.armed() || ap.land_complete) {
        attitude_control.init_targets();
        attitude_control.set_throttle_out(0, false);
        return;
    }

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);

    // Grab inertial velocity
    Vector3f vel = inertial_nav.get_velocity();

    // rotate roll, pitch input from north facing to vehicle's perspective
    float roll_vel =  vel.y * ahrs.cos_yaw() - vel.x * ahrs.sin_yaw(); // body roll vel
    float pitch_vel = vel.y * ahrs.sin_yaw() + vel.x * ahrs.cos_yaw(); // body pitch vel

    float pitch_vel2 = min(fabs(pitch_vel), 800);

    // simple gain scheduling for yaw input
    target_yaw_rate = (float)(target_roll/2) * (1.0 - (pitch_vel2 / 2400.0));

    roll_vel = constrain_float(roll_vel, -322, 322);
    pitch_vel = constrain_float(pitch_vel, -322, 322);

    // always limit roll
    target_roll = roll_vel * -DRIFT_SPEEDGAIN;

    // If we let go of sticks, bring us to a stop
    if(target_pitch == 0){
        // .14/ (.03 * 100) = 4.6 seconds till full breaking
        breaker += .03;
        breaker = min(breaker, DRIFT_SPEEDGAIN);
        target_pitch = pitch_vel * breaker;
    }else{
        breaker = 0.0;
    }

    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // output pilot's throttle with angle boost
    attitude_control.set_throttle_out(pilot_throttle_scaled, true);
}
