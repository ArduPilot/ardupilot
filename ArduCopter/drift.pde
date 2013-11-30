/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

////////////////////////////////////////////////////////////////////////////////
// Drift Mode
////////////////////////////////////////////////////////////////////////////////

#ifndef DRIFT_SPEEDGAIN
 # define DRIFT_SPEEDGAIN 14.0
#endif

// The function call for managing the flight mode drift
static void
get_roll_pitch_drift()
{
}

// get_yaw_drift - roll-pitch and yaw controller for drift mode
static void
get_yaw_drift()
{
    static float breaker = 0.0;
    // convert pilot input to lean angles
    // moved to Yaw since it is called before get_roll_pitch_drift();
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, control_roll, control_pitch);

    // Grab inertial velocity
    Vector3f vel = inertial_nav.get_velocity();

    // rotate roll, pitch input from north facing to vehicle's perspective
    float roll_vel =  vel.y * cos_yaw - vel.x * sin_yaw; // body roll vel
    float pitch_vel = vel.y * sin_yaw + vel.x * cos_yaw; // body pitch vel

    float pitch_vel2 = min(fabs(pitch_vel), 800);
    // simple gain scheduling for yaw input
    get_yaw_rate_stabilized_ef((float)(control_roll/2) * (1.0 - (pitch_vel2 / 2400.0)));

    roll_vel = constrain_float(roll_vel, -322, 322);
    pitch_vel = constrain_float(pitch_vel, -322, 322);

    // always limit roll
    control_roll = roll_vel * -DRIFT_SPEEDGAIN;
    get_stabilize_roll(control_roll);

    if(control_pitch == 0){
        // .14/ (.03 * 100) = 4.6 seconds till full breaking
        breaker+= .03;
        breaker = min(breaker, DRIFT_SPEEDGAIN);
        // If we let go of sticks, bring us to a stop
        control_pitch = pitch_vel * breaker;
    }else{
        breaker = 0.0;
    }

    // stabilize pitch
    get_stabilize_pitch(control_pitch);
}
