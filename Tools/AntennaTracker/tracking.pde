/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/**
   state of the vehicle we are tracking
 */
static struct {
    Location location; // lat, long in degrees * 10^7; alt in meters * 100
    uint32_t last_update_us;
    float heading; // degrees
    float ground_speed; // m/s
} vehicle;

/**
   update the pitch (elevation) servo. The aim is to drive the boards ahrs pitch to the
   requested pitch, so the board (and therefore the antenna) will be pointing at the target
 */
static void update_pitch_servo(float pitch)
{
    // degrees(ahrs.pitch) is -90 to 90, where 0 is horizontal
    // pitch argument is -90 to 90, where 0 is horizontal
    // servo_out is in 100ths of a degree
    float ahrs_pitch = degrees(ahrs.pitch);
    int32_t err = (ahrs_pitch - pitch) * 100.0; 
    // Need to configure your servo so that increasing servo_out causes increase in pitch/elevation (ie pointing higher into the sky,
    // above the horizon. On my antenna tracker this requires the pitch/elevation servo to be reversed
    // param set RC2_REV -1
    //
    // The pitch servo (RC channel 2) is configured for servo_out of -9000-0-9000 servo_out, 
    // which will drive the servo from RC2_MIN to RC2_MAX usec pulse width.
    // Therefore, you must set RC2_MIN and RC2_MAX so that your servo drives the antenna altitude between -90 to 90 exactly
    // To drive my HS-645MG servos through their full 180 degrees of rotational range, I have to set:
    // param set RC2_MAX 2540
    // param set RC2_MIN 640
    //
    // You will also need to tune the pitch PID to suit your antenna and servos. I use:
    // PITCH2SRV_P      0.100000
    // PITCH2SRV_I      0.020000
    // PITCH2SRV_D      0.000000
    // PITCH2SRV_IMAX   4000.000000
    int32_t new_servo_out = channel_pitch.servo_out - g.pidPitch2Srv.get_pid(err);
    channel_pitch.servo_out = constrain_float(new_servo_out, -9000, 9000);
    channel_pitch.calc_pwm();
    channel_pitch.output();
}

/**
   update the yaw (azimuth) servo. The aim is to drive the boards ahrs yaw to the
   requested yaw, so the board (and therefore the antenna) will be pinting at the target
 */
static void update_yaw_servo(float yaw)
{
    // degrees(ahrs.yaw) is -180 to 180, where 0 is north
    float ahrs_yaw = degrees(ahrs.yaw);
    // yaw argument is 0 to 360 where 0 and 360 are north
    // Make yaw -180-0-180 too
    if (yaw > 180)
        yaw = yaw - 360;

    // Antenna as Ballerina. Use with antenna that do not have continuously rotating servos, ie at some point in rotation
    // the servo limits are reached and the servo has to slew 360 degrees to the 'other side' to keep tracking.
    //
    // This algorithm accounts for the fact that the antenna mount may not be aligned with North 
    // (in fact, any alignment is permissable), and that the alignment may change (possibly rapidly) over time 
    // (as when the antenna is mounted on a moving, turning vehicle)
    // When the servo is being forced beyond its limits, it rapidly slews to the 'other side' then normal tracking takes over.
    //
    // Caution: this whole system is compromised by the fact that the ahrs_yaw reported by the compass system lags 
    // the actual yaw by about 5 seconds, and also periodically realigns itself with about a 30 second period, 
    // which makes it very hard to be completely sure exactly where the antenna is _really_ pointing
    // especially during the high speed slew. This can cause odd pointing artifacts from time to time. The best strategy is to
    // position and point the mount so the aircraft does not 'go behind' the antenna (if possible)
    //
    // With my antenna mount, large pwm output drives the antenna anticlockise, so need:
    // param set RC1_REV -1
    // to reverse the servo. Yours may be different
    //
    // You MUST set RC1_MIN and RC1_MAX so that your servo drives the antenna azimuth from -180 to 180 relative
    // to the mount.
    // To drive my HS-645MG servos through their full 180 degrees of rotational range and therefore the
    // antenna through a full 360 degrees, I have to set:
    // param set RC1_MAX 2380
    // param set RC1_MIN 680
    // According to the specs at https://www.servocity.com/html/hs-645mg_ultra_torque.html,
    // that should be 600 through 2400, but the azimuth gearing in my antenna pointer is not exactly 2:1
    int32_t err = (ahrs_yaw - yaw) * 100.0;
    static int32_t last_err = 0.0;

    // Correct for wrapping yaw at +-180
    // so we get the error to the _closest_ version of the target azimuth
    // +ve error requires anticlockwise motion (ie towards more negative yaw)
    if (err > 18000)
        err -= 36000;
    else if (err < -18000)
        err += 36000;

    static int32_t slew_to = 0;

    int32_t servo_err = channel_yaw.servo_out - err; // Servo position we need to get to
    if (   !slew_to
        && servo_err > 19000 // 10 degreee deadband
        && err < 0
        && last_err > err)
    {
        // We need to be beyond the servo limits and the error magnitude is increasing
        // Slew most of the way to the other side at high speed...
        slew_to = servo_err - 27000;
    }
    else if (   !slew_to
             && servo_err < -19000 // 10 degreee deadband
             && err > 0
             && last_err < err)
    {
        // We need to be beyond the servo limits and the error magnitude is increasing
        // Slew most of the way to the other side at high speed...
        slew_to = servo_err + 27000;
    }
    else if (   slew_to < 0
             && err > 0
             && last_err < err)
    {
        // ...then let normal tracking take over
        slew_to = 0;
    }
    else if (   slew_to > 0
             && err < 0
             && last_err > err)
    {
        // ...then let normal tracking take over
        slew_to = 0;
    }

    if (slew_to)
    {
        channel_yaw.servo_out = slew_to;
    }
    else
    {
        // Normal tracking
        // You will need to tune the yaw PID to suit your antenna and servos
        // For my servos, suitable PID settings are:
        // param set YAW2SRV_P 0.1
        // param set YAW2SRV_I 0.05
        // param set YAW2SRV_D 0
        // param set YAW2SRV_IMAX 4000
        
        int32_t new_servo_out = channel_yaw.servo_out - g.pidYaw2Srv.get_pid(err);
        channel_yaw.servo_out = constrain_float(new_servo_out, -18000, 18000);
    }
    last_err = err;
    channel_yaw.calc_pwm();
    channel_yaw.output();
}


/**
  main antenna tracking code, called at 50Hz
 */
static void update_tracking(void)
{
    // project the vehicle position to take account of lost radio packets
    Location vpos = vehicle.location;
    float dt = (hal.scheduler->micros() - vehicle.last_update_us) * 1.0e-6f;
    location_update(vpos, vehicle.heading, vehicle.ground_speed * dt);

    // update our position if we have at least a 2D fix
    if (g_gps->status() >= GPS::GPS_OK_FIX_2D) {
        current_loc.lat = g_gps->latitude;
        current_loc.lng = g_gps->longitude;
        current_loc.alt = 0; // assume ground level for now REVISIT: WHY?
    }
    else {
        current_loc = home_loc; // dont know any better
    }
    // calculate the bearing to the vehicle
    float bearing  = get_bearing_cd(current_loc, vehicle.location) * 0.01f;
    float distance = get_distance(current_loc, vehicle.location);
    float pitch    = degrees(atan2((vehicle.location.alt - current_loc.alt)/100, distance));
    // update the servos
    update_pitch_servo(pitch);
    update_yaw_servo(bearing);

    // update nav_status for NAV_CONTROLLER_OUTPUT
    nav_status.bearing  = bearing;
    nav_status.pitch    = pitch;
    nav_status.distance = distance;
}


/**
   handle an updated position from the aircraft
 */
static void tracking_update_position(const mavlink_global_position_int_t &msg)
{
    vehicle.location.lat = msg.lat;
    vehicle.location.lng = msg.lon;
    vehicle.location.alt = msg.relative_alt/10;
    vehicle.heading      = msg.hdg * 0.01f;
    vehicle.ground_speed = pythagorous2(msg.vx, msg.vy) * 0.01f;
    vehicle.last_update_us = hal.scheduler->micros();    
}

