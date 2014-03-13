/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/**
   state of the vehicle we are tracking
 */
static struct {
    Location location;      // lat, long in degrees * 10^7; alt in meters * 100
    int32_t  relative_alt;  // meters * 100
    uint32_t last_update_us;
    float heading;          // degrees
    float ground_speed;     // m/s
} vehicle;

/**
   update the pitch (elevation) servo. The aim is to drive the boards ahrs pitch to the
   requested pitch, so the board (and therefore the antenna) will be pointing at the target
 */
static void update_pitch_servo(float pitch)
{
//    pitch = 0.0; // TEST
    // degrees(ahrs.pitch) is -90 to 90, where 0 is horizontal
    // pitch argument is -90 to 90, where 0 is horizontal
    // servo_out is in 100ths of a degree
    float ahrs_pitch = ahrs.pitch_sensor*0.01f;
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
   update the yaw (azimuth) servo. The aim is to drive the boards ahrs
   yaw to the requested yaw, so the board (and therefore the antenna)
   will be pointing at the target
 */
static void update_yaw_servo(float yaw)
{
//    yaw = 0.0; // TEST

    int32_t ahrs_yaw_cd = wrap_180_cd(ahrs.yaw_sensor);
    int32_t yaw_cd   = wrap_180_cd(yaw*100);
    const int16_t margin = 500; // 5 degrees slop

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
    int32_t err = wrap_180_cd(ahrs_yaw_cd - yaw_cd);

    /*
      a positive error means that we need to rotate counter-clockwise
      a negative error means that we need to rotate clockwise

      Use our current yawspeed to determine if we are moving in the
      right direction
     */
    static int8_t slew_dir = 0;
    int8_t new_slew_dir = slew_dir;

    Vector3f omega = ahrs.get_gyro();
    if (abs(channel_yaw.servo_out) == 18000 && abs(err) > margin && err * omega.z >= 0) {
        // we are at the limit of the servo and are not moving in the
        // right direction, so slew the other way
        new_slew_dir = -channel_yaw.servo_out / 18000;
    }

    /*
      stop slewing and revert to normal control when normal control
      should move us in the right direction
     */
    if (slew_dir * err < -margin) {
        new_slew_dir = 0;
    }

#if 0    
    ::printf("err=%d slew_dir=%d new_slew_dir=%d servo=%d\n", 
             err, slew_dir, new_slew_dir, channel_yaw.servo_out);
#endif

    slew_dir = new_slew_dir;

    int16_t new_servo_out;
    if (slew_dir != 0) {
        new_servo_out = slew_dir * 18000;
        g.pidYaw2Srv.reset_I();
    } else {
        float servo_change = g.pidYaw2Srv.get_pid(err);
        servo_change = constrain_float(servo_change, -18000, 18000);
        new_servo_out = constrain_float(channel_yaw.servo_out - servo_change, -18000, 18000);
    }

    if (new_servo_out - channel_yaw.servo_out > 100) {
        new_servo_out = channel_yaw.servo_out + 100;
    } else if (new_servo_out - channel_yaw.servo_out < -100) {
        new_servo_out = channel_yaw.servo_out - 100;
    }
    channel_yaw.servo_out = new_servo_out;

    {
        // Normal tracking
        // You will need to tune the yaw PID to suit your antenna and servos
        // For my servos, suitable PID settings are:
        // param set YAW2SRV_P 0.1
        // param set YAW2SRV_I 0.05
        // param set YAW2SRV_D 0
        // param set YAW2SRV_IMAX 4000
        
    }
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
    // REVISIT: what if we lose lock during a mission and the antenna is moving?
    if (g_gps->status() >= GPS::GPS_OK_FIX_2D) {
        current_loc.lat = g_gps->latitude;
        current_loc.lng = g_gps->longitude;
        current_loc.alt = g_gps->altitude_cm;
        current_loc.options = 0; // Absolute altitude
    }

    if (control_mode == AUTO)
    {
        // calculate the bearing to the vehicle
        float bearing  = get_bearing_cd(current_loc, vehicle.location) * 0.01f;
        float distance = get_distance(current_loc, vehicle.location);
        float pitch    = degrees(atan2f(nav_status.altitude_difference, distance));

        // update the servos
        update_pitch_servo(pitch);
        update_yaw_servo(bearing);
        
        // update nav_status for NAV_CONTROLLER_OUTPUT
        nav_status.bearing  = bearing;
        nav_status.pitch    = pitch;
        nav_status.distance = distance;
    }

}

/**
   handle an updated position from the aircraft
 */
static void tracking_update_position(const mavlink_global_position_int_t &msg)
{
    vehicle.location.lat = msg.lat;
    vehicle.location.lng = msg.lon;
    vehicle.location.alt = msg.alt/10;
    vehicle.relative_alt = msg.relative_alt/10;
    vehicle.heading      = msg.hdg * 0.01f;
    vehicle.ground_speed = pythagorous2(msg.vx, msg.vy) * 0.01f;
    vehicle.last_update_us = hal.scheduler->micros();    
}


/**
   handle an updated pressure reading from the aircraft
 */
static void tracking_update_pressure(const mavlink_scaled_pressure_t &msg)
{
    float local_pressure = barometer.get_pressure();
    float aircraft_pressure = msg.press_abs*100.0f;
    float ground_temp = barometer.get_temperature();
    float scaling = local_pressure / aircraft_pressure;

    // calculate altitude difference based on difference in barometric pressure
    float alt_diff = logf(scaling) * (ground_temp+273.15f) * 29271.267 * 0.001f;
    if (!isnan(alt_diff)) {
        nav_status.altitude_difference = alt_diff;
    }
}
