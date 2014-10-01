/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/**
   state of the vehicle we are tracking
 */
static struct {
    bool location_valid;    // true if we have a valid location for the vehicle
    Location location;      // lat, long in degrees * 10^7; alt in meters * 100
    Location location_estimate; // lat, long in degrees * 10^7; alt in meters * 100
    uint32_t last_update_us;    // last position update in micxroseconds
    uint32_t last_update_ms;    // last position update in milliseconds
    float heading;          // last known direction vehicle is moving
    float ground_speed;     // vehicle's last known ground speed in m/s
} vehicle;

/**
   update the pitch (elevation) servo. The aim is to drive the boards ahrs pitch to the
   requested pitch, so the board (and therefore the antenna) will be pointing at the target
 */
static void update_pitch_position_servo(float pitch)
{
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

    // add slew rate limit
    if (g.pitch_slew_time > 0.02f) {
        uint16_t max_change = 0.02f * 18000 / g.pitch_slew_time;
        if (max_change < 1) {
            max_change = 1;
        }
        new_servo_out = constrain_float(new_servo_out,
                                        channel_pitch.servo_out - max_change, 
                                        channel_pitch.servo_out + max_change);
    }
    channel_pitch.servo_out = new_servo_out;
}


/**
   update the pitch (elevation) servo. The aim is to drive the boards ahrs pitch to the
   requested pitch, so the board (and therefore the antenna) will be pointing at the target
 */
static void update_pitch_onoff_servo(float pitch)
{
    // degrees(ahrs.pitch) is -90 to 90, where 0 is horizontal
    // pitch argument is -90 to 90, where 0 is horizontal
    // servo_out is in 100ths of a degree
    float ahrs_pitch = degrees(ahrs.pitch);
    float err = ahrs_pitch - pitch; 

    float acceptable_error = g.onoff_pitch_rate * g.onoff_pitch_mintime;
    if (fabsf(err) < acceptable_error) {
        channel_pitch.servo_out = 0;
    } else if (err > 0) {
        // positive error means we are pointing too high, so push the
        // servo down
        channel_pitch.servo_out = -9000;        
    } else {
        // negative error means we are pointing too low, so push the
        // servo up
        channel_pitch.servo_out = 9000;
    }
}


/**
   update the pitch (elevation) servo. The aim is to drive the boards ahrs pitch to the
   requested pitch, so the board (and therefore the antenna) will be pointing at the target
 */
static void update_pitch_servo(float pitch)
{
    switch ((enum ServoType)g.servo_type.get()) {
    case SERVO_TYPE_ONOFF:
        update_pitch_onoff_servo(pitch);
        break;

    case SERVO_TYPE_POSITION:
    default:
        update_pitch_position_servo(pitch);
        break;
    }
    channel_pitch.calc_pwm();
    channel_pitch.output();
}


/**
   update the yaw (azimuth) servo. The aim is to drive the boards ahrs
   yaw to the requested yaw, so the board (and therefore the antenna)
   will be pointing at the target
 */
static void update_yaw_position_servo(float yaw)
{
    int32_t ahrs_yaw_cd = wrap_180_cd(ahrs.yaw_sensor);
    int32_t yaw_cd   = wrap_180_cd(yaw*100);
    const int16_t margin = 500; // 5 degrees slop

    static uint16_t count = 0;
    static uint32_t last_millis= 0;
    uint32_t millis = hal.scheduler->millis();
    if (millis > last_millis + 1000)
    {
//        hal.uartA->printf("ahrs_yaw_cd %ld yaw_cd %ld\n", ahrs_yaw_cd, yaw_cd);
//        hal.uartA->printf("count %d\n", count);
        last_millis = millis;
        count = 0;
    }
    count++;

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
    static int8_t slew_dir;
    static uint32_t slew_start_ms;
    int8_t new_slew_dir = slew_dir;

    Vector3f earth_rotation = ahrs.get_gyro() * ahrs.get_dcm_matrix();
    bool making_progress;
    if (slew_dir != 0) {
        making_progress = (-slew_dir * earth_rotation.z >= 0);
    } else {
        making_progress = (err * earth_rotation.z >= 0);
    }
    if (abs(channel_yaw.servo_out) == 18000 && 
        abs(err) > margin && 
        making_progress && 
        hal.scheduler->millis() - slew_start_ms > g.min_reverse_time*1000) {
        // we are at the limit of the servo and are not moving in the
        // right direction, so slew the other way
        new_slew_dir = -channel_yaw.servo_out / 18000;
        slew_start_ms = hal.scheduler->millis();
    }

    /*
      stop slewing and revert to normal control when normal control
      should move us in the right direction
     */
    if (slew_dir * err < -margin) {
        new_slew_dir = 0;
    }

    if (new_slew_dir != slew_dir) {
        gcs_send_text_fmt(PSTR("SLEW: %d/%d err=%ld servo=%ld ez=%.3f"), 
                          (int)slew_dir, (int)new_slew_dir, 
                          (long)err, 
                          (long)channel_yaw.servo_out,
                          degrees(ahrs.get_gyro().z));
    }

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

    // add slew rate limit
    if (g.yaw_slew_time > 0.02f) {
        uint16_t max_change = 0.02f * 36000.0f / g.yaw_slew_time;
        if (max_change < 1) {
            max_change = 1;
        }
        new_servo_out = constrain_float(new_servo_out,
                                        channel_yaw.servo_out - max_change, 
                                        channel_yaw.servo_out + max_change);
    }

    channel_yaw.servo_out = new_servo_out;
}


/**
   update the yaw (azimuth) servo. The aim is to drive the boards ahrs
   yaw to the requested yaw, so the board (and therefore the antenna)
   will be pointing at the target
 */
static void update_yaw_onoff_servo(float yaw)
{
    int32_t ahrs_yaw_cd = wrap_180_cd(ahrs.yaw_sensor);
    int32_t yaw_cd   = wrap_180_cd(yaw*100);
    int32_t err_cd = wrap_180_cd(ahrs_yaw_cd - yaw_cd);
    float err = err_cd * 0.01f;
    
    float acceptable_error = g.onoff_yaw_rate * g.onoff_yaw_mintime;
    if (fabsf(err) < acceptable_error) {
        channel_yaw.servo_out = 0;
    } else if (err > 0) {
        // positive error means we are clockwise of the target, so
        // move anti-clockwise
        channel_yaw.servo_out = -18000;        
    } else {
        // negative error means we are anti-clockwise of the target, so
        // move clockwise
        channel_yaw.servo_out = 18000;        
    }
}

/**
   update the yaw (azimuth) servo. 
 */
static void update_yaw_servo(float yaw)
{
    switch ((enum ServoType)g.servo_type.get()) {
    case SERVO_TYPE_ONOFF:
        update_yaw_onoff_servo(yaw);
        break;

    case SERVO_TYPE_POSITION:
    default:
        update_yaw_position_servo(yaw);
        break;
    }
    channel_yaw.calc_pwm();
    channel_yaw.output();
}


/*
  control servos for AUTO mode
 */
static void update_auto(void)
{    
    if (g.startup_delay > 0 && 
        hal.scheduler->millis() - start_time_ms < g.startup_delay*1000) {
        return;
    }
    float yaw = wrap_180_cd((nav_status.bearing+g.yaw_trim)*100) * 0.01f;
    float pitch = constrain_float(nav_status.pitch+g.pitch_trim, -90, 90);
    update_pitch_servo(pitch);
    update_yaw_servo(yaw);
}


/*
  control servos for MANUAL mode
 */
static void update_manual(void)
{
    channel_yaw.radio_out = channel_yaw.radio_in;
    channel_pitch.radio_out = channel_pitch.radio_in;
    channel_yaw.output();
    channel_pitch.output();
}

/*
  control servos for SCAN mode
 */
static void update_scan(void)
{    
    if (!nav_status.manual_control_yaw) {
        float yaw_delta = g.scan_speed * 0.02f;
        nav_status.bearing   += yaw_delta   * (nav_status.scan_reverse_yaw?-1:1);
        if (nav_status.bearing < 0 && nav_status.scan_reverse_yaw) {
            nav_status.scan_reverse_yaw = false;
        }
        if (nav_status.bearing > 360 && !nav_status.scan_reverse_yaw) {
            nav_status.scan_reverse_yaw = true;
        }
        nav_status.bearing = constrain_float(nav_status.bearing, 0, 360);
    }

    if (!nav_status.manual_control_pitch) {
        float pitch_delta = g.scan_speed * 0.02f;
        nav_status.pitch += pitch_delta * (nav_status.scan_reverse_pitch?-1:1);
        if (nav_status.pitch < -90 && nav_status.scan_reverse_pitch) {
            nav_status.scan_reverse_pitch = false;
        }
        if (nav_status.pitch > 90 && !nav_status.scan_reverse_pitch) {
            nav_status.scan_reverse_pitch = true;
        }
        nav_status.pitch = constrain_float(nav_status.pitch, -90, 90);
    }

    update_auto();
}

/**
  update_vehicle_position_estimate - updates estimate of vehicle positions
  should be called at 50hz
 */
static void update_vehicle_pos_estimate()
{
    // calculate time since last actual position update
    float dt = (hal.scheduler->micros() - vehicle.last_update_us) * 1.0e-6f;

    // if less than 5 seconds since last position update estimate the position
    if (dt < TRACKING_TIMEOUT_SEC) {
        // project the vehicle position to take account of lost radio packets
        vehicle.location_estimate = vehicle.location;
        location_update(vehicle.location_estimate, vehicle.heading, vehicle.ground_speed * dt);
        // set valid_location flag
        vehicle.location_valid = true;
    } else {
        // vehicle has been lost, set lost flag
        vehicle.location_valid = false;
    }
}

/**
  main antenna tracking code, called at 50Hz
 */
static void update_tracking(void)
{
    // update vehicle position estimate
    update_vehicle_pos_estimate();

    // update our position if we have at least a 2D fix
    // REVISIT: what if we lose lock during a mission and the antenna is moving?
    if (gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
        current_loc = gps.location();
    }

    // calculate the bearing to the vehicle
    float bearing  = get_bearing_cd(current_loc, vehicle.location_estimate) * 0.01f;
    float distance = get_distance(current_loc, vehicle.location_estimate);
    float pitch    = degrees(atan2f(nav_status.altitude_difference, distance));

    // update nav_status for NAV_CONTROLLER_OUTPUT
    if (control_mode != SCAN && !nav_status.manual_control_yaw) {
        nav_status.bearing = bearing;
    }
    if (control_mode != SCAN && !nav_status.manual_control_pitch) {
        nav_status.pitch = pitch;
    }
    nav_status.distance = distance;

    switch (control_mode) {
    case AUTO:
        update_auto();
        break;

    case MANUAL:
        update_manual();
        break;

    case SCAN:
        update_scan();
        break;

    case STOP:
    case INITIALISING:
        break;
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
    vehicle.heading      = msg.hdg * 0.01f;
    vehicle.ground_speed = pythagorous2(msg.vx, msg.vy) * 0.01f;
    vehicle.last_update_us = hal.scheduler->micros();    
    vehicle.last_update_ms = hal.scheduler->millis();
}


/**
   handle an updated pressure reading from the aircraft
 */
static void tracking_update_pressure(const mavlink_scaled_pressure_t &msg)
{
    float local_pressure = barometer.get_pressure();
    float aircraft_pressure = msg.press_abs*100.0f;

    // calculate altitude difference based on difference in barometric pressure
    float alt_diff = barometer.get_altitude_difference(local_pressure, aircraft_pressure);
    if (!isnan(alt_diff)) {
        nav_status.altitude_difference = alt_diff + nav_status.altitude_offset;
    }

    if (nav_status.need_altitude_calibration) {
        // we have done a baro calibration - zero the altitude
        // difference to the aircraft
        nav_status.altitude_offset = -nav_status.altitude_difference;
        nav_status.altitude_difference = 0;
        nav_status.need_altitude_calibration = false;
    }
}

/**
   handle a manual control message by using the data to command yaw and pitch
 */
static void tracking_manual_control(const mavlink_manual_control_t &msg)
{
    nav_status.bearing = msg.x;
    nav_status.pitch   = msg.y;
    nav_status.distance = 0.0;
    nav_status.manual_control_yaw   = (msg.x != 0x7FFF);
    nav_status.manual_control_pitch = (msg.y != 0x7FFF);
    // z, r and buttons are not used
}

/**
   update_armed_disarmed - set armed LED if we have received a position update within the last 5 seconds
 */
static void update_armed_disarmed()
{
    if (vehicle.last_update_ms != 0 && (hal.scheduler->millis() - vehicle.last_update_ms) < TRACKING_TIMEOUT_MS) {
        AP_Notify::flags.armed = true;
    } else {
        AP_Notify::flags.armed = false;
    }
}
