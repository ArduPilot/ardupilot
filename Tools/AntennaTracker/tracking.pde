/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/**
   state of the vehicle we are tracking
 */
static struct {
    Location location;
    uint32_t last_update_us;
    float heading; // degrees
    float ground_speed; // m/s
} vehicle;

static Location our_location;

/**
   update the pitch servo. The aim is to drive the boards pitch to the
   requested pitch
 */
static void update_pitch_servo(float pitch)
{
    channel_pitch.servo_out = g.pidPitch2Srv.get_pid_4500(degrees(ahrs.pitch) - pitch);
    channel_pitch.calc_pwm();
    channel_pitch.output();
}

/**
   update the yaw servo
 */
static void update_yaw_servo(float yaw)
{
    channel_yaw.servo_out = g.pidYaw2Srv.get_pid_4500(degrees(ahrs.yaw) - yaw);    
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
        our_location.lat = g_gps->latitude;
        our_location.lng = g_gps->longitude;
        our_location.alt = 0; // assume ground level for now
    }

    // calculate the bearing to the vehicle
    float bearing  = get_bearing_cd(our_location, vehicle.location) * 0.01f;
    float distance = get_distance(our_location, vehicle.location);
    float pitch    = degrees(atan2(vehicle.location.alt - our_location.alt, distance));
    
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
