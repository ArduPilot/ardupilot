// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

// default sensors are present and healthy: gyro, accelerometer, barometer, rate_control, attitude_stabilization, yaw_position, altitude control, x/y position control, motor_control
#define MAVLINK_SENSOR_PRESENT_DEFAULT (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_YAW_POSITION | MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS | MAV_SYS_STATUS_AHRS | MAV_SYS_STATUS_SENSOR_RC_RECEIVER)

void Plane::send_heartbeat(mavlink_channel_t chan)
{
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status;
    uint32_t custom_mode = control_mode;
    
    if (failsafe.state != FAILSAFE_NONE) {
        system_status = MAV_STATE_CRITICAL;
    } else if (plane.crash_state.is_crashed) {
        system_status = MAV_STATE_EMERGENCY;
    } else if (is_flying()) {
        system_status = MAV_STATE_ACTIVE;
    } else {
        system_status = MAV_STATE_STANDBY;
    }

    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    switch (control_mode) {
    case MANUAL:
    case TRAINING:
    case ACRO:
        base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;
    case STABILIZE:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
    case FLY_BY_WIRE_B:
    case QSTABILIZE:
    case QHOVER:
    case QLOITER:
    case CRUISE:
        base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
        break;
    case AUTO:
    case RTL:
    case LOITER:
    case GUIDED:
    case CIRCLE:
        base_mode = MAV_MODE_FLAG_GUIDED_ENABLED |
                    MAV_MODE_FLAG_STABILIZE_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    case INITIALISING:
        system_status = MAV_STATE_CALIBRATING;
        break;
    }

    if (!training_manual_pitch || !training_manual_roll) {
        base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;        
    }

    if (control_mode != MANUAL && control_mode != INITIALISING) {
        // stabiliser of some form is enabled
        base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }

    if (g.stick_mixing != STICK_MIXING_DISABLED && control_mode != INITIALISING) {
        // all modes except INITIALISING have some form of manual
        // override if stick mixing is enabled
        base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }

#if HIL_SUPPORT
    if (g.hil_mode == 1) {
        base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
    }
#endif

    // we are armed if we are not initialising
    if (control_mode != INITIALISING && hal.util->get_soft_armed()) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    mavlink_msg_heartbeat_send(
        chan,
        MAV_TYPE_FIXED_WING,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        custom_mode,
        system_status);
}

void Plane::send_attitude(mavlink_channel_t chan)
{
    const Vector3f &omega = ahrs.get_gyro();
    mavlink_msg_attitude_send(
        chan,
        millis(),
        ahrs.roll,
        ahrs.pitch - radians(g.pitch_trim_cd*0.01f),
        ahrs.yaw,
        omega.x,
        omega.y,
        omega.z);
}

#if GEOFENCE_ENABLED == ENABLED
void Plane::send_fence_status(mavlink_channel_t chan)
{
    geofence_send_status(chan);
}
#endif


void Plane::send_extended_status1(mavlink_channel_t chan)
{
    uint32_t control_sensors_present;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;

    // default sensors present
    control_sensors_present = MAVLINK_SENSOR_PRESENT_DEFAULT;

    // first what sensors/controllers we have
    if (g.compass_enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG; // compass present
    }

    if (airspeed.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    }
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#if OPTFLOW == ENABLED
    if (optflow.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
    if (geofence_present()) {
        control_sensors_present |= MAV_SYS_STATUS_GEOFENCE;
    }

    if (aparm.throttle_min < 0) {
        control_sensors_present |= MAV_SYS_STATUS_REVERSE_MOTOR;
    }

    // all present sensors enabled by default except rate control, attitude stabilization, yaw, altitude, position control, geofence and motor output which we will set individually
    control_sensors_enabled = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL & ~MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION & ~MAV_SYS_STATUS_SENSOR_YAW_POSITION & ~MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL & ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL & ~MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS & ~MAV_SYS_STATUS_GEOFENCE);

    if (airspeed.enabled() && airspeed.use()) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    }

    if (geofence_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_GEOFENCE;
    }

    switch (control_mode) {
    case MANUAL:
        break;

    case ACRO:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        break;

    case STABILIZE:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
    case QSTABILIZE:
    case QHOVER:
    case QLOITER:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation
        break;

    case FLY_BY_WIRE_B:
    case CRUISE:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation
        break;

    case TRAINING:
        if (!training_manual_roll || !training_manual_pitch) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation        
        }
        break;

    case AUTO:
    case RTL:
    case LOITER:
    case GUIDED:
    case CIRCLE:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_YAW_POSITION; // yaw position
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL; // altitude control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL; // X/Y position control
        break;

    case INITIALISING:
        break;
    }

    // set motors outputs as enabled if safety switch is not disarmed (i.e. either NONE or ARMED)
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }

    // default: all present sensors healthy except baro, 3D_MAG, GPS, DIFFERNTIAL_PRESSURE.   GEOFENCE always defaults to healthy.
    control_sensors_health = control_sensors_present & ~(MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                         MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                         MAV_SYS_STATUS_SENSOR_GPS |
                                                         MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
    control_sensors_health |= MAV_SYS_STATUS_GEOFENCE;

    if (ahrs.initialised() && !ahrs.healthy()) {
        // AHRS subsystem is unhealthy
        control_sensors_health &= ~MAV_SYS_STATUS_AHRS;
    }
    if (ahrs.have_inertial_nav() && !ins.accel_calibrated_ok_all()) {
        // trying to use EKF without properly calibrated accelerometers
        control_sensors_health &= ~MAV_SYS_STATUS_AHRS;
    }

    if (barometer.all_healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    }
    if (g.compass_enabled && compass.healthy(0) && ahrs.use_compass()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#if OPTFLOW == ENABLED
    if (optflow.healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
    if (!ins.get_gyro_health_all() || !ins.gyro_calibrated_ok_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_GYRO;
    }
    if (!ins.get_accel_health_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    }
    if (airspeed.healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    }
#if GEOFENCE_ENABLED
    if (geofence_breached()) {
        control_sensors_health &= ~MAV_SYS_STATUS_GEOFENCE;
    }
#endif

    if (millis() - failsafe.last_valid_rc_ms < 200) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    } else {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }
    
    int16_t battery_current = -1;
    int8_t battery_remaining = -1;

    if (battery.has_current() && battery.healthy()) {
        battery_remaining = battery.capacity_remaining_pct();
        battery_current = battery.current_amps() * 100;
    }

#if AP_TERRAIN_AVAILABLE
    switch (terrain.status()) {
    case AP_Terrain::TerrainStatusDisabled:
        break;
    case AP_Terrain::TerrainStatusUnhealthy:
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        break;
    case AP_Terrain::TerrainStatusOK:
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_health  |= MAV_SYS_STATUS_TERRAIN;
        break;
    }
#endif

#if RANGEFINDER_ENABLED == ENABLED
    if (rangefinder.num_sensors() > 0) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (g.rangefinder_landing) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
        if (rangefinder.has_data()) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;            
        }
    }
#endif

    if (aparm.throttle_min < 0 && channel_throttle->servo_out < 0) {
        control_sensors_enabled |= MAV_SYS_STATUS_REVERSE_MOTOR;
        control_sensors_health |= MAV_SYS_STATUS_REVERSE_MOTOR;
    }

    if (AP_Notify::flags.initialising) {
        // while initialising the gyros and accels are not enabled
        control_sensors_enabled &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
        control_sensors_health &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
    }

    mavlink_msg_sys_status_send(
        chan,
        control_sensors_present,
        control_sensors_enabled,
        control_sensors_health,
        (uint16_t)(scheduler.load_average(20000) * 1000),
        battery.voltage() * 1000, // mV
        battery_current,        // in 10mA units
        battery_remaining,      // in %
        0, // comm drops %,
        0, // comm drops in pkts,
        0, 0, 0, 0);
}

void Plane::send_location(mavlink_channel_t chan)
{
    uint32_t fix_time_ms;
    // if we have a GPS fix, take the time as the last fix time. That
    // allows us to correctly calculate velocities and extrapolate
    // positions.
    // If we don't have a GPS fix then we are dead reckoning, and will
    // use the current boot time as the fix time.    
    if (gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
        fix_time_ms = gps.last_fix_time_ms();
    } else {
        fix_time_ms = millis();
    }
    const Vector3f &vel = gps.velocity();
    mavlink_msg_global_position_int_send(
        chan,
        fix_time_ms,
        current_loc.lat,                // in 1E7 degrees
        current_loc.lng,                // in 1E7 degrees
        current_loc.alt * 10UL,         // millimeters above sea level
        relative_altitude() * 1.0e3f,    // millimeters above ground
        vel.x * 100,  // X speed cm/s (+ve North)
        vel.y * 100,  // Y speed cm/s (+ve East)
        vel.z * -100, // Z speed cm/s (+ve up)
        ahrs.yaw_sensor);
}

void Plane::send_nav_controller_output(mavlink_channel_t chan)
{
    mavlink_msg_nav_controller_output_send(
        chan,
        nav_roll_cd * 0.01f,
        nav_pitch_cd * 0.01f,
        nav_controller->nav_bearing_cd() * 0.01f,
        nav_controller->target_bearing_cd() * 0.01f,
        auto_state.wp_distance,
        altitude_error_cm * 0.01f,
        airspeed_error_cm,
        nav_controller->crosstrack_error());
}


void Plane::send_servo_out(mavlink_channel_t chan)
{
    // normalized values scaled to -10000 to 10000
    // This is used for HIL.  Do not change without discussing with
    // HIL maintainers
    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0, // port 0
        10000 * channel_roll->norm_output() * (channel_roll->get_reverse()?-1:1),
        10000 * channel_pitch->norm_output() * (channel_pitch->get_reverse()?-1:1),
        10000 * channel_throttle->norm_output() * (channel_throttle->get_reverse()?-1:1),
        10000 * channel_rudder->norm_output() * (channel_rudder->get_reverse()?-1:1),
        0,
        0,
        0,
        0,
        receiver_rssi);
}

void Plane::send_radio_out(mavlink_channel_t chan)
{
#if HIL_SUPPORT
    if (g.hil_mode==1 && !g.hil_servos) {
        mavlink_msg_servo_output_raw_send(
            chan,
            micros(),
            0,     // port
            RC_Channel::rc_channel(0)->radio_out,
            RC_Channel::rc_channel(1)->radio_out,
            RC_Channel::rc_channel(2)->radio_out,
            RC_Channel::rc_channel(3)->radio_out,
            RC_Channel::rc_channel(4)->radio_out,
            RC_Channel::rc_channel(5)->radio_out,
            RC_Channel::rc_channel(6)->radio_out,
            RC_Channel::rc_channel(7)->radio_out);
        return;
    }
#endif
    mavlink_msg_servo_output_raw_send(
        chan,
        micros(),
        0,     // port
        hal.rcout->read(0),
        hal.rcout->read(1),
        hal.rcout->read(2),
        hal.rcout->read(3),
        hal.rcout->read(4),
        hal.rcout->read(5),
        hal.rcout->read(6),
        hal.rcout->read(7));
}

void Plane::send_vfr_hud(mavlink_channel_t chan)
{
    float aspeed;
    if (airspeed.enabled()) {
        aspeed = airspeed.get_airspeed();
    } else if (!ahrs.airspeed_estimate(&aspeed)) {
        aspeed = 0;
    }
    mavlink_msg_vfr_hud_send(
        chan,
        aspeed,
        gps.ground_speed(),
        (ahrs.yaw_sensor / 100) % 360,
        abs(throttle_percentage()),
        current_loc.alt / 100.0f,
        barometer.get_climb_rate());
}

/*
  keep last HIL_STATE message to allow sending SIM_STATE
 */
#if HIL_SUPPORT
static mavlink_hil_state_t last_hil_state;
#endif

// report simulator state
void Plane::send_simstate(mavlink_channel_t chan)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    sitl.simstate_send(chan);
#elif HIL_SUPPORT
    if (g.hil_mode == 1) {
        mavlink_msg_simstate_send(chan,
                                  last_hil_state.roll,
                                  last_hil_state.pitch,
                                  last_hil_state.yaw,
                                  last_hil_state.xacc*0.001f*GRAVITY_MSS,
                                  last_hil_state.yacc*0.001f*GRAVITY_MSS,
                                  last_hil_state.zacc*0.001f*GRAVITY_MSS,
                                  last_hil_state.rollspeed,
                                  last_hil_state.pitchspeed,
                                  last_hil_state.yawspeed,
                                  last_hil_state.lat,
                                  last_hil_state.lon);
    }
#endif
}

void Plane::send_hwstatus(mavlink_channel_t chan)
{
    mavlink_msg_hwstatus_send(
        chan,
        hal.analogin->board_voltage()*1000,
        hal.i2c->lockup_count());
}

void Plane::send_wind(mavlink_channel_t chan)
{
    Vector3f wind = ahrs.wind_estimate();
    mavlink_msg_wind_send(
        chan,
        degrees(atan2f(-wind.y, -wind.x)), // use negative, to give
                                          // direction wind is coming from
        wind.length(),
        wind.z);
}

/*
  send RPM packet
 */
void NOINLINE Plane::send_rpm(mavlink_channel_t chan)
{
    if (rpm_sensor.healthy(0) || rpm_sensor.healthy(1)) {
        mavlink_msg_rpm_send(
            chan,
            rpm_sensor.get_rpm(0),
            rpm_sensor.get_rpm(1));
    }
}

/*
  send PID tuning message
 */
void Plane::send_pid_tuning(mavlink_channel_t chan)
{
    const Vector3f &gyro = ahrs.get_gyro();
    if (g.gcs_pid_mask & 1) {
        const DataFlash_Class::PID_Info &pid_info = rollController.get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_ROLL, 
                                    pid_info.desired,
                                    degrees(gyro.x),
                                    pid_info.FF,
                                    pid_info.P,
                                    pid_info.I,
                                    pid_info.D);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 2) {
        const DataFlash_Class::PID_Info &pid_info = pitchController.get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_PITCH, 
                                    pid_info.desired,
                                    degrees(gyro.y),
                                    pid_info.FF,
                                    pid_info.P,
                                    pid_info.I,
                                    pid_info.D);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 4) {
        const DataFlash_Class::PID_Info &pid_info = yawController.get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_YAW,
                                    pid_info.desired,
                                    degrees(gyro.z),
                                    pid_info.FF,
                                    pid_info.P,
                                    pid_info.I,
                                    pid_info.D);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 8) {
        const DataFlash_Class::PID_Info &pid_info = steerController.get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_STEER, 
                                    pid_info.desired,
                                    degrees(gyro.z),
                                    pid_info.FF,
                                    pid_info.P,
                                    pid_info.I,
                                    pid_info.D);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
}

void Plane::send_rangefinder(mavlink_channel_t chan)
{
#if RANGEFINDER_ENABLED == ENABLED
    if (!rangefinder.has_data()) {
        // no sonar to report
        return;
    }
    mavlink_msg_rangefinder_send(
        chan,
        rangefinder.distance_cm() * 0.01f,
        rangefinder.voltage_mv()*0.001f);
#endif
}

void Plane::send_current_waypoint(mavlink_channel_t chan)
{
    mavlink_msg_mission_current_send(chan, mission.get_current_nav_index());
}

// are we still delaying telemetry to try to avoid Xbee bricking?
bool Plane::telemetry_delayed(mavlink_channel_t chan)
{
    uint32_t tnow = millis() >> 10;
    if (tnow > (uint32_t)g.telem_delay) {
        return false;
    }
    if (chan == MAVLINK_COMM_0 && hal.gpio->usb_connected()) {
        // this is USB telemetry, so won't be an Xbee
        return false;
    }
    // we're either on the 2nd UART, or no USB cable is connected
    // we need to delay telemetry by the TELEM_DELAY time
    return true;
}

/*
 *  a delay() callback that processes MAVLink packets and processes
 *  Notify events (blinks LEDs).  We set this as the callback in long
 *  running library initialisation routines to allow MAVLink to
 *  process packets while waiting for the initialisation to complete.
 *  And to entertain the user with blinking lights.
 */
void Plane::mavlink_delay_cb()
{
    static uint32_t last_50hz;

    if (in_mavlink_delay) {
        return;
    }

    in_mavlink_delay = true;

    // process MAVLink packets:
    gcs_frontend.delay_cb();

    // update Notify events:
    uint32_t tnow = AP_HAL::millis();
    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
        notify.update();
    }

    // see if USB has been connected or disconnected:
    check_usb_mux();

    in_mavlink_delay = false;
}

/*
 *  send data streams in the given rate range on both links
 */
void Plane::gcs_data_stream_send(void)
{
    gcs_frontend.data_stream_send();
}

/*
 *  look for incoming commands on the GCS links
 */
void Plane::gcs_update(void)
{
    gcs_frontend.update();
}



/**
   retry any deferred messages
 */
void Plane::gcs_retry_deferred(void)
{
    gcs_frontend.send_message(MSG_RETRY_DEFERRED);
}
