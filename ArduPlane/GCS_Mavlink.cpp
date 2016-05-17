// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"
#include "version.h"

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
    case QLAND:
    case CRUISE:
        base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
        break;
    case AUTO:
    case RTL:
    case LOITER:
    case GUIDED:
    case CIRCLE:
    case QRTL:
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
    if (control_mode != INITIALISING && arming.is_armed()) {
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
    case QLAND:
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
    case QRTL:
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

    if (aparm.throttle_min < 0 && channel_throttle->get_servo_out() < 0) {
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
        airspeed_error * 100,
        nav_controller->crosstrack_error());
}

void Plane::send_position_target_global_int(mavlink_channel_t chan)
{
    mavlink_msg_position_target_global_int_send(
        chan,
        AP_HAL::millis(), // time_boot_ms
        MAV_FRAME_GLOBAL_INT, // targets are always global altitude
        0xFFF8, // ignore everything except the x/y/z components
        next_WP_loc.lat, // latitude as 1e7
        next_WP_loc.lng, // longitude as 1e7
        next_WP_loc.alt * 0.01f, // altitude is sent as a float
        0.0f, // vx
        0.0f, // vy
        0.0f, // vz
        0.0f, // afx
        0.0f, // afy
        0.0f, // afz
        0.0f, // yaw
        0.0f); // yaw_rate
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
            RC_Channel::rc_channel(0)->get_radio_out(),
            RC_Channel::rc_channel(1)->get_radio_out(),
            RC_Channel::rc_channel(2)->get_radio_out(),
            RC_Channel::rc_channel(3)->get_radio_out(),
            RC_Channel::rc_channel(4)->get_radio_out(),
            RC_Channel::rc_channel(5)->get_radio_out(),
            RC_Channel::rc_channel(6)->get_radio_out(),
            RC_Channel::rc_channel(7)->get_radio_out());
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
    const DataFlash_Class::PID_Info *pid_info;
    if (g.gcs_pid_mask & 1) {
        if (quadplane.in_vtol_mode()) {
            pid_info = &quadplane.attitude_control->get_rate_roll_pid().get_pid_info();
        } else {
            pid_info = &rollController.get_pid_info();
        }
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_ROLL, 
                                    pid_info->desired,
                                    degrees(gyro.x),
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 2) {
        if (quadplane.in_vtol_mode()) {
            pid_info = &quadplane.attitude_control->get_rate_pitch_pid().get_pid_info();
        } else {
            pid_info = &pitchController.get_pid_info();
        }
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_PITCH, 
                                    pid_info->desired,
                                    degrees(gyro.y),
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 4) {
        if (quadplane.in_vtol_mode()) {
            pid_info = &quadplane.attitude_control->get_rate_yaw_pid().get_pid_info();
        } else {
            pid_info = &yawController.get_pid_info();
        }
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_YAW,
                                    pid_info->desired,
                                    degrees(gyro.z),
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 8) {
        pid_info = &steerController.get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_STEER, 
                                    pid_info->desired,
                                    degrees(gyro.z),
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D);
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


// try to send a message, return false if it won't fit in the serial tx buffer
bool GCS_MAVLINK::try_send_message(enum ap_message id)
{
    if (plane.telemetry_delayed(chan)) {
        return false;
    }

    // if we don't have at least 0.2ms remaining before the main loop
    // wants to fire then don't send a mavlink message. We want to
    // prioritise the main flight control loop over communications
    if (!plane.in_mavlink_delay && plane.scheduler.time_available_usec() < 200) {
        plane.gcs_out_of_time = true;
        return false;
    }

    switch (id) {
    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        last_heartbeat_time = AP_HAL::millis();
        plane.send_heartbeat(chan);
        return true;

    case MSG_EXTENDED_STATUS1:
        CHECK_PAYLOAD_SIZE(SYS_STATUS);
        plane.send_extended_status1(chan);
        CHECK_PAYLOAD_SIZE2(POWER_STATUS);
        send_power_status();
        break;

    case MSG_EXTENDED_STATUS2:
        CHECK_PAYLOAD_SIZE(MEMINFO);
        send_meminfo();
        break;

    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        plane.send_attitude(chan);
        break;

    case MSG_LOCATION:
        CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
        plane.send_location(chan);
        break;

    case MSG_LOCAL_POSITION:
        CHECK_PAYLOAD_SIZE(LOCAL_POSITION_NED);
        send_local_position(plane.ahrs);
        break;

    case MSG_NAV_CONTROLLER_OUTPUT:
        if (plane.control_mode != MANUAL) {
            CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
            plane.send_nav_controller_output(chan);
        }
        break;

    case MSG_POSITION_TARGET_GLOBAL_INT:
        if (plane.control_mode != MANUAL) {
            CHECK_PAYLOAD_SIZE(POSITION_TARGET_GLOBAL_INT);
            plane.send_position_target_global_int(chan);
        }
        break;

    case MSG_GPS_RAW:
        CHECK_PAYLOAD_SIZE(GPS_RAW_INT);
        send_gps_raw(plane.gps);
        break;

    case MSG_SYSTEM_TIME:
        CHECK_PAYLOAD_SIZE(SYSTEM_TIME);
        send_system_time(plane.gps);
        break;

    case MSG_SERVO_OUT:
#if HIL_SUPPORT
        if (plane.g.hil_mode == 1) {
            CHECK_PAYLOAD_SIZE(RC_CHANNELS_SCALED);
            plane.send_servo_out(chan);
        }
#endif
        break;

    case MSG_RADIO_IN:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_RAW);
        send_radio_in(plane.receiver_rssi);
        break;

    case MSG_RADIO_OUT:
        CHECK_PAYLOAD_SIZE(SERVO_OUTPUT_RAW);
        plane.send_radio_out(chan);
        break;

    case MSG_VFR_HUD:
        CHECK_PAYLOAD_SIZE(VFR_HUD);
        plane.send_vfr_hud(chan);
        break;

    case MSG_RAW_IMU1:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
        send_raw_imu(plane.ins, plane.compass);
        break;

    case MSG_RAW_IMU2:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE);
        send_scaled_pressure(plane.barometer);
        break;

    case MSG_RAW_IMU3:
        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
        send_sensor_offsets(plane.ins, plane.compass, plane.barometer);
        break;

    case MSG_CURRENT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_CURRENT);
        plane.send_current_waypoint(chan);
        break;

    case MSG_NEXT_PARAM:
        CHECK_PAYLOAD_SIZE(PARAM_VALUE);
        queued_param_send();
        break;

    case MSG_NEXT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_REQUEST);
        queued_waypoint_send();
        break;

    case MSG_STATUSTEXT:
        // depreciated, use GCS_MAVLINK::send_statustext*
        return false;

#if GEOFENCE_ENABLED == ENABLED
    case MSG_FENCE_STATUS:
        CHECK_PAYLOAD_SIZE(FENCE_STATUS);
        plane.send_fence_status(chan);
        break;
#endif

    case MSG_AHRS:
        CHECK_PAYLOAD_SIZE(AHRS);
        send_ahrs(plane.ahrs);
        break;

    case MSG_SIMSTATE:
        CHECK_PAYLOAD_SIZE(SIMSTATE);
        plane.send_simstate(chan);
        CHECK_PAYLOAD_SIZE2(AHRS2);
        send_ahrs2(plane.ahrs);
        break;

    case MSG_HWSTATUS:
        CHECK_PAYLOAD_SIZE(HWSTATUS);
        plane.send_hwstatus(chan);
        break;

    case MSG_RANGEFINDER:
        CHECK_PAYLOAD_SIZE(RANGEFINDER);
        plane.send_rangefinder(chan);
        break;

    case MSG_TERRAIN:
#if AP_TERRAIN_AVAILABLE
        CHECK_PAYLOAD_SIZE(TERRAIN_REQUEST);
        plane.terrain.send_request(chan);
#endif
        break;

    case MSG_CAMERA_FEEDBACK:
#if CAMERA == ENABLED
        CHECK_PAYLOAD_SIZE(CAMERA_FEEDBACK);
        plane.camera.send_feedback(chan, plane.gps, plane.ahrs, plane.current_loc);
#endif
        break;

    case MSG_BATTERY2:
        CHECK_PAYLOAD_SIZE(BATTERY2);
        send_battery2(plane.battery);
        break;

    case MSG_WIND:
        CHECK_PAYLOAD_SIZE(WIND);
        plane.send_wind(chan);
        break;

    case MSG_MOUNT_STATUS:
#if MOUNT == ENABLED
        CHECK_PAYLOAD_SIZE(MOUNT_STATUS);
        plane.camera_mount.status_msg(chan);
#endif // MOUNT == ENABLED
        break;

    case MSG_OPTICAL_FLOW:
#if OPTFLOW == ENABLED
        CHECK_PAYLOAD_SIZE(OPTICAL_FLOW);
        send_opticalflow(plane.ahrs, plane.optflow);
#endif
        break;

    case MSG_EKF_STATUS_REPORT:
#if AP_AHRS_NAVEKF_AVAILABLE
        CHECK_PAYLOAD_SIZE(EKF_STATUS_REPORT);
        plane.ahrs.send_ekf_status_report(chan);
#endif
        break;

    case MSG_GIMBAL_REPORT:
#if MOUNT == ENABLED
        CHECK_PAYLOAD_SIZE(GIMBAL_REPORT);
        plane.camera_mount.send_gimbal_report(chan);
#endif
        break;

    case MSG_RETRY_DEFERRED:
        break; // just here to prevent a warning

    case MSG_LIMITS_STATUS:
        // unused
        break;

    case MSG_PID_TUNING:
        CHECK_PAYLOAD_SIZE(PID_TUNING);
        plane.send_pid_tuning(chan);
        break;

    case MSG_VIBRATION:
        CHECK_PAYLOAD_SIZE(VIBRATION);
        send_vibration(plane.ins);
        break;

    case MSG_RPM:
        CHECK_PAYLOAD_SIZE(RPM);
        plane.send_rpm(chan);
        break;

    case MSG_MISSION_ITEM_REACHED:
        CHECK_PAYLOAD_SIZE(MISSION_ITEM_REACHED);
        mavlink_msg_mission_item_reached_send(chan, mission_item_reached_index);
        break;

    case MSG_MAG_CAL_PROGRESS:
        CHECK_PAYLOAD_SIZE(MAG_CAL_PROGRESS);
        plane.compass.send_mag_cal_progress(chan);
        break;

    case MSG_MAG_CAL_REPORT:
        CHECK_PAYLOAD_SIZE(MAG_CAL_REPORT);
        plane.compass.send_mag_cal_report(chan);
        break;
    }
    return true;
}


/*
  default stream rates to 1Hz
 */
const AP_Param::GroupInfo GCS_MAVLINK::var_info[] = {
    // @Param: RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: Raw sensor stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK, streamRates[0],  1),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Extended status stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK, streamRates[1],  1),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: RC Channel stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK, streamRates[2],  1),

    // @Param: RAW_CTRL
    // @DisplayName: Raw Control stream rate to ground station
    // @Description: Raw Control stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK, streamRates[3],  1),

    // @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Position stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK, streamRates[4],  1),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Extra data type 1 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK, streamRates[5],  1),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate to ground station
    // @Description: Extra data type 2 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK, streamRates[6],  1),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Extra data type 3 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK, streamRates[7],  1),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate to ground station
    // @Description: Parameter stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK, streamRates[8],  10),
    AP_GROUPEND
};

float GCS_MAVLINK::adjust_rate_for_stream_trigger(enum streams stream_num)
{
    // send at a much lower rate while handling waypoints and
    // parameter sends
    if ((stream_num != STREAM_PARAMS) && 
        (waypoint_receiving || _queued_parameter != NULL)) {
        return 0.25f;
    }

    return 1.0f;
}

void
GCS_MAVLINK::data_stream_send(void)
{
    plane.gcs_out_of_time = false;

    if (!plane.in_mavlink_delay) {
        handle_log_send(plane.DataFlash);
    }

    if (_queued_parameter != NULL) {
        if (streamRates[STREAM_PARAMS].get() <= 0) {
            streamRates[STREAM_PARAMS].set(10);
        }
        if (stream_trigger(STREAM_PARAMS)) {
            send_message(MSG_NEXT_PARAM);
        }
    }

    if (plane.gcs_out_of_time) return;

    if (plane.in_mavlink_delay) {
#if HIL_SUPPORT
        if (plane.g.hil_mode == 1) {
            // in HIL we need to keep sending servo values to ensure
            // the simulator doesn't pause, otherwise our sensor
            // calibration could stall
            if (stream_trigger(STREAM_RAW_CONTROLLER)) {
                send_message(MSG_SERVO_OUT);
            }
            if (stream_trigger(STREAM_RC_CHANNELS)) {
                send_message(MSG_RADIO_OUT);
            }
        }
#endif
        // don't send any other stream types while in the delay callback
        return;
    }

    if (plane.gcs_out_of_time) return;

    if (stream_trigger(STREAM_RAW_SENSORS)) {
        send_message(MSG_RAW_IMU1);
        send_message(MSG_RAW_IMU2);
        send_message(MSG_RAW_IMU3);
    }

    if (plane.gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTENDED_STATUS)) {
        send_message(MSG_EXTENDED_STATUS1);
        send_message(MSG_EXTENDED_STATUS2);
        send_message(MSG_CURRENT_WAYPOINT);
        send_message(MSG_GPS_RAW);
        send_message(MSG_NAV_CONTROLLER_OUTPUT);
        send_message(MSG_FENCE_STATUS);
        send_message(MSG_POSITION_TARGET_GLOBAL_INT);
    }

    if (plane.gcs_out_of_time) return;

    if (stream_trigger(STREAM_POSITION)) {
        // sent with GPS read
        send_message(MSG_LOCATION);
        send_message(MSG_LOCAL_POSITION);
    }

    if (plane.gcs_out_of_time) return;

    if (stream_trigger(STREAM_RAW_CONTROLLER)) {
        send_message(MSG_SERVO_OUT);
    }

    if (plane.gcs_out_of_time) return;

    if (stream_trigger(STREAM_RC_CHANNELS)) {
        send_message(MSG_RADIO_OUT);
        send_message(MSG_RADIO_IN);
    }

    if (plane.gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA1)) {
        send_message(MSG_ATTITUDE);
        send_message(MSG_SIMSTATE);
        send_message(MSG_RPM);
        if (plane.control_mode != MANUAL) {
            send_message(MSG_PID_TUNING);
        }
    }

    if (plane.gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA2)) {
        send_message(MSG_VFR_HUD);
    }

    if (plane.gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA3)) {
        send_message(MSG_AHRS);
        send_message(MSG_HWSTATUS);
        send_message(MSG_WIND);
        send_message(MSG_RANGEFINDER);
        send_message(MSG_SYSTEM_TIME);
#if AP_TERRAIN_AVAILABLE
        send_message(MSG_TERRAIN);
#endif
        send_message(MSG_MAG_CAL_REPORT);
        send_message(MSG_MAG_CAL_PROGRESS);
        send_message(MSG_BATTERY2);
        send_message(MSG_MOUNT_STATUS);
        send_message(MSG_OPTICAL_FLOW);
        send_message(MSG_EKF_STATUS_REPORT);
        send_message(MSG_GIMBAL_REPORT);
        send_message(MSG_VIBRATION);
    }
}


/*
  handle a request to switch to guided mode. This happens via a
  callback from handle_mission_item()
 */
bool GCS_MAVLINK::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
    if (plane.control_mode != GUIDED) {
        // only accept position updates when in GUIDED mode
        return false;
    }
    plane.guided_WP_loc = cmd.content.location;
    
    // add home alt if needed
    if (plane.guided_WP_loc.flags.relative_alt) {
        plane.guided_WP_loc.alt += plane.home.alt;
        plane.guided_WP_loc.flags.relative_alt = 0;
    }

    plane.set_guided_WP();
    return true;
}

/*
  handle a request to change current WP altitude. This happens via a
  callback from handle_mission_item()
 */
void GCS_MAVLINK::handle_change_alt_request(AP_Mission::Mission_Command &cmd)
{
    plane.next_WP_loc.alt = cmd.content.location.alt;
    if (cmd.content.location.flags.relative_alt) {
        plane.next_WP_loc.alt += plane.home.alt;
    }
    plane.next_WP_loc.flags.relative_alt = false;
    plane.next_WP_loc.flags.terrain_alt = cmd.content.location.flags.terrain_alt;
    plane.reset_offset_altitude();
}

void GCS_MAVLINK::handleMessage(mavlink_message_t* msg)
{
    switch (msg->msgid) {

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
    {
        handle_request_data_stream(msg, true);
        break;
    }

    case MAVLINK_MSG_ID_COMMAND_INT:
    {
        // decode
        mavlink_command_int_t packet;
        mavlink_msg_command_int_decode(msg, &packet);

        uint8_t result = MAV_RESULT_UNSUPPORTED;

        switch(packet.command) {

        case MAV_CMD_DO_REPOSITION:
            Location requested_position {};
            requested_position.lat = packet.x;
            requested_position.lng = packet.y;

            // check the floating representation for overflow of altitude
            if (abs(packet.z * 100.0f) >= 0x7fffff) {
                result = MAV_RESULT_FAILED;
                break;
            }
            requested_position.alt = (int32_t)(packet.z * 100.0f);

            // load option flags
            if (packet.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) {
                requested_position.flags.relative_alt = 1;
            }
            else if (packet.frame == MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) {
                requested_position.flags.terrain_alt = 1;
            }
            else if (packet.frame != MAV_FRAME_GLOBAL_INT) {
                // not a supported frame
                break;
            }

            if (is_zero(packet.param4)) {
                requested_position.flags.loiter_ccw = 0;
            } else {
                requested_position.flags.loiter_ccw = 1;
            }

            if (location_sanitize(plane.current_loc, requested_position)) {
                // if the location wasn't already sane don't load it
                result = MAV_RESULT_FAILED; // failed as the location is not valid
                break;
            }

            // location is valid load and set
            if (((int32_t)packet.param2 & MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) ||
                (plane.control_mode == GUIDED)) {
                plane.set_mode(GUIDED);
                plane.guided_WP_loc = requested_position;

                // add home alt if needed
                if (plane.guided_WP_loc.flags.relative_alt) {
                    plane.guided_WP_loc.alt += plane.home.alt;
                    plane.guided_WP_loc.flags.relative_alt = 0;
                }

                plane.set_guided_WP();

                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_FAILED; // failed as we are not in guided
            }
            break;
        }

        mavlink_msg_command_ack_send_buf(
            msg,
            chan,
            packet.command,
            result);

        break;
    }

    case MAVLINK_MSG_ID_COMMAND_LONG:
    {
        // decode
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(msg, &packet);

        uint8_t result = MAV_RESULT_UNSUPPORTED;

        // do command

        switch(packet.command) {

        case MAV_CMD_DO_CHANGE_SPEED:
            result = MAV_RESULT_FAILED;
            AP_Mission::Mission_Command cmd;
            if (AP_Mission::mavlink_cmd_long_to_mission_cmd(packet, cmd)
                    == MAV_MISSION_ACCEPTED) {
                plane.do_change_speed(cmd);
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_START_RX_PAIR:
            // initiate bind procedure
            if (!hal.rcin->rc_bind(packet.param1)) {
                result = MAV_RESULT_FAILED;
            } else {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_NAV_LOITER_UNLIM:
            plane.set_mode(LOITER);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
            plane.set_mode(RTL);
            result = MAV_RESULT_ACCEPTED;
            break;

#if MOUNT == ENABLED
        // Sets the region of interest (ROI) for the camera
        case MAV_CMD_DO_SET_ROI:
            // sanity check location
            if (fabsf(packet.param5) > 90.0f || fabsf(packet.param6) > 180.0f) {
                break;
            }
            Location roi_loc;
            roi_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
            roi_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
            roi_loc.alt = (int32_t)(packet.param7 * 100.0f);
            if (roi_loc.lat == 0 && roi_loc.lng == 0 && roi_loc.alt == 0) {
                // switch off the camera tracking if enabled
                if (plane.camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
                    plane.camera_mount.set_mode_to_default();
                }
            } else {
                // send the command to the camera mount
                plane.camera_mount.set_roi_target(roi_loc);
            }
            result = MAV_RESULT_ACCEPTED;
            break;
#endif

#if CAMERA == ENABLED
        case MAV_CMD_DO_DIGICAM_CONFIGURE:
            plane.camera.configure(packet.param1,
                                   packet.param2,
                                   packet.param3,
                                   packet.param4,
                                   packet.param5,
                                   packet.param6,
                                   packet.param7);

            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_DO_DIGICAM_CONTROL:
            if (plane.camera.control(packet.param1,
                                     packet.param2,
                                     packet.param3,
                                     packet.param4,
                                     packet.param5,
                                     packet.param6)) {
                plane.log_picture();
            }
            result = MAV_RESULT_ACCEPTED;
            break;
#endif // CAMERA == ENABLED

        case MAV_CMD_DO_MOUNT_CONTROL:
#if MOUNT == ENABLED
            plane.camera_mount.control(packet.param1, packet.param2, packet.param3, (MAV_MOUNT_MODE) packet.param7);
            result = MAV_RESULT_ACCEPTED;
#endif
            break;

        case MAV_CMD_MISSION_START:
            plane.set_mode(AUTO);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_PREFLIGHT_CALIBRATION:
            if(hal.util->get_soft_armed()) {
                result = MAV_RESULT_FAILED;
                break;
            }
            plane.in_calibration = true;
            if (is_equal(packet.param1,1.0f)) {
                plane.ins.init_gyro();
                if (plane.ins.gyro_calibrated_ok_all()) {
                    plane.ahrs.reset_gyro_drift();
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            } else if (is_equal(packet.param3,1.0f)) {
                plane.init_barometer();
                if (plane.airspeed.enabled()) {
                    plane.zero_airspeed(false);
                }
                result = MAV_RESULT_ACCEPTED;
            } else if (is_equal(packet.param4,1.0f)) {
                plane.trim_radio();
                result = MAV_RESULT_ACCEPTED;
            } else if (is_equal(packet.param5,1.0f)) {
                result = MAV_RESULT_ACCEPTED;
                // start with gyro calibration
                plane.ins.init_gyro();
                // reset ahrs gyro bias
                if (plane.ins.gyro_calibrated_ok_all()) {
                    plane.ahrs.reset_gyro_drift();
                } else {
                    result = MAV_RESULT_FAILED;
                }
                plane.ins.acal_init();
                plane.ins.get_acal()->start(this);

            } else if (is_equal(packet.param5,2.0f)) {
                // start with gyro calibration
                plane.ins.init_gyro();
                // accel trim
                float trim_roll, trim_pitch;
                if(plane.ins.calibrate_trim(trim_roll, trim_pitch)) {
                    // reset ahrs's trim to suggested values from calibration routine
                    plane.ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            }
            else {
                    send_text(MAV_SEVERITY_WARNING, "Unsupported preflight calibration");
            }
            plane.in_calibration = false;
            break;

        case MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
            if (is_equal(packet.param1,2.0f)) {
                // save first compass's offsets
                plane.compass.set_and_save_offsets(0, packet.param2, packet.param3, packet.param4);
                result = MAV_RESULT_ACCEPTED;
            }
            if (is_equal(packet.param1,5.0f)) {
                // save secondary compass's offsets
                plane.compass.set_and_save_offsets(1, packet.param2, packet.param3, packet.param4);
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_COMPONENT_ARM_DISARM:
            if (is_equal(packet.param1,1.0f)) {
                // run pre_arm_checks and arm_checks and display failures
                if (plane.arm_motors(AP_Arming::MAVLINK)) {
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            } else if (is_zero(packet.param1))  {
                if (plane.disarm_motors()) {
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            } else {
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_GET_HOME_POSITION:
            if (plane.home_is_set != HOME_UNSET) {
                send_home(plane.ahrs.get_home());
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_SET_MODE:
            switch ((uint16_t)packet.param1) {
            case MAV_MODE_MANUAL_ARMED:
            case MAV_MODE_MANUAL_DISARMED:
                plane.set_mode(MANUAL);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_MODE_AUTO_ARMED:
            case MAV_MODE_AUTO_DISARMED:
                plane.set_mode(AUTO);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_MODE_STABILIZE_DISARMED:
            case MAV_MODE_STABILIZE_ARMED:
                plane.set_mode(FLY_BY_WIRE_A);
                result = MAV_RESULT_ACCEPTED;
                break;

            default:
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_DO_SET_SERVO:
            if (plane.ServoRelayEvents.do_set_servo(packet.param1, packet.param2)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_REPEAT_SERVO:
            if (plane.ServoRelayEvents.do_repeat_servo(packet.param1, packet.param2, packet.param3, packet.param4*1000)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_SET_RELAY:
            if (plane.ServoRelayEvents.do_set_relay(packet.param1, packet.param2)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_REPEAT_RELAY:
            if (plane.ServoRelayEvents.do_repeat_relay(packet.param1, packet.param2, packet.param3*1000)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if (is_equal(packet.param1,1.0f) || is_equal(packet.param1,3.0f)) {
                // when packet.param1 == 3 we reboot to hold in bootloader
                hal.scheduler->reboot(is_equal(packet.param1,3.0f));
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_LAND_START:
            result = MAV_RESULT_FAILED;
            
            // attempt to switch to next DO_LAND_START command in the mission
            if (plane.jump_to_landing_sequence()) {
                result = MAV_RESULT_ACCEPTED;
            } 
            break;

        case MAV_CMD_DO_GO_AROUND:
            result = MAV_RESULT_FAILED;

            //Not allowing go around at FLIGHT_LAND_FINAL stage on purpose --
            //if plane is close to the ground a go around coudld be dangerous.
            if (plane.flight_stage == AP_SpdHgtControl::FLIGHT_LAND_APPROACH ||
                plane.flight_stage == AP_SpdHgtControl::FLIGHT_LAND_PREFLARE ||
                plane.flight_stage == AP_SpdHgtControl::FLIGHT_LAND_FINAL) {
                // Initiate an aborted landing. This will trigger a pitch-up and
                // climb-out to a safe altitude holding heading then one of the
                // following actions will occur, check for in this order:
                // - If MAV_CMD_CONTINUE_AND_CHANGE_ALT is next command in mission,
                //      increment mission index to execute it
                // - else if DO_LAND_START is available, jump to it
                // - else decrement the mission index to repeat the landing approach

                if (!is_zero(packet.param1)) {
                    plane.auto_state.takeoff_altitude_rel_cm = packet.param1 * 100;
                }
                plane.auto_state.commanded_go_around = true;
               
                result = MAV_RESULT_ACCEPTED;
                plane.gcs_send_text(MAV_SEVERITY_INFO,"Go around command accepted");
            } else {
                plane.gcs_send_text(MAV_SEVERITY_NOTICE,"Rejected go around command");
            }
            break;

        case MAV_CMD_DO_FENCE_ENABLE:
            result = MAV_RESULT_ACCEPTED;
            
            if (!plane.geofence_present()) {
                result = MAV_RESULT_FAILED;
            } switch((uint16_t)packet.param1) {
                case 0:
                    if (! plane.geofence_set_enabled(false, GCS_TOGGLED)) {
                        result = MAV_RESULT_FAILED;
                    }
                    break;
                case 1:
                    if (! plane.geofence_set_enabled(true, GCS_TOGGLED)) {
                        result = MAV_RESULT_FAILED; 
                    }
                    break;
                case 2: //disable fence floor only 
                    if (! plane.geofence_set_floor_enabled(false)) {
                        result = MAV_RESULT_FAILED;
                    } else {
                        plane.gcs_send_text(MAV_SEVERITY_NOTICE,"Fence floor disabled");
                    }
                    break;
                default:
                    result = MAV_RESULT_FAILED;
                    break;
            }
            break;

        case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES: {
            if (is_equal(packet.param1,1.0f)) {
                send_autopilot_version(FIRMWARE_VERSION);
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_SET_HOME:
            // param1 : use current (1=use current location, 0=use specified location)
            // param5 : latitude
            // param6 : longitude
            // param7 : altitude (absolute)
            result = MAV_RESULT_FAILED; // assume failure
            if (is_equal(packet.param1,1.0f)) {
                plane.init_home();
            } else {
                if (is_zero(packet.param5) && is_zero(packet.param6) && is_zero(packet.param7)) {
                    // don't allow the 0,0 position
                    break;
                }
                // sanity check location
                if (fabsf(packet.param5) > 90.0f || fabsf(packet.param6) > 180.0f) {
                    break;
                }
                Location new_home_loc {};
                new_home_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
                new_home_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
                new_home_loc.alt = (int32_t)(packet.param7 * 100.0f);
                plane.ahrs.set_home(new_home_loc);
                plane.home_is_set = HOME_SET_NOT_LOCKED;
                plane.Log_Write_Home_And_Origin();
                GCS_MAVLINK::send_home_all(new_home_loc);
                result = MAV_RESULT_ACCEPTED;
                plane.gcs_send_text_fmt(MAV_SEVERITY_INFO, "Set HOME to %.6f %.6f at %um",
                                        (double)(new_home_loc.lat*1.0e-7f),
                                        (double)(new_home_loc.lng*1.0e-7f),
                                        (uint32_t)(new_home_loc.alt*0.01f));
            }
            break;
        }

        case MAV_CMD_DO_AUTOTUNE_ENABLE:
            // param1 : enable/disable
            plane.autotune_enable(!is_zero(packet.param1));
            break;

        case MAV_CMD_DO_START_MAG_CAL:
        case MAV_CMD_DO_ACCEPT_MAG_CAL:
        case MAV_CMD_DO_CANCEL_MAG_CAL:
            result = plane.compass.handle_mag_cal_command(packet);
            break;

#if PARACHUTE == ENABLED
        case MAV_CMD_DO_PARACHUTE:
            // configure or release parachute
            result = MAV_RESULT_ACCEPTED;
            switch ((uint16_t)packet.param1) {
                case PARACHUTE_DISABLE:
                    plane.parachute.enabled(false);
                    break;
                case PARACHUTE_ENABLE:
                    plane.parachute.enabled(true);
                    break;
                case PARACHUTE_RELEASE:
                    // treat as a manual release which performs some additional check of altitude
                    if (plane.parachute.released()) {
                        plane.gcs_send_text_fmt(MAV_SEVERITY_NOTICE, "Parachute already released");
                        result = MAV_RESULT_FAILED;
                    } else if (!plane.parachute.enabled()) {
                        plane.gcs_send_text_fmt(MAV_SEVERITY_NOTICE, "Parachute not enabled");
                        result = MAV_RESULT_FAILED;
                    } else {
                        if (!plane.parachute_manual_release()) {
                            result = MAV_RESULT_FAILED;
                        }
                    }
                    break;
                default:
                    result = MAV_RESULT_FAILED;
                    break;
            }
            break;
#endif

        case MAV_CMD_DO_MOTOR_TEST:
            // param1 : motor sequence number (a number from 1 to max number of motors on the vehicle)
            // param2 : throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
            // param3 : throttle (range depends upon param2)
            // param4 : timeout (in seconds)
            // param5 : motor count (number of motors to test in sequence)
            result = plane.quadplane.mavlink_motor_test_start(chan, (uint8_t)packet.param1, (uint8_t)packet.param2, (uint16_t)packet.param3, packet.param4, (uint8_t)packet.param5);
            break;
            
        case MAV_CMD_DO_VTOL_TRANSITION:
            if (!plane.quadplane.handle_do_vtol_transition((enum MAV_VTOL_STATE)packet.param1)) {
                result = MAV_RESULT_FAILED;
            } else {
                result = MAV_RESULT_ACCEPTED;
            }
            break;
            
        default:
            break;
        }

        mavlink_msg_command_ack_send_buf(
            msg,
            chan,
            packet.command,
            result);

        break;
    }

    case MAVLINK_MSG_ID_SET_MODE:
    {
        handle_set_mode(msg, FUNCTOR_BIND(&plane, &Plane::mavlink_set_mode, bool, uint8_t));
        break;
    }

    // GCS request the full list of commands, we return just the number and leave the GCS to then request each command individually
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
    {
        handle_mission_request_list(plane.mission, msg);
        break;
    }

    // XXX read a WP from EEPROM and send it to the GCS
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
    case MAVLINK_MSG_ID_MISSION_REQUEST:
    {
        handle_mission_request(plane.mission, msg);
        break;
    }


    case MAVLINK_MSG_ID_MISSION_ACK:
    {
        // nothing to do
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
        // mark the firmware version in the tlog
        send_text(MAV_SEVERITY_INFO, FIRMWARE_STRING);

#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
        send_text(MAV_SEVERITY_INFO, "PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION);
#endif
        handle_param_request_list(msg);
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    {
        handle_param_request_read(msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
    {
        handle_mission_clear_all(plane.mission, msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
    {
        // disable cross-track when user asks for WP change, to
        // prevent unexpected flight paths
        plane.auto_state.next_wp_no_crosstrack = true;
        handle_mission_set_current(plane.mission, msg);
        if (plane.control_mode == AUTO && plane.mission.state() == AP_Mission::MISSION_STOPPED) {
            plane.mission.resume();
        }
        break;
    }

    // GCS provides the full number of commands it wishes to upload
    //  individual commands will then be sent from the GCS using the MAVLINK_MSG_ID_MISSION_ITEM message
    case MAVLINK_MSG_ID_MISSION_COUNT:
    {
        handle_mission_count(plane.mission, msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
    {
        handle_mission_write_partial_list(plane.mission, msg);
        break;
    }

    // GCS has sent us a mission item, store to EEPROM
    case MAVLINK_MSG_ID_MISSION_ITEM:
    {
        if (handle_mission_item(msg, plane.mission)) {
            plane.DataFlash.Log_Write_EntireMission(plane.mission);
        }
        break;
    }
    
    // GCS has sent us a mission item, store to EEPROM
    case MAVLINK_MSG_ID_MISSION_ITEM_INT:
    {
        if (handle_mission_item(msg, plane.mission)) {
            plane.DataFlash.Log_Write_EntireMission(plane.mission);
        }
        break;
    }

#if GEOFENCE_ENABLED == ENABLED
    // receive a fence point from GCS and store in EEPROM
    case MAVLINK_MSG_ID_FENCE_POINT: {
        mavlink_fence_point_t packet;
        mavlink_msg_fence_point_decode(msg, &packet);
        if (plane.g.fence_action != FENCE_ACTION_NONE) {
            send_text(MAV_SEVERITY_WARNING,"Fencing must be disabled");
        } else if (packet.count != plane.g.fence_total) {
            send_text(MAV_SEVERITY_WARNING,"Bad fence point");
        } else if (fabsf(packet.lat) > 90.0f || fabsf(packet.lng) > 180.0f) {
            send_text(MAV_SEVERITY_WARNING,"Invalid fence point, lat or lng too large");
        } else {
            Vector2l point;
            point.x = packet.lat*1.0e7f;
            point.y = packet.lng*1.0e7f;
            plane.set_fence_point_with_index(point, packet.idx);
        }
        break;
    }

    // send a fence point to GCS
    case MAVLINK_MSG_ID_FENCE_FETCH_POINT: {
        mavlink_fence_fetch_point_t packet;
        mavlink_msg_fence_fetch_point_decode(msg, &packet);
        if (packet.idx >= plane.g.fence_total) {
            send_text(MAV_SEVERITY_WARNING,"Bad fence point");
        } else {
            Vector2l point = plane.get_fence_point_with_index(packet.idx);
            mavlink_msg_fence_point_send_buf(msg, chan, msg->sysid, msg->compid, packet.idx, plane.g.fence_total,
                                             point.x*1.0e-7f, point.y*1.0e-7f);
        }
        break;
    }
#endif // GEOFENCE_ENABLED

    // receive a rally point from GCS and store in EEPROM
    case MAVLINK_MSG_ID_RALLY_POINT: {
        mavlink_rally_point_t packet;
        mavlink_msg_rally_point_decode(msg, &packet);
        
        if (packet.idx >= plane.rally.get_rally_total() || 
            packet.idx >= plane.rally.get_rally_max()) {
            send_text(MAV_SEVERITY_WARNING,"Bad rally point message ID");
            break;
        }

        if (packet.count != plane.rally.get_rally_total()) {
            send_text(MAV_SEVERITY_WARNING,"Bad rally point message count");
            break;
        }

        RallyLocation rally_point;
        rally_point.lat = packet.lat;
        rally_point.lng = packet.lng;
        rally_point.alt = packet.alt;
        rally_point.break_alt = packet.break_alt;
        rally_point.land_dir = packet.land_dir;
        rally_point.flags = packet.flags;
        plane.rally.set_rally_point_with_index(packet.idx, rally_point);
        break;
    }

    //send a rally point to the GCS
    case MAVLINK_MSG_ID_RALLY_FETCH_POINT: {
        mavlink_rally_fetch_point_t packet;
        mavlink_msg_rally_fetch_point_decode(msg, &packet);
        if (packet.idx > plane.rally.get_rally_total()) {
            send_text(MAV_SEVERITY_WARNING, "Bad rally point index");
            break;
        }
        RallyLocation rally_point;
        if (!plane.rally.get_rally_point_with_index(packet.idx, rally_point)) {
            send_text(MAV_SEVERITY_WARNING, "Failed to set rally point");
            break;
        }

        mavlink_msg_rally_point_send_buf(msg,
                                         chan, msg->sysid, msg->compid, packet.idx, 
                                         plane.rally.get_rally_total(), rally_point.lat, rally_point.lng, 
                                         rally_point.alt, rally_point.break_alt, rally_point.land_dir, 
                                         rally_point.flags);
        break;
    }    

    case MAVLINK_MSG_ID_PARAM_SET:
    {
        handle_param_set(msg, &plane.DataFlash);
        break;
    }

    case MAVLINK_MSG_ID_GIMBAL_REPORT:
    {
#if MOUNT == ENABLED
        handle_gimbal_report(plane.camera_mount, msg);
#endif
        break;
    }

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
    {
        // allow override of RC channel values for HIL
        // or for complete GCS control of switch position
        // and RC PWM values.
        if(msg->sysid != plane.g.sysid_my_gcs) break;                         // Only accept control from our gcs
        mavlink_rc_channels_override_t packet;
        int16_t v[8];
        mavlink_msg_rc_channels_override_decode(msg, &packet);

        v[0] = packet.chan1_raw;
        v[1] = packet.chan2_raw;
        v[2] = packet.chan3_raw;
        v[3] = packet.chan4_raw;
        v[4] = packet.chan5_raw;
        v[5] = packet.chan6_raw;
        v[6] = packet.chan7_raw;
        v[7] = packet.chan8_raw;

        if (hal.rcin->set_overrides(v, 8)) {
            plane.failsafe.last_valid_rc_ms = AP_HAL::millis();
            plane.failsafe.AFS_last_valid_rc_ms =  plane.failsafe.last_valid_rc_ms;
        }

        // a RC override message is consiered to be a 'heartbeat' from
        // the ground station for failsafe purposes
        plane.failsafe.last_heartbeat_ms = AP_HAL::millis();
        break;
    }

    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        // We keep track of the last time we received a heartbeat from
        // our GCS for failsafe purposes
        if (msg->sysid != plane.g.sysid_my_gcs) break;
        plane.failsafe.last_heartbeat_ms = AP_HAL::millis();
        break;
    }

    case MAVLINK_MSG_ID_HIL_STATE:
    {
#if HIL_SUPPORT
        if (plane.g.hil_mode != 1) {
            break;
        }
        mavlink_hil_state_t packet;
        mavlink_msg_hil_state_decode(msg, &packet);

        last_hil_state = packet;

        // set gps hil sensor
        Location loc;
        memset(&loc, 0, sizeof(loc));
        loc.lat = packet.lat;
        loc.lng = packet.lon;
        loc.alt = packet.alt/10;
        Vector3f vel(packet.vx, packet.vy, packet.vz);
        vel *= 0.01f;

        // setup airspeed pressure based on 3D speed, no wind
        plane.airspeed.setHIL(sq(vel.length()) / 2.0f + 2013);

        plane.gps.setHIL(0, AP_GPS::GPS_OK_FIX_3D,
                         packet.time_usec/1000,
                         loc, vel, 10, 0);

        // rad/sec
        Vector3f gyros;
        gyros.x = packet.rollspeed;
        gyros.y = packet.pitchspeed;
        gyros.z = packet.yawspeed;

        // m/s/s
        Vector3f accels;
        accels.x = packet.xacc * GRAVITY_MSS*0.001f;
        accels.y = packet.yacc * GRAVITY_MSS*0.001f;
        accels.z = packet.zacc * GRAVITY_MSS*0.001f;

        plane.ins.set_gyro(0, gyros);
        plane.ins.set_accel(0, accels);

        plane.barometer.setHIL(packet.alt*0.001f);
        plane.compass.setHIL(0, packet.roll, packet.pitch, packet.yaw);
        plane.compass.setHIL(1, packet.roll, packet.pitch, packet.yaw);

        // cope with DCM getting badly off due to HIL lag
        if (plane.g.hil_err_limit > 0 &&
            (fabsf(packet.roll - plane.ahrs.roll) > ToRad(plane.g.hil_err_limit) ||
             fabsf(packet.pitch - plane.ahrs.pitch) > ToRad(plane.g.hil_err_limit) ||
             wrap_PI(fabsf(packet.yaw - plane.ahrs.yaw)) > ToRad(plane.g.hil_err_limit))) {
            plane.ahrs.reset_attitude(packet.roll, packet.pitch, packet.yaw);
        }
#endif
        break;
    }

#if CAMERA == ENABLED
    //deprecated. Use MAV_CMD_DO_DIGICAM_CONFIGURE
    case MAVLINK_MSG_ID_DIGICAM_CONFIGURE:
    {
        break;
    }

    //deprecated. Use MAV_CMD_DO_DIGICAM_CONTROL
    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
    {
        plane.camera.control_msg(msg);
        plane.log_picture();
        break;
    }
#endif // CAMERA == ENABLED

#if MOUNT == ENABLED
    //deprecated. Use MAV_CMD_DO_MOUNT_CONFIGURE
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:
    {
        plane.camera_mount.configure_msg(msg);
        break;
    }

    //deprecated. Use MAV_CMD_DO_MOUNT_CONTROL
    case MAVLINK_MSG_ID_MOUNT_CONTROL:
    {
        plane.camera_mount.control_msg(msg);
        break;
    }
#endif // MOUNT == ENABLED

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS:
    {
        handle_radio_status(msg, plane.DataFlash, plane.should_log(MASK_LOG_PM));
        break;
    }

    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
    case MAVLINK_MSG_ID_LOG_ERASE:
        plane.in_log_download = true;
        /* no break */
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        if (!plane.in_mavlink_delay) {
            handle_log_message(msg, plane.DataFlash);
        }
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        plane.in_log_download = false;
        if (!plane.in_mavlink_delay) {
            handle_log_message(msg, plane.DataFlash);
        }
        break;

    case MAVLINK_MSG_ID_SERIAL_CONTROL:
        handle_serial_control(msg, plane.gps);
        break;

    case MAVLINK_MSG_ID_GPS_INJECT_DATA:
        handle_gps_inject(msg, plane.gps);
        break;

#if RANGEFINDER_ENABLED == ENABLED
    case MAVLINK_MSG_ID_DISTANCE_SENSOR:
        plane.rangefinder.handle_msg(msg);
        break;
#endif

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
#if AP_TERRAIN_AVAILABLE
        plane.terrain.handle_data(chan, msg);
#endif
        break;

    case MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST:
        send_autopilot_version(FIRMWARE_VERSION);
        break;

    case MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS:
        plane.DataFlash.remote_log_block_status_msg(chan, msg);
        break;

    case MAVLINK_MSG_ID_SET_HOME_POSITION:
    {
        mavlink_set_home_position_t packet;
        mavlink_msg_set_home_position_decode(msg, &packet);
        if((packet.latitude == 0) && (packet.longitude == 0) && (packet.altitude == 0)) {
            // don't allow the 0,0 position
            break;
        }
        // sanity check location
        if (labs(packet.latitude) > 90*10e7 || labs(packet.longitude) > 180 * 10e7) {
            break;
        }
        Location new_home_loc {};
        new_home_loc.lat = packet.latitude;
        new_home_loc.lng = packet.longitude;
        new_home_loc.alt = packet.altitude / 10;
        plane.ahrs.set_home(new_home_loc);
        plane.home_is_set = HOME_SET_NOT_LOCKED;
        plane.Log_Write_Home_And_Origin();
        GCS_MAVLINK::send_home_all(new_home_loc);
        plane.gcs_send_text_fmt(MAV_SEVERITY_INFO, "Set HOME to %.6f %.6f at %um",
                                (double)(new_home_loc.lat*1.0e-7f),
                                (double)(new_home_loc.lng*1.0e-7f),
                                (uint32_t)(new_home_loc.alt*0.01f));
        break;
    }

    case MAVLINK_MSG_ID_ADSB_VEHICLE:
        plane.adsb.update_vehicle(msg);
        break;
    } // end switch
} // end handle mavlink

/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
void Plane::mavlink_delay_cb()
{
    static uint32_t last_1hz, last_50hz, last_5s;
    if (!gcs[0].initialised || in_mavlink_delay) return;

    in_mavlink_delay = true;

    uint32_t tnow = millis();
    if (tnow - last_1hz > 1000) {
        last_1hz = tnow;
        gcs_send_message(MSG_HEARTBEAT);
        gcs_send_message(MSG_EXTENDED_STATUS1);
    }
    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
        gcs_update();
        gcs_data_stream_send();
        notify.update();
    }
    if (tnow - last_5s > 5000) {
        last_5s = tnow;
        gcs_send_text(MAV_SEVERITY_INFO, "Initialising APM");
    }
    check_usb_mux();

    in_mavlink_delay = false;
}

/*
 *  send a message on both GCS links
 */
void Plane::gcs_send_message(enum ap_message id)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].send_message(id);
        }
    }
}

/*
 *  send a mission item reached message and load the index before the send attempt in case it may get delayed
 */
void Plane::gcs_send_mission_item_reached_message(uint16_t mission_index)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].mission_item_reached_index = mission_index;
            gcs[i].send_message(MSG_MISSION_ITEM_REACHED);
        }
    }
}

/*
 *  send data streams in the given rate range on both links
 */
void Plane::gcs_data_stream_send(void)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].data_stream_send();
        }
    }
}

/*
 *  look for incoming commands on the GCS links
 */
void Plane::gcs_update(void)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
#if CLI_ENABLED == ENABLED
            gcs[i].update(g.cli_enabled == 1 ? FUNCTOR_BIND_MEMBER(&Plane::run_cli, void, AP_HAL::UARTDriver *):NULL);
#else
            gcs[i].update(NULL);
#endif
        }
    }
}

void Plane::gcs_send_text(MAV_SEVERITY severity, const char *str)
{
    GCS_MAVLINK::send_statustext(severity, 0xFF, str);
}

/*
 *  send a low priority formatted message to the GCS
 *  only one fits in the queue, so if you send more than one before the
 *  last one gets into the serial buffer then the old one will be lost
 */
void Plane::gcs_send_text_fmt(MAV_SEVERITY severity, const char *fmt, ...)
{
    char str[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] {};
    va_list arg_list;
    va_start(arg_list, fmt);
    hal.util->vsnprintf((char *)str, sizeof(str), fmt, arg_list);
    va_end(arg_list);
    GCS_MAVLINK::send_statustext(severity, 0xFF, str);
}

/*
  send airspeed calibration data
 */
void Plane::gcs_send_airspeed_calibration(const Vector3f &vg)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            if (comm_get_txspace((mavlink_channel_t)i) - MAVLINK_NUM_NON_PAYLOAD_BYTES >= 
                MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN) {
                airspeed.log_mavlink_send((mavlink_channel_t)i, vg);
            }
        }
    }
}

/**
   retry any deferred messages
 */
void Plane::gcs_retry_deferred(void)
{
    gcs_send_message(MSG_RETRY_DEFERRED);
    GCS_MAVLINK::service_statustext();
}
