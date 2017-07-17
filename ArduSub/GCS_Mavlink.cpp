#include "Sub.h"
#include "version.h"

#include "GCS_Mavlink.h"

// default sensors are present and healthy: gyro, accelerometer, barometer, rate_control, attitude_stabilization, yaw_position, altitude control, x/y position control, motor_control
#define MAVLINK_SENSOR_PRESENT_DEFAULT (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_YAW_POSITION | MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS | MAV_SYS_STATUS_AHRS)

void Sub::gcs_send_heartbeat(void)
{
    gcs().send_message(MSG_HEARTBEAT);
}

void Sub::gcs_send_deferred(void)
{
    gcs().send_message(MSG_RETRY_DEFERRED);
    gcs().service_statustext();
}

/*
 *  !!NOTE!!
 *
 *  the use of NOINLINE separate functions for each message type avoids
 *  a compiler bug in gcc that would cause it to use far more stack
 *  space than is needed. Without the NOINLINE we use the sum of the
 *  stack needed for each message type. Please be careful to follow the
 *  pattern below when adding any new messages
 */

NOINLINE void Sub::send_heartbeat(mavlink_channel_t chan)
{
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = motors.armed() ? MAV_STATE_STANDBY : MAV_STATE_ACTIVE;
    uint32_t custom_mode = control_mode;

    // set system as critical if any failsafe have triggered
    if (failsafe.pilot_input || failsafe.battery || failsafe.gcs || failsafe.ekf || failsafe.terrain)  {
        system_status = MAV_STATE_CRITICAL;
    }

    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
    switch (control_mode) {
    case AUTO:
    case GUIDED:
    case CIRCLE:
    case POSHOLD:
        base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    default:
        break;
    }

    // all modes except INITIALISING have some form of manual
    // override if stick mixing is enabled
    base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

    // we are armed if we are not initialising
    if (motors.armed()) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    uint8_t mav_type;
    mav_type = MAV_TYPE_SUBMARINE;

    gcs().chan(chan-MAVLINK_COMM_0).send_heartbeat(mav_type,
                                            base_mode,
                                            custom_mode,
                                            system_status);
}

NOINLINE void Sub::send_attitude(mavlink_channel_t chan)
{
    const Vector3f &gyro = ins.get_gyro();
    mavlink_msg_attitude_send(
        chan,
        millis(),
        ahrs.roll,
        ahrs.pitch,
        ahrs.yaw,
        gyro.x,
        gyro.y,
        gyro.z);
}

#if AC_FENCE == ENABLED
NOINLINE void Sub::send_limits_status(mavlink_channel_t chan)
{
    fence_send_mavlink_status(chan);
}
#endif


NOINLINE void Sub::send_extended_status1(mavlink_channel_t chan)
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
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#if OPTFLOW == ENABLED
    if (optflow.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif

    // all present sensors enabled by default except altitude and position control and motors which we will set individually
    control_sensors_enabled = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL &
                              ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL &
                              ~MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS);

    switch (control_mode) {
    case ALT_HOLD:
    case AUTO:
    case GUIDED:
    case CIRCLE:
    case SURFACE:
    case POSHOLD:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        break;
    default:
        break;
    }

    // set motors outputs as enabled if safety switch is not disarmed (i.e. either NONE or ARMED)
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }

    // default to all healthy except baro, compass, gps and receiver which we set individually
    control_sensors_health = control_sensors_present & ~(MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                             MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR_RC_RECEIVER);

    if (sensor_health.depth) { // check the internal barometer only
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    }
    if (g.compass_enabled && compass.healthy() && ahrs.use_compass()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (gps.status() > AP_GPS::NO_GPS) {
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

    if (ahrs.initialised() && !ahrs.healthy()) {
        // AHRS subsystem is unhealthy
        control_sensors_health &= ~MAV_SYS_STATUS_AHRS;
    }

    int16_t battery_current = -1;
    int8_t battery_remaining = -1;

    if (battery.has_current() && battery.healthy()) {
        // percent remaining is not necessarily accurate at the moment
        //battery_remaining = battery.capacity_remaining_pct();
        battery_current = battery.current_amps() * 100;
    }

#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    switch (terrain.status()) {
    case AP_Terrain::TerrainStatusDisabled:
        break;
    case AP_Terrain::TerrainStatusUnhealthy:
        // To-Do: restore unhealthy terrain status reporting once terrain is used in Sub
        //control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        //control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        //break;
    case AP_Terrain::TerrainStatusOK:
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_health  |= MAV_SYS_STATUS_TERRAIN;
        break;
    }
#endif

#if RANGEFINDER_ENABLED == ENABLED
    if (rangefinder_state.enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (rangefinder.has_data_orient(ROTATION_PITCH_270)) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
    }
#endif

    if (!ap.initialised || ins.calibrating()) {
        // while initialising the gyros and accels are not enabled
        control_sensors_enabled &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
        control_sensors_health &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
    }

    mavlink_msg_sys_status_send(
        chan,
        control_sensors_present,
        control_sensors_enabled,
        control_sensors_health,
        (uint16_t)(scheduler.load_average(MAIN_LOOP_MICROS) * 1000),
        battery.voltage() * 1000, // mV
        battery_current,        // in 10mA units
        battery_remaining,      // in %
        0, // comm drops %,
        0, // comm drops in pkts,
        0, 0, 0, 0);

}

void NOINLINE Sub::send_location(mavlink_channel_t chan)
{
    uint32_t fix_time;
    // if we have a GPS fix, take the time as the last fix time. That
    // allows us to correctly calculate velocities and extrapolate
    // positions.
    // If we don't have a GPS fix then we are dead reckoning, and will
    // use the current boot time as the fix time.
    if (gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
        fix_time = gps.last_fix_time_ms();
    } else {
        fix_time = millis();
    }
    const Vector3f &vel = inertial_nav.get_velocity();
    mavlink_msg_global_position_int_send(
        chan,
        fix_time,
        current_loc.lat,                // in 1E7 degrees
        current_loc.lng,                // in 1E7 degrees
        (ahrs.get_home().alt + current_loc.alt) * 10UL,      // millimeters above sea level
        current_loc.alt * 10,           // millimeters above ground
        vel.x,                          // X speed cm/s (+ve North)
        vel.y,                          // Y speed cm/s (+ve East)
        vel.z,                          // Z speed cm/s (+ve up)
        ahrs.yaw_sensor);               // compass heading in 1/100 degree
}

void NOINLINE Sub::send_nav_controller_output(mavlink_channel_t chan)
{
    const Vector3f &targets = attitude_control.get_att_target_euler_cd();
    mavlink_msg_nav_controller_output_send(
        chan,
        targets.x * 1.0e-2f,
        targets.y * 1.0e-2f,
        targets.z * 1.0e-2f,
        wp_nav.get_wp_bearing_to_destination() * 1.0e-2f,
        MIN(wp_nav.get_wp_distance_to_destination() * 1.0e-2f, UINT16_MAX),
        pos_control.get_alt_error() * 1.0e-2f,
        0,
        0);
}

// report simulator state
void NOINLINE Sub::send_simstate(mavlink_channel_t chan)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    sitl.simstate_send(chan);
#endif
}

void NOINLINE Sub::send_hwstatus(mavlink_channel_t chan)
{
    mavlink_msg_hwstatus_send(
        chan,
        hal.analogin->board_voltage()*1000,
        0);
}

void NOINLINE Sub::send_radio_out(mavlink_channel_t chan)
{
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
        hal.rcout->read(7),
        hal.rcout->read(8),
        hal.rcout->read(9),
        hal.rcout->read(10),
        hal.rcout->read(11),
        hal.rcout->read(12),
        hal.rcout->read(13),
        hal.rcout->read(14),
        hal.rcout->read(15));
}

void NOINLINE Sub::send_vfr_hud(mavlink_channel_t chan)
{
    mavlink_msg_vfr_hud_send(
        chan,
        gps.ground_speed(),
        gps.ground_speed(),
        (ahrs.yaw_sensor / 100) % 360,
        (int16_t)(motors.get_throttle() * 100),
        current_loc.alt / 100.0f,
        climb_rate / 100.0f);
}

void NOINLINE Sub::send_current_waypoint(mavlink_channel_t chan)
{
    mavlink_msg_mission_current_send(chan, mission.get_current_nav_index());
}

/*
  send RPM packet
 */
#if RPM_ENABLED == ENABLED
void NOINLINE Sub::send_rpm(mavlink_channel_t chan)
{
    if (rpm_sensor.enabled(0) || rpm_sensor.enabled(1)) {
        mavlink_msg_rpm_send(
            chan,
            rpm_sensor.get_rpm(0),
            rpm_sensor.get_rpm(1));
    }
}
#endif

// Work around to get temperature sensor data out
void NOINLINE Sub::send_temperature(mavlink_channel_t chan)
{
    if (!celsius.healthy()) {
        return;
    }
    mavlink_msg_scaled_pressure3_send(
        chan,
        AP_HAL::millis(),
        0,
        0,
        celsius.temperature() * 100);
}

/*
  send PID tuning message
 */
void Sub::send_pid_tuning(mavlink_channel_t chan)
{
    const Vector3f &gyro = ahrs.get_gyro();
    if (g.gcs_pid_mask & 1) {
        const DataFlash_Class::PID_Info &pid_info = attitude_control.get_rate_roll_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_ROLL,
                                    pid_info.desired*0.01f,
                                    degrees(gyro.x),
                                    pid_info.FF*0.01f,
                                    pid_info.P*0.01f,
                                    pid_info.I*0.01f,
                                    pid_info.D*0.01f);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 2) {
        const DataFlash_Class::PID_Info &pid_info = attitude_control.get_rate_pitch_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_PITCH,
                                    pid_info.desired*0.01f,
                                    degrees(gyro.y),
                                    pid_info.FF*0.01f,
                                    pid_info.P*0.01f,
                                    pid_info.I*0.01f,
                                    pid_info.D*0.01f);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 4) {
        const DataFlash_Class::PID_Info &pid_info = attitude_control.get_rate_yaw_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_YAW,
                                    pid_info.desired*0.01f,
                                    degrees(gyro.z),
                                    pid_info.FF*0.01f,
                                    pid_info.P*0.01f,
                                    pid_info.I*0.01f,
                                    pid_info.D*0.01f);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 8) {
        const DataFlash_Class::PID_Info &pid_info = g.pid_accel_z.get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_ACCZ,
                                    pid_info.desired*0.01f,
                                    -(ahrs.get_accel_ef_blended().z + GRAVITY_MSS),
                                    pid_info.FF*0.01f,
                                    pid_info.P*0.01f,
                                    pid_info.I*0.01f,
                                    pid_info.D*0.01f);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
}

uint8_t GCS_MAVLINK_Sub::sysid_my_gcs() const
{
    return sub.g.sysid_my_gcs;
}

// try to send a message, return false if it won't fit in the serial tx buffer
bool GCS_MAVLINK_Sub::try_send_message(enum ap_message id)
{
    if (telemetry_delayed()) {
        return false;
    }

    // if we don't have at least 250 micros remaining before the main loop
    // wants to fire then don't send a mavlink message. We want to
    // prioritise the main flight control loop over communications
    if (sub.scheduler.time_available_usec() < 250 && sub.motors.armed()) {
        sub.gcs_out_of_time = true;
        return false;
    }

    switch (id) {
    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        last_heartbeat_time = AP_HAL::millis();
        sub.send_heartbeat(chan);
        break;

    case MSG_EXTENDED_STATUS1:
        // send extended status only once vehicle has been initialised
        // to avoid unnecessary errors being reported to user
        if (sub.ap.initialised) {
            CHECK_PAYLOAD_SIZE(SYS_STATUS);
            sub.send_extended_status1(chan);
            CHECK_PAYLOAD_SIZE(POWER_STATUS);
            send_power_status();
        }
        break;

    case MSG_EXTENDED_STATUS2:
        CHECK_PAYLOAD_SIZE(MEMINFO);
        send_meminfo();
        break;

    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        sub.send_attitude(chan);
        break;

    case MSG_LOCATION:
        CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
        sub.send_location(chan);
        break;

    case MSG_LOCAL_POSITION:
        CHECK_PAYLOAD_SIZE(LOCAL_POSITION_NED);
        send_local_position(sub.ahrs);
        break;

    case MSG_NAV_CONTROLLER_OUTPUT:
        CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
        sub.send_nav_controller_output(chan);
        break;

    case MSG_GPS_RAW:
        send_gps_raw(sub.gps);
        break;

    case MSG_SYSTEM_TIME:
        CHECK_PAYLOAD_SIZE(SYSTEM_TIME);
        send_system_time(sub.gps);
        break;

    case MSG_SERVO_OUT:
        break;

    case MSG_RADIO_IN:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_RAW);
        send_radio_in(0);
        break;

    case MSG_SERVO_OUTPUT_RAW:
        CHECK_PAYLOAD_SIZE(SERVO_OUTPUT_RAW);
        sub.send_radio_out(chan);
        break;

    case MSG_VFR_HUD:
        CHECK_PAYLOAD_SIZE(VFR_HUD);
        sub.send_vfr_hud(chan);
        break;

    case MSG_RAW_IMU1:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
        send_raw_imu(sub.ins, sub.compass);
        break;

    case MSG_RAW_IMU2:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE);
        send_scaled_pressure(sub.barometer);
        sub.send_temperature(chan);
        break;

    case MSG_RAW_IMU3:
        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
        send_sensor_offsets(sub.ins, sub.compass, sub.barometer);
        break;

    case MSG_CURRENT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_CURRENT);
        sub.send_current_waypoint(chan);
        break;

    case MSG_NEXT_PARAM:
        CHECK_PAYLOAD_SIZE(PARAM_VALUE);
        queued_param_send();
        break;

    case MSG_NEXT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_REQUEST);
        queued_waypoint_send();
        break;

    case MSG_RANGEFINDER:
#if RANGEFINDER_ENABLED == ENABLED
        CHECK_PAYLOAD_SIZE(RANGEFINDER);
        send_rangefinder_downward(sub.rangefinder);
        CHECK_PAYLOAD_SIZE(DISTANCE_SENSOR);
        send_distance_sensor_downward(sub.rangefinder);
#endif
        break;

    case MSG_RPM:
#if RPM_ENABLED == ENABLED
        CHECK_PAYLOAD_SIZE(RPM);
        sub.send_rpm(chan);
#endif
        break;

    case MSG_TERRAIN:
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
        CHECK_PAYLOAD_SIZE(TERRAIN_REQUEST);
        sub.terrain.send_request(chan);
#endif
        break;

    case MSG_CAMERA_FEEDBACK:
#if CAMERA == ENABLED
        CHECK_PAYLOAD_SIZE(CAMERA_FEEDBACK);
        sub.camera.send_feedback(chan, sub.gps, sub.ahrs, sub.current_loc);
#endif
        break;

    case MSG_STATUSTEXT:
        // deprecated, use gcs().send_statustext*
        return false;

    case MSG_LIMITS_STATUS:
#if AC_FENCE == ENABLED
        CHECK_PAYLOAD_SIZE(LIMITS_STATUS);
        sub.send_limits_status(chan);
#endif
        break;

    case MSG_AHRS:
        CHECK_PAYLOAD_SIZE(AHRS);
        send_ahrs(sub.ahrs);
        break;

    case MSG_SIMSTATE:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        CHECK_PAYLOAD_SIZE(SIMSTATE);
        sub.send_simstate(chan);
#endif
        CHECK_PAYLOAD_SIZE(AHRS2);
        send_ahrs2(sub.ahrs);
        break;

    case MSG_HWSTATUS:
        CHECK_PAYLOAD_SIZE(HWSTATUS);
        sub.send_hwstatus(chan);
        break;

    case MSG_MOUNT_STATUS:
#if MOUNT == ENABLED
        CHECK_PAYLOAD_SIZE(MOUNT_STATUS);
        sub.camera_mount.status_msg(chan);
#endif // MOUNT == ENABLED
        break;

    case MSG_BATTERY2:
        CHECK_PAYLOAD_SIZE(BATTERY2);
        send_battery2(sub.battery);
        break;

    case MSG_OPTICAL_FLOW:
#if OPTFLOW == ENABLED
        CHECK_PAYLOAD_SIZE(OPTICAL_FLOW);
        send_opticalflow(sub.ahrs, sub.optflow);
#endif
        break;

    case MSG_GIMBAL_REPORT:
#if MOUNT == ENABLED
        CHECK_PAYLOAD_SIZE(GIMBAL_REPORT);
        sub.camera_mount.send_gimbal_report(chan);
#endif
        break;

    case MSG_EKF_STATUS_REPORT:
        CHECK_PAYLOAD_SIZE(EKF_STATUS_REPORT);
        sub.ahrs.send_ekf_status_report(chan);
        break;

    case MSG_FENCE_STATUS:
    case MSG_WIND:
    case MSG_POSITION_TARGET_GLOBAL_INT:
    case MSG_AOA_SSA:
    case MSG_LANDING:
        // unused
        break;

    case MSG_PID_TUNING:
        CHECK_PAYLOAD_SIZE(PID_TUNING);
        sub.send_pid_tuning(chan);
        break;

    case MSG_VIBRATION:
        CHECK_PAYLOAD_SIZE(VIBRATION);
        send_vibration(sub.ins);
        break;

    case MSG_MISSION_ITEM_REACHED:
        CHECK_PAYLOAD_SIZE(MISSION_ITEM_REACHED);
        mavlink_msg_mission_item_reached_send(chan, mission_item_reached_index);
        break;

    case MSG_RETRY_DEFERRED:
        break; // just here to prevent a warning

    case MSG_MAG_CAL_PROGRESS:
        sub.compass.send_mag_cal_progress(chan);
        break;

    case MSG_MAG_CAL_REPORT:
        sub.compass.send_mag_cal_report(chan);
        break;

    case MSG_ADSB_VEHICLE:
        break; // Do nothing for Sub, here to prevent warning
    case MSG_BATTERY_STATUS:
        send_battery_status(sub.battery);
        break;
    }

    return true;
}


const AP_Param::GroupInfo GCS_MAVLINK::var_info[] = {
    // @Param: RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK, streamRates[STREAM_RAW_SENSORS],  0),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK, streamRates[STREAM_EXTENDED_STATUS],  0),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS_RAW to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK, streamRates[STREAM_RC_CHANNELS],  0),

    // @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Stream rate of GLOBAL_POSITION_INT to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK, streamRates[STREAM_POSITION],  0),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK, streamRates[STREAM_EXTRA1],  0),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate to ground station
    // @Description: Stream rate of VFR_HUD to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK, streamRates[STREAM_EXTRA2],  0),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK, streamRates[STREAM_EXTRA3],  0),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate to ground station
    // @Description: Stream rate of PARAM_VALUE to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK, streamRates[STREAM_PARAMS],  0),
    AP_GROUPEND
};

void
GCS_MAVLINK_Sub::data_stream_send(void)
{
    if (waypoint_receiving) {
        // don't interfere with mission transfer
        return;
    }

    if (!sub.in_mavlink_delay && !sub.motors.armed()) {
        sub.DataFlash.handle_log_send(*this);
    }

    sub.gcs_out_of_time = false;

    send_queued_parameters();

    if (sub.gcs_out_of_time) {
        return;
    }

    if (sub.in_mavlink_delay) {
        // don't send any other stream types while in the delay callback
        return;
    }

    if (stream_trigger(STREAM_RAW_SENSORS)) {
        send_message(MSG_RAW_IMU1);
        send_message(MSG_RAW_IMU2);
        send_message(MSG_RAW_IMU3);
    }

    if (sub.gcs_out_of_time) {
        return;
    }

    if (stream_trigger(STREAM_EXTENDED_STATUS)) {
        send_message(MSG_EXTENDED_STATUS1);
        send_message(MSG_EXTENDED_STATUS2);
        send_message(MSG_CURRENT_WAYPOINT);
        send_message(MSG_GPS_RAW);
        send_message(MSG_NAV_CONTROLLER_OUTPUT);
        send_message(MSG_LIMITS_STATUS);
    }

    if (sub.gcs_out_of_time) {
        return;
    }

    if (stream_trigger(STREAM_POSITION)) {
        send_message(MSG_LOCATION);
        send_message(MSG_LOCAL_POSITION);
    }

    if (sub.gcs_out_of_time) {
        return;
    }

    if (stream_trigger(STREAM_RAW_CONTROLLER)) {
        send_message(MSG_SERVO_OUT);
    }

    if (sub.gcs_out_of_time) {
        return;
    }

    if (stream_trigger(STREAM_RC_CHANNELS)) {
        send_message(MSG_SERVO_OUTPUT_RAW);
        send_message(MSG_RADIO_IN);
    }

    if (sub.gcs_out_of_time) {
        return;
    }

    if (stream_trigger(STREAM_EXTRA1)) {
        send_message(MSG_ATTITUDE);
        send_message(MSG_SIMSTATE);
        send_message(MSG_PID_TUNING);
    }

    if (sub.gcs_out_of_time) {
        return;
    }

    if (stream_trigger(STREAM_EXTRA2)) {
        send_message(MSG_VFR_HUD);
    }

    if (sub.gcs_out_of_time) {
        return;
    }

    if (stream_trigger(STREAM_EXTRA3)) {
        send_message(MSG_AHRS);
        send_message(MSG_HWSTATUS);
        send_message(MSG_SYSTEM_TIME);
        send_message(MSG_RANGEFINDER);
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
        send_message(MSG_TERRAIN);
#endif
        send_message(MSG_BATTERY2);
        send_message(MSG_BATTERY_STATUS);
        send_message(MSG_MOUNT_STATUS);
        send_message(MSG_OPTICAL_FLOW);
        send_message(MSG_GIMBAL_REPORT);
        send_message(MSG_MAG_CAL_REPORT);
        send_message(MSG_MAG_CAL_PROGRESS);
        send_message(MSG_EKF_STATUS_REPORT);
        send_message(MSG_VIBRATION);
#if RPM_ENABLED == ENABLED
        send_message(MSG_RPM);
#endif
    }

    if (sub.gcs_out_of_time) {
        return;
    }
}


bool GCS_MAVLINK_Sub::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
    return sub.do_guided(cmd);
}

void GCS_MAVLINK_Sub::handle_change_alt_request(AP_Mission::Mission_Command &cmd)
{
    // add home alt if needed
    if (cmd.content.location.flags.relative_alt) {
        cmd.content.location.alt += sub.ahrs.get_home().alt;
    }

    // To-Do: update target altitude for loiter or waypoint controller depending upon nav mode
}

void GCS_MAVLINK_Sub::handleMessage(mavlink_message_t* msg)
{
    uint8_t result = MAV_RESULT_FAILED;         // assume failure.  Each messages id is responsible for return ACK or NAK if required

    switch (msg->msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT: {    // MAV ID: 0
        // We keep track of the last time we received a heartbeat from our GCS for failsafe purposes
        if (msg->sysid != sub.g.sysid_my_gcs) {
            break;
        }
        sub.failsafe.last_heartbeat_ms = AP_HAL::millis();
        sub.pmTest1++;
        break;
    }

    case MAVLINK_MSG_ID_SET_MODE: {     // MAV ID: 11
        handle_set_mode(msg, FUNCTOR_BIND(&sub, &Sub::gcs_set_mode, bool, uint8_t));
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {       // MAV ID: 21
        // mark the firmware version in the tlog
        send_text(MAV_SEVERITY_INFO, FIRMWARE_STRING);

#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
        send_text(MAV_SEVERITY_INFO, "PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION);
#endif
        handle_param_request_list(msg);
        break;
    }

    case MAVLINK_MSG_ID_PARAM_SET: {   // 23
        handle_param_set(msg, &sub.DataFlash);
        break;
    }

    case MAVLINK_MSG_ID_PARAM_VALUE: {
        sub.camera_mount.handle_param_value(msg);
        break;
    }

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {  // MAV ID: 66
        handle_request_data_stream(msg, false);
        break;
    }

    case MAVLINK_MSG_ID_GIMBAL_REPORT: {
#if MOUNT == ENABLED
        handle_gimbal_report(sub.camera_mount, msg);
#endif
        break;
    }

    case MAVLINK_MSG_ID_MANUAL_CONTROL: {     // MAV ID: 69
        if (msg->sysid != sub.g.sysid_my_gcs) {
            break;    // Only accept control from our gcs
        }
        mavlink_manual_control_t packet;
        mavlink_msg_manual_control_decode(msg, &packet);

        sub.transform_manual_control_to_rc_override(packet.x,packet.y,packet.z,packet.r,packet.buttons);

        sub.failsafe.last_pilot_input_ms = AP_HAL::millis();
        // a RC override message is considered to be a 'heartbeat' from the ground station for failsafe purposes
        sub.failsafe.last_heartbeat_ms = AP_HAL::millis();
        break;
    }

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {     // MAV ID: 70
        // allow override of RC input
        if (msg->sysid != sub.g.sysid_my_gcs) {
            break;    // Only accept control from our gcs
        }
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

        hal.rcin->set_overrides(v, 8);

        sub.failsafe.last_pilot_input_ms = AP_HAL::millis();
        // a RC override message is considered to be a 'heartbeat' from the ground station for failsafe purposes
        sub.failsafe.last_heartbeat_ms = AP_HAL::millis();
        break;
    }


    case MAVLINK_MSG_ID_COMMAND_INT: {
        // decode packet
        mavlink_command_int_t packet;
        mavlink_msg_command_int_decode(msg, &packet);
        switch (packet.command) {
        case MAV_CMD_DO_SET_ROI: {
            // param1 : /* Region of interest mode (not used)*/
            // param2 : /* MISSION index/ target ID (not used)*/
            // param3 : /* ROI index (not used)*/
            // param4 : /* empty */
            // x : lat
            // y : lon
            // z : alt
            // sanity check location
            if (!check_latlng(packet.x, packet.y)) {
                break;
            }
            Location roi_loc;
            roi_loc.lat = packet.x;
            roi_loc.lng = packet.y;
            roi_loc.alt = (int32_t)(packet.z * 100.0f);
            sub.set_auto_yaw_roi(roi_loc);
            result = MAV_RESULT_ACCEPTED;
            break;
        }
        default:
            result = MAV_RESULT_UNSUPPORTED;
            break;
        }

        // send ACK or NAK
        mavlink_msg_command_ack_send_buf(msg, chan, packet.command, result);
        break;
    }

    // Pre-Flight calibration requests
    case MAVLINK_MSG_ID_COMMAND_LONG: {     // MAV ID: 76
        // decode packet
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(msg, &packet);

        switch (packet.command) {
        case MAV_CMD_PREFLIGHT_STORAGE:
            if (is_equal(packet.param1, 2.0f)) {
                AP_Param::erase_all();
                gcs().send_text(MAV_SEVERITY_WARNING, "All parameters reset, reboot board");
                result= MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_NAV_LOITER_UNLIM:
            if (sub.set_mode(POSHOLD, MODE_REASON_GCS_COMMAND)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_NAV_LAND:
            if (sub.set_mode(SURFACE, MODE_REASON_GCS_COMMAND)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_CONDITION_YAW:
            // param1 : target angle [0-360]
            // param2 : speed during change [deg per second]
            // param3 : direction (-1:ccw, +1:cw)
            // param4 : relative offset (1) or absolute angle (0)
            if ((packet.param1 >= 0.0f)   &&
                    (packet.param1 <= 360.0f) &&
                    (is_zero(packet.param4) || is_equal(packet.param4,1.0f))) {
                sub.set_auto_yaw_look_at_heading(packet.param1, packet.param2, (int8_t)packet.param3, (uint8_t)packet.param4);
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_FAILED;
            }
            break;

        case MAV_CMD_DO_CHANGE_SPEED:
            // param1 : unused
            // param2 : new speed in m/s
            // param3 : unused
            // param4 : unused
            if (packet.param2 > 0.0f) {
                sub.wp_nav.set_speed_xy(packet.param2 * 100.0f);
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_FAILED;
            }
            break;

        case MAV_CMD_DO_SET_HOME:
            // param1 : use current (1=use current location, 0=use specified location)
            // param5 : latitude
            // param6 : longitude
            // param7 : altitude (absolute)
            result = MAV_RESULT_FAILED; // assume failure
            if (is_equal(packet.param1,1.0f) || (is_zero(packet.param5) && is_zero(packet.param6) && is_zero(packet.param7))) {
                if (sub.set_home_to_current_location(true)) {
                    result = MAV_RESULT_ACCEPTED;
                }
            } else {
                // sanity check location
                if (!check_latlng(packet.param5, packet.param6)) {
                    break;
                }
                Location new_home_loc;
                new_home_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
                new_home_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
                new_home_loc.alt = (int32_t)(packet.param7 * 100.0f);
                if (!sub.far_from_EKF_origin(new_home_loc)) {
                    if (sub.set_home(new_home_loc, true)) {
                        result = MAV_RESULT_ACCEPTED;
                    }
                }
            }
            break;

        case MAV_CMD_DO_FLIGHTTERMINATION:
            if (packet.param1 > 0.5f) {
                sub.init_disarm_motors();
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_SET_ROI:
            // param1 : regional of interest mode (not supported)
            // param2 : mission index/ target id (not supported)
            // param3 : ROI index (not supported)
            // param5 : x / lat
            // param6 : y / lon
            // param7 : z / alt
            // sanity check location
            if (!check_latlng(packet.param5, packet.param6)) {
                break;
            }
            Location roi_loc;
            roi_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
            roi_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
            roi_loc.alt = (int32_t)(packet.param7 * 100.0f);
            sub.set_auto_yaw_roi(roi_loc);
            result = MAV_RESULT_ACCEPTED;
            break;

#if CAMERA == ENABLED
        case MAV_CMD_DO_DIGICAM_CONFIGURE:
            sub.camera.configure(packet.param1,
                                 packet.param2,
                                 packet.param3,
                                 packet.param4,
                                 packet.param5,
                                 packet.param6,
                                 packet.param7);

            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_DO_DIGICAM_CONTROL:
            if (sub.camera.control(packet.param1,
                                   packet.param2,
                                   packet.param3,
                                   packet.param4,
                                   packet.param5,
                                   packet.param6)) {
                sub.log_picture();
            }
            result = MAV_RESULT_ACCEPTED;
            break;
        case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
            sub.camera.set_trigger_distance(packet.param1);
            result = MAV_RESULT_ACCEPTED;
            break;
#endif // CAMERA == ENABLED
        case MAV_CMD_DO_MOUNT_CONTROL:
#if MOUNT == ENABLED
            sub.camera_mount.control(packet.param1, packet.param2, packet.param3, (MAV_MOUNT_MODE) packet.param7);
            result = MAV_RESULT_ACCEPTED;
#endif
            break;

        case MAV_CMD_MISSION_START:
            if (sub.motors.armed() && sub.set_mode(AUTO, MODE_REASON_GCS_COMMAND)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_PREFLIGHT_CALIBRATION:
            // exit immediately if armed
            if (sub.motors.armed()) {
                result = MAV_RESULT_FAILED;
                break;
            }
            if (is_equal(packet.param1,1.0f)) {
                if (sub.calibrate_gyros()) {
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            } else if (is_equal(packet.param3,1.0f)) {
                if (!sub.sensor_health.depth || sub.motors.armed() || sub.barometer.get_pressure() > 110000) {
                    result = MAV_RESULT_FAILED;
                } else {
                    sub.init_barometer(true);
                    result = MAV_RESULT_ACCEPTED;
                }
            } else if (is_equal(packet.param4,1.0f)) {
                result = MAV_RESULT_UNSUPPORTED;
            } else if (is_equal(packet.param5,1.0f)) {
                // 3d accel calibration
                result = MAV_RESULT_ACCEPTED;
                if (!sub.calibrate_gyros()) {
                    result = MAV_RESULT_FAILED;
                    break;
                }
                sub.ins.acal_init();
                sub.ins.get_acal()->start(this);

            } else if (is_equal(packet.param5,2.0f)) {
                // calibrate gyros
                if (!sub.calibrate_gyros()) {
                    result = MAV_RESULT_FAILED;
                    break;
                }
                // accel trim
                float trim_roll, trim_pitch;
                if (sub.ins.calibrate_trim(trim_roll, trim_pitch)) {
                    // reset ahrs's trim to suggested values from calibration routine
                    sub.ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            } else if (is_equal(packet.param6,1.0f)) {
                // compassmot calibration
                //result = sub.mavlink_compassmot(chan);
                gcs().send_text(MAV_SEVERITY_INFO, "#CompassMot calibration not supported");
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_COMPONENT_ARM_DISARM:
            if (is_equal(packet.param1,1.0f)) {
                // attempt to arm and return success or failure
                if (sub.init_arm_motors(true)) {
                    result = MAV_RESULT_ACCEPTED;
                }
            } else if (is_zero(packet.param1))  {
                // force disarming by setting param2 = 21196 is deprecated
                // see COMMAND_LONG DO_FLIGHTTERMINATION
                sub.init_disarm_motors();
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_GET_HOME_POSITION:
            if (sub.ap.home_state != HOME_UNSET) {
                send_home(sub.ahrs.get_home());
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if (is_equal(packet.param1,1.0f) || is_equal(packet.param1,3.0f)) {
                // Send an invalid signal to the motors to prevent spinning due to neutral (1500) pwm pulse being cut short
                // For that matter, send an invalid signal to all channels to prevent undesired/unexpected behavior
                hal.rcout->cork();
                for (int i=0; i<NUM_RC_CHANNELS; i++) {
                    // Set to 1 because 0 is interpreted as flag to ignore update
                    hal.rcout->write(i, 1);
                }
                hal.rcout->push();

                AP_Notify::flags.firmware_update = 1;
                sub.update_notify();
                hal.scheduler->delay(200);
                // when packet.param1 == 3 we reboot to hold in bootloader
                hal.scheduler->reboot(is_equal(packet.param1,3.0f));
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_FENCE_ENABLE:
#if AC_FENCE == ENABLED
            result = MAV_RESULT_ACCEPTED;
            switch ((uint16_t)packet.param1) {
            case 0:
                sub.fence.enable(false);
                break;
            case 1:
                sub.fence.enable(true);
                break;
            default:
                result = MAV_RESULT_FAILED;
                break;
            }
#else
            // if fence code is not included return failure
            result = MAV_RESULT_FAILED;
#endif
            break;

        case MAV_CMD_DO_MOTOR_TEST:
            // param1 : motor sequence number (a number from 1 to max number of motors on the vehicle)
            // param2 : throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
            // param3 : throttle (range depends upon param2)
            // param4 : timeout (in seconds)
            break;

#if GRIPPER_ENABLED == ENABLED
        case MAV_CMD_DO_GRIPPER:
            // param1 : gripper number (ignored)
            // param2 : action (0=release, 1=grab). See GRIPPER_ACTIONS enum.
            if (!sub.g2.gripper.enabled()) {
                result = MAV_RESULT_FAILED;
            } else {
                result = MAV_RESULT_ACCEPTED;
                switch ((uint8_t)packet.param2) {
                case GRIPPER_ACTION_RELEASE:
                    sub.g2.gripper.release();
                    break;
                case GRIPPER_ACTION_GRAB:
                    sub.g2.gripper.grab();
                    break;
                default:
                    result = MAV_RESULT_FAILED;
                    break;
                }
            }
            break;
#endif

        case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES: {
            if (is_equal(packet.param1,1.0f)) {
                send_autopilot_version(FIRMWARE_VERSION);
                result = MAV_RESULT_ACCEPTED;
            }
            break;
        }

        case MAV_CMD_DO_SEND_BANNER: {
            result = MAV_RESULT_ACCEPTED;

            send_text(MAV_SEVERITY_INFO, FIRMWARE_STRING);

#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
            send_text(MAV_SEVERITY_INFO, "PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION);
#endif

            // send system ID if we can
            char sysid[40];
            if (hal.util->get_system_id(sysid)) {
                send_text(MAV_SEVERITY_INFO, sysid);
            }

            break;
        }

        default:
            result = handle_command_long_message(packet);
            break;
        }

        // send ACK or NAK
        mavlink_msg_command_ack_send_buf(msg, chan, packet.command, result);

        break;
    }

    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET: { // MAV ID: 82
        // decode packet
        mavlink_set_attitude_target_t packet;
        mavlink_msg_set_attitude_target_decode(msg, &packet);

        // ensure type_mask specifies to use attitude and thrust
        if ((packet.type_mask & ((1<<7)|(1<<6))) != 0) {
            break;
        }

        // convert thrust to climb rate
        packet.thrust = constrain_float(packet.thrust, 0.0f, 1.0f);
        float climb_rate_cms = 0.0f;
        if (is_equal(packet.thrust, 0.5f)) {
            climb_rate_cms = 0.0f;
        } else if (packet.thrust > 0.5f) {
            // climb at up to WPNAV_SPEED_UP
            climb_rate_cms = (packet.thrust - 0.5f) * 2.0f * sub.wp_nav.get_speed_up();
        } else {
            // descend at up to WPNAV_SPEED_DN
            climb_rate_cms = (0.5f - packet.thrust) * 2.0f * -fabsf(sub.wp_nav.get_speed_down());
        }
        sub.guided_set_angle(Quaternion(packet.q[0],packet.q[1],packet.q[2],packet.q[3]), climb_rate_cms);
        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED: {   // MAV ID: 84
        // decode packet
        mavlink_set_position_target_local_ned_t packet;
        mavlink_msg_set_position_target_local_ned_decode(msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if ((sub.control_mode != GUIDED) && !(sub.control_mode == AUTO && sub.auto_mode == Auto_NavGuided)) {
            break;
        }

        // check for supported coordinate frames
        if (packet.coordinate_frame != MAV_FRAME_LOCAL_NED &&
                packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED &&
                packet.coordinate_frame != MAV_FRAME_BODY_NED &&
                packet.coordinate_frame != MAV_FRAME_BODY_OFFSET_NED) {
            break;
        }

        bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
        bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;

        /*
         * for future use:
         * bool force           = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE;
         * bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
         * bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;
         */

        // prepare position
        Vector3f pos_vector;
        if (!pos_ignore) {
            // convert to cm
            pos_vector = Vector3f(packet.x * 100.0f, packet.y * 100.0f, -packet.z * 100.0f);
            // rotate to body-frame if necessary
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED ||
                    packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                sub.rotate_body_frame_to_NE(pos_vector.x, pos_vector.y);
            }
            // add body offset if necessary
            if (packet.coordinate_frame == MAV_FRAME_LOCAL_OFFSET_NED ||
                    packet.coordinate_frame == MAV_FRAME_BODY_NED ||
                    packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                pos_vector += sub.inertial_nav.get_position();
            } else {
                // convert from alt-above-home to alt-above-ekf-origin
                pos_vector.z = sub.pv_alt_above_origin(pos_vector.z);
            }
        }

        // prepare velocity
        Vector3f vel_vector;
        if (!vel_ignore) {
            // convert to cm
            vel_vector = Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f);
            // rotate to body-frame if necessary
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                sub.rotate_body_frame_to_NE(vel_vector.x, vel_vector.y);
            }
        }

        // send request
        if (!pos_ignore && !vel_ignore && acc_ignore) {
            sub.guided_set_destination_posvel(pos_vector, vel_vector);
        } else if (pos_ignore && !vel_ignore && acc_ignore) {
            sub.guided_set_velocity(vel_vector);
        } else if (!pos_ignore && vel_ignore && acc_ignore) {
            if (!sub.guided_set_destination(pos_vector)) {
                result = MAV_RESULT_FAILED;
            }
        } else {
            result = MAV_RESULT_FAILED;
        }

        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: {  // MAV ID: 86
        // decode packet
        mavlink_set_position_target_global_int_t packet;
        mavlink_msg_set_position_target_global_int_decode(msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if ((sub.control_mode != GUIDED) && !(sub.control_mode == AUTO && sub.auto_mode == Auto_NavGuided)) {
            break;
        }

        // check for supported coordinate frames
        if (packet.coordinate_frame != MAV_FRAME_GLOBAL_INT &&
                packet.coordinate_frame != MAV_FRAME_GLOBAL_RELATIVE_ALT && // solo shot manager incorrectly sends RELATIVE_ALT instead of RELATIVE_ALT_INT
                packet.coordinate_frame != MAV_FRAME_GLOBAL_RELATIVE_ALT_INT &&
                packet.coordinate_frame != MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) {
            break;
        }

        bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
        bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;

        /*
         * for future use:
         * bool force           = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE;
         * bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
         * bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;
         */

        Vector3f pos_ned;

        if (!pos_ignore) {
            // sanity check location
            if (!check_latlng(packet.lat_int, packet.lon_int)) {
                result = MAV_RESULT_FAILED;
                break;
            }
            Location loc;
            loc.lat = packet.lat_int;
            loc.lng = packet.lon_int;
            loc.alt = packet.alt*100;
            switch (packet.coordinate_frame) {
            case MAV_FRAME_GLOBAL_RELATIVE_ALT: // solo shot manager incorrectly sends RELATIVE_ALT instead of RELATIVE_ALT_INT
            case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
                loc.flags.relative_alt = true;
                loc.flags.terrain_alt = false;
                break;
            case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
                loc.flags.relative_alt = true;
                loc.flags.terrain_alt = true;
                break;
            case MAV_FRAME_GLOBAL_INT:
            default:
                loc.flags.relative_alt = false;
                loc.flags.terrain_alt = false;
                break;
            }
            pos_ned = sub.pv_location_to_vector(loc);
        }

        if (!pos_ignore && !vel_ignore && acc_ignore) {
            sub.guided_set_destination_posvel(pos_ned, Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f));
        } else if (pos_ignore && !vel_ignore && acc_ignore) {
            sub.guided_set_velocity(Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f));
        } else if (!pos_ignore && vel_ignore && acc_ignore) {
            if (!sub.guided_set_destination(pos_ned)) {
                result = MAV_RESULT_FAILED;
            }
        } else {
            result = MAV_RESULT_FAILED;
        }

        break;
    }

    case MAVLINK_MSG_ID_DISTANCE_SENSOR: {
        result = MAV_RESULT_ACCEPTED;
        sub.rangefinder.handle_msg(msg);
        break;
    }

    case MAVLINK_MSG_ID_GPS_INPUT: {
        result = MAV_RESULT_ACCEPTED;
        sub.gps.handle_msg(msg);
        break;
    }

    case MAVLINK_MSG_ID_SERIAL_CONTROL:
        handle_serial_control(msg, sub.gps);
        break;

    case MAVLINK_MSG_ID_GPS_INJECT_DATA:
        handle_gps_inject(msg, sub.gps);
        result = MAV_RESULT_ACCEPTED;
        break;

#if AC_FENCE == ENABLED
        // send or receive fence points with GCS
    case MAVLINK_MSG_ID_FENCE_POINT:            // MAV ID: 160
    case MAVLINK_MSG_ID_FENCE_FETCH_POINT:
        sub.fence.handle_msg(*this, msg);
        break;
#endif // AC_FENCE == ENABLED

#if CAMERA == ENABLED
        //deprecated.  Use MAV_CMD_DO_DIGICAM_CONFIGURE
    case MAVLINK_MSG_ID_DIGICAM_CONFIGURE:      // MAV ID: 202
        break;

        //deprecated.  Use MAV_CMD_DO_DIGICAM_CONTROL
    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
        sub.camera.control_msg(msg);
        sub.log_picture();
        break;
#endif // CAMERA == ENABLED

#if MOUNT == ENABLED
        //deprecated. Use MAV_CMD_DO_MOUNT_CONFIGURE
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:        // MAV ID: 204
        sub.camera_mount.configure_msg(msg);
        break;
        //deprecated. Use MAV_CMD_DO_MOUNT_CONTROL
    case MAVLINK_MSG_ID_MOUNT_CONTROL:
        sub.camera_mount.control_msg(msg);
        break;
#endif // MOUNT == ENABLED

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
        sub.terrain.handle_data(chan, msg);
#endif
        break;

    case MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST:
        send_autopilot_version(FIRMWARE_VERSION);
        break;

    case MAVLINK_MSG_ID_LED_CONTROL:
        // send message to Notify
        AP_Notify::handle_led_control(msg);
        break;

    case MAVLINK_MSG_ID_SET_HOME_POSITION: {
        mavlink_set_home_position_t packet;
        mavlink_msg_set_home_position_decode(msg, &packet);
        if ((packet.latitude == 0) && (packet.longitude == 0) && (packet.altitude == 0)) {
            sub.set_home_to_current_location(true);
        } else {
            // sanity check location
            if (!check_latlng(packet.latitude, packet.longitude)) {
                break;
            }
            Location new_home_loc;
            new_home_loc.lat = packet.latitude;
            new_home_loc.lng = packet.longitude;
            new_home_loc.alt = packet.altitude / 10;
            if (sub.far_from_EKF_origin(new_home_loc)) {
                break;
            }
            sub.set_home(new_home_loc, true);
        }
        break;
    }

    // This adds support for leak detectors in a separate enclosure
    // connected to a mavlink enabled subsystem
    case MAVLINK_MSG_ID_SYS_STATUS: {
        uint32_t MAV_SENSOR_WATER = 0x20000000;
        mavlink_sys_status_t packet;
        mavlink_msg_sys_status_decode(msg, &packet);
        if ((packet.onboard_control_sensors_enabled & MAV_SENSOR_WATER) && !(packet.onboard_control_sensors_health & MAV_SENSOR_WATER)) {
            sub.leak_detector.set_detect();
        }
    }
        break;

    default:
        handle_common_message(msg);
        break;
    }     // end switch
} // end handle mavlink


/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
void Sub::mavlink_delay_cb()
{
    static uint32_t last_1hz, last_50hz, last_5s;
    if (!gcs().chan(0).initialised || in_mavlink_delay) {
        return;
    }

    in_mavlink_delay = true;
    DataFlash.EnableWrites(false);

    uint32_t tnow = millis();
    if (tnow - last_1hz > 1000) {
        last_1hz = tnow;
        gcs_send_heartbeat();
        gcs().send_message(MSG_EXTENDED_STATUS1);
    }
    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
        gcs_check_input();
        gcs_data_stream_send();
        gcs_send_deferred();
        notify.update();
    }
    if (tnow - last_5s > 5000) {
        last_5s = tnow;
        gcs().send_text(MAV_SEVERITY_INFO, "Initialising APM");
    }

    DataFlash.EnableWrites(true);
    in_mavlink_delay = false;
}

/*
 *  send data streams in the given rate range on both links
 */
void Sub::gcs_data_stream_send(void)
{
    gcs().data_stream_send();
}

/*
 *  look for incoming commands on the GCS links
 */
void Sub::gcs_check_input(void)
{
    gcs().update();
}

Compass *GCS_MAVLINK_Sub::get_compass() const
{
    return &sub.compass;
}

AP_Mission *GCS_MAVLINK_Sub::get_mission()
{
    return &sub.mission;
}

AP_ServoRelayEvents *GCS_MAVLINK_Sub::get_servorelayevents() const
{
    return &sub.ServoRelayEvents;
}

AP_Rally *GCS_MAVLINK_Sub::get_rally() const
{
#if AC_RALLY == ENABLED
    return &sub.rally;
#else
    return nullptr;
#endif
}
