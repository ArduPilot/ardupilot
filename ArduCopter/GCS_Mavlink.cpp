#include "Copter.h"

#include "GCS_Mavlink.h"

void Copter::gcs_send_heartbeat(void)
{
    gcs().send_message(MSG_HEARTBEAT);
}

void Copter::gcs_send_deferred(void)
{
    gcs().retry_deferred();
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

NOINLINE void Copter::send_heartbeat(mavlink_channel_t chan)
{
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = ap.land_complete ? MAV_STATE_STANDBY : MAV_STATE_ACTIVE;
    uint32_t custom_mode = control_mode;

    // set system as critical if any failsafe have triggered
    if (failsafe.radio || failsafe.battery || failsafe.gcs || failsafe.ekf || failsafe.terrain || failsafe.adsb)  {
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
    case RTL:
    case LOITER:
    case AVOID_ADSB:
    case GUIDED:
    case CIRCLE:
    case POSHOLD:
    case BRAKE:
    case SMART_RTL:
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

#if HIL_MODE != HIL_MODE_DISABLED
    base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
#endif

    // we are armed if we are not initialising
    if (motors != nullptr && motors->armed()) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    gcs().chan(chan-MAVLINK_COMM_0).send_heartbeat(get_frame_mav_type(),
                                            base_mode,
                                            custom_mode,
                                            system_status);
}

NOINLINE void Copter::send_attitude(mavlink_channel_t chan)
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
NOINLINE void Copter::send_fence_status(mavlink_channel_t chan)
{
    fence_send_mavlink_status(chan);
}
#endif


NOINLINE void Copter::send_extended_status1(mavlink_channel_t chan)
{
    int16_t battery_current = -1;
    int8_t battery_remaining = -1;

    if (battery.has_current() && battery.healthy()) {
        battery_remaining = battery.capacity_remaining_pct();
        battery_current = battery.current_amps() * 100;
    }

    update_sensor_status_flags();
    
    mavlink_msg_sys_status_send(
        chan,
        control_sensors_present,
        control_sensors_enabled,
        control_sensors_health,
        (uint16_t)(scheduler.load_average() * 1000),
        battery.voltage() * 1000, // mV
        battery_current,        // in 10mA units
        battery_remaining,      // in %
        0, // comm drops %,
        0, // comm drops in pkts,
        0, 0, 0, 0);
}

void NOINLINE Copter::send_location(mavlink_channel_t chan)
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

void NOINLINE Copter::send_nav_controller_output(mavlink_channel_t chan)
{
    const Vector3f &targets = attitude_control->get_att_target_euler_cd();
    mavlink_msg_nav_controller_output_send(
        chan,
        targets.x * 1.0e-2f,
        targets.y * 1.0e-2f,
        targets.z * 1.0e-2f,
        flightmode->wp_bearing() * 1.0e-2f,
        MIN(flightmode->wp_distance() * 1.0e-2f, UINT16_MAX),
        pos_control->get_alt_error() * 1.0e-2f,
        0,
        0);
}

// report simulator state
void NOINLINE Copter::send_simstate(mavlink_channel_t chan)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    sitl.simstate_send(chan);
#endif
}

void NOINLINE Copter::send_vfr_hud(mavlink_channel_t chan)
{
    mavlink_msg_vfr_hud_send(
        chan,
        gps.ground_speed(),
        ahrs.groundspeed(),
        (ahrs.yaw_sensor / 100) % 360,
        (int16_t)(motors->get_throttle() * 100),
        current_loc.alt / 100.0f,
        climb_rate / 100.0f);
}

/*
  send RPM packet
 */
void NOINLINE Copter::send_rpm(mavlink_channel_t chan)
{
    if (rpm_sensor.enabled(0) || rpm_sensor.enabled(1)) {
        mavlink_msg_rpm_send(
            chan,
            rpm_sensor.get_rpm(0),
            rpm_sensor.get_rpm(1));
    }
}


/*
  send PID tuning message
 */
void Copter::send_pid_tuning(mavlink_channel_t chan)
{
    const Vector3f &gyro = ahrs.get_gyro();
    if (g.gcs_pid_mask & 1) {
        const DataFlash_Class::PID_Info &pid_info = attitude_control->get_rate_roll_pid().get_pid_info();
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
        const DataFlash_Class::PID_Info &pid_info = attitude_control->get_rate_pitch_pid().get_pid_info();
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
        const DataFlash_Class::PID_Info &pid_info = attitude_control->get_rate_yaw_pid().get_pid_info();
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
        const DataFlash_Class::PID_Info &pid_info = copter.pos_control->get_accel_z_pid().get_pid_info();
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

uint8_t GCS_MAVLINK_Copter::sysid_my_gcs() const
{
    return copter.g.sysid_my_gcs;
}

uint32_t GCS_MAVLINK_Copter::telem_delay() const
{
    return (uint32_t)(copter.g.telem_delay);
}

// try to send a message, return false if it wasn't sent
bool GCS_MAVLINK_Copter::try_send_message(enum ap_message id)
{
    if (telemetry_delayed()) {
        return false;
    }

#if HIL_MODE != HIL_MODE_SENSORS
    // if we don't have at least 250 micros remaining before the main loop
    // wants to fire then don't send a mavlink message. We want to
    // prioritise the main flight control loop over communications

    // the check for nullptr here doesn't just save a nullptr
    // dereference; it means that we send messages out even if we're
    // failing to detect a PX4 board type (see delay(3000) in px_drivers).
    if (copter.motors != nullptr && copter.scheduler.time_available_usec() < 250 && copter.motors->armed()) {
        gcs().set_out_of_time(true);
        return false;
    }
#endif

    switch(id) {
    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        last_heartbeat_time = AP_HAL::millis();
        copter.send_heartbeat(chan);
        break;

    case MSG_EXTENDED_STATUS1:
        // send extended status only once vehicle has been initialised
        // to avoid unnecessary errors being reported to user
        if (copter.ap.initialised) {
            CHECK_PAYLOAD_SIZE(SYS_STATUS);
            copter.send_extended_status1(chan);
            CHECK_PAYLOAD_SIZE(POWER_STATUS);
            send_power_status();
        }
        break;

    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        copter.send_attitude(chan);
        break;

    case MSG_LOCATION:
        CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
        copter.send_location(chan);
        break;

    case MSG_LOCAL_POSITION:
        CHECK_PAYLOAD_SIZE(LOCAL_POSITION_NED);
        send_local_position(copter.ahrs);
        break;

    case MSG_NAV_CONTROLLER_OUTPUT:
        CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
        copter.send_nav_controller_output(chan);
        break;

    case MSG_RADIO_IN:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS);
        send_radio_in(copter.receiver_rssi);
        break;

    case MSG_SERVO_OUTPUT_RAW:
        CHECK_PAYLOAD_SIZE(SERVO_OUTPUT_RAW);
        send_servo_output_raw(false);
        break;

    case MSG_VFR_HUD:
        CHECK_PAYLOAD_SIZE(VFR_HUD);
        copter.send_vfr_hud(chan);
        break;

    case MSG_RAW_IMU1:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
        send_raw_imu(copter.ins, copter.compass);
        break;

    case MSG_RAW_IMU2:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE);
        send_scaled_pressure(copter.barometer);
        break;

    case MSG_RAW_IMU3:
        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
        send_sensor_offsets(copter.ins, copter.compass, copter.barometer);
        break;

    case MSG_RANGEFINDER:
#if RANGEFINDER_ENABLED == ENABLED
        CHECK_PAYLOAD_SIZE(RANGEFINDER);
        send_rangefinder_downward(copter.rangefinder);
        CHECK_PAYLOAD_SIZE(DISTANCE_SENSOR);
        send_distance_sensor_downward(copter.rangefinder);
#endif
#if PROXIMITY_ENABLED == ENABLED
        send_proximity(copter.g2.proximity);
#endif
        break;

    case MSG_RPM:
        CHECK_PAYLOAD_SIZE(RPM);
        copter.send_rpm(chan);
        break;

    case MSG_TERRAIN:
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
        CHECK_PAYLOAD_SIZE(TERRAIN_REQUEST);
        copter.terrain.send_request(chan);
#endif
        break;

    case MSG_FENCE_STATUS:
#if AC_FENCE == ENABLED
        CHECK_PAYLOAD_SIZE(FENCE_STATUS);
        copter.send_fence_status(chan);
#endif
        break;

    case MSG_AHRS:
        CHECK_PAYLOAD_SIZE(AHRS);
        send_ahrs(copter.ahrs);
        break;

    case MSG_SIMSTATE:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        CHECK_PAYLOAD_SIZE(SIMSTATE);
        copter.send_simstate(chan);
#endif
        CHECK_PAYLOAD_SIZE(AHRS2);
        send_ahrs2(copter.ahrs);
        break;

    case MSG_MOUNT_STATUS:
#if MOUNT == ENABLED
        CHECK_PAYLOAD_SIZE(MOUNT_STATUS);
        copter.camera_mount.status_msg(chan);
#endif // MOUNT == ENABLED
        break;

    case MSG_BATTERY2:
        CHECK_PAYLOAD_SIZE(BATTERY2);
        send_battery2(copter.battery);
        break;

    case MSG_OPTICAL_FLOW:
#if OPTFLOW == ENABLED
        CHECK_PAYLOAD_SIZE(OPTICAL_FLOW);
        send_opticalflow(copter.ahrs, copter.optflow);
#endif
        break;

    case MSG_GIMBAL_REPORT:
#if MOUNT == ENABLED
        CHECK_PAYLOAD_SIZE(GIMBAL_REPORT);
        copter.camera_mount.send_gimbal_report(chan);
#endif
        break;

    case MSG_EKF_STATUS_REPORT:
        CHECK_PAYLOAD_SIZE(EKF_STATUS_REPORT);
        copter.ahrs.send_ekf_status_report(chan);
        break;

    case MSG_LIMITS_STATUS:
    case MSG_WIND:
    case MSG_POSITION_TARGET_GLOBAL_INT:
    case MSG_SERVO_OUT:
    case MSG_AOA_SSA:
    case MSG_LANDING:
        // unused
        break;

    case MSG_PID_TUNING:
        CHECK_PAYLOAD_SIZE(PID_TUNING);
        copter.send_pid_tuning(chan);
        break;

    case MSG_VIBRATION:
        CHECK_PAYLOAD_SIZE(VIBRATION);
        send_vibration(copter.ins);
        break;

    case MSG_ADSB_VEHICLE:
        CHECK_PAYLOAD_SIZE(ADSB_VEHICLE);
        copter.adsb.send_adsb_vehicle(chan);
        break;
    case MSG_BATTERY_STATUS:
        send_battery_status(copter.battery);
        break;
    default:
        return GCS_MAVLINK::try_send_message(id);
    }
    return true;
}


const AP_Param::GroupInfo GCS_MAVLINK::var_info[] = {
    // @Param: RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and SENSOR_OFFSETS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK, streamRates[0],  0),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Stream rate of SYS_STATUS, POWER_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, and FENCE_STATUS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK, streamRates[1],  0),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK, streamRates[2],  0),

    // @Param: RAW_CTRL
    // @DisplayName: Raw Control stream rate to ground station
    // @Description: Stream rate of RC_CHANNELS_SCALED (HIL only) to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK, streamRates[3],  0),

    // @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK, streamRates[4],  0),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Stream rate of ATTITUDE, SIMSTATE (SITL only), AHRS2 and PID_TUNING to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK, streamRates[5],  0),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate to ground station
    // @Description: Stream rate of VFR_HUD to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK, streamRates[6],  0),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Stream rate of AHRS, HWSTATUS, SYSTEM_TIME, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, BATTERY2, MOUNT_STATUS, OPTICAL_FLOW, GIMBAL_REPORT, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION and RPM to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK, streamRates[7],  0),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate to ground station
    // @Description: Stream rate of PARAM_VALUE to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK, streamRates[8],  0),

    // @Param: ADSB
    // @DisplayName: ADSB stream rate to ground station
    // @Description: ADSB stream rate to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ADSB",   9, GCS_MAVLINK, streamRates[9],  5),
AP_GROUPEND
};

void
GCS_MAVLINK_Copter::data_stream_send(void)
{
    if (waypoint_receiving) {
        // don't interfere with mission transfer
        return;
    }

    if (!copter.in_mavlink_delay && !copter.motors->armed()) {
        copter.DataFlash.handle_log_send(*this);
    }

    gcs().set_out_of_time(false);

    send_queued_parameters();

    if (gcs().out_of_time()) return;

    if (copter.in_mavlink_delay) {
        // don't send any other stream types while in the delay callback
        return;
    }

    if (stream_trigger(STREAM_RAW_SENSORS)) {
        send_message(MSG_RAW_IMU1);  // RAW_IMU, SCALED_IMU2, SCALED_IMU3
        send_message(MSG_RAW_IMU2);  // SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3
        send_message(MSG_RAW_IMU3);  // SENSOR_OFFSETS
    }

    if (gcs().out_of_time()) return;

    if (stream_trigger(STREAM_EXTENDED_STATUS)) {
        send_message(MSG_EXTENDED_STATUS1); // SYS_STATUS, POWER_STATUS
        send_message(MSG_EXTENDED_STATUS2); // MEMINFO
        send_message(MSG_CURRENT_WAYPOINT);
        send_message(MSG_GPS_RAW);
        send_message(MSG_GPS_RTK);
        send_message(MSG_GPS2_RAW);
        send_message(MSG_GPS2_RTK);
        send_message(MSG_NAV_CONTROLLER_OUTPUT);
        send_message(MSG_FENCE_STATUS);
    }

    if (gcs().out_of_time()) return;

    if (stream_trigger(STREAM_POSITION)) {
        send_message(MSG_LOCATION);
        send_message(MSG_LOCAL_POSITION);
    }

    if (gcs().out_of_time()) return;

    if (stream_trigger(STREAM_RAW_CONTROLLER)) {
    }

    if (gcs().out_of_time()) return;

    if (stream_trigger(STREAM_RC_CHANNELS)) {
        send_message(MSG_SERVO_OUTPUT_RAW);
        send_message(MSG_RADIO_IN);
    }

    if (gcs().out_of_time()) return;

    if (stream_trigger(STREAM_EXTRA1)) {
        send_message(MSG_ATTITUDE);
        send_message(MSG_SIMSTATE); // SIMSTATE, AHRS2
        send_message(MSG_PID_TUNING);
    }

    if (gcs().out_of_time()) return;

    if (stream_trigger(STREAM_EXTRA2)) {
        send_message(MSG_VFR_HUD);
    }

    if (gcs().out_of_time()) return;

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
        send_message(MSG_RPM);
    }

    if (gcs().out_of_time()) return;

    if (stream_trigger(STREAM_ADSB)) {
        send_message(MSG_ADSB_VEHICLE);
    }
}


bool GCS_MAVLINK_Copter::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
    return copter.mode_auto.do_guided(cmd);
}

void GCS_MAVLINK_Copter::handle_change_alt_request(AP_Mission::Mission_Command &cmd)
{
    // add home alt if needed
    if (cmd.content.location.flags.relative_alt) {
        cmd.content.location.alt += copter.ahrs.get_home().alt;
    }

    // To-Do: update target altitude for loiter or waypoint controller depending upon nav mode
}

void GCS_MAVLINK_Copter::packetReceived(const mavlink_status_t &status,
                                        mavlink_message_t &msg)
{
    if (copter.g2.dev_options.get() & DevOptionADSBMAVLink) {
        // optional handling of GLOBAL_POSITION_INT as a MAVLink based avoidance source
        copter.avoidance_adsb.handle_msg(msg);
    }
    GCS_MAVLINK::packetReceived(status, msg);
}

bool GCS_MAVLINK_Copter::params_ready() const
{
    if (AP_BoardConfig::in_sensor_config_error()) {
        // we may never have parameters "initialised" in this case
        return true;
    }
    // if we have not yet initialised (including allocating the motors
    // object) we drop this request. That prevents the GCS from getting
    // a confusing parameter count during bootup
    return copter.ap.initialised_params;
}

void GCS_MAVLINK_Copter::send_banner()
{
    GCS_MAVLINK::send_banner();
    send_text(MAV_SEVERITY_INFO, "Frame: %s", copter.get_frame_string());
}


void GCS_MAVLINK_Copter::handleMessage(mavlink_message_t* msg)
{
    MAV_RESULT result = MAV_RESULT_FAILED;         // assume failure.  Each messages id is responsible for return ACK or NAK if required

    switch (msg->msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT:      // MAV ID: 0
    {
        // We keep track of the last time we received a heartbeat from our GCS for failsafe purposes
        if(msg->sysid != copter.g.sysid_my_gcs) break;
        copter.failsafe.last_heartbeat_ms = AP_HAL::millis();
        copter.pmTest1++;
        break;
    }

    case MAVLINK_MSG_ID_PARAM_VALUE:
    {
#if MOUNT == ENABLED
        copter.camera_mount.handle_param_value(msg);
#endif
        break;
    }

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:    // MAV ID: 66
    {
        handle_request_data_stream(msg, false);
        break;
    }

    case MAVLINK_MSG_ID_GIMBAL_REPORT:
    {
#if MOUNT == ENABLED
        handle_gimbal_report(copter.camera_mount, msg);
#endif
        break;
    }

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:       // MAV ID: 70
    {
        // allow override of RC channel values for HIL
        // or for complete GCS control of switch position
        // and RC PWM values.
        if(msg->sysid != copter.g.sysid_my_gcs) break;                         // Only accept control from our gcs
        if (!copter.ap.rc_override_enable) {
            if (copter.failsafe.rc_override_active) {  // if overrides were active previously, disable them
                copter.failsafe.rc_override_active = false;
                hal.rcin->clear_overrides();
            }
            break;
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

        // record that rc are overwritten so we can trigger a failsafe if we lose contact with groundstation
        copter.failsafe.rc_override_active = hal.rcin->set_overrides(v, 8);

        // a RC override message is considered to be a 'heartbeat' from the ground station for failsafe purposes
        copter.failsafe.last_heartbeat_ms = AP_HAL::millis();
        break;
    }

    case MAVLINK_MSG_ID_MANUAL_CONTROL:
    {
        if(msg->sysid != copter.g.sysid_my_gcs) break;                         // Only accept control from our gcs

        mavlink_manual_control_t packet;
        mavlink_msg_manual_control_decode(msg, &packet);

        if (packet.z < 0) { // Copter doesn't do negative thrust
            break;
        }

        bool override_active = false;
        int16_t roll = (packet.y == INT16_MAX) ? 0 : copter.channel_roll->get_radio_min() + (copter.channel_roll->get_radio_max() - copter.channel_roll->get_radio_min()) * (packet.y + 1000) / 2000.0f;
        int16_t pitch = (packet.x == INT16_MAX) ? 0 : copter.channel_pitch->get_radio_min() + (copter.channel_pitch->get_radio_max() - copter.channel_pitch->get_radio_min()) * (-packet.x + 1000) / 2000.0f;
        int16_t throttle = (packet.z == INT16_MAX) ? 0 : copter.channel_throttle->get_radio_min() + (copter.channel_throttle->get_radio_max() - copter.channel_throttle->get_radio_min()) * (packet.z) / 1000.0f;
        int16_t yaw = (packet.r == INT16_MAX) ? 0 : copter.channel_yaw->get_radio_min() + (copter.channel_yaw->get_radio_max() - copter.channel_yaw->get_radio_min()) * (packet.r + 1000) / 2000.0f;

        override_active |= hal.rcin->set_override(uint8_t(copter.rcmap.roll() - 1), roll);
        override_active |= hal.rcin->set_override(uint8_t(copter.rcmap.pitch() - 1), pitch);
        override_active |= hal.rcin->set_override(uint8_t(copter.rcmap.throttle() - 1), throttle);
        override_active |= hal.rcin->set_override(uint8_t(copter.rcmap.yaw() - 1), yaw);

        // record that rc are overwritten so we can trigger a failsafe if we lose contact with groundstation
        copter.failsafe.rc_override_active = override_active;

        // a manual control message is considered to be a 'heartbeat' from the ground station for failsafe purposes
        copter.failsafe.last_heartbeat_ms = AP_HAL::millis();
        break;
    }

    case MAVLINK_MSG_ID_COMMAND_INT:
    {
        // decode packet
        mavlink_command_int_t packet;
        mavlink_msg_command_int_decode(msg, &packet);
        switch(packet.command)
        {
            case MAV_CMD_DO_SET_HOME: {
                // assume failure
                result = MAV_RESULT_FAILED;
                if (is_equal(packet.param1, 1.0f)) {
                    // if param1 is 1, use current location
                    if (copter.set_home_to_current_location(true)) {
                        result = MAV_RESULT_ACCEPTED;
                    }
                    break;
                }
                // ensure param1 is zero
                if (!is_zero(packet.param1)) {
                    break;
                }
                // check frame type is supported
                if (packet.frame != MAV_FRAME_GLOBAL &&
                    packet.frame != MAV_FRAME_GLOBAL_INT &&
                    packet.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT &&
                    packet.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) {
                    break;
                }
                // sanity check location
                if (!check_latlng(packet.x, packet.y)) {
                    break;
                }
                Location new_home_loc {};
                new_home_loc.lat = packet.x;
                new_home_loc.lng = packet.y;
                new_home_loc.alt = packet.z * 100;
                // handle relative altitude
                if (packet.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT || packet.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) {
                    if (copter.ap.home_state == HOME_UNSET) {
                        // cannot use relative altitude if home is not set
                        break;
                    }
                    new_home_loc.alt += copter.ahrs.get_home().alt;
                }
                if (copter.set_home(new_home_loc, true)) {
                    result = MAV_RESULT_ACCEPTED;
                }
                break;
            }

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
                copter.set_auto_yaw_roi(roi_loc);
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
    case MAVLINK_MSG_ID_COMMAND_LONG:       // MAV ID: 76
    {
        // decode packet
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(msg, &packet);

        switch(packet.command) {

        case MAV_CMD_NAV_TAKEOFF: {
            // param3 : horizontal navigation by pilot acceptable
            // param4 : yaw angle   (not supported)
            // param5 : latitude    (not supported)
            // param6 : longitude   (not supported)
            // param7 : altitude [metres]

            float takeoff_alt = packet.param7 * 100;      // Convert m to cm

            if(copter.do_user_takeoff(takeoff_alt, is_zero(packet.param3))) {
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_FAILED;
            }
            break;
        }


        case MAV_CMD_NAV_LOITER_UNLIM:
            if (copter.set_mode(LOITER, MODE_REASON_GCS_COMMAND)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
            if (copter.set_mode(RTL, MODE_REASON_GCS_COMMAND)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_NAV_LAND:
            if (copter.set_mode(LAND, MODE_REASON_GCS_COMMAND)) {
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
            	copter.set_auto_yaw_look_at_heading(packet.param1, packet.param2, (int8_t)packet.param3, is_positive(packet.param4));
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
                copter.wp_nav->set_speed_xy(packet.param2 * 100.0f);
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
            if (is_equal(packet.param1,1.0f)) {
                if (copter.set_home_to_current_location(true)) {
                    result = MAV_RESULT_ACCEPTED;
                }
            } else {
                // ensure param1 is zero
                if (!is_zero(packet.param1)) {
                    break;
                }
                // sanity check location
                if (!check_latlng(packet.param5, packet.param6)) {
                    break;
                }
                Location new_home_loc;
                new_home_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
                new_home_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
                new_home_loc.alt = (int32_t)(packet.param7 * 100.0f);
                if (copter.set_home(new_home_loc, true)) {
                    result = MAV_RESULT_ACCEPTED;
                }
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
            copter.set_auto_yaw_roi(roi_loc);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_DO_MOUNT_CONTROL:
#if MOUNT == ENABLED
            if(!copter.camera_mount.has_pan_control()) {
                copter.set_auto_yaw_look_at_heading((float)packet.param3 / 100.0f,0.0f,0,0);
            }
            copter.camera_mount.control(packet.param1, packet.param2, packet.param3, (MAV_MOUNT_MODE) packet.param7);
            result = MAV_RESULT_ACCEPTED;
#endif
            break;

        case MAV_CMD_MISSION_START:
            if (copter.motors->armed() && copter.set_mode(AUTO, MODE_REASON_GCS_COMMAND)) {
                copter.set_auto_armed(true);
                if (copter.mission.state() != AP_Mission::MISSION_RUNNING) {
                    copter.mission.start_or_resume();
                }
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_PREFLIGHT_CALIBRATION:
            // exit immediately if armed
            if (copter.motors->armed()) {
                result = MAV_RESULT_FAILED;
                break;
            }
            if (is_equal(packet.param1,1.0f)) {
                if (copter.calibrate_gyros()) {
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            } else if (is_equal(packet.param3,1.0f)) {
                // fast barometer calibration
                copter.init_barometer(false);
                result = MAV_RESULT_ACCEPTED;
            } else if (is_equal(packet.param4,1.0f)) {
                result = MAV_RESULT_UNSUPPORTED;
            } else if (is_equal(packet.param5,1.0f)) {
                // 3d accel calibration
                result = MAV_RESULT_ACCEPTED;
                if (!copter.calibrate_gyros()) {
                    result = MAV_RESULT_FAILED;
                    break;
                }
                copter.ins.acal_init();
                copter.ins.get_acal()->start(this);
                
            } else if (is_equal(packet.param5,2.0f)) {
                // calibrate gyros
                if (!copter.calibrate_gyros()) {
                    result = MAV_RESULT_FAILED;
                    break;
                }
                // accel trim
                float trim_roll, trim_pitch;
                if(copter.ins.calibrate_trim(trim_roll, trim_pitch)) {
                    // reset ahrs's trim to suggested values from calibration routine
                    copter.ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
                
            } else if (is_equal(packet.param5,4.0f)) {
                // simple accel calibration
                result = copter.ins.simple_accel_cal(copter.ahrs);

            } else if (is_equal(packet.param6,1.0f)) {
                // compassmot calibration
                result = copter.mavlink_compassmot(chan);
            }
            break;

        case MAV_CMD_COMPONENT_ARM_DISARM:
            if (is_equal(packet.param1,1.0f)) {
                // attempt to arm and return success or failure
                if (copter.init_arm_motors(true)) {
                    result = MAV_RESULT_ACCEPTED;
                }
            } else if (is_zero(packet.param1) && (copter.ap.land_complete || is_equal(packet.param2,21196.0f)))  {
                // force disarming by setting param2 = 21196 is deprecated
                copter.init_disarm_motors();
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_GET_HOME_POSITION:
            if (copter.ap.home_state != HOME_UNSET) {
                send_home(copter.ahrs.get_home());
                Location ekf_origin;
                if (copter.ahrs.get_origin(ekf_origin)) {
                    send_ekf_origin(ekf_origin);
                }
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_FAILED;
            }
            break;

        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if (is_equal(packet.param1,1.0f) || is_equal(packet.param1,3.0f)) {
                AP_Notify::flags.firmware_update = 1;
                copter.notify.update();
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
                    copter.fence.enable(false);
                    break;
                case 1:
                    copter.fence.enable(true);
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

#if PARACHUTE == ENABLED
        case MAV_CMD_DO_PARACHUTE:
            // configure or release parachute
            result = MAV_RESULT_ACCEPTED;
            switch ((uint16_t)packet.param1) {
                case PARACHUTE_DISABLE:
                    copter.parachute.enabled(false);
                    copter.Log_Write_Event(DATA_PARACHUTE_DISABLED);
                    break;
                case PARACHUTE_ENABLE:
                    copter.parachute.enabled(true);
                    copter.Log_Write_Event(DATA_PARACHUTE_ENABLED);
                    break;
                case PARACHUTE_RELEASE:
                    // treat as a manual release which performs some additional check of altitude
                    copter.parachute_manual_release();
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
            // param5 : num_motors (in sequence)
            // param6 : compass learning (0: disabled, 1: enabled)
            result = copter.mavlink_motor_test_start(chan, (uint8_t)packet.param1, (uint8_t)packet.param2, (uint16_t)packet.param3,
                                                     packet.param4, (uint8_t)packet.param5);
            break;

#if GRIPPER_ENABLED == ENABLED
        case MAV_CMD_DO_GRIPPER:
            // param1 : gripper number (ignored)
            // param2 : action (0=release, 1=grab). See GRIPPER_ACTIONS enum.
            if(!copter.g2.gripper.enabled()) {
                result = MAV_RESULT_FAILED;
            } else {
                result = MAV_RESULT_ACCEPTED;
                switch ((uint8_t)packet.param2) {
                    case GRIPPER_ACTION_RELEASE:
                        copter.g2.gripper.release();
                        break;
                    case GRIPPER_ACTION_GRAB:
                        copter.g2.gripper.grab();
                        break;
                    default:
                        result = MAV_RESULT_FAILED;
                        break;
                }
            }
            break;
#endif

#if WINCH_ENABLED == ENABLED
        case MAV_CMD_DO_WINCH:
            // param1 : winch number (ignored)
            // param2 : action (0=relax, 1=relative length control, 2=rate control). See WINCH_ACTIONS enum.
            if (!copter.g2.winch.enabled()) {
                result = MAV_RESULT_FAILED;
            } else {
                result = MAV_RESULT_ACCEPTED;
                switch ((uint8_t)packet.param2) {
                    case WINCH_RELAXED:
                        copter.g2.winch.relax();
                        copter.Log_Write_Event(DATA_WINCH_RELAXED);
                        break;
                    case WINCH_RELATIVE_LENGTH_CONTROL: {
                        copter.g2.winch.release_length(packet.param3, fabsf(packet.param4));
                        copter.Log_Write_Event(DATA_WINCH_LENGTH_CONTROL);
                        break;
                    }
                    case WINCH_RATE_CONTROL: {
                        if (fabsf(packet.param4) <= copter.g2.winch.get_rate_max()) {
                            copter.g2.winch.set_desired_rate(packet.param4);
                            copter.Log_Write_Event(DATA_WINCH_RATE_CONTROL);
                        } else {
                            result = MAV_RESULT_FAILED;
                        }
                        break;
                    }
                    default:
                        result = MAV_RESULT_FAILED;
                        break;
                }
            }
            break;
#endif

        /* Solo user presses Fly button */
        case MAV_CMD_SOLO_BTN_FLY_CLICK: {
            result = MAV_RESULT_ACCEPTED;

            if (copter.failsafe.radio) {
                break;
            }

            // set mode to Loiter or fall back to AltHold
            if (!copter.set_mode(LOITER, MODE_REASON_GCS_COMMAND)) {
                copter.set_mode(ALT_HOLD, MODE_REASON_GCS_COMMAND);
            }
            break;
        }

        /* Solo user holds down Fly button for a couple of seconds */
        case MAV_CMD_SOLO_BTN_FLY_HOLD: {
            result = MAV_RESULT_ACCEPTED;

            if (copter.failsafe.radio) {
                break;
            }

            if (!copter.motors->armed()) {
                // if disarmed, arm motors
                copter.init_arm_motors(true);
            } else if (copter.ap.land_complete) {
                // if armed and landed, takeoff
                if (copter.set_mode(LOITER, MODE_REASON_GCS_COMMAND)) {
                    copter.do_user_takeoff(packet.param1*100, true);
                }
            } else {
                // if flying, land
                copter.set_mode(LAND, MODE_REASON_GCS_COMMAND);
            }
            break;
        }

        /* Solo user presses pause button */
        case MAV_CMD_SOLO_BTN_PAUSE_CLICK: {
            result = MAV_RESULT_ACCEPTED;

            if (copter.failsafe.radio) {
                break;
            }

            if (copter.motors->armed()) {
                if (copter.ap.land_complete) {
                    // if landed, disarm motors
                    copter.init_disarm_motors();
                } else {
                    // assume that shots modes are all done in guided.
                    // NOTE: this may need to change if we add a non-guided shot mode
                    bool shot_mode = (!is_zero(packet.param1) && (copter.control_mode == GUIDED || copter.control_mode == GUIDED_NOGPS));

                    if (!shot_mode) {
                        if (copter.set_mode(BRAKE, MODE_REASON_GCS_COMMAND)) {
                            copter.mode_brake.timeout_to_loiter_ms(2500);
                        } else {
                            copter.set_mode(ALT_HOLD, MODE_REASON_GCS_COMMAND);
                        }
                    } else {
                        // SoloLink is expected to handle pause in shots
                    }
                }
            }
            break;
        }

        case MAV_CMD_ACCELCAL_VEHICLE_POS:
            result = MAV_RESULT_FAILED;

            if (copter.ins.get_acal()->gcs_vehicle_position(packet.param1)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        default:
            result = handle_command_long_message(packet);
            break;
        }

        // send ACK or NAK
        mavlink_msg_command_ack_send_buf(msg, chan, packet.command, result);

        break;
    }

    case MAVLINK_MSG_ID_COMMAND_ACK:        // MAV ID: 77
    {
        copter.command_ack_counter++;
        break;
    }

    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:   // MAV ID: 82
    {
        // decode packet
        mavlink_set_attitude_target_t packet;
        mavlink_msg_set_attitude_target_decode(msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if ((copter.control_mode != GUIDED) && (copter.control_mode != GUIDED_NOGPS) && !(copter.control_mode == AUTO && copter.mode_auto.mode() == Auto_NavGuided)) {
            break;
        }

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
            climb_rate_cms = (packet.thrust - 0.5f) * 2.0f * copter.wp_nav->get_speed_up();
        } else {
            // descend at up to WPNAV_SPEED_DN
            climb_rate_cms = (0.5f - packet.thrust) * 2.0f * -fabsf(copter.wp_nav->get_speed_down());
        }

        // if the body_yaw_rate field is ignored, use the commanded yaw position
        // otherwise use the commanded yaw rate
        bool use_yaw_rate = false;
        if ((packet.type_mask & (1<<2)) == 0) {
            use_yaw_rate = true;
        }

        copter.mode_guided.set_angle(Quaternion(packet.q[0],packet.q[1],packet.q[2],packet.q[3]),
            climb_rate_cms, use_yaw_rate, packet.body_yaw_rate);

        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:     // MAV ID: 84
    {
        // decode packet
        mavlink_set_position_target_local_ned_t packet;
        mavlink_msg_set_position_target_local_ned_decode(msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if ((copter.control_mode != GUIDED) && !(copter.control_mode == AUTO && copter.mode_auto.mode() == Auto_NavGuided)) {
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
        bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
        bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;

        /*
         * for future use:
         * bool force           = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE;
         */

        // prepare position
        Vector3f pos_vector;
        if (!pos_ignore) {
            // convert to cm
            pos_vector = Vector3f(packet.x * 100.0f, packet.y * 100.0f, -packet.z * 100.0f);
            // rotate to body-frame if necessary
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED ||
                packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                copter.rotate_body_frame_to_NE(pos_vector.x, pos_vector.y);
            }
            // add body offset if necessary
            if (packet.coordinate_frame == MAV_FRAME_LOCAL_OFFSET_NED ||
                packet.coordinate_frame == MAV_FRAME_BODY_NED ||
                packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                pos_vector += copter.inertial_nav.get_position();
            } else {
                // convert from alt-above-home to alt-above-ekf-origin
                pos_vector.z = copter.pv_alt_above_origin(pos_vector.z);
            }
        }

        // prepare velocity
        Vector3f vel_vector;
        if (!vel_ignore) {
            // convert to cm
            vel_vector = Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f);
            // rotate to body-frame if necessary
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                copter.rotate_body_frame_to_NE(vel_vector.x, vel_vector.y);
            }
        }

        // prepare yaw
        float yaw_cd = 0.0f;
        bool yaw_relative = false;
        float yaw_rate_cds = 0.0f;
        if (!yaw_ignore) {
            yaw_cd = ToDeg(packet.yaw) * 100.0f;
            yaw_relative = packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED;
        }
        if (!yaw_rate_ignore) {
            yaw_rate_cds = ToDeg(packet.yaw_rate) * 100.0f;
        }

        // send request
        if (!pos_ignore && !vel_ignore && acc_ignore) {
            if (copter.mode_guided.set_destination_posvel(pos_vector, vel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative)) {
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_FAILED;
            }
        } else if (pos_ignore && !vel_ignore && acc_ignore) {
            copter.mode_guided.set_velocity(vel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
            result = MAV_RESULT_ACCEPTED;
        } else if (!pos_ignore && vel_ignore && acc_ignore) {
            if (copter.mode_guided.set_destination(pos_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative)) {
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_FAILED;
            }
        } else {
            result = MAV_RESULT_FAILED;
        }

        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:    // MAV ID: 86
    {
        // decode packet
        mavlink_set_position_target_global_int_t packet;
        mavlink_msg_set_position_target_global_int_decode(msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if ((copter.control_mode != GUIDED) && !(copter.control_mode == AUTO && copter.mode_auto.mode() == Auto_NavGuided)) {
            break;
        }

        // check for supported coordinate frames
        if (packet.coordinate_frame != MAV_FRAME_GLOBAL &&
            packet.coordinate_frame != MAV_FRAME_GLOBAL_INT &&
            packet.coordinate_frame != MAV_FRAME_GLOBAL_RELATIVE_ALT && // solo shot manager incorrectly sends RELATIVE_ALT instead of RELATIVE_ALT_INT
            packet.coordinate_frame != MAV_FRAME_GLOBAL_RELATIVE_ALT_INT &&
            packet.coordinate_frame != MAV_FRAME_GLOBAL_TERRAIN_ALT &&
            packet.coordinate_frame != MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) {
            break;
        }

        bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
        bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
        bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
        bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;

        /*
         * for future use:
         * bool force           = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE;
         */

        Vector3f pos_neu_cm;  // position (North, East, Up coordinates) in centimeters

        if(!pos_ignore) {
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
                case MAV_FRAME_GLOBAL_TERRAIN_ALT:
                case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
                    loc.flags.relative_alt = true;
                    loc.flags.terrain_alt = true;
                    break;
                case MAV_FRAME_GLOBAL:
                case MAV_FRAME_GLOBAL_INT:
                default:
                    // pv_location_to_vector does not support absolute altitudes.
                    // Convert the absolute altitude to a home-relative altitude before calling pv_location_to_vector
                    loc.alt -= copter.ahrs.get_home().alt;
                    loc.flags.relative_alt = true;
                    loc.flags.terrain_alt = false;
                    break;
            }
            pos_neu_cm = copter.pv_location_to_vector(loc);
        }

        // prepare yaw
        float yaw_cd = 0.0f;
        bool yaw_relative = false;
        float yaw_rate_cds = 0.0f;
        if (!yaw_ignore) {
            yaw_cd = ToDeg(packet.yaw) * 100.0f;
            yaw_relative = packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED;
        }
        if (!yaw_rate_ignore) {
            yaw_rate_cds = ToDeg(packet.yaw_rate) * 100.0f;
        }

        if (!pos_ignore && !vel_ignore && acc_ignore) {
            if (copter.mode_guided.set_destination_posvel(pos_neu_cm, Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f), !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative)) {
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_FAILED;
            }
        } else if (pos_ignore && !vel_ignore && acc_ignore) {
            copter.mode_guided.set_velocity(Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f), !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
            result = MAV_RESULT_ACCEPTED;
        } else if (!pos_ignore && vel_ignore && acc_ignore) {
            if (copter.mode_guided.set_destination(pos_neu_cm, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative)) {
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_FAILED;
            }
        } else {
            result = MAV_RESULT_FAILED;
        }

        break;
    }

    case MAVLINK_MSG_ID_DISTANCE_SENSOR:
    {
        result = MAV_RESULT_ACCEPTED;
        copter.rangefinder.handle_msg(msg);
#if PROXIMITY_ENABLED == ENABLED
        copter.g2.proximity.handle_msg(msg);
#endif
        break;
    }

#if HIL_MODE != HIL_MODE_DISABLED
    case MAVLINK_MSG_ID_HIL_STATE:          // MAV ID: 90
    {
        mavlink_hil_state_t packet;
        mavlink_msg_hil_state_decode(msg, &packet);

        // sanity check location
        if (!check_latlng(packet.lat, packet.lon)) {
            break;
        }

        // set gps hil sensor
        Location loc;
        loc.lat = packet.lat;
        loc.lng = packet.lon;
        loc.alt = packet.alt/10;
        Vector3f vel(packet.vx, packet.vy, packet.vz);
        vel *= 0.01f;

        gps.setHIL(0, AP_GPS::GPS_OK_FIX_3D,
                   packet.time_usec/1000,
                   loc, vel, 10, 0);

        // rad/sec
        Vector3f gyros;
        gyros.x = packet.rollspeed;
        gyros.y = packet.pitchspeed;
        gyros.z = packet.yawspeed;

        // m/s/s
        Vector3f accels;
        accels.x = packet.xacc * (GRAVITY_MSS/1000.0f);
        accels.y = packet.yacc * (GRAVITY_MSS/1000.0f);
        accels.z = packet.zacc * (GRAVITY_MSS/1000.0f);

        ins.set_gyro(0, gyros);

        ins.set_accel(0, accels);

        copter.barometer.setHIL(packet.alt*0.001f);
        copter.compass.setHIL(0, packet.roll, packet.pitch, packet.yaw);
        copter.compass.setHIL(1, packet.roll, packet.pitch, packet.yaw);

        break;
    }
#endif //  HIL_MODE != HIL_MODE_DISABLED

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS:       // MAV ID: 109
    {
        handle_radio_status(msg, copter.DataFlash, copter.should_log(MASK_LOG_PM));
        break;
    }

#if PRECISION_LANDING == ENABLED
    case MAVLINK_MSG_ID_LANDING_TARGET:
        result = MAV_RESULT_ACCEPTED;
        copter.precland.handle_msg(msg);
        break;
#endif

#if AC_FENCE == ENABLED
    // send or receive fence points with GCS
    case MAVLINK_MSG_ID_FENCE_POINT:            // MAV ID: 160
    case MAVLINK_MSG_ID_FENCE_FETCH_POINT:
        copter.fence.handle_msg(*this, msg);
        break;
#endif // AC_FENCE == ENABLED

#if MOUNT == ENABLED
    //deprecated. Use MAV_CMD_DO_MOUNT_CONFIGURE
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:        // MAV ID: 204
        copter.camera_mount.configure_msg(msg);
        break;
    //deprecated. Use MAV_CMD_DO_MOUNT_CONTROL
    case MAVLINK_MSG_ID_MOUNT_CONTROL:
        if(!copter.camera_mount.has_pan_control()) {
            copter.set_auto_yaw_look_at_heading(mavlink_msg_mount_control_get_input_c(msg)/100.0f, 0.0f, 0, 0);
        }
        copter.camera_mount.control_msg(msg);
        break;
#endif // MOUNT == ENABLED

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
        copter.terrain.handle_data(chan, msg);
#endif
        break;

    case MAVLINK_MSG_ID_SET_HOME_POSITION:
    {
        mavlink_set_home_position_t packet;
        mavlink_msg_set_home_position_decode(msg, &packet);
        if((packet.latitude == 0) && (packet.longitude == 0) && (packet.altitude == 0)) {
            copter.set_home_to_current_location(true);
        } else {
            // sanity check location
            if (!check_latlng(packet.latitude, packet.longitude)) {
                break;
            }
            Location new_home_loc;
            new_home_loc.lat = packet.latitude;
            new_home_loc.lng = packet.longitude;
            new_home_loc.alt = packet.altitude / 10;
            copter.set_home(new_home_loc, true);
        }
        break;
    }

    case MAVLINK_MSG_ID_ADSB_VEHICLE:
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG:
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC:
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT:
#if ADSB_ENABLED == ENABLED
        copter.adsb.handle_message(chan, msg);
#endif
        break;

    case MAVLINK_MSG_ID_VISION_POSITION_DELTA:
#if VISUAL_ODOMETRY_ENABLED == ENABLED
        copter.g2.visual_odom.handle_msg(msg);
#endif
        break;

#if TOY_MODE_ENABLED == ENABLED
    case MAVLINK_MSG_ID_NAMED_VALUE_INT:
        copter.g2.toy_mode.handle_message(msg);
        break;
#endif
        
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
void Copter::mavlink_delay_cb()
{
    static uint32_t last_1hz, last_50hz, last_5s;
    if (!gcs().chan(0).initialised || in_mavlink_delay) return;

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
    check_usb_mux();

    DataFlash.EnableWrites(true);
    in_mavlink_delay = false;
}

/*
 *  send data streams in the given rate range on both links
 */
void Copter::gcs_data_stream_send(void)
{
    gcs().data_stream_send();
}

/*
 *  look for incoming commands on the GCS links
 */
void Copter::gcs_check_input(void)
{
    gcs().update();
}

/*
  return true if we will accept this packet. Used to implement SYSID_ENFORCE
 */
bool GCS_MAVLINK_Copter::accept_packet(const mavlink_status_t &status, mavlink_message_t &msg)
{
    if (!copter.g2.sysid_enforce) {
        return true;
    }
    if (msg.msgid == MAVLINK_MSG_ID_RADIO || msg.msgid == MAVLINK_MSG_ID_RADIO_STATUS) {
        return true;
    }
    return (msg.sysid == copter.g.sysid_my_gcs);
}

AP_Mission *GCS_MAVLINK_Copter::get_mission()
{
    return &copter.mission;
}

Compass *GCS_MAVLINK_Copter::get_compass() const
{
    return &copter.compass;
}

AP_Camera *GCS_MAVLINK_Copter::get_camera() const
{
#if CAMERA == ENABLED
    return &copter.camera;
#else
    return nullptr;
#endif
}

AP_ServoRelayEvents *GCS_MAVLINK_Copter::get_servorelayevents() const
{
    return &copter.ServoRelayEvents;
}

AP_AdvancedFailsafe *GCS_MAVLINK_Copter::get_advanced_failsafe() const
{
#if ADVANCED_FAILSAFE == ENABLED
    return &copter.g2.afs;
#else
    return nullptr;
#endif
}

MAV_RESULT GCS_MAVLINK_Copter::handle_flight_termination(const mavlink_command_long_t &packet) {
    MAV_RESULT result = MAV_RESULT_FAILED;

#if ADVANCED_FAILSAFE == ENABLED
    if (GCS_MAVLINK::handle_flight_termination(packet) != MAV_RESULT_ACCEPTED) {
#endif
        if (packet.param1 > 0.5f) {
            copter.init_disarm_motors();
            result = MAV_RESULT_ACCEPTED;
        }
#if ADVANCED_FAILSAFE == ENABLED
    } else {
        result = MAV_RESULT_ACCEPTED;
    }
#endif

    return result;
}

AP_Rally *GCS_MAVLINK_Copter::get_rally() const
{
#if AC_RALLY == ENABLED
    return &copter.rally;
#else
    return nullptr;
#endif
}

bool GCS_MAVLINK_Copter::set_mode(const uint8_t mode)
{
#ifdef DISALLOW_GCS_MODE_CHANGE_DURING_RC_FAILSAFE
    if (copter.failsafe.radio) {
        // don't allow mode changes while in radio failsafe
        return false;
    }
#endif
    return copter.set_mode((control_mode_t)mode, MODE_REASON_GCS_COMMAND);
}

const AP_FWVersion &GCS_MAVLINK_Copter::get_fwver() const
{
    return copter.fwver;
}


void GCS_MAVLINK_Copter::set_ekf_origin(const Location& loc)
{
    copter.set_ekf_origin(loc);
}
