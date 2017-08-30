#include "Rover.h"
#include "version.h"

#include "GCS_Mavlink.h"

#include <AP_RangeFinder/RangeFinder_Backend.h>

void Rover::send_heartbeat(mavlink_channel_t chan)
{
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = MAV_STATE_ACTIVE;

    if (failsafe.triggered != 0) {
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
    if (control_mode->has_manual_input()) {
        base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }

    if (control_mode->is_autopilot_mode()) {
        base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
    }

#if defined(ENABLE_STICK_MIXING) && (ENABLE_STICK_MIXING == ENABLED) // TODO ???? Remove !
    if (control_mode->stick_mixing_enabled()) {
        // all modes except INITIALISING have some form of manual
        // override if stick mixing is enabled
        base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }
#endif

#if HIL_MODE != HIL_MODE_DISABLED
    base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
#endif
    if (control_mode == &mode_initializing) {
        system_status = MAV_STATE_CALIBRATING;
    }
    if (control_mode == &mode_hold) {
        system_status = MAV_STATE_STANDBY;
    }
    // we are armed if we are not initialising
    if (control_mode != &mode_initializing && arming.is_armed()) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    gcs().chan(chan-MAVLINK_COMM_0).send_heartbeat(MAV_TYPE_GROUND_ROVER,
                                            base_mode,
                                            control_mode->mode_number(),
                                            system_status);
}

void Rover::send_attitude(mavlink_channel_t chan)
{
    const Vector3f omega = ahrs.get_gyro();
    mavlink_msg_attitude_send(
        chan,
        millis(),
        ahrs.roll,
        ahrs.pitch,
        ahrs.yaw,
        omega.x,
        omega.y,
        omega.z);
}

void Rover::send_extended_status1(mavlink_channel_t chan)
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
        static_cast<uint16_t>(scheduler.load_average() * 1000),
        battery.voltage() * 1000,  // mV
        battery_current,        // in 10mA units
        battery_remaining,      // in %
        0,  // comm drops %,
        0,  // comm drops in pkts,
        0, 0, 0, 0);
}

void Rover::send_location(mavlink_channel_t chan)
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
    const Vector3f &vel = gps.velocity();
    mavlink_msg_global_position_int_send(
        chan,
        fix_time,
        current_loc.lat,                    // in 1E7 degrees
        current_loc.lng,                    // in 1E7 degrees
        current_loc.alt * 10UL,             // millimeters above sea level
        (current_loc.alt - home.alt) * 10,  // millimeters above ground
        vel.x * 100,   // X speed cm/s (+ve North)
        vel.y * 100,   // Y speed cm/s (+ve East)
        vel.z * -100,  // Z speed cm/s (+ve up)
        ahrs.yaw_sensor);
}

void Rover::send_nav_controller_output(mavlink_channel_t chan)
{
    mavlink_msg_nav_controller_output_send(
        chan,
        control_mode->lateral_acceleration,  // use nav_roll to hold demanded Y accel
        ahrs.groundspeed() * ins.get_gyro().z,  // use nav_pitch to hold actual Y accel
        nav_controller->nav_bearing_cd() * 0.01f,
        nav_controller->target_bearing_cd() * 0.01f,
        MIN(control_mode->get_distance_to_destination(), UINT16_MAX),
        0,
        control_mode->speed_error(),
        nav_controller->crosstrack_error());
}

void Rover::send_servo_out(mavlink_channel_t chan)
{
#if HIL_MODE != HIL_MODE_DISABLED
    // normalized values scaled to -10000 to 10000
    // This is used for HIL.  Do not change without discussing with
    // HIL maintainers
    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0,  // port 0
        10000 * channel_steer->norm_output(),
        0,
        g2.motors.get_throttle(),
        0,
        0,
        0,
        0,
        0,
        receiver_rssi);
#endif
}

void Rover::send_vfr_hud(mavlink_channel_t chan)
{
    mavlink_msg_vfr_hud_send(
        chan,
        gps.ground_speed(),
        ahrs.groundspeed(),
        (ahrs.yaw_sensor / 100) % 360,
        g2.motors.get_throttle(),
        current_loc.alt / 100.0f,
        0);
}

// report simulator state
void Rover::send_simstate(mavlink_channel_t chan)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    sitl.simstate_send(chan);
#endif
}

void Rover::send_rangefinder(mavlink_channel_t chan)
{
    float distance_cm;
    float voltage;
    bool got_one = false;

    // report smaller distance of all rangefinders
    for (uint8_t i=0; i<rangefinder.num_sensors(); i++) {
        AP_RangeFinder_Backend *s = rangefinder.get_backend(i);
        if (s == nullptr) {
            continue;
        }
        if (!got_one ||
            s->distance_cm() < distance_cm) {
            distance_cm = s->distance_cm();
            voltage = s->voltage_mv();
            got_one = true;
        }
    }
    if (!got_one) {
        // no relevant data found
        return;
    }

    mavlink_msg_rangefinder_send(
        chan,
        distance_cm * 0.01f,
        voltage);
}

/*
  send PID tuning message
 */
void Rover::send_pid_tuning(mavlink_channel_t chan)
{
    const Vector3f &gyro = ahrs.get_gyro();
    const DataFlash_Class::PID_Info *pid_info;
    if (g.gcs_pid_mask & 1) {
        pid_info = &g2.attitude_control.get_steering_rate_pid().get_pid_info();
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
    if (g.gcs_pid_mask & 2) {
        pid_info = &g2.attitude_control.get_throttle_speed_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_ACCZ,
                                    pid_info->desired,
                                    0,
                                    0,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
}

void Rover::send_wheel_encoder(mavlink_channel_t chan)
{
    // send wheel encoder data using rpm message
    if (g2.wheel_encoder.enabled(0) || g2.wheel_encoder.enabled(1)) {
        mavlink_msg_rpm_send(chan, wheel_encoder_rpm[0], wheel_encoder_rpm[1]);
    }
}

uint8_t GCS_MAVLINK_Rover::sysid_my_gcs() const
{
    return rover.g.sysid_my_gcs;
}

uint32_t GCS_MAVLINK_Rover::telem_delay() const
{
    return static_cast<uint32_t>(rover.g.telem_delay);
}

// try to send a message, return false if it won't fit in the serial tx buffer
bool GCS_MAVLINK_Rover::try_send_message(enum ap_message id)
{
    if (telemetry_delayed()) {
        return false;
    }

    // if we don't have at least 1ms remaining before the main loop
    // wants to fire then don't send a mavlink message. We want to
    // prioritise the main flight control loop over communications
    if (!rover.in_mavlink_delay && rover.scheduler.time_available_usec() < 1200) {
        rover.gcs_out_of_time = true;
        return false;
    }

    switch (id) {
    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        last_heartbeat_time = AP_HAL::millis();
        rover.send_heartbeat(chan);
        return true;

    case MSG_EXTENDED_STATUS1:
        // send extended status only once vehicle has been initialised
        // to avoid unnecessary errors being reported to user
        if (initialised) {
            CHECK_PAYLOAD_SIZE(SYS_STATUS);
            rover.send_extended_status1(chan);
            CHECK_PAYLOAD_SIZE(POWER_STATUS);
            send_power_status();
        }
        break;

    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        rover.send_attitude(chan);
        break;

    case MSG_LOCATION:
        CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
        rover.send_location(chan);
        break;

    case MSG_LOCAL_POSITION:
        CHECK_PAYLOAD_SIZE(LOCAL_POSITION_NED);
        send_local_position(rover.ahrs);
        break;

    case MSG_NAV_CONTROLLER_OUTPUT:
        if (rover.control_mode->is_autopilot_mode()) {
            CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
            rover.send_nav_controller_output(chan);
        }
        break;

    case MSG_SERVO_OUT:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_SCALED);
        rover.send_servo_out(chan);
        break;

    case MSG_RADIO_IN:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS);
        send_radio_in(rover.receiver_rssi);
        break;

    case MSG_SERVO_OUTPUT_RAW:
        CHECK_PAYLOAD_SIZE(SERVO_OUTPUT_RAW);
        send_servo_output_raw(false);
        break;

    case MSG_VFR_HUD:
        CHECK_PAYLOAD_SIZE(VFR_HUD);
        rover.send_vfr_hud(chan);
        break;

    case MSG_RAW_IMU1:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
        send_raw_imu(rover.ins, rover.compass);
        break;

    case MSG_RAW_IMU3:
        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
        send_sensor_offsets(rover.ins, rover.compass, rover.barometer);
        break;

    case MSG_AHRS:
        CHECK_PAYLOAD_SIZE(AHRS);
        send_ahrs(rover.ahrs);
        break;

    case MSG_SIMSTATE:
        CHECK_PAYLOAD_SIZE(SIMSTATE);
        rover.send_simstate(chan);
        break;

    case MSG_RANGEFINDER:
        CHECK_PAYLOAD_SIZE(RANGEFINDER);
        rover.send_rangefinder(chan);
        send_distance_sensor(rover.rangefinder);
        break;

    case MSG_RPM:
        CHECK_PAYLOAD_SIZE(RPM);
        rover.send_wheel_encoder(chan);
        break;

    case MSG_MOUNT_STATUS:
#if MOUNT == ENABLED
        CHECK_PAYLOAD_SIZE(MOUNT_STATUS);
        rover.camera_mount.status_msg(chan);
#endif  // MOUNT == ENABLED
        break;

    case MSG_VIBRATION:
        CHECK_PAYLOAD_SIZE(VIBRATION);
        send_vibration(rover.ins);
        break;

    case MSG_BATTERY2:
        CHECK_PAYLOAD_SIZE(BATTERY2);
        send_battery2(rover.battery);
        break;

    case MSG_EKF_STATUS_REPORT:
#if AP_AHRS_NAVEKF_AVAILABLE
        CHECK_PAYLOAD_SIZE(EKF_STATUS_REPORT);
        rover.ahrs.send_ekf_status_report(chan);
#endif
        break;

    case MSG_PID_TUNING:
        CHECK_PAYLOAD_SIZE(PID_TUNING);
        rover.send_pid_tuning(chan);
        break;

    case MSG_BATTERY_STATUS:
        send_battery_status(rover.battery);
        break;

    default:
        return GCS_MAVLINK::try_send_message(id);
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

void
GCS_MAVLINK_Rover::data_stream_send(void)
{
    rover.gcs_out_of_time = false;

    if (!rover.in_mavlink_delay) {
        rover.DataFlash.handle_log_send(*this);
    }

    send_queued_parameters();

    if (rover.gcs_out_of_time) {
      return;
    }

    if (rover.in_mavlink_delay) {
#if HIL_MODE != HIL_MODE_DISABLED
        // in HIL we need to keep sending servo values to ensure
        // the simulator doesn't pause, otherwise our sensor
        // calibration could stall
        if (stream_trigger(STREAM_RAW_CONTROLLER)) {
            send_message(MSG_SERVO_OUT);
        }
        if (stream_trigger(STREAM_RC_CHANNELS)) {
            send_message(MSG_SERVO_OUTPUT_RAW);
        }
#endif
        // don't send any other stream types while in the delay callback
        return;
    }

    if (rover.gcs_out_of_time) {
      return;
    }

    if (stream_trigger(STREAM_RAW_SENSORS)) {
        send_message(MSG_RAW_IMU1);
        send_message(MSG_RAW_IMU3);
    }

    if (rover.gcs_out_of_time) {
      return;
    }

    if (stream_trigger(STREAM_EXTENDED_STATUS)) {
        send_message(MSG_EXTENDED_STATUS1);
        send_message(MSG_EXTENDED_STATUS2);
        send_message(MSG_CURRENT_WAYPOINT);
        send_message(MSG_GPS_RAW);
        send_message(MSG_GPS_RTK);
        send_message(MSG_GPS2_RAW);
        send_message(MSG_GPS2_RTK);
        send_message(MSG_NAV_CONTROLLER_OUTPUT);
    }

    if (rover.gcs_out_of_time) {
      return;
    }

    if (stream_trigger(STREAM_POSITION)) {
        // sent with GPS read
        send_message(MSG_LOCATION);
        send_message(MSG_LOCAL_POSITION);
    }

    if (rover.gcs_out_of_time) {
      return;
    }

    if (stream_trigger(STREAM_RAW_CONTROLLER)) {
        send_message(MSG_SERVO_OUT);
    }

    if (rover.gcs_out_of_time) {
      return;
    }

    if (stream_trigger(STREAM_RC_CHANNELS)) {
        send_message(MSG_SERVO_OUTPUT_RAW);
        send_message(MSG_RADIO_IN);
    }

    if (rover.gcs_out_of_time) {
      return;
    }

    if (stream_trigger(STREAM_EXTRA1)) {
        send_message(MSG_ATTITUDE);
        send_message(MSG_SIMSTATE);
        if (rover.control_mode->is_autopilot_mode()) {
            send_message(MSG_PID_TUNING);
        }
    }

    if (rover.gcs_out_of_time) {
      return;
    }

    if (stream_trigger(STREAM_EXTRA2)) {
        send_message(MSG_VFR_HUD);
    }

    if (rover.gcs_out_of_time) {
      return;
    }

    if (stream_trigger(STREAM_EXTRA3)) {
        send_message(MSG_AHRS);
        send_message(MSG_HWSTATUS);
        send_message(MSG_RANGEFINDER);
        send_message(MSG_SYSTEM_TIME);
        send_message(MSG_BATTERY2);
        send_message(MSG_BATTERY_STATUS);
        send_message(MSG_MAG_CAL_REPORT);
        send_message(MSG_MAG_CAL_PROGRESS);
        send_message(MSG_MOUNT_STATUS);
        send_message(MSG_EKF_STATUS_REPORT);
        send_message(MSG_VIBRATION);
        send_message(MSG_RPM);
    }
}



bool GCS_MAVLINK_Rover::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
    if (rover.control_mode != &rover.mode_guided) {
        // only accept position updates when in GUIDED mode
        return false;
    }

    // make any new wp uploaded instant (in case we are already in Guided mode)
    rover.mode_guided.set_desired_location(cmd.content.location);
    return true;
}

void GCS_MAVLINK_Rover::handle_change_alt_request(AP_Mission::Mission_Command &cmd)
{
    // nothing to do
}

void GCS_MAVLINK_Rover::handleMessage(mavlink_message_t* msg)
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
        {
            handle_request_data_stream(msg, true);
            break;
        }

    case MAVLINK_MSG_ID_COMMAND_INT: {
        // decode packet
        mavlink_command_int_t packet;
        mavlink_msg_command_int_decode(msg, &packet);
        uint8_t result = MAV_RESULT_UNSUPPORTED;

        switch (packet.command) {
#if MOUNT == ENABLED
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
            if (roi_loc.lat == 0 && roi_loc.lng == 0 && roi_loc.alt == 0) {
                // switch off the camera tracking if enabled
                if (rover.camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
                    rover.camera_mount.set_mode_to_default();
                }
            } else {
                // send the command to the camera mount
                rover.camera_mount.set_roi_target(roi_loc);
            }
            result = MAV_RESULT_ACCEPTED;
            break;
        }
#endif

        default:
            result = MAV_RESULT_UNSUPPORTED;
            break;
        }

        // send ACK or NAK
        mavlink_msg_command_ack_send_buf(msg, chan, packet.command, result);
        break;
    }

    case MAVLINK_MSG_ID_COMMAND_LONG:
        {
            // decode
            mavlink_command_long_t packet;
            mavlink_msg_command_long_decode(msg, &packet);

            uint8_t result = MAV_RESULT_UNSUPPORTED;

            // do command

            switch (packet.command) {

            case MAV_CMD_NAV_RETURN_TO_LAUNCH:
                rover.set_mode(rover.mode_rtl, MODE_REASON_GCS_COMMAND);
                result = MAV_RESULT_ACCEPTED;
                break;

#if MOUNT == ENABLED
            // Sets the region of interest (ROI) for the camera
            case MAV_CMD_DO_SET_ROI:
                // sanity check location
                if (!check_latlng(packet.param5, packet.param6)) {
                    break;
                }
                Location roi_loc;
                roi_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
                roi_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
                roi_loc.alt = (int32_t)(packet.param7 * 100.0f);
                if (roi_loc.lat == 0 && roi_loc.lng == 0 && roi_loc.alt == 0) {
                    // switch off the camera tracking if enabled
                    if (rover.camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
                        rover.camera_mount.set_mode_to_default();
                    }
                } else {
                    // send the command to the camera mount
                    rover.camera_mount.set_roi_target(roi_loc);
                }
                result = MAV_RESULT_ACCEPTED;
                break;
#endif

            case MAV_CMD_DO_MOUNT_CONTROL:
#if MOUNT == ENABLED
                rover.camera_mount.control(packet.param1, packet.param2, packet.param3, (MAV_MOUNT_MODE) packet.param7);
                result = MAV_RESULT_ACCEPTED;
#endif
                break;

            case MAV_CMD_MISSION_START:
                rover.set_mode(rover.mode_auto, MODE_REASON_GCS_COMMAND);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_CMD_PREFLIGHT_CALIBRATION:
                if (hal.util->get_soft_armed()) {
                    result = MAV_RESULT_FAILED;
                    break;
                }
                if (is_equal(packet.param1, 1.0f)) {
                    rover.ins.init_gyro();
                    if (rover.ins.gyro_calibrated_ok_all()) {
                        rover.ahrs.reset_gyro_drift();
                        result = MAV_RESULT_ACCEPTED;
                    } else {
                        result = MAV_RESULT_FAILED;
                    }
                } else if (is_equal(packet.param3, 1.0f)) {
                    rover.init_barometer(false);
                    result = MAV_RESULT_ACCEPTED;
                } else if (is_equal(packet.param4, 1.0f)) {
                    rover.trim_radio();
                    result = MAV_RESULT_ACCEPTED;
                } else if (is_equal(packet.param5, 1.0f)) {
                    result = MAV_RESULT_ACCEPTED;
                    // start with gyro calibration
                    rover.ins.init_gyro();
                    // reset ahrs gyro bias
                    if (rover.ins.gyro_calibrated_ok_all()) {
                        rover.ahrs.reset_gyro_drift();
                    } else {
                        result = MAV_RESULT_FAILED;
                    }
                    rover.ins.acal_init();
                    rover.ins.get_acal()->start(this);

                } else if (is_equal(packet.param5, 2.0f)) {
                    // start with gyro calibration
                    rover.ins.init_gyro();
                    // accel trim
                    float trim_roll, trim_pitch;
                    if (rover.ins.calibrate_trim(trim_roll, trim_pitch)) {
                        // reset ahrs's trim to suggested values from calibration routine
                        rover.ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
                        result = MAV_RESULT_ACCEPTED;
                    } else {
                        result = MAV_RESULT_FAILED;
                    }
                } else {
                    send_text(MAV_SEVERITY_WARNING, "Unsupported preflight calibration");
                }
                break;

        case MAV_CMD_DO_SET_MODE:
            switch (static_cast<uint16_t>(packet.param1)) {
            case MAV_MODE_MANUAL_ARMED:
            case MAV_MODE_MANUAL_DISARMED:
                rover.set_mode(rover.mode_manual, MODE_REASON_GCS_COMMAND);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_MODE_AUTO_ARMED:
            case MAV_MODE_AUTO_DISARMED:
                rover.set_mode(rover.mode_auto, MODE_REASON_GCS_COMMAND);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_MODE_STABILIZE_DISARMED:
            case MAV_MODE_STABILIZE_ARMED:
                rover.set_mode(rover.mode_steering, MODE_REASON_GCS_COMMAND);
                result = MAV_RESULT_ACCEPTED;
                break;

            default:
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if (is_equal(packet.param1, 1.0f) || is_equal(packet.param1, 3.0f)) {
                // when packet.param1 == 3 we reboot to hold in bootloader
                hal.scheduler->reboot(is_equal(packet.param1, 3.0f));
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_COMPONENT_ARM_DISARM:
            if (is_equal(packet.param1, 1.0f)) {
                // run pre_arm_checks and arm_checks and display failures
                if (rover.arm_motors(AP_Arming::MAVLINK)) {
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            } else if (is_zero(packet.param1))  {
                if (rover.disarm_motors()) {
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            } else {
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_GET_HOME_POSITION:
            if (rover.home_is_set != HOME_UNSET) {
                send_home(rover.ahrs.get_home());
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_FAILED;
            }
            break;

        case MAV_CMD_DO_SET_HOME:
        {
            // param1 : use current (1=use current location, 0=use specified location)
            // param5 : latitude
            // param6 : longitude
            // param7 : altitude
            result = MAV_RESULT_FAILED;  // assume failure
            if (is_equal(packet.param1, 1.0f)) {
                if (rover.set_home_to_current_location(true)) {
                    result = MAV_RESULT_ACCEPTED;
                }
            } else {
                Location new_home_loc {};
                new_home_loc.lat = static_cast<int32_t>(packet.param5 * 1.0e7f);
                new_home_loc.lng = static_cast<int32_t>(packet.param6 * 1.0e7f);
                new_home_loc.alt = static_cast<int32_t>(packet.param7 * 100.0f);
                if (rover.set_home(new_home_loc, true)) {
                    result = MAV_RESULT_ACCEPTED;
                }
            }
            break;
        }

        case MAV_CMD_NAV_SET_YAW_SPEED:
        {
            // param1 : yaw angle to adjust direction by in centidegress
            // param2 : Speed - normalized to 0 .. 1

            // exit if vehicle is not in Guided mode
            if (rover.control_mode != &rover.mode_guided) {
                break;
            }

            // send yaw change and target speed to guided mode controller
            float target_speed = constrain_float(packet.param2 * rover.g.speed_cruise, -rover.g.speed_cruise, rover.g.speed_cruise);
            rover.mode_guided.set_desired_heading_delta_and_speed(packet.param1, target_speed);
            result = MAV_RESULT_ACCEPTED;
            break;
        }

        case MAV_CMD_ACCELCAL_VEHICLE_POS:
            result = MAV_RESULT_FAILED;

            if (rover.ins.get_acal()->gcs_vehicle_position(packet.param1)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_MOTOR_TEST:
            // param1 : motor sequence number (a number from 1 to max number of motors on the vehicle)
            // param2 : throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
            // param3 : throttle (range depends upon param2)
            // param4 : timeout (in seconds)
            result = rover.mavlink_motor_test_start(chan, static_cast<uint8_t>(packet.param1),
                                                    static_cast<uint8_t>(packet.param2),
                                                    static_cast<int16_t>(packet.param3),
                                                    packet.param4);
            break;

        default:
            result = handle_command_long_message(packet);
                break;
            }

            mavlink_msg_command_ack_send_buf(
                msg,
                chan,
                packet.command,
                result);

            break;
        }

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
    {
        // allow override of RC channel values for HIL
        // or for complete GCS control of switch position
        // and RC PWM values.
        if (msg->sysid != rover.g.sysid_my_gcs) {  // Only accept control from our gcs
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

        hal.rcin->set_overrides(v, 8);

        rover.failsafe.rc_override_timer = AP_HAL::millis();
        rover.failsafe_trigger(FAILSAFE_EVENT_RC, false);
        break;
    }

    case MAVLINK_MSG_ID_HEARTBEAT:
        {
            // We keep track of the last time we received a heartbeat from our GCS for failsafe purposes
            if (msg->sysid != rover.g.sysid_my_gcs) {
                break;
            }

            rover.last_heartbeat_ms = rover.failsafe.rc_override_timer = AP_HAL::millis();
            rover.failsafe_trigger(FAILSAFE_EVENT_GCS, false);
            break;
        }

    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:   // MAV ID: 82
        {
            // decode packet
            mavlink_set_attitude_target_t packet;
            mavlink_msg_set_attitude_target_decode(msg, &packet);

            // exit if vehicle is not in Guided mode
            if (rover.control_mode != &rover.mode_guided) {
                break;
            }

            // ensure type_mask specifies to use thrust
            if ((packet.type_mask & MAVLINK_SET_ATT_TYPE_MASK_THROTTLE_IGNORE) != 0) {
                break;
            }

            // convert thrust to ground speed
            packet.thrust = constrain_float(packet.thrust, -1.0f, 1.0f);
            float target_speed = rover.g.speed_cruise * packet.thrust;

            // if the body_yaw_rate field is ignored, convert quaternion to heading
            if ((packet.type_mask & MAVLINK_SET_ATT_TYPE_MASK_YAW_RATE_IGNORE) != 0) {
                // convert quaternion to heading
                float target_heading_cd = degrees(Quaternion(packet.q[0], packet.q[1], packet.q[2], packet.q[3]).get_euler_yaw()) * 100.0f;
                rover.mode_guided.set_desired_heading_and_speed(target_heading_cd, target_speed);
            } else {
                // use body_yaw_rate field
                rover.mode_guided.set_desired_turn_rate_and_speed((RAD_TO_DEG * packet.body_yaw_rate) * 100.0f, target_speed);
            }
            break;
        }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:     // MAV ID: 84
        {
            // decode packet
            mavlink_set_position_target_local_ned_t packet;
            mavlink_msg_set_position_target_local_ned_decode(msg, &packet);

            // exit if vehicle is not in Guided mode
            if (rover.control_mode != &rover.mode_guided) {
                break;
            }

            // check for supported coordinate frames
            if (packet.coordinate_frame != MAV_FRAME_LOCAL_NED &&
                packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED &&
                packet.coordinate_frame != MAV_FRAME_BODY_NED &&
                packet.coordinate_frame != MAV_FRAME_BODY_OFFSET_NED) {
                break;
            }

            bool pos_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
            bool vel_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
            bool acc_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
            bool yaw_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
            bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;

            // prepare target position
            Location target_loc = rover.current_loc;
            if (!pos_ignore) {
                switch (packet.coordinate_frame) {
                case MAV_FRAME_BODY_NED:
                case MAV_FRAME_BODY_OFFSET_NED: {
                    // rotate from body-frame to NE frame
                    const float ne_x = packet.x * rover.ahrs.cos_yaw() - packet.y * rover.ahrs.sin_yaw();
                    const float ne_y = packet.x * rover.ahrs.sin_yaw() + packet.y * rover.ahrs.cos_yaw();
                    // add offset to current location
                    location_offset(target_loc, ne_x, ne_y);
                    }
                    break;

                case MAV_FRAME_LOCAL_OFFSET_NED:
                    // add offset to current location
                    location_offset(target_loc, packet.x, packet.y);
                    break;

                default:
                    // MAV_FRAME_LOCAL_NED interpret as an offset from home
                    target_loc = rover.ahrs.get_home();
                    location_offset(target_loc, packet.x, packet.y);
                    break;
                }
            }

            float target_speed = 0.0f;
            float target_yaw_cd = 0.0f;

            // consume velocity and convert to target speed and heading
            if (!vel_ignore) {
                // convert vector length into a speed
                target_speed = constrain_float(safe_sqrt(sq(packet.vx) + sq(packet.vy)), -rover.g.speed_cruise, rover.g.speed_cruise);
                // convert vector direction to target yaw
                target_yaw_cd = degrees(atan2f(packet.vy, packet.vx)) * 100.0f;
                // rotate target yaw if provided in body-frame
                if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                    target_yaw_cd = wrap_180_cd(target_yaw_cd + rover.ahrs.yaw_sensor);
                }
            }

            float target_turn_rate_cds = 0.0f;

            // consume yaw heading and yaw rate
            if (!yaw_ignore) {
                target_yaw_cd = ToDeg(packet.yaw) * 100.0f;
                // rotate target yaw if provided in body-frame
                if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                    target_yaw_cd = wrap_180_cd(target_yaw_cd + rover.ahrs.yaw_sensor);
                }
            }
            if (!yaw_rate_ignore) {
                target_turn_rate_cds = ToDeg(packet.yaw_rate) * 100.0f;
            }

            // set guided mode targets
            if (!pos_ignore && vel_ignore && acc_ignore && yaw_ignore && yaw_rate_ignore) {
                // consume position target
                rover.mode_guided.set_desired_location(target_loc);
            } else if (pos_ignore && !vel_ignore && acc_ignore && yaw_ignore && yaw_rate_ignore) {
                // consume velocity
                rover.mode_guided.set_desired_heading_and_speed(target_yaw_cd, target_speed);
            } else if (pos_ignore && vel_ignore && acc_ignore && !yaw_ignore && yaw_rate_ignore) {
                // consume just target heading (probably only skid steering vehicles can do this)
                rover.mode_guided.set_desired_heading_and_speed(target_yaw_cd, 0.0f);
            } else if (pos_ignore && vel_ignore && acc_ignore && yaw_ignore && !yaw_rate_ignore) {
                // consume just turn rate(probably only skid steering vehicles can do this)
                rover.mode_guided.set_desired_turn_rate_and_speed(target_turn_rate_cds, 0.0f);
            }
            break;
        }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:    // MAV ID: 86
        {
            // decode packet
            mavlink_set_position_target_global_int_t packet;
            mavlink_msg_set_position_target_global_int_decode(msg, &packet);

            // exit if vehicle is not in Guided mode
            if (rover.control_mode != &rover.mode_guided) {
                break;
            }
            // check for supported coordinate frames
            if (packet.coordinate_frame != MAV_FRAME_GLOBAL_INT &&
                packet.coordinate_frame != MAV_FRAME_GLOBAL_RELATIVE_ALT &&
                packet.coordinate_frame != MAV_FRAME_GLOBAL_RELATIVE_ALT_INT &&
                packet.coordinate_frame != MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) {
                break;
            }
            bool pos_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
            bool vel_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
            bool acc_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
            bool yaw_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
            bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;

            // prepare target position
            Location target_loc = rover.current_loc;
            if (!pos_ignore) {
                // sanity check location
                if (!check_latlng(packet.lat_int, packet.lon_int)) {
                    // result = MAV_RESULT_FAILED;
                    break;
                }
                target_loc.lat = packet.lat_int;
                target_loc.lng = packet.lon_int;
            }

            float target_speed = 0.0f;
            float target_yaw_cd = 0.0f;

            // consume velocity and convert to target speed and heading
            if (!vel_ignore) {
                // convert vector length into a speed
                target_speed = constrain_float(safe_sqrt(sq(packet.vx) + sq(packet.vy)), -rover.g.speed_cruise, rover.g.speed_cruise);
                // convert vector direction to target yaw
                target_yaw_cd = degrees(atan2f(packet.vy, packet.vx)) * 100.0f;
            }

            float target_turn_rate_cds = 0.0f;

            // consume yaw heading and yaw rate
            if (!yaw_ignore) {
                target_yaw_cd = ToDeg(packet.yaw) * 100.0f;
                // rotate target yaw if provided in body-frame
                if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                    target_yaw_cd = wrap_180_cd(target_yaw_cd + rover.ahrs.yaw_sensor);
                }
            }
            if (!yaw_rate_ignore) {
                target_turn_rate_cds = ToDeg(packet.yaw_rate) * 100.0f;
            }

            // set guided mode targets
            if (!pos_ignore && vel_ignore && acc_ignore && yaw_ignore && yaw_rate_ignore) {
                // consume position target
                rover.mode_guided.set_desired_location(target_loc);
            } else if (pos_ignore && !vel_ignore && acc_ignore && yaw_ignore && yaw_rate_ignore) {
                // consume velocity
                rover.mode_guided.set_desired_heading_and_speed(target_yaw_cd, target_speed);
            } else if (pos_ignore && vel_ignore && acc_ignore && !yaw_ignore && yaw_rate_ignore) {
                // consume just target heading (probably only skid steering vehicles can do this)
                rover.mode_guided.set_desired_heading_and_speed(target_yaw_cd, 0.0f);
            } else if (pos_ignore && vel_ignore && acc_ignore && yaw_ignore && !yaw_rate_ignore) {
                // consume just turn rate(probably only skid steering vehicles can do this)
                rover.mode_guided.set_desired_turn_rate_and_speed(target_turn_rate_cds, 0.0f);
            }
            break;
        }

#if HIL_MODE != HIL_MODE_DISABLED
    case MAVLINK_MSG_ID_HIL_STATE:
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
            compass.setHIL(0, packet.roll, packet.pitch, packet.yaw);
            compass.setHIL(1, packet.roll, packet.pitch, packet.yaw);
            break;
        }
#endif  // HIL_MODE

#if MOUNT == ENABLED
    // deprecated. Use MAV_CMD_DO_MOUNT_CONFIGURE
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:
        {
            rover.camera_mount.configure_msg(msg);
            break;
        }

    // deprecated. Use MAV_CMD_DO_MOUNT_CONTROL
    case MAVLINK_MSG_ID_MOUNT_CONTROL:
        {
            rover.camera_mount.control_msg(msg);
            break;
        }
#endif  // MOUNT == ENABLED

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS:
        {
            handle_radio_status(msg, rover.DataFlash, rover.should_log(MASK_LOG_PM));
            break;
        }

    case MAVLINK_MSG_ID_DISTANCE_SENSOR:
        rover.rangefinder.handle_msg(msg);
        break;

    case MAVLINK_MSG_ID_VISION_POSITION_DELTA:
        rover.g2.visual_odom.handle_msg(msg);
        break;

    default:
        handle_common_message(msg);
        break;
    }  // end switch
}  // end handle mavlink

/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
void Rover::mavlink_delay_cb()
{
    static uint32_t last_1hz, last_50hz, last_5s;
    if (!gcs().chan(0).initialised || in_mavlink_delay) {
        return;
    }

    in_mavlink_delay = true;
    // don't allow potentially expensive logging calls:
    DataFlash.EnableWrites(false);

    const uint32_t tnow = millis();
    if (tnow - last_1hz > 1000) {
        last_1hz = tnow;
        gcs().send_message(MSG_HEARTBEAT);
        gcs().send_message(MSG_EXTENDED_STATUS1);
    }
    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
        gcs_update();
        gcs_data_stream_send();
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
void Rover::gcs_data_stream_send(void)
{
    gcs().data_stream_send();
}

/*
 *  look for incoming commands on the GCS links
 */
void Rover::gcs_update(void)
{
    gcs().update();
}

/**
   retry any deferred messages
 */
void Rover::gcs_retry_deferred(void)
{
    gcs().retry_deferred();
}

/*
  return true if we will accept this packet. Used to implement SYSID_ENFORCE
 */
bool GCS_MAVLINK_Rover::accept_packet(const mavlink_status_t &status, mavlink_message_t &msg)
{
    if (!rover.g2.sysid_enforce) {
      return true;
    }
    if (msg.msgid == MAVLINK_MSG_ID_RADIO || msg.msgid == MAVLINK_MSG_ID_RADIO_STATUS) {
        return true;
    }
    return (msg.sysid == rover.g.sysid_my_gcs);
}

AP_GPS *GCS_MAVLINK_Rover::get_gps() const
{
    return &rover.gps;
}

AP_Camera *GCS_MAVLINK_Rover::get_camera() const
{
#if CAMERA == ENABLED
    return &rover.camera;
#else
    return nullptr;
#endif
}

AP_ServoRelayEvents *GCS_MAVLINK_Rover::get_servorelayevents() const
{
    return &rover.ServoRelayEvents;
}

AP_AdvancedFailsafe *GCS_MAVLINK_Rover::get_advanced_failsafe() const
{
#if ADVANCED_FAILSAFE == ENABLED
    return &rover.g2.afs;
#else
    return nullptr;
#endif
}

Compass *GCS_MAVLINK_Rover::get_compass() const
{
    return &rover.compass;
}

AP_Mission *GCS_MAVLINK_Rover::get_mission()
{
    return &rover.mission;
}

bool GCS_MAVLINK_Rover::set_mode(const uint8_t mode)
{
    Mode *new_mode = rover.control_mode_from_num((enum mode)mode);
    if (new_mode == nullptr) {
        return false;
    }
    return rover.set_mode(*new_mode, MODE_REASON_GCS_COMMAND);
}

const AP_FWVersion &GCS_MAVLINK_Rover::get_fwver() const
{
    return rover.fwver;
}
