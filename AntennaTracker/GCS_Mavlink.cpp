// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "GCS_Mavlink.h"

#include "Tracker.h"
#include "version.h"

// default sensors are present and healthy: gyro, accelerometer, barometer, rate_control, attitude_stabilization, yaw_position, altitude control, x/y position control, motor_control
#define MAVLINK_SENSOR_PRESENT_DEFAULT (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_YAW_POSITION | MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS)

/*
 *  !!NOTE!!
 *
 *  the use of NOINLINE separate functions for each message type avoids
 *  a compiler bug in gcc that would cause it to use far more stack
 *  space than is needed. Without the NOINLINE we use the sum of the
 *  stack needed for each message type. Please be careful to follow the
 *  pattern below when adding any new messages
 */

void Tracker::send_heartbeat(mavlink_channel_t chan)
{
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = MAV_STATE_ACTIVE;
    uint32_t custom_mode = control_mode;

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
        base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;

    case STOP:
        break;

    case SCAN:
    case SERVO_TEST:
    case AUTO:
        base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED |
            MAV_MODE_FLAG_STABILIZE_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;

    case INITIALISING:
        system_status = MAV_STATE_CALIBRATING;
        break;
    }

    // we are armed if safety switch is not disarmed
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    gcs[chan-MAVLINK_COMM_0].send_heartbeat(MAV_TYPE_ANTENNA_TRACKER,
                                            base_mode,
                                            custom_mode,
                                            system_status);
}

void Tracker::send_attitude(mavlink_channel_t chan)
{
    Vector3f omega = ahrs.get_gyro();
    mavlink_msg_attitude_send(
        chan,
        AP_HAL::millis(),
        ahrs.roll,
        ahrs.pitch,
        ahrs.yaw,
        omega.x,
        omega.y,
        omega.z);
}


void Tracker::send_location(mavlink_channel_t chan)
{
    uint32_t fix_time;
    if (gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
        fix_time = gps.last_fix_time_ms();
    } else {
        fix_time = AP_HAL::millis();
    }
    const Vector3f &vel = gps.velocity();
    mavlink_msg_global_position_int_send(
        chan,
        fix_time,
        current_loc.lat,                // in 1E7 degrees
        current_loc.lng,                // in 1E7 degrees
        current_loc.alt * 10,        // millimeters above sea level
        0,
        vel.x * 100,  // X speed cm/s (+ve North)
        vel.y * 100,  // Y speed cm/s (+ve East)
        vel.z * -100, // Z speed cm/s (+ve up)
        ahrs.yaw_sensor);
}

void Tracker::send_hwstatus(mavlink_channel_t chan)
{
    mavlink_msg_hwstatus_send(
        chan,
        0,
        0);
}

void Tracker::send_waypoint_request(mavlink_channel_t chan)
{
    gcs[chan-MAVLINK_COMM_0].queued_waypoint_send();
}

void Tracker::send_nav_controller_output(mavlink_channel_t chan)
{
	float alt_diff = (g.alt_source == ALT_SOURCE_BARO) ? nav_status.alt_difference_baro : nav_status.alt_difference_gps;

    mavlink_msg_nav_controller_output_send(
        chan,
        0,
        nav_status.pitch,
        nav_status.bearing,
        nav_status.bearing,
        nav_status.distance,
        alt_diff,
        0,
        0);
}


// report simulator state
void Tracker::send_simstate(mavlink_channel_t chan)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    sitl.simstate_send(chan);
#endif
}

bool GCS_MAVLINK_Tracker::handle_guided_request(AP_Mission::Mission_Command&)
{
    // do nothing
    return false;
}

void GCS_MAVLINK_Tracker::handle_change_alt_request(AP_Mission::Mission_Command&)
{
    // do nothing
}

// try to send a message, return false if it won't fit in the serial tx buffer
bool GCS_MAVLINK_Tracker::try_send_message(enum ap_message id)
{
    switch (id) {
    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        last_heartbeat_time = AP_HAL::millis();
        tracker.send_heartbeat(chan);
        return true;

    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        tracker.send_attitude(chan);
        break;

    case MSG_LOCATION:
        CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
        tracker.send_location(chan);
        break;

    case MSG_LOCAL_POSITION:
        CHECK_PAYLOAD_SIZE(LOCAL_POSITION_NED);
        send_local_position(tracker.ahrs);
        break;

    case MSG_NAV_CONTROLLER_OUTPUT:
        CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
        tracker.send_nav_controller_output(chan);
        break;

    case MSG_GPS_RAW:
        CHECK_PAYLOAD_SIZE(GPS_RAW_INT);
        send_gps_raw(tracker.gps);
        break;

    case MSG_RADIO_IN:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_RAW);
        send_radio_in(0);
        break;

    case MSG_RADIO_OUT:
        CHECK_PAYLOAD_SIZE(SERVO_OUTPUT_RAW);
        send_servo_output_raw(false);
        break;

    case MSG_RAW_IMU1:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
        send_raw_imu(tracker.ins, tracker.compass);
        break;

    case MSG_RAW_IMU2:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE);
        send_scaled_pressure(tracker.barometer);
        break;

    case MSG_RAW_IMU3:
        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
        send_sensor_offsets(tracker.ins, tracker.compass, tracker.barometer);
        break;

    case MSG_NEXT_PARAM:
        CHECK_PAYLOAD_SIZE(PARAM_VALUE);
        queued_param_send();
        break;

    case MSG_NEXT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_REQUEST);
        tracker.send_waypoint_request(chan);
        break;

    case MSG_STATUSTEXT:
        // depreciated, use GCS_MAVLINK::send_statustext*
        return false;

    case MSG_AHRS:
        CHECK_PAYLOAD_SIZE(AHRS);
        send_ahrs(tracker.ahrs);
        break;

    case MSG_SIMSTATE:
        CHECK_PAYLOAD_SIZE(SIMSTATE);
        tracker.send_simstate(chan);
        break;

    case MSG_HWSTATUS:
        CHECK_PAYLOAD_SIZE(HWSTATUS);
        tracker.send_hwstatus(chan);
        break;
    case MSG_MAG_CAL_PROGRESS:
        CHECK_PAYLOAD_SIZE(MAG_CAL_PROGRESS);
        tracker.compass.send_mag_cal_progress(chan);
        break;

    case MSG_MAG_CAL_REPORT:
        CHECK_PAYLOAD_SIZE(MAG_CAL_REPORT);
        tracker.compass.send_mag_cal_report(chan);
        break;

    case MSG_SERVO_OUT:
    case MSG_EXTENDED_STATUS1:
    case MSG_EXTENDED_STATUS2:
    case MSG_RETRY_DEFERRED:
    case MSG_ADSB_VEHICLE:
    case MSG_CURRENT_WAYPOINT:
    case MSG_VFR_HUD:
    case MSG_SYSTEM_TIME:
    case MSG_LIMITS_STATUS:
    case MSG_FENCE_STATUS:
    case MSG_WIND:
    case MSG_RANGEFINDER:
    case MSG_TERRAIN:
    case MSG_BATTERY2:
    case MSG_CAMERA_FEEDBACK:
    case MSG_MOUNT_STATUS:
    case MSG_OPTICAL_FLOW:
    case MSG_GIMBAL_REPORT:
    case MSG_EKF_STATUS_REPORT:
    case MSG_PID_TUNING:
    case MSG_VIBRATION:
    case MSG_RPM:
    case MSG_MISSION_ITEM_REACHED:
    case MSG_POSITION_TARGET_GLOBAL_INT:
        break; // just here to prevent a warning
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
GCS_MAVLINK_Tracker::data_stream_send(void)
{
    if (_queued_parameter != NULL) {
        if (streamRates[STREAM_PARAMS].get() <= 0) {
            streamRates[STREAM_PARAMS].set(10);
        }
        if (stream_trigger(STREAM_PARAMS)) {
            send_message(MSG_NEXT_PARAM);
        }
    }

    if (tracker.in_mavlink_delay) {
        // don't send any other stream types while in the delay callback
        return;
    }

    if (stream_trigger(STREAM_RAW_SENSORS)) {
        send_message(MSG_RAW_IMU1);
        send_message(MSG_RAW_IMU2);
        send_message(MSG_RAW_IMU3);
    }

    if (stream_trigger(STREAM_EXTENDED_STATUS)) {
        send_message(MSG_EXTENDED_STATUS1);
        send_message(MSG_EXTENDED_STATUS2);
        send_message(MSG_NAV_CONTROLLER_OUTPUT);
        send_message(MSG_GPS_RAW);
    }

    if (stream_trigger(STREAM_POSITION)) {
        send_message(MSG_LOCATION);
        send_message(MSG_LOCAL_POSITION);
    }

    if (stream_trigger(STREAM_RAW_CONTROLLER)) {
        send_message(MSG_SERVO_OUT);
    }

    if (stream_trigger(STREAM_RC_CHANNELS)) {
        send_message(MSG_RADIO_IN);
        send_message(MSG_RADIO_OUT);
    }

    if (stream_trigger(STREAM_EXTRA1)) {
        send_message(MSG_ATTITUDE);
    }

    if (stream_trigger(STREAM_EXTRA3)) {
        send_message(MSG_AHRS);
        send_message(MSG_HWSTATUS);
        send_message(MSG_SIMSTATE);
        send_message(MSG_MAG_CAL_REPORT);
        send_message(MSG_MAG_CAL_PROGRESS);
    }
}

/*
  We eavesdrop on MAVLINK_MSG_ID_GLOBAL_POSITION_INT and
  MAVLINK_MSG_ID_SCALED_PRESSUREs
*/
void Tracker::mavlink_snoop(const mavlink_message_t* msg)
{
    // return immediately if sysid doesn't match our target sysid
    if ((g.sysid_target != 0) && (g.sysid_target != msg->sysid)) {
        return;
    }

    switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        mavlink_check_target(msg);
        break;
    }

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    {
        // decode
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(msg, &packet);
        tracking_update_position(packet);
        break;
    }
    
    case MAVLINK_MSG_ID_SCALED_PRESSURE:
    {
        // decode
        mavlink_scaled_pressure_t packet;
        mavlink_msg_scaled_pressure_decode(msg, &packet);
        tracking_update_pressure(packet);
        break;
    }
    }
}

// locks onto a particular target sysid and sets it's position data stream to at least 1hz
void Tracker::mavlink_check_target(const mavlink_message_t* msg)
{
    // exit immediately if the target has already been set
    if (target_set) {
        return;
    }

    // decode
    mavlink_heartbeat_t packet;
    mavlink_msg_heartbeat_decode(msg, &packet);

    // exit immediately if this is not a vehicle we would track
    if ((packet.type == MAV_TYPE_ANTENNA_TRACKER) ||
        (packet.type == MAV_TYPE_GCS) ||
        (packet.type == MAV_TYPE_ONBOARD_CONTROLLER) ||
        (packet.type == MAV_TYPE_GIMBAL)) {
        return;
    }

    // set our sysid to the target, this ensures we lock onto a single vehicle
    if (g.sysid_target == 0) {
        g.sysid_target = msg->sysid;
    }

    // send data stream request to target on all channels
    //  Note: this doesn't check success for all sends meaning it's not guaranteed the vehicle's positions will be sent at 1hz
    for (uint8_t i=0; i < num_gcs; i++) {
        if (gcs[i].initialised) {
            // request position
            if (HAVE_PAYLOAD_SPACE((mavlink_channel_t)i, DATA_STREAM)) {
                mavlink_msg_request_data_stream_send(
                    (mavlink_channel_t)i,
                    msg->sysid,
                    msg->compid,
                    MAV_DATA_STREAM_POSITION,
                    g.mavlink_update_rate,
                    1); // start streaming
            }
            // request air pressure
            if (HAVE_PAYLOAD_SPACE((mavlink_channel_t)i, DATA_STREAM)) {
                mavlink_msg_request_data_stream_send(
                    (mavlink_channel_t)i,
                    msg->sysid,
                    msg->compid,
                    MAV_DATA_STREAM_RAW_SENSORS,
                    g.mavlink_update_rate,
                    1); // start streaming
            }
        }
    }

    // flag target has been set
    target_set = true;
}

void GCS_MAVLINK_Tracker::handleMessage(mavlink_message_t* msg)
{
    switch (msg->msgid) {

    // If we are currently operating as a proxy for a remote, 
    // alas we have to look inside each packet to see if it's for us or for the remote
    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
    {
        handle_request_data_stream(msg, false);
        break;
    }


    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
        handle_param_request_list(msg);
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    {
        handle_param_request_read(msg);
        break;
    }

    case MAVLINK_MSG_ID_PARAM_SET:
    {
        handle_param_set(msg, NULL);
        break;
    }

    case MAVLINK_MSG_ID_HEARTBEAT:
        break;

    case MAVLINK_MSG_ID_COMMAND_LONG:
    {
        // decode
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(msg, &packet);
        
        uint8_t result = MAV_RESULT_UNSUPPORTED;
        
        // do command
        send_text(MAV_SEVERITY_INFO,"Command received: ");
        
        switch(packet.command) {
            
            case MAV_CMD_PREFLIGHT_CALIBRATION:
            {
                if (is_equal(packet.param1,1.0f)) {
                    tracker.ins.init_gyro();
                    if (tracker.ins.gyro_calibrated_ok_all()) {
                        tracker.ahrs.reset_gyro_drift();
                        result = MAV_RESULT_ACCEPTED;
                    } else {
                        result = MAV_RESULT_FAILED;
                    }
                }
                if (is_equal(packet.param3,1.0f)) {
                    tracker.init_barometer(false);
                    // zero the altitude difference on next baro update
                    tracker.nav_status.need_altitude_calibration = true;
                    result = MAV_RESULT_ACCEPTED;
                }
                if (is_equal(packet.param4,1.0f)) {
                    // Can't trim radio
                    result = MAV_RESULT_UNSUPPORTED;
                } else if (is_equal(packet.param5,1.0f)) {
                    result = MAV_RESULT_ACCEPTED;
                    // start with gyro calibration
                    tracker.ins.init_gyro();
                    // reset ahrs gyro bias
                    if (tracker.ins.gyro_calibrated_ok_all()) {
                        tracker.ahrs.reset_gyro_drift();
                    } else {
                        result = MAV_RESULT_FAILED;
                    }
                    // start accel cal
                    tracker.ins.acal_init();
                    tracker.ins.get_acal()->start(this);
                } else if (is_equal(packet.param5,2.0f)) {
                    // start with gyro calibration
                    tracker.ins.init_gyro();
                    // accel trim
                    float trim_roll, trim_pitch;
                    if (tracker.ins.calibrate_trim(trim_roll, trim_pitch)) {
                        // reset ahrs's trim to suggested values from calibration routine
                        tracker.ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
                        result = MAV_RESULT_ACCEPTED;
                    } else {
                        result = MAV_RESULT_FAILED;
                    }
                }
                break;
            }

            case MAV_CMD_COMPONENT_ARM_DISARM:
                if (packet.target_component == MAV_COMP_ID_SYSTEM_CONTROL) {
                    if (is_equal(packet.param1,1.0f)) {
                        tracker.arm_servos();
                        result = MAV_RESULT_ACCEPTED;
                    } else if (is_zero(packet.param1))  {
                        tracker.disarm_servos();
                        result = MAV_RESULT_ACCEPTED;
                    } else {
                        result = MAV_RESULT_UNSUPPORTED;
                    }
                } else {
                    result = MAV_RESULT_UNSUPPORTED;
                }
            break;

            case MAV_CMD_GET_HOME_POSITION:
                send_home(tracker.ahrs.get_home());
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_CMD_DO_SET_MODE:
                switch ((uint16_t)packet.param1) {
                    case MAV_MODE_MANUAL_ARMED:
                    case MAV_MODE_MANUAL_DISARMED:
                        tracker.set_mode(MANUAL);
                        result = MAV_RESULT_ACCEPTED;
                        break;

                    case MAV_MODE_AUTO_ARMED:
                    case MAV_MODE_AUTO_DISARMED:
                        tracker.set_mode(AUTO);
                        result = MAV_RESULT_ACCEPTED;
                        break;

                    default:
                        result = MAV_RESULT_UNSUPPORTED;
                }
                break;

            case MAV_CMD_DO_SET_SERVO:
                if (tracker.servo_test_set_servo(packet.param1, packet.param2)) {
                    result = MAV_RESULT_ACCEPTED;
                }
                break;

                // mavproxy/mavutil sends this when auto command is entered 
            case MAV_CMD_MISSION_START:
                tracker.set_mode(AUTO);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            {
                if (is_equal(packet.param1,1.0f) || is_equal(packet.param1,3.0f)) {
                    // when packet.param1 == 3 we reboot to hold in bootloader
                    hal.scheduler->reboot(is_equal(packet.param1,3.0f));
                    result = MAV_RESULT_ACCEPTED;
                }
                break;
            }

            case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES: {
                if (is_equal(packet.param1,1.0f)) {
                    send_autopilot_version(FIRMWARE_VERSION);
                    result = MAV_RESULT_ACCEPTED;
                }
                break;
            }

            case MAV_CMD_DO_START_MAG_CAL:
            case MAV_CMD_DO_ACCEPT_MAG_CAL:
            case MAV_CMD_DO_CANCEL_MAG_CAL:
                result = tracker.compass.handle_mag_cal_command(packet);
                break;

            default:
                break;
        }
        mavlink_msg_command_ack_send(
            chan,
            packet.command,
            result);
        
        break;
    }
         
    // When mavproxy 'wp sethome' 
    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
    {
        // decode
        mavlink_mission_write_partial_list_t packet;
        mavlink_msg_mission_write_partial_list_decode(msg, &packet);
        if (packet.start_index == 0)
        {
            // New home at wp index 0. Ask for it
            waypoint_receiving = true;
            waypoint_request_i = 0;
            waypoint_request_last = 0;
            send_message(MSG_NEXT_WAYPOINT);
            waypoint_receiving = true;
        }
        break;
    }

    // XXX receive a WP from GCS and store in EEPROM if it is HOME
    case MAVLINK_MSG_ID_MISSION_ITEM:
    {
        // decode
        mavlink_mission_item_t packet;
        uint8_t result = MAV_MISSION_ACCEPTED;

        mavlink_msg_mission_item_decode(msg, &packet);

        struct Location tell_command = {};

        switch (packet.frame)
        {
        case MAV_FRAME_MISSION:
        case MAV_FRAME_GLOBAL:
        {
            tell_command.lat = 1.0e7f*packet.x;                                     // in as DD converted to * t7
            tell_command.lng = 1.0e7f*packet.y;                                     // in as DD converted to * t7
            tell_command.alt = packet.z*1.0e2f;                                     // in as m converted to cm
            tell_command.options = 0;                                     // absolute altitude
            break;
        }

#ifdef MAV_FRAME_LOCAL_NED
        case MAV_FRAME_LOCAL_NED:                         // local (relative to home position)
        {
            tell_command.lat = 1.0e7f*ToDeg(packet.x/
                                           (RADIUS_OF_EARTH*cosf(ToRad(home.lat/1.0e7f)))) + home.lat;
            tell_command.lng = 1.0e7f*ToDeg(packet.y/RADIUS_OF_EARTH) + home.lng;
            tell_command.alt = -packet.z*1.0e2f;
            tell_command.options = MASK_OPTIONS_RELATIVE_ALT;
            break;
        }
#endif

#ifdef MAV_FRAME_LOCAL
        case MAV_FRAME_LOCAL:                         // local (relative to home position)
        {
            tell_command.lat = 1.0e7f*ToDeg(packet.x/
                                           (RADIUS_OF_EARTH*cosf(ToRad(home.lat/1.0e7f)))) + home.lat;
            tell_command.lng = 1.0e7f*ToDeg(packet.y/RADIUS_OF_EARTH) + home.lng;
            tell_command.alt = packet.z*1.0e2f;
            tell_command.options = MASK_OPTIONS_RELATIVE_ALT;
            break;
        }
#endif

        case MAV_FRAME_GLOBAL_RELATIVE_ALT:                         // absolute lat/lng, relative altitude
        {
            tell_command.lat = 1.0e7f * packet.x;                                     // in as DD converted to * t7
            tell_command.lng = 1.0e7f * packet.y;                                     // in as DD converted to * t7
            tell_command.alt = packet.z * 1.0e2f;
            tell_command.options = MASK_OPTIONS_RELATIVE_ALT;                                     // store altitude relative!! Always!!
            break;
        }

        default:
            result = MAV_MISSION_UNSUPPORTED_FRAME;
            break;
        }

        if (result != MAV_MISSION_ACCEPTED) goto mission_failed;

        // Check if receiving waypoints (mission upload expected)
        if (!waypoint_receiving) {
            result = MAV_MISSION_ERROR;
            goto mission_failed;
        }

        // check if this is the HOME wp
        if (packet.seq == 0) {
            tracker.set_home(tell_command); // New home in EEPROM
            send_text(MAV_SEVERITY_INFO,"New HOME received");
            waypoint_receiving = false;
        }

mission_failed:
        // we are rejecting the mission/waypoint
        mavlink_msg_mission_ack_send(
            chan,
            msg->sysid,
            msg->compid,
            result);
        break;
    }

    case MAVLINK_MSG_ID_MANUAL_CONTROL:
    {
        mavlink_manual_control_t packet;
        mavlink_msg_manual_control_decode(msg, &packet);
        tracker.tracking_manual_control(packet);
        break;
    }

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 
    {
        // decode
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(msg, &packet);
        tracker.tracking_update_position(packet);
        break;
    }

    case MAVLINK_MSG_ID_SCALED_PRESSURE: 
    {
        // decode
        mavlink_scaled_pressure_t packet;
        mavlink_msg_scaled_pressure_decode(msg, &packet);
        tracker.tracking_update_pressure(packet);
        break;
    }

    case MAVLINK_MSG_ID_SET_MODE:
    {
        handle_set_mode(msg, FUNCTOR_BIND(&tracker, &Tracker::mavlink_set_mode, bool, uint8_t));
        break;
    }

    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
    case MAVLINK_MSG_ID_LOG_ERASE:
        tracker.in_log_download = true;
        /* no break */
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        if (!tracker.in_mavlink_delay) {
            handle_log_message(msg, tracker.DataFlash);
        }
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        tracker.in_log_download = false;
        if (!tracker.in_mavlink_delay) {
            handle_log_message(msg, tracker.DataFlash);
        }
        break;

    case MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS:
        tracker.DataFlash.remote_log_block_status_msg(chan, msg);
        break;

    case MAVLINK_MSG_ID_SERIAL_CONTROL:
        handle_serial_control(msg, tracker.gps);
        break;

    case MAVLINK_MSG_ID_GPS_INJECT_DATA:
        handle_gps_inject(msg, tracker.gps);
        break;

    case MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST:
        send_autopilot_version(FIRMWARE_VERSION);
        break;

    case MAVLINK_MSG_ID_SETUP_SIGNING:
        handle_setup_signing(msg);
        break;
    } // end switch
} // end handle mavlink


/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
void Tracker::mavlink_delay_cb()
{
    static uint32_t last_1hz, last_50hz, last_5s;
    if (!gcs[0].initialised) return;

    tracker.in_mavlink_delay = true;

    uint32_t tnow = AP_HAL::millis();
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
    tracker.in_mavlink_delay = false;
}

/*
 *  send a message on both GCS links
 */
void Tracker::gcs_send_message(enum ap_message id)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].send_message(id);
        }
    }
}

/*
 *  send data streams in the given rate range on both links
 */
void Tracker::gcs_data_stream_send(void)
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
void Tracker::gcs_update(void)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].update(NULL);
        }
    }
}

void Tracker::gcs_send_text(MAV_SEVERITY severity, const char *str)
{
    GCS_MAVLINK::send_statustext(severity, 0xFF, str);
}

/*
 *  send a low priority formatted message to the GCS
 *  only one fits in the queue, so if you send more than one before the
 *  last one gets into the serial buffer then the old one will be lost
 */
void Tracker::gcs_send_text_fmt(MAV_SEVERITY severity, const char *fmt, ...)
{
    char str[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] {};
    va_list arg_list;
    va_start(arg_list, fmt);
    hal.util->vsnprintf((char *)str, sizeof(str), fmt, arg_list);
    va_end(arg_list);
    GCS_MAVLINK::send_statustext(severity, 0xFF, str);
}

/**
   retry any deferred messages
 */
void Tracker::gcs_retry_deferred(void)
{
    gcs_send_message(MSG_RETRY_DEFERRED);
    GCS_MAVLINK::service_statustext();
}
