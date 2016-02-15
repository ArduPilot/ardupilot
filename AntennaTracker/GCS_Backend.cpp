#include "Tracker.h" // for access to tracker global

AP_GPS &GCS_Backend_Tracker::_gps() const { return tracker.gps; }

#if AP_AHRS_NAVEKF_AVAILABLE
AP_AHRS_NavEKF &GCS_Backend_Tracker::_ahrs() const { return tracker.ahrs; }
#else
AP_AHRS &GCS_Backend_Tracker::_ahrs() const { return tracker.ahrs; }
#endif

Compass &GCS_Backend_Tracker::_compass() const { return tracker.compass; }

AP_Baro &GCS_Backend_Tracker::_barometer() const { return tracker.barometer; }

AP_InertialSensor &GCS_Backend_Tracker::_ins() const { return tracker.ins; }

void GCS_Backend_Tracker::handle_guided_request(AP_Mission::Mission_Command&)
{
    // do nothing
}

void GCS_Backend_Tracker::handle_change_alt_request(AP_Mission::Mission_Command&)
{
    // do nothing
}

bool GCS_Backend_Tracker::should_try_send_message(enum ap_message id)
{
    return true;
}

bool GCS_Backend_Tracker::send_ATTITUDE()
{
    tracker.send_attitude(chan);
    return true;
}
bool GCS_Backend_Tracker::send_GLOBAL_POSITION_INT()
{
    tracker.send_location(chan);
    return true;
}
bool GCS_Backend_Tracker::send_HEARTBEAT()
{
    tracker.send_heartbeat(chan);
    return true;
}
bool GCS_Backend_Tracker::send_NAV_CONTROLLER_OUTPUT()
{
    tracker.send_nav_controller_output(chan);
    return true;
}
bool GCS_Backend_Tracker::send_RC_CHANNELS_RAW()
{
    send_radio_in(0);
    return true;
}
bool GCS_Backend_Tracker::send_SERVO_OUTPUT_RAW()
{
    tracker.send_radio_out(chan);
    return true;
}
bool GCS_Backend_Tracker::send_SIMSTATE()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    tracker.send_simstate(chan);
#endif
    return true;
}


// see if we should send a stream now. Called at 50Hz
bool GCS_Backend_Tracker::stream_trigger(enum streams stream_num)
{
    if (stream_num >= NUM_STREAMS) {
        return false;
    }
    float rate = (uint8_t)streamRate(stream_num).get();

    // send at a much lower rate during parameter sends
    if (queued_parameter() != NULL) {
        rate *= 0.25f;
    }

    if (rate <= 0) {
        return false;
    }

    if (stream_ticks[stream_num] == 0) {
        // we're triggering now, setup the next trigger point
        if (rate > 50) {
            rate = 50;
        }
        stream_ticks[stream_num] = (50 / rate) -1 + stream_slowdown;
        return true;
    }

    // count down at 50Hz
    stream_ticks[stream_num]--;
    return false;
}

void
GCS_Backend_Tracker::data_stream_send(void)
{
    if (queued_parameter() != NULL) {
        if (streamRate(STREAM_PARAMS).get() <= 0) {
            streamRate(STREAM_PARAMS).set(10);
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

void GCS_Backend_Tracker::handleMessage(mavlink_message_t* msg)
{
    switch (msg->msgid) {

    // If we are currently operating as a proxy for a remote, 
    // alas we have to look inside each packet to see if its for us or for the remote
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
                    tracker.init_barometer();
                    // zero the altitude difference on next baro update
                    tracker.nav_status.need_altitude_calibration = true;
                }
                if (is_equal(packet.param4,1.0f)) {
                    // Cant trim radio
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
                    if(tracker.ins.calibrate_trim(trim_roll, trim_pitch)) {
                        // reset ahrs's trim to suggested values from calibration routine
                        tracker.ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
                        result = MAV_RESULT_ACCEPTED;
                    } else {
                        result = MAV_RESULT_FAILED;
                    }
                }
                result = MAV_RESULT_ACCEPTED;
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
        ::mavlink_msg_command_ack_send(
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
            set_waypoint_receiving(true);
            set_waypoint_request_i(0);
            set_waypoint_request_last(0);
            send_message(MSG_NEXT_WAYPOINT);
            set_waypoint_receiving(true);
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
        if (!waypoint_receiving()) {
            result = MAV_MISSION_ERROR;
            goto mission_failed;
        }

        // check if this is the HOME wp
        if (packet.seq == 0) {
            tracker.set_home(tell_command); // New home in EEPROM
            send_text(MAV_SEVERITY_INFO,"New HOME received");
            set_waypoint_receiving(false);
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

    } // end switch
} // end handle mavlink


/*
 *  a delay() callback that processes MAVLink packets and processes
 *  Notify events (blinks LEDs).  We set this as the callback in long
 *  running library initialisation routines to allow MAVLink to
 *  process packets while waiting for the initialisation to complete.
 *  And to entertain the user with blinking lights.
 */
