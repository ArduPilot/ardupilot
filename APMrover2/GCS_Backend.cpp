#include "Rover.h" // for global rover object

AP_GPS &GCS_Backend_Rover::_gps() const { return rover.gps; }

#if AP_AHRS_NAVEKF_AVAILABLE
AP_AHRS_NavEKF &GCS_Backend_Rover::_ahrs() const { return rover.ahrs; }
#else
AP_AHRS &GCS_Backend_Rover::_ahrs() const { return rover.ahrs; }
#endif

bool GCS_Backend_Rover::should_try_send_message(enum ap_message id)
{
    if (telemetry_delayed()) {
        return false;
    }

    // if we don't have a minimum amount of remaining before the main
    // loop wants to fire then don't send a mavlink message. We want
    // to prioritise the main flight control loop over communications
    if (!rover.in_mavlink_delay &&
        rover.scheduler.time_available_usec() < 1200) {
        out_of_time = true;
        return false;
    }

    return true;
}

bool GCS_Backend_Rover::send_AHRS()
{
    send_ahrs(rover.ahrs);
    return true;
}
bool GCS_Backend_Rover::send_AHRS2()
{
    return true;
}
bool GCS_Backend_Rover::send_ATTITUDE()
{
    rover.send_attitude(chan);
    return true;
}
bool GCS_Backend_Rover::send_BATTERY2()
{
    send_battery2(rover.battery);
    return true;
}
bool GCS_Backend_Rover::send_CAMERA_FEEDBACK()
{
#if CAMERA == ENABLED
    rover.camera.send_feedback(chan, rover.gps, rover.ahrs, rover.current_loc);
#endif
    return true;
}
bool GCS_Backend_Rover::send_EKF_STATUS_REPORT()
{
#if AP_AHRS_NAVEKF_AVAILABLE
    rover.ahrs.send_ekf_status_report(chan);
#endif
    return true;
}
bool GCS_Backend_Rover::send_GLOBAL_POSITION_INT()
{
    rover.send_location(chan);
    return true;
}
bool GCS_Backend_Rover::send_HEARTBEAT()
{
    rover.send_heartbeat(chan);
    return true;
}
bool GCS_Backend_Rover::send_LOCAL_POSITION_NED()
{
    send_local_position(rover.ahrs);
    return true;
}
bool GCS_Backend_Rover::send_MISSION_CURRENT()
{
    rover.send_current_waypoint(chan);
    return true;
}
bool GCS_Backend_Rover::send_MISSION_ITEM_REACHED()
{
    mavlink_msg_mission_item_reached_send(chan, mission_item_reached_index);
    return true;
}
bool GCS_Backend_Rover::send_MAG_CAL_PROGRESS()
{
    rover.compass.send_mag_cal_progress(chan);
    return true;
}
bool GCS_Backend_Rover::send_MAG_CAL_REPORT()
{
    rover.compass.send_mag_cal_report(chan);
    return true;
}
bool GCS_Backend_Rover::send_MOUNT_STATUS()
{
#if MOUNT == ENABLED
    rover.camera_mount.status_msg(chan);
#endif
    return true;
}
bool GCS_Backend_Rover::send_NAV_CONTROLLER_OUTPUT()
{
    if (rover.control_mode != MANUAL) {
        rover.send_nav_controller_output(chan);
    }
    return true;
}
bool GCS_Backend_Rover::send_PID_TUNING()
{
    rover.send_pid_tuning(chan);
    return true;
}
bool GCS_Backend_Rover::send_RC_CHANNELS_RAW()
{
    send_radio_in(rover.receiver_rssi);
    return true;
}
bool GCS_Backend_Rover::send_SERVO_OUTPUT_RAW()
{
    rover.send_radio_out(chan);
    return true;
}
bool GCS_Backend_Rover::send_RANGEFINDER()
{
    rover.send_rangefinder(chan);
    return true;
}
bool GCS_Backend_Rover::send_RAW_IMU()
{
    send_raw_imu(rover.ins, rover.compass);
    return true;
}
bool GCS_Backend_Rover::send_RC_CHANNELS_SCALED()
{
    rover.send_servo_out(chan);
    return true;
}
bool GCS_Backend_Rover::send_SENSOR_OFFSETS()
{
    send_sensor_offsets(rover.ins, rover.compass, rover.barometer);
    return true;
}
bool GCS_Backend_Rover::send_SIMSTATE()
{
    rover.send_simstate(chan);
    return true;
}
bool GCS_Backend_Rover::send_STATUSTEXT()
{
    // this looks wrong; if a Backend is told to send its statustext
    // then it should not be talking to the frontend:
    rover.gcs_frontend.send_statustext(chan);
    return true;
}
bool GCS_Backend_Rover::send_SYS_STATUS()
{
    rover.send_extended_status1(chan);
    return true;
}
bool GCS_Backend_Rover::send_VFR_HUD()
{
    rover.send_vfr_hud(chan);
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


// see if we should send a stream now. Called at 50Hz
bool GCS_Backend_Rover::stream_trigger(enum streams stream_num)
{
    if (stream_num >= NUM_STREAMS) {
        return false;
    }
    float rate = (uint8_t)streamRate(stream_num).get();

    // send at a much lower rate while handling waypoints and
    // parameter sends
    if ((stream_num != STREAM_PARAMS) && 
        (waypoint_receiving() || queued_parameter() != NULL)) {
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
        stream_ticks[stream_num] = (50 / rate) - 1 + stream_slowdown;
        return true;
    }

    // count down at 50Hz
    stream_ticks[stream_num]--;
    return false;
}

void
GCS_Backend_Rover::data_stream_send(void)
{
    out_of_time = false;

    if (!rover.in_mavlink_delay) {
        handle_log_send(rover.DataFlash);
    }

    if (queued_parameter() != NULL) {
        if (streamRate(STREAM_PARAMS).get() <= 0) {
            streamRate(STREAM_PARAMS).set(10);
        }
        if (stream_trigger(STREAM_PARAMS)) {
            send_message(MSG_NEXT_PARAM);
        }
    }

    if (out_of_time) return;

    if (rover.in_mavlink_delay) {
#if HIL_MODE != HIL_MODE_DISABLED
        // in HIL we need to keep sending servo values to ensure
        // the simulator doesn't pause, otherwise our sensor
        // calibration could stall
        if (stream_trigger(STREAM_RAW_CONTROLLER)) {
            send_message(MSG_SERVO_OUT);
        }
        if (stream_trigger(STREAM_RC_CHANNELS)) {
            send_message(MSG_RADIO_OUT);
        }
#endif
        // don't send any other stream types while in the delay callback
        return;
    }

    if (out_of_time) return;

    if (stream_trigger(STREAM_RAW_SENSORS)) {
        send_message(MSG_RAW_IMU1);
        send_message(MSG_RAW_IMU3);
    }

    if (out_of_time) return;

    if (stream_trigger(STREAM_EXTENDED_STATUS)) {
        send_message(MSG_EXTENDED_STATUS1);
        send_message(MSG_EXTENDED_STATUS2);
        send_message(MSG_CURRENT_WAYPOINT);
        send_message(MSG_GPS_RAW);            // TODO - remove this message after location message is working
        send_message(MSG_NAV_CONTROLLER_OUTPUT);
    }

    if (out_of_time) return;

    if (stream_trigger(STREAM_POSITION)) {
        // sent with GPS read
        send_message(MSG_LOCATION);
        send_message(MSG_LOCAL_POSITION);
    }

    if (out_of_time) return;

    if (stream_trigger(STREAM_RAW_CONTROLLER)) {
        send_message(MSG_SERVO_OUT);
    }

    if (out_of_time) return;

    if (stream_trigger(STREAM_RC_CHANNELS)) {
        send_message(MSG_RADIO_OUT);
        send_message(MSG_RADIO_IN);
    }

    if (out_of_time) return;

    if (stream_trigger(STREAM_EXTRA1)) {
        send_message(MSG_ATTITUDE);
        send_message(MSG_SIMSTATE);
        if (rover.control_mode != MANUAL) {
            send_message(MSG_PID_TUNING);
        }
    }
    
    if (out_of_time) return;

    if (stream_trigger(STREAM_EXTRA2)) {
        send_message(MSG_VFR_HUD);
    }

    if (out_of_time) return;

    if (stream_trigger(STREAM_EXTRA3)) {
        send_message(MSG_AHRS);
        send_message(MSG_HWSTATUS);
        send_message(MSG_RANGEFINDER);
        send_message(MSG_SYSTEM_TIME);
        send_message(MSG_BATTERY2);
        send_message(MSG_MAG_CAL_REPORT);
        send_message(MSG_MAG_CAL_PROGRESS);
        send_message(MSG_MOUNT_STATUS);
        send_message(MSG_EKF_STATUS_REPORT);
    }
}



void GCS_Backend_Rover::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
    if (rover.control_mode != GUIDED) {
        // only accept position updates when in GUIDED mode
        return;
    }
        
    rover.guided_WP = cmd.content.location;

    // make any new wp uploaded instant (in case we are already in Guided mode)
    rover.rtl_complete = false;
    rover.set_guided_WP();
}

void GCS_Backend_Rover::handle_change_alt_request(AP_Mission::Mission_Command &cmd)
{
    // nothing to do
}

void GCS_Backend_Rover::handleMessage(mavlink_message_t* msg)
{
    switch (msg->msgid) {

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
        {
            handle_request_data_stream(msg, true);
            break;
        }

    case MAVLINK_MSG_ID_COMMAND_LONG:
        {
            // decode
            mavlink_command_long_t packet;
            mavlink_msg_command_long_decode(msg, &packet);

            uint8_t result = MAV_RESULT_UNSUPPORTED;

            // do command
            send_text(MAV_SEVERITY_INFO,"Command received: ");

            switch(packet.command) {

            case MAV_CMD_START_RX_PAIR:
                // initiate bind procedure
                if (!hal.rcin->rc_bind(packet.param1)) {
                    result = MAV_RESULT_FAILED;
                } else {
                    result = MAV_RESULT_ACCEPTED;
                }
                break;

            case MAV_CMD_NAV_RETURN_TO_LAUNCH:
                rover.set_mode(RTL);
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

#if CAMERA == ENABLED
        case MAV_CMD_DO_DIGICAM_CONFIGURE:
            rover.camera.configure(packet.param1,
                                   packet.param2,
                                   packet.param3,
                                   packet.param4,
                                   packet.param5,
                                   packet.param6,
                                   packet.param7);

            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_DO_DIGICAM_CONTROL:
            if (rover.camera.control(packet.param1,
                                     packet.param2,
                                     packet.param3,
                                     packet.param4,
                                     packet.param5,
                                     packet.param6)) {
                rover.log_picture();
            }
            result = MAV_RESULT_ACCEPTED;
            break;
#endif // CAMERA == ENABLED

            case MAV_CMD_DO_MOUNT_CONTROL:
#if MOUNT == ENABLED
                rover.camera_mount.control(packet.param1, packet.param2, packet.param3, (MAV_MOUNT_MODE) packet.param7);
#endif
                break;

            case MAV_CMD_MISSION_START:
                rover.set_mode(AUTO);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_CMD_PREFLIGHT_CALIBRATION:
                if(hal.util->get_soft_armed()) {
                    result = MAV_RESULT_FAILED;
                    break;
                }
                if (is_equal(packet.param1,1.0f)) {
                    rover.ins.init_gyro();
                    if (rover.ins.gyro_calibrated_ok_all()) {
                        rover.ahrs.reset_gyro_drift();
                        result = MAV_RESULT_ACCEPTED;
                    } else {
                        result = MAV_RESULT_FAILED;
                    }
                } else if (is_equal(packet.param3,1.0f)) {
                    rover.init_barometer();
                    result = MAV_RESULT_ACCEPTED;
                } else if (is_equal(packet.param4,1.0f)) {
                    rover.trim_radio();
                    result = MAV_RESULT_ACCEPTED;
                } else if (is_equal(packet.param5,1.0f)) {
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

                } else if (is_equal(packet.param5,2.0f)) {
                    // start with gyro calibration
                    rover.ins.init_gyro();
                    // accel trim
                    float trim_roll, trim_pitch;
                    if(rover.ins.calibrate_trim(trim_roll, trim_pitch)) {
                        // reset ahrs's trim to suggested values from calibration routine
                        rover.ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
                        result = MAV_RESULT_ACCEPTED;
                    } else {
                        result = MAV_RESULT_FAILED;
                    }
                }
                else {
                    send_text(MAV_SEVERITY_WARNING, "Unsupported preflight calibration");
                }
                break;

            case MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                if (is_equal(packet.param1,2.0f)) {
                    // save first compass's offsets
                    rover.compass.set_and_save_offsets(0, packet.param2, packet.param3, packet.param4);
                    result = MAV_RESULT_ACCEPTED;
                }
                if (is_equal(packet.param1,5.0f)) {
                    // save secondary compass's offsets
                    rover.compass.set_and_save_offsets(1, packet.param2, packet.param3, packet.param4);
                    result = MAV_RESULT_ACCEPTED;
                }
                break;

        case MAV_CMD_DO_SET_MODE:
            switch ((uint16_t)packet.param1) {
            case MAV_MODE_MANUAL_ARMED:
            case MAV_MODE_MANUAL_DISARMED:
                rover.set_mode(MANUAL);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_MODE_AUTO_ARMED:
            case MAV_MODE_AUTO_DISARMED:
                rover.set_mode(AUTO);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_MODE_STABILIZE_DISARMED:
            case MAV_MODE_STABILIZE_ARMED:
                rover.set_mode(LEARNING);
                result = MAV_RESULT_ACCEPTED;
                break;

            default:
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_DO_SET_SERVO:
            if (rover.ServoRelayEvents.do_set_servo(packet.param1, packet.param2)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_REPEAT_SERVO:
            if (rover.ServoRelayEvents.do_repeat_servo(packet.param1, packet.param2, packet.param3, packet.param4*1000)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_SET_RELAY:
            if (rover.ServoRelayEvents.do_set_relay(packet.param1, packet.param2)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_REPEAT_RELAY:
            if (rover.ServoRelayEvents.do_repeat_relay(packet.param1, packet.param2, packet.param3*1000)) {
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

        case MAV_CMD_COMPONENT_ARM_DISARM:
            if (is_equal(packet.param1,1.0f)) {
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
            }
            break;

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
            result = rover.compass.handle_mag_cal_command(packet);
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
            handle_set_mode(msg, FUNCTOR_BIND(&rover, &Rover::mavlink_set_mode, bool, uint8_t));
            break;
        }

    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        {
            handle_mission_request_list(rover.mission, msg);
            break;
        }


	// XXX read a WP from EEPROM and send it to the GCS
    case MAVLINK_MSG_ID_MISSION_REQUEST:
    {
        handle_mission_request(rover.mission, msg);
        break;
    }


    case MAVLINK_MSG_ID_MISSION_ACK:
        {
            // not used
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
            handle_mission_clear_all(rover.mission, msg);
            break;
        }

    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
        {
            handle_mission_set_current(rover.mission, msg);
            break;
        }

    case MAVLINK_MSG_ID_MISSION_COUNT:
        {
            handle_mission_count(rover.mission, msg);
            break;
        }

    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
    {
        handle_mission_write_partial_list(rover.mission, msg);
        break;
    }

    // GCS has sent us a mission item, store to EEPROM
    case MAVLINK_MSG_ID_MISSION_ITEM:
        {
            if (handle_mission_item(msg, rover.mission)) {
                rover.DataFlash.Log_Write_EntireMission(rover.mission);
            }
            break;
        }


    case MAVLINK_MSG_ID_PARAM_SET:
        {
            handle_param_set(msg, &rover.DataFlash);
            break;
        }

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
    {
        // allow override of RC channel values for HIL
        // or for complete GCS control of switch position
        // and RC PWM values.
        if(msg->sysid != rover.g.sysid_my_gcs) break;                         // Only accept control from our gcs
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
			if(msg->sysid != rover.g.sysid_my_gcs) break;
            rover.last_heartbeat_ms = rover.failsafe.rc_override_timer = AP_HAL::millis();
            rover.failsafe_trigger(FAILSAFE_EVENT_GCS, false);
            break;
        }

#if HIL_MODE != HIL_MODE_DISABLED
	case MAVLINK_MSG_ID_HIL_STATE:
		{
			mavlink_hil_state_t packet;
			mavlink_msg_hil_state_decode(msg, &packet);
			
            // set gps hil sensor
            Location loc;
            loc.lat = packet.lat;
            loc.lng = packet.lon;
            loc.alt = packet.alt/10;
            Vector3f vel(packet.vx, packet.vy, packet.vz);
            vel *= 0.01f;
            
            gps.setHIL(0, AP_GPS::GPS_OK_FIX_3D,
                       packet.time_usec/1000,
                       loc, vel, 10, 0, true);
			
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
#endif // HIL_MODE

#if CAMERA == ENABLED
    //deprecated. Use MAV_CMD_DO_DIGICAM_CONFIGURE
    case MAVLINK_MSG_ID_DIGICAM_CONFIGURE:
    {
        break;
    }

    //deprecated. Use MAV_CMD_DO_DIGICAM_CONFIGURE
    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
    {
        rover.camera.control_msg(msg);
        rover.log_picture();
        break;
    }
#endif // CAMERA == ENABLED

#if MOUNT == ENABLED
    //deprecated. Use MAV_CMD_DO_MOUNT_CONFIGURE
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:
		{
			rover.camera_mount.configure_msg(msg);
			break;
		}

    //deprecated. Use MAV_CMD_DO_MOUNT_CONTROL
    case MAVLINK_MSG_ID_MOUNT_CONTROL:
		{
			rover.camera_mount.control_msg(msg);
			break;
		}
#endif // MOUNT == ENABLED

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS:
        {
            handle_radio_status(msg, rover.DataFlash, rover.should_log(MASK_LOG_PM));
            break;
        }

    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
    case MAVLINK_MSG_ID_LOG_ERASE:
        rover.in_log_download = true;
        /* no break */
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        if (!rover.in_mavlink_delay) {
            handle_log_message(msg, rover.DataFlash);
        }
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        rover.in_log_download = false;
        if (!rover.in_mavlink_delay) {
            handle_log_message(msg, rover.DataFlash);
        }
        break;

    case MAVLINK_MSG_ID_SERIAL_CONTROL:
        handle_serial_control(msg, rover.gps);
        break;

    case MAVLINK_MSG_ID_GPS_INJECT_DATA:
        handle_gps_inject(msg, rover.gps);
        break;

    case MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS:
        rover.DataFlash.remote_log_block_status_msg(chan, msg);
        break;

    case MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST:
        send_autopilot_version(FIRMWARE_VERSION);
        break;

    } // end switch
} // end handle mavlink
