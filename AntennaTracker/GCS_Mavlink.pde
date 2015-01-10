// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// default sensors are present and healthy: gyro, accelerometer, barometer, rate_control, attitude_stabilization, yaw_position, altitude control, x/y position control, motor_control
#define MAVLINK_SENSOR_PRESENT_DEFAULT (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_YAW_POSITION | MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS)

// use this to prevent recursion during sensor init
static bool in_mavlink_delay;

// check if a message will fit in the payload space available
#define CHECK_PAYLOAD_SIZE(id) if (txspace < MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_ ## id ## _LEN) return false

/*
 *  !!NOTE!!
 *
 *  the use of NOINLINE separate functions for each message type avoids
 *  a compiler bug in gcc that would cause it to use far more stack
 *  space than is needed. Without the NOINLINE we use the sum of the
 *  stack needed for each message type. Please be careful to follow the
 *  pattern below when adding any new messages
 */

static NOINLINE void send_heartbeat(mavlink_channel_t chan)
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

    mavlink_msg_heartbeat_send(
        chan,
        MAV_TYPE_ANTENNA_TRACKER,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        custom_mode,
        system_status);
}

static NOINLINE void send_attitude(mavlink_channel_t chan)
{
    Vector3f omega = ahrs.get_gyro();
    mavlink_msg_attitude_send(
        chan,
        hal.scheduler->millis(),
        ahrs.roll,
        ahrs.pitch,
        ahrs.yaw,
        omega.x,
        omega.y,
        omega.z);
}


static void NOINLINE send_location(mavlink_channel_t chan)
{
    uint32_t fix_time;
    if (gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
        fix_time = gps.last_fix_time_ms();
    } else {
        fix_time = hal.scheduler->millis();
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

static void NOINLINE send_radio_out(mavlink_channel_t chan)
{
    mavlink_msg_servo_output_raw_send(
        chan,
        hal.scheduler->micros(),
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

static void NOINLINE send_hwstatus(mavlink_channel_t chan)
{
    mavlink_msg_hwstatus_send(
        chan,
        0,
        hal.i2c->lockup_count());
}

static void NOINLINE send_waypoint_request(mavlink_channel_t chan)
{
    gcs[chan-MAVLINK_COMM_0].queued_waypoint_send();
}

static void NOINLINE send_statustext(mavlink_channel_t chan)
{
    mavlink_statustext_t *s = &gcs[chan-MAVLINK_COMM_0].pending_status;
    mavlink_msg_statustext_send(
        chan,
        s->severity,
        s->text);
}

static void NOINLINE send_nav_controller_output(mavlink_channel_t chan)
{
    mavlink_msg_nav_controller_output_send(
        chan,
        0,
        nav_status.pitch,
        nav_status.bearing,
        nav_status.bearing,
        nav_status.distance,
        nav_status.altitude_difference,
        0,
        0);
}


// report simulator state
static void NOINLINE send_simstate(mavlink_channel_t chan)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    sitl.simstate_send(chan);
#endif
}


// try to send a message, return false if it won't fit in the serial tx buffer
bool GCS_MAVLINK::try_send_message(enum ap_message id)
{
    uint16_t txspace = comm_get_txspace(chan);
    switch (id) {
    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        send_heartbeat(chan);
        return true;

    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        send_attitude(chan);
        break;

    case MSG_LOCATION:
        CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
        send_location(chan);
        break;

    case MSG_NAV_CONTROLLER_OUTPUT:
        CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
        send_nav_controller_output(chan);
        break;

    case MSG_GPS_RAW:
        CHECK_PAYLOAD_SIZE(GPS_RAW_INT);
        gcs[chan-MAVLINK_COMM_0].send_gps_raw(gps);
        break;

    case MSG_RADIO_IN:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_RAW);
        gcs[chan-MAVLINK_COMM_0].send_radio_in(0);
        break;

    case MSG_RADIO_OUT:
        CHECK_PAYLOAD_SIZE(SERVO_OUTPUT_RAW);
        send_radio_out(chan);
        break;

    case MSG_RAW_IMU1:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
        gcs[chan-MAVLINK_COMM_0].send_raw_imu(ins, compass);
        break;

    case MSG_RAW_IMU2:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE);
        gcs[chan-MAVLINK_COMM_0].send_scaled_pressure(barometer);
        break;

    case MSG_RAW_IMU3:
        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
        gcs[chan-MAVLINK_COMM_0].send_sensor_offsets(ins, compass, barometer);
        break;

    case MSG_NEXT_PARAM:
        CHECK_PAYLOAD_SIZE(PARAM_VALUE);
        gcs[chan-MAVLINK_COMM_0].queued_param_send();
        break;

    case MSG_NEXT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_REQUEST);
        send_waypoint_request(chan);
        break;

    case MSG_STATUSTEXT:
        CHECK_PAYLOAD_SIZE(STATUSTEXT);
        send_statustext(chan);
        break;

    case MSG_AHRS:
        CHECK_PAYLOAD_SIZE(AHRS);
        gcs[chan-MAVLINK_COMM_0].send_ahrs(ahrs);
        break;

    case MSG_SIMSTATE:
        CHECK_PAYLOAD_SIZE(SIMSTATE);
        send_simstate(chan);
        break;

    case MSG_HWSTATUS:
        CHECK_PAYLOAD_SIZE(HWSTATUS);
        send_hwstatus(chan);
        break;

    case MSG_SERVO_OUT:
    case MSG_EXTENDED_STATUS1:
    case MSG_EXTENDED_STATUS2:
    case MSG_RETRY_DEFERRED:
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
        break; // just here to prevent a warning
    }
    return true;
}


/*
  default stream rates to 1Hz
 */
const AP_Param::GroupInfo GCS_MAVLINK::var_info[] PROGMEM = {
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
bool GCS_MAVLINK::stream_trigger(enum streams stream_num)
{
    if (stream_num >= NUM_STREAMS) {
        return false;
    }
    float rate = (uint8_t)streamRates[stream_num].get();

    // send at a much lower rate during parameter sends
    if (_queued_parameter != NULL) {
        rate *= 0.25;
    }

    if (rate <= 0) {
        return false;
    }

    if (stream_ticks[stream_num] == 0) {
        // we're triggering now, setup the next trigger point
        if (rate > 50) {
            rate = 50;
        }
        stream_ticks[stream_num] = (50 / rate) + stream_slowdown;
        return true;
    }

    // count down at 50Hz
    stream_ticks[stream_num]--;
    return false;
}

void
GCS_MAVLINK::data_stream_send(void)
{
    if (_queued_parameter != NULL) {
        if (streamRates[STREAM_PARAMS].get() <= 0) {
            streamRates[STREAM_PARAMS].set(10);
        }
        if (stream_trigger(STREAM_PARAMS)) {
            send_message(MSG_NEXT_PARAM);
        }
    }

    if (in_mavlink_delay) {
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
    }
}

/*
  We eavesdrop on MAVLINK_MSG_ID_GLOBAL_POSITION_INT and
  MAVLINK_MSG_ID_SCALED_PRESSUREs
*/
void mavlink_snoop(const mavlink_message_t* msg)
{
    switch (msg->msgid) {
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

void GCS_MAVLINK::handleMessage(mavlink_message_t* msg)
{
    switch (msg->msgid) {

    // If we are currently operating as a proxy for a remote, 
    // alas we have to look inside each packet to see if its for us or for the remote
    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
    {
        handle_request_data_stream(msg, true);
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
        send_text_P(SEVERITY_LOW,PSTR("command received: "));
        
        switch(packet.command) {
            
            case MAV_CMD_PREFLIGHT_CALIBRATION:
            {
                if (packet.param1 == 1 ||
                    packet.param2 == 1) {
                    calibrate_ins();
                } else if (packet.param3 == 1) {
                    init_barometer();
                    // zero the altitude difference on next baro update
                    nav_status.need_altitude_calibration = true;
                }
                if (packet.param4 == 1) {
                    // Cant trim radio
                }
#if !defined( __AVR_ATmega1280__ )
                if (packet.param5 == 1) {
                    float trim_roll, trim_pitch;
                    AP_InertialSensor_UserInteract_MAVLink interact(chan);
                    if(ins.calibrate_accel(&interact, trim_roll, trim_pitch)) {
                        // reset ahrs's trim to suggested values from calibration routine
                        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
                    }
                }
#endif
                result = MAV_RESULT_ACCEPTED;
                break;
            }

            case MAV_CMD_COMPONENT_ARM_DISARM:
                if (packet.target_component == MAV_COMP_ID_SYSTEM_CONTROL) {
                    if (packet.param1 == 1.0f) {
                        arm_servos();
                        result = MAV_RESULT_ACCEPTED;
                    } else if (packet.param1 == 0.0f)  {
                        disarm_servos();
                        result = MAV_RESULT_ACCEPTED;
                    } else {
                        result = MAV_RESULT_UNSUPPORTED;
                    }
                } else {
                    result = MAV_RESULT_UNSUPPORTED;
                }
            break;
            
            case MAV_CMD_DO_SET_MODE:
                switch ((uint16_t)packet.param1) {
                    case MAV_MODE_MANUAL_ARMED:
                    case MAV_MODE_MANUAL_DISARMED:
                        set_mode(MANUAL);
                        result = MAV_RESULT_ACCEPTED;
                        break;

                    case MAV_MODE_AUTO_ARMED:
                    case MAV_MODE_AUTO_DISARMED:
                        set_mode(AUTO);
                        result = MAV_RESULT_ACCEPTED;
                        break;

                    default:
                        result = MAV_RESULT_UNSUPPORTED;
                }
                break;

                // mavproxy/mavutil sends this when auto command is entered 
            case MAV_CMD_MISSION_START:
                set_mode(AUTO);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            {
                if (packet.param1 == 1 || packet.param1 == 3) {
                    // when packet.param1 == 3 we reboot to hold in bootloader
                    hal.scheduler->reboot(packet.param1 == 3);
                    result = MAV_RESULT_ACCEPTED;
                }
                break;
            }

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
            set_home(tell_command); // New home in EEPROM
            send_text_P(SEVERITY_LOW,PSTR("new HOME received"));
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
        tracking_manual_control(packet);
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

    case MAVLINK_MSG_ID_SET_MODE:
    {
        handle_set_mode(msg, mavlink_set_mode);
        break;
    }

#if HAL_CPU_CLASS > HAL_CPU_CLASS_16
    case MAVLINK_MSG_ID_SERIAL_CONTROL:
        handle_serial_control(msg, gps);
        break;
#endif

    } // end switch
} // end handle mavlink


/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
static void mavlink_delay_cb()
{
    static uint32_t last_1hz, last_50hz, last_5s;
    if (!gcs[0].initialised) return;

    in_mavlink_delay = true;

    uint32_t tnow = hal.scheduler->millis();
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
        gcs_send_text_P(SEVERITY_LOW, PSTR("Initialising APM..."));
    }
    in_mavlink_delay = false;
}

/*
 *  send a message on both GCS links
 */
static void gcs_send_message(enum ap_message id)
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
static void gcs_data_stream_send(void)
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
static void gcs_update(void)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].update(NULL);
        }
    }
}

static void gcs_send_text_P(gcs_severity severity, const prog_char_t *str)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].send_text_P(severity, str);
        }
    }
#if LOGGING_ENABLED == ENABLED
    DataFlash.Log_Write_Message_P(str);
#endif
}

/*
 *  send a low priority formatted message to the GCS
 *  only one fits in the queue, so if you send more than one before the
 *  last one gets into the serial buffer then the old one will be lost
 */
void gcs_send_text_fmt(const prog_char_t *fmt, ...)
{
    va_list arg_list;
    gcs[0].pending_status.severity = (uint8_t)SEVERITY_LOW;
    va_start(arg_list, fmt);
    hal.util->vsnprintf_P((char *)gcs[0].pending_status.text,
            sizeof(gcs[0].pending_status.text), fmt, arg_list);
    va_end(arg_list);
#if LOGGING_ENABLED == ENABLED
    DataFlash.Log_Write_Message(gcs[0].pending_status.text);
#endif
    gcs[0].send_message(MSG_STATUSTEXT);
    for (uint8_t i=1; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].pending_status = gcs[0].pending_status;
            gcs[i].send_message(MSG_STATUSTEXT);
        }
    }
}

/**
   retry any deferred messages
 */
static void gcs_retry_deferred(void)
{
    gcs_send_message(MSG_RETRY_DEFERRED);
}

