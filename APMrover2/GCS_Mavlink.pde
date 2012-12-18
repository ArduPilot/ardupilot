// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// use this to prevent recursion during sensor init
static bool in_mavlink_delay;

// true when we have received at least 1 MAVLink packet
static bool mavlink_active;

// check if a message will fit in the payload space available
#define CHECK_PAYLOAD_SIZE(id) if (payload_space < MAVLINK_MSG_ID_## id ##_LEN) return false

// prototype this for use inside the GCS class
void gcs_send_text_fmt(const prog_char_t *fmt, ...);

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
        base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;
    case LEARNING:
        base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;
    case AUTO:
    case RTL:
    case GUIDED:
    case CIRCLE:
        base_mode = MAV_MODE_FLAG_GUIDED_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    case INITIALISING:
        system_status = MAV_STATE_CALIBRATING;
        break;
    }

#if ENABLE_STICK_MIXING==ENABLED
    if (control_mode != INITIALISING) {
        // all modes except INITIALISING have some form of manual
        // override if stick mixing is enabled
        base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }
#endif

#if HIL_MODE != HIL_MODE_DISABLED
    base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
#endif

    // we are armed if we are not initialising
    if (control_mode != INITIALISING) {
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

static NOINLINE void send_attitude(mavlink_channel_t chan)
{
    Vector3f omega = ahrs.get_gyro();
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

static NOINLINE void send_extended_status1(mavlink_channel_t chan, uint16_t packet_drops)
{
    uint32_t control_sensors_present = 0;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;

    // first what sensors/controllers we have
    control_sensors_present |= (1<<0); // 3D gyro present
    control_sensors_present |= (1<<1); // 3D accelerometer present
    if (g.compass_enabled) {
        control_sensors_present |= (1<<2); // compass present
    }
    control_sensors_present |= (1<<3); // absolute pressure sensor present
    if (g_gps != NULL && g_gps->status() == GPS::GPS_OK) {
        control_sensors_present |= (1<<5); // GPS present
    }
    control_sensors_present |= (1<<10); // 3D angular rate control
    control_sensors_present |= (1<<11); // attitude stabilisation
    control_sensors_present |= (1<<12); // yaw position
    control_sensors_present |= (1<<13); // altitude control
    control_sensors_present |= (1<<14); // X/Y position control
    control_sensors_present |= (1<<15); // motor control

    // now what sensors/controllers are enabled

    // first the sensors
    control_sensors_enabled = control_sensors_present & 0x1FF;

    // now the controllers
    control_sensors_enabled = control_sensors_present & 0x1FF;

    switch (control_mode) {
    case MANUAL:
        break;

    case LEARNING:
        control_sensors_enabled |= (1<<10); // 3D angular rate control
        control_sensors_enabled |= (1<<11); // attitude stabilisation
        break;

    case AUTO:
    case RTL:
        control_sensors_enabled |= (1<<10); // 3D angular rate control
        control_sensors_enabled |= (1<<11); // attitude stabilisation
        control_sensors_enabled |= (1<<12); // yaw position
        control_sensors_enabled |= (1<<13); // altitude control
        control_sensors_enabled |= (1<<14); // X/Y position control
        control_sensors_enabled |= (1<<15); // motor control
        break;

    case INITIALISING:
        break;
    }

    // at the moment all sensors/controllers are assumed healthy
    control_sensors_health = control_sensors_present;

    if (!compass.healthy) {
        control_sensors_health &= ~(1<<2); // compass
    }
    if (!compass.use_for_yaw()) {
        control_sensors_enabled &= ~(1<<2); // compass
    }

    uint16_t battery_current = -1;
    uint8_t  battery_remaining = -1;

    if (current_total1 != 0 && g.pack_capacity != 0) {
        battery_remaining = (100.0 * (g.pack_capacity - current_total1) / g.pack_capacity);
    }
    if (current_total1 != 0) {
        battery_current = current_amps1 * 100;
    }

    if (g.battery_monitoring == 3) {
        /*setting a out-of-range value.
         *  It informs to external devices that
         *  it cannot be calculated properly just by voltage*/
        battery_remaining = 150;
    }

    mavlink_msg_sys_status_send(
        chan,
        control_sensors_present,
        control_sensors_enabled,
        control_sensors_health,
        (uint16_t)(load * 1000),
        battery_voltage1 * 1000, // mV
        battery_current,        // in 10mA units
        battery_remaining,      // in %
        0, // comm drops %,
        0, // comm drops in pkts,
        0, 0, 0, 0);

}

static void NOINLINE send_meminfo(mavlink_channel_t chan)
{
#if CONFIG_HAL_BOARD != HAL_BOARD_AVR_SITL
    extern unsigned __brkval;
    mavlink_msg_meminfo_send(chan, __brkval, memcheck_available_memory());
#endif
}

static void NOINLINE send_location(mavlink_channel_t chan)
{
    uint32_t fix_time;
    // if we have a GPS fix, take the time as the last fix time. That
    // allows us to correctly calculate velocities and extrapolate
    // positions.
    // If we don't have a GPS fix then we are dead reckoning, and will
    // use the current boot time as the fix time.    
    if (g_gps->status() == GPS::GPS_OK) {
        fix_time = g_gps->last_fix_time;
    } else {
        fix_time = millis();
    }
    mavlink_msg_global_position_int_send(
        chan,
        fix_time,
        current_loc.lat,                // in 1E7 degrees
        current_loc.lng,                // in 1E7 degrees
        g_gps->altitude * 10,             // millimeters above sea level
        (current_loc.alt - home.alt) * 10,           // millimeters above ground
        g_gps->velocity_north() * 100,  // X speed cm/s (+ve North)
        g_gps->velocity_east()  * 100,  // Y speed cm/s (+ve East)
        g_gps->velocity_down()  * -100, // Z speed cm/s (+ve up)
        ahrs.yaw_sensor);
}

static void NOINLINE send_nav_controller_output(mavlink_channel_t chan)
{
    int16_t bearing = nav_bearing / 100;
    mavlink_msg_nav_controller_output_send(
        chan,
        nav_roll / 1.0e2,
        0,
        bearing,
        target_bearing / 100,
        wp_distance,
        altitude_error / 1.0e2,
        groundspeed_error,
        crosstrack_error);
}

static void NOINLINE send_gps_raw(mavlink_channel_t chan)
{
    uint8_t fix = g_gps->status();
    if (fix == GPS::GPS_OK) {
        fix = 3;
    }

    mavlink_msg_gps_raw_int_send(
        chan,
        g_gps->last_fix_time*(uint64_t)1000,
        fix,
        g_gps->latitude,      // in 1E7 degrees
        g_gps->longitude,     // in 1E7 degrees
        g_gps->altitude * 10, // in mm
        g_gps->hdop,
        65535,
        g_gps->ground_speed,  // cm/s
        g_gps->ground_course, // 1/100 degrees,
        g_gps->num_sats);
}

static void NOINLINE send_servo_out(mavlink_channel_t chan)
{
    // normalized values scaled to -10000 to 10000
    // This is used for HIL.  Do not change without discussing with
    // HIL maintainers
    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0, // port 0
        10000 * g.channel_roll.norm_output(),
        10000 * g.channel_pitch.norm_output(),
        10000 * g.channel_throttle.norm_output(),
        10000 * g.channel_rudder.norm_output(),
        0,
        0,
        0,
        0,
        receiver_rssi);
}

static void NOINLINE send_radio_in(mavlink_channel_t chan)
{
    mavlink_msg_rc_channels_raw_send(
        chan,
        millis(),
        0, // port
        hal.rcin->read(CH_1),
        hal.rcin->read(CH_2),
        hal.rcin->read(CH_3),
        hal.rcin->read(CH_4),
        hal.rcin->read(CH_5),
        hal.rcin->read(CH_6),
        hal.rcin->read(CH_7),
        hal.rcin->read(CH_8),
        receiver_rssi);
}

static void NOINLINE send_radio_out(mavlink_channel_t chan)
{
#if HIL_MODE == HIL_MODE_DISABLED || HIL_SERVOS
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
#else
    extern RC_Channel* rc_ch[8];
    mavlink_msg_servo_output_raw_send(
        chan,
        micros(),
        0,     // port
        rc_ch[0]->radio_out,
        rc_ch[1]->radio_out,
        rc_ch[2]->radio_out,
        rc_ch[3]->radio_out,
        rc_ch[4]->radio_out,
        rc_ch[5]->radio_out,
        rc_ch[6]->radio_out,
        rc_ch[7]->radio_out);
#endif
}

static void NOINLINE send_vfr_hud(mavlink_channel_t chan)
{
    mavlink_msg_vfr_hud_send(
        chan,
        (float)g_gps->ground_speed / 100.0,
        (float)g_gps->ground_speed / 100.0,
        (ahrs.yaw_sensor / 100) % 360,
        (uint16_t)(100 * g.channel_throttle.norm_output()),
        current_loc.alt / 100.0,
        0);
}

static void NOINLINE send_raw_imu1(mavlink_channel_t chan)
{
    Vector3f accel = ins.get_accel();
    Vector3f gyro = ins.get_gyro();

    mavlink_msg_raw_imu_send(
        chan,
        micros(),
        accel.x * 1000.0 / gravity,
        accel.y * 1000.0 / gravity,
        accel.z * 1000.0 / gravity,
        gyro.x * 1000.0,
        gyro.y * 1000.0,
        gyro.z * 1000.0,
        compass.mag_x,
        compass.mag_y,
        compass.mag_z);
}

static void NOINLINE send_raw_imu3(mavlink_channel_t chan)
{
    Vector3f mag_offsets = compass.get_offsets();
    Vector3f accel_offsets = ins.get_accel_offsets();
    Vector3f gyro_offsets = ins.get_gyro_offsets();

    mavlink_msg_sensor_offsets_send(chan,
                                    mag_offsets.x,
                                    mag_offsets.y,
                                    mag_offsets.z,
                                    compass.get_declination(),
                                    0, 0,
                                    gyro_offsets.x,
                                    gyro_offsets.y,
                                    gyro_offsets.z,
                                    accel_offsets.x,
                                    accel_offsets.y,
                                    accel_offsets.z);
}

static void NOINLINE send_ahrs(mavlink_channel_t chan)
{
    Vector3f omega_I = ahrs.get_gyro_drift();
    mavlink_msg_ahrs_send(
        chan,
        omega_I.x,
        omega_I.y,
        omega_I.z,
        0,
        0,
        ahrs.get_error_rp(),
        ahrs.get_error_yaw());
}

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
// report simulator state
static void NOINLINE send_simstate(mavlink_channel_t chan)
{
    sitl.simstate_send(chan);
}
#endif

static void NOINLINE send_hwstatus(mavlink_channel_t chan)
{
    mavlink_msg_hwstatus_send(
        chan,
        board_voltage(),
        hal.i2c->lockup_count());
}

static void NOINLINE send_current_waypoint(mavlink_channel_t chan)
{
    mavlink_msg_mission_current_send(
        chan,
        g.command_index);
}

static void NOINLINE send_statustext(mavlink_channel_t chan)
{
    mavlink_statustext_t *s = (chan == MAVLINK_COMM_0?&gcs0.pending_status:&gcs3.pending_status);
    mavlink_msg_statustext_send(
        chan,
        s->severity,
        s->text);
}

// are we still delaying telemetry to try to avoid Xbee bricking?
static bool telemetry_delayed(mavlink_channel_t chan)
{
    uint32_t tnow = millis() >> 10;
    if (tnow > g.telem_delay) {
        return false;
    }
#if USB_MUX_PIN > 0
    if (chan == MAVLINK_COMM_0 && usb_connected) {
        // this is an APM2 with USB telemetry
        return false;
    }
    // we're either on the 2nd UART, or no USB cable is connected
    // we need to delay telemetry
    return true;
#else
    if (chan == MAVLINK_COMM_0) {
        // we're on the USB port
        return false;
    }
    // don't send telemetry yet
    return true;
#endif
}


// try to send a message, return false if it won't fit in the serial tx buffer
static bool mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    int16_t payload_space = comm_get_txspace(chan) - MAVLINK_NUM_NON_PAYLOAD_BYTES;

    if (telemetry_delayed(chan)) {
        return false;
    }

    switch (id) {
    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        send_heartbeat(chan);
        return true;

    case MSG_EXTENDED_STATUS1:
        CHECK_PAYLOAD_SIZE(SYS_STATUS);
        send_extended_status1(chan, packet_drops);
        break;

    case MSG_EXTENDED_STATUS2:
        CHECK_PAYLOAD_SIZE(MEMINFO);
        send_meminfo(chan);
        break;

    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        send_attitude(chan);
        break;

    case MSG_LOCATION:
        CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
        send_location(chan);
        break;

    case MSG_NAV_CONTROLLER_OUTPUT:
        if (control_mode != MANUAL) {
            CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
            send_nav_controller_output(chan);
        }
        break;

    case MSG_GPS_RAW:
        CHECK_PAYLOAD_SIZE(GPS_RAW_INT);
        send_gps_raw(chan);
        break;

    case MSG_SERVO_OUT:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_SCALED);
        send_servo_out(chan);
        break;

    case MSG_RADIO_IN:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_RAW);
        send_radio_in(chan);
        break;

    case MSG_RADIO_OUT:
        CHECK_PAYLOAD_SIZE(SERVO_OUTPUT_RAW);
        send_radio_out(chan);
        break;

    case MSG_VFR_HUD:
        CHECK_PAYLOAD_SIZE(VFR_HUD);
        send_vfr_hud(chan);
        break;

    case MSG_RAW_IMU1:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
        send_raw_imu1(chan);
        break;

    case MSG_RAW_IMU3:
        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
        send_raw_imu3(chan);
        break;

    case MSG_CURRENT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_CURRENT);
        send_current_waypoint(chan);
        break;

    case MSG_NEXT_PARAM:
        CHECK_PAYLOAD_SIZE(PARAM_VALUE);
        if (chan == MAVLINK_COMM_0) {
            gcs0.queued_param_send();
        } else if (gcs3.initialised) {
            gcs3.queued_param_send();
        }
        break;

    case MSG_NEXT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_REQUEST);
        if (chan == MAVLINK_COMM_0) {
            gcs0.queued_waypoint_send();
        } else if (gcs3.initialised) {
            gcs3.queued_waypoint_send();
        }
        break;

    case MSG_STATUSTEXT:
        CHECK_PAYLOAD_SIZE(STATUSTEXT);
        send_statustext(chan);
        break;

    case MSG_AHRS:
        CHECK_PAYLOAD_SIZE(AHRS);
        send_ahrs(chan);
        break;

    case MSG_SIMSTATE:
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
        CHECK_PAYLOAD_SIZE(SIMSTATE);
        send_simstate(chan);
#endif
        break;

    case MSG_HWSTATUS:
        CHECK_PAYLOAD_SIZE(HWSTATUS);
        send_hwstatus(chan);
        break;

    case MSG_RETRY_DEFERRED:
        break; // just here to prevent a warning
	}
    return true;
}


#define MAX_DEFERRED_MESSAGES MSG_RETRY_DEFERRED
static struct mavlink_queue {
    enum ap_message deferred_messages[MAX_DEFERRED_MESSAGES];
    uint8_t next_deferred_message;
    uint8_t num_deferred_messages;
} mavlink_queue[2];

// send a message using mavlink
static void mavlink_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    uint8_t i, nextid;
    struct mavlink_queue *q = &mavlink_queue[(uint8_t)chan];

    // see if we can send the deferred messages, if any
    while (q->num_deferred_messages != 0) {
        if (!mavlink_try_send_message(chan,
                                      q->deferred_messages[q->next_deferred_message],
                                      packet_drops)) {
            break;
        }
        q->next_deferred_message++;
        if (q->next_deferred_message == MAX_DEFERRED_MESSAGES) {
            q->next_deferred_message = 0;
        }
        q->num_deferred_messages--;
    }

    if (id == MSG_RETRY_DEFERRED) {
        return;
    }

    // this message id might already be deferred
    for (i=0, nextid = q->next_deferred_message; i < q->num_deferred_messages; i++) {
        if (q->deferred_messages[nextid] == id) {
            // its already deferred, discard
            return;
        }
        nextid++;
        if (nextid == MAX_DEFERRED_MESSAGES) {
            nextid = 0;
        }
    }

    if (q->num_deferred_messages != 0 ||
        !mavlink_try_send_message(chan, id, packet_drops)) {
        // can't send it now, so defer it
        if (q->num_deferred_messages == MAX_DEFERRED_MESSAGES) {
            // the defer buffer is full, discard
            return;
        }
        nextid = q->next_deferred_message + q->num_deferred_messages;
        if (nextid >= MAX_DEFERRED_MESSAGES) {
            nextid -= MAX_DEFERRED_MESSAGES;
        }
        q->deferred_messages[nextid] = id;
        q->num_deferred_messages++;
    }
}

void mavlink_send_text(mavlink_channel_t chan, gcs_severity severity, const char *str)
{
    if (telemetry_delayed(chan)) {
        return;
    }

    if (severity == SEVERITY_LOW) {
        // send via the deferred queuing system
        mavlink_statustext_t *s = (chan == MAVLINK_COMM_0?&gcs0.pending_status:&gcs3.pending_status);
        s->severity = (uint8_t)severity;
        strncpy((char *)s->text, str, sizeof(s->text));
        mavlink_send_message(chan, MSG_STATUSTEXT, 0);
    } else {
        // send immediately
        mavlink_msg_statustext_send(chan, severity, str);
    }
}

const AP_Param::GroupInfo GCS_MAVLINK::var_info[] PROGMEM = {
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK, streamRateRawSensors,      0),
	AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK, streamRateExtendedStatus,  0),
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK, streamRateRCChannels,      0),
	AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK, streamRateRawController,   0),
	AP_GROUPINFO("POSITION", 4, GCS_MAVLINK, streamRatePosition,        0),
	AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK, streamRateExtra1,          0),
	AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK, streamRateExtra2,          0),
	AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK, streamRateExtra3,          0),
	AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK, streamRateParams,          0),
    AP_GROUPEND
};


GCS_MAVLINK::GCS_MAVLINK() :
    packet_drops(0),
    waypoint_send_timeout(1000), // 1 second
    waypoint_receive_timeout(1000) // 1 second
{
}

void
GCS_MAVLINK::init(AP_HAL::UARTDriver *port)
{
    GCS_Class::init(port);
    if (port == (AP_HAL::BetterStream*)hal.uartA) {
        mavlink_comm_0_port = port;
        chan = MAVLINK_COMM_0;
    }else{
        mavlink_comm_1_port = port;
        chan = MAVLINK_COMM_1;
    }
    _queued_parameter = NULL;
    reset_cli_timeout();
}

void
GCS_MAVLINK::update(void)
{
    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;
	status.packet_rx_drop_count = 0;

    // process received bytes
    while (comm_get_available(chan))
    {
        uint8_t c = comm_receive_ch(chan);

#if CLI_ENABLED == ENABLED
        /* allow CLI to be started by hitting enter 3 times, if no
         *  heartbeat packets have been received */
        if (mavlink_active == 0 && (millis() - _cli_timeout) < 30000) {
            if (c == '\n' || c == '\r') {
                crlf_count++;
            } else {
                crlf_count = 0;
            }
            if (crlf_count == 3) {
                run_cli(_port);
            }
        }
#endif

        // Try to get a new message
        if (mavlink_parse_char(chan, c, &msg, &status)) {
            // we exclude radio packets to make it possible to use the
            // CLI over the radio
            if (msg.msgid != MAVLINK_MSG_ID_RADIO) {
                mavlink_active = true;
            }
            handleMessage(&msg);
        }
    }

    // Update packet drops counter
    packet_drops += status.packet_rx_drop_count;

    if (!waypoint_receiving) {
        return;
    }

    uint32_t tnow = millis();

    if (waypoint_receiving &&
        waypoint_request_i <= waypoint_request_last &&
        tnow > waypoint_timelast_request + 500 + (stream_slowdown*20)) {
        waypoint_timelast_request = tnow;
        send_message(MSG_NEXT_WAYPOINT);
    }

    // stop waypoint receiving if timeout
    if (waypoint_receiving && (millis() - waypoint_timelast_receive) > waypoint_receive_timeout){
        waypoint_receiving = false;
    }
}

// see if we should send a stream now. Called at 50Hz
bool GCS_MAVLINK::stream_trigger(enum streams stream_num)
{
    AP_Int16 *stream_rates = &streamRateRawSensors;
    float rate = (uint8_t)stream_rates[stream_num].get();

    // send at a much lower rate while handling waypoints and
    // parameter sends
    if (waypoint_receiving || _queued_parameter != NULL) {
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
        if (streamRateParams.get() <= 0) {
            streamRateParams.set(50);
        }
        if (stream_trigger(STREAM_PARAMS)) {
            send_message(MSG_NEXT_PARAM);
        }
    }

    if (in_mavlink_delay) {
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

    if (stream_trigger(STREAM_RAW_SENSORS)) {
        send_message(MSG_RAW_IMU1);
        send_message(MSG_RAW_IMU3);
    }

    if (stream_trigger(STREAM_EXTENDED_STATUS)) {
        send_message(MSG_EXTENDED_STATUS1);
        send_message(MSG_EXTENDED_STATUS2);
        send_message(MSG_CURRENT_WAYPOINT);
        send_message(MSG_GPS_RAW);            // TODO - remove this message after location message is working
        send_message(MSG_NAV_CONTROLLER_OUTPUT);
    }

    if (stream_trigger(STREAM_POSITION)) {
        // sent with GPS read
        send_message(MSG_LOCATION);
    }

    if (stream_trigger(STREAM_RAW_CONTROLLER)) {
        send_message(MSG_SERVO_OUT);
    }

    if (stream_trigger(STREAM_RC_CHANNELS)) {
        send_message(MSG_RADIO_OUT);
        send_message(MSG_RADIO_IN);
    }

    if (stream_trigger(STREAM_EXTRA1)) {
        send_message(MSG_ATTITUDE);
        send_message(MSG_SIMSTATE);
    }

    if (stream_trigger(STREAM_EXTRA2)) {
        send_message(MSG_VFR_HUD);
    }

    if (stream_trigger(STREAM_EXTRA3)) {
        send_message(MSG_AHRS);
        send_message(MSG_HWSTATUS);
    }
}



void
GCS_MAVLINK::send_message(enum ap_message id)
{
    mavlink_send_message(chan,id, packet_drops);
}

void
GCS_MAVLINK::send_text_P(gcs_severity severity, const prog_char_t *str)
{
    mavlink_statustext_t m;
    uint8_t i;
    for (i=0; i<sizeof(m.text); i++) {
        m.text[i] = pgm_read_byte((const prog_char *)(str++));
    }
    if (i < sizeof(m.text)) m.text[i] = 0;
    mavlink_send_text(chan, severity, (const char *)m.text);
}

void GCS_MAVLINK::handleMessage(mavlink_message_t* msg)
{
    struct Location tell_command = {};                // command for telemetry

    switch (msg->msgid) {

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
        {
            // decode
            mavlink_request_data_stream_t packet;
            mavlink_msg_request_data_stream_decode(msg, &packet);

			if (mavlink_check_target(packet.target_system, packet.target_component))
				break;

        int16_t freq = 0;     // packet frequency

			if (packet.start_stop == 0)
				freq = 0; // stop sending
			else if (packet.start_stop == 1)
				freq = packet.req_message_rate; // start sending
			else
				break;

            switch(packet.req_stream_id){

                case MAV_DATA_STREAM_ALL:
                    streamRateRawSensors.set_and_save_ifchanged(freq);
                    streamRateExtendedStatus.set_and_save_ifchanged(freq);
                    streamRateRCChannels.set_and_save_ifchanged(freq);
                    streamRateRawController.set_and_save_ifchanged(freq);
                    streamRatePosition.set_and_save_ifchanged(freq);
                    streamRateExtra1.set_and_save_ifchanged(freq);
                    streamRateExtra2.set_and_save_ifchanged(freq);
                    streamRateExtra3.set_and_save_ifchanged(freq);
                    break;

                case MAV_DATA_STREAM_RAW_SENSORS:
            if (freq <= 5) {
                streamRateRawSensors.set_and_save_ifchanged(freq);
            } else {
                // We do not set and save this one so that if HIL is shut down incorrectly
														// we will not continue to broadcast raw sensor data at 50Hz.
                streamRateRawSensors = freq;
            }
                    break;

                case MAV_DATA_STREAM_EXTENDED_STATUS:
                    streamRateExtendedStatus.set_and_save_ifchanged(freq);
                    break;

                case MAV_DATA_STREAM_RC_CHANNELS:
                    streamRateRCChannels.set_and_save_ifchanged(freq);
                    break;

                case MAV_DATA_STREAM_RAW_CONTROLLER:
                    streamRateRawController.set_and_save_ifchanged(freq);
                    break;

                case MAV_DATA_STREAM_POSITION:
                    streamRatePosition.set_and_save_ifchanged(freq);
                    break;

                case MAV_DATA_STREAM_EXTRA1:
                    streamRateExtra1.set_and_save_ifchanged(freq);
                    break;

                case MAV_DATA_STREAM_EXTRA2:
                    streamRateExtra2.set_and_save_ifchanged(freq);
                    break;

                case MAV_DATA_STREAM_EXTRA3:
                    streamRateExtra3.set_and_save_ifchanged(freq);
                    break;

                default:
                    break;
            }
            break;
        }

    case MAVLINK_MSG_ID_COMMAND_LONG:
        {
            // decode
            mavlink_command_long_t packet;
            mavlink_msg_command_long_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system, packet.target_component)) break;

        uint8_t result = MAV_RESULT_UNSUPPORTED;

            // do command
            send_text_P(SEVERITY_LOW,PSTR("command received: "));

            switch(packet.command) {

            case MAV_CMD_NAV_RETURN_TO_LAUNCH:
                set_mode(RTL);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_CMD_MISSION_START:
                set_mode(AUTO);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_CMD_PREFLIGHT_CALIBRATION:
                if (packet.param1 == 1 ||
                    packet.param2 == 1 ||
                    packet.param3 == 1) {
            #if LITE == DISABLED                      
                    startup_INS_ground(true);
            #endif
                }
                if (packet.param4 == 1) {
                    trim_radio();
                }
                result = MAV_RESULT_ACCEPTED;
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

            case MAV_MODE_STABILIZE_DISARMED:
            case MAV_MODE_STABILIZE_ARMED:
                set_mode(LEARNING);
                result = MAV_RESULT_ACCEPTED;
                break;

            default:
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_DO_SET_SERVO:
            hal.rcout->enable_ch(packet.param1 - 1);
            hal.rcout->write(packet.param1 - 1, packet.param2);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if (packet.param1 == 1) {
                reboot_apm();
                result = MAV_RESULT_ACCEPTED;
            }
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


    case MAVLINK_MSG_ID_SET_MODE:
		{
            // decode
            mavlink_set_mode_t packet;
            mavlink_msg_set_mode_decode(msg, &packet);

            if (!(packet.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) {
                // we ignore base_mode as there is no sane way to map
                // from that bitmap to a APM flight mode. We rely on
                // custom_mode instead.
                break;
            }
            switch (packet.custom_mode) {
            case MANUAL:
            case LEARNING:
            case AUTO:
            case RTL:
                set_mode(packet.custom_mode);
                break;
            }

            break;
		}

    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        {
            // decode
            mavlink_mission_request_list_t packet;
            mavlink_msg_mission_request_list_decode(msg, &packet);
			if (mavlink_check_target(packet.target_system, packet.target_component))
				break;

            // Start sending waypoints
            mavlink_msg_mission_count_send(
                chan,msg->sysid,
                msg->compid,
                g.command_total + 1); // + home

            waypoint_timelast_send   = millis();
            waypoint_receiving       = false;
            waypoint_dest_sysid      = msg->sysid;
            waypoint_dest_compid     = msg->compid;
            break;
        }


	// XXX read a WP from EEPROM and send it to the GCS
    case MAVLINK_MSG_ID_MISSION_REQUEST:
        {
            // decode
            mavlink_mission_request_t packet;
            mavlink_msg_mission_request_decode(msg, &packet);

 			if (mavlink_check_target(packet.target_system, packet.target_component))
 				break;

            // send waypoint
            tell_command = get_cmd_with_index(packet.seq);

            // set frame of waypoint
            uint8_t frame;

			if (tell_command.options & MASK_OPTIONS_RELATIVE_ALT) {
                frame = MAV_FRAME_GLOBAL_RELATIVE_ALT; // reference frame
            } else {
                frame = MAV_FRAME_GLOBAL; // reference frame
            }

            float param1 = 0, param2 = 0 , param3 = 0, param4 = 0;

        // time that the mav should loiter in milliseconds
            uint8_t current = 0; // 1 (true), 0 (false)

			if (packet.seq == (uint16_t)g.command_index)
            	current = 1;

            uint8_t autocontinue = 1; // 1 (true), 0 (false)

            float x = 0, y = 0, z = 0;

            if (tell_command.id < MAV_CMD_NAV_LAST || tell_command.id == MAV_CMD_CONDITION_CHANGE_ALT) {
                // command needs scaling
                x = tell_command.lat/1.0e7; // local (x), global (latitude)
                y = tell_command.lng/1.0e7; // local (y), global (longitude)
            z = tell_command.alt/1.0e2;
            }

			switch (tell_command.id) {				// Switch to map APM command fields inot MAVLink command fields

				case MAV_CMD_NAV_TAKEOFF:
				case MAV_CMD_DO_SET_HOME:
					param1 = tell_command.p1;
					break;

				case MAV_CMD_CONDITION_CHANGE_ALT:
					x=0;	// Clear fields loaded above that we don't want sent for this command
					y=0;
				case MAV_CMD_CONDITION_DELAY:
				case MAV_CMD_CONDITION_DISTANCE:
					param1 = tell_command.lat;
					break;

				case MAV_CMD_DO_JUMP:
					param2 = tell_command.lat;
					param1 = tell_command.p1;
					break;

				case MAV_CMD_DO_REPEAT_SERVO:
					param4 = tell_command.lng;
				case MAV_CMD_DO_REPEAT_RELAY:
				case MAV_CMD_DO_CHANGE_SPEED:
					param3 = tell_command.lat;
					param2 = tell_command.alt;
					param1 = tell_command.p1;
					break;

				case MAV_CMD_DO_SET_PARAMETER:
				case MAV_CMD_DO_SET_RELAY:
				case MAV_CMD_DO_SET_SERVO:
					param2 = tell_command.alt;
					param1 = tell_command.p1;
					break;
			}

			mavlink_msg_mission_item_send(chan,msg->sysid,
										msg->compid,
										packet.seq,
										frame,
										tell_command.id,
										current,
										autocontinue,
										param1,
										param2,
										param3,
										param4,
										x,
										y,
										z);

            // update last waypoint comm stamp
            waypoint_timelast_send = millis();
            break;
        }


    case MAVLINK_MSG_ID_MISSION_ACK:
        {
            // decode
            mavlink_mission_ack_t packet;
            mavlink_msg_mission_ack_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;
            break;
        }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        {
            // decode
            mavlink_param_request_list_t packet;
            mavlink_msg_param_request_list_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // Start sending parameters - next call to ::update will kick the first one out

            _queued_parameter = AP_Param::first(&_queued_parameter_token, &_queued_parameter_type);
            _queued_parameter_index = 0;
            _queued_parameter_count = _count_parameters();
            break;
        }

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    {
        // decode
        mavlink_param_request_read_t packet;
        mavlink_msg_param_request_read_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;
        enum ap_var_type p_type;
        AP_Param *vp;
        if (packet.param_index != -1) {
            vp = AP_Param::find_by_index(packet.param_index, &p_type);
            if (vp == NULL) {
                gcs_send_text_fmt(PSTR("Unknown parameter index %d"), packet.param_index);
                break;
            }
        } else {
            vp = AP_Param::find(packet.param_id, &p_type);
            if (vp == NULL) {
                gcs_send_text_fmt(PSTR("Unknown parameter %.16s"), packet.param_id);
                break;
            }
        }
        char param_name[AP_MAX_NAME_SIZE];
        vp->copy_name(param_name, sizeof(param_name), true);

        float value = vp->cast_to_float(p_type);
        mavlink_msg_param_value_send(
            chan,
            param_name,
            value,
            mav_var_type(p_type),
            _count_parameters(),
            packet.param_index);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
        {
            // decode
            mavlink_mission_clear_all_t packet;
            mavlink_msg_mission_clear_all_decode(msg, &packet);
			if (mavlink_check_target(packet.target_system, packet.target_component)) break;

            // clear all commands
            g.command_total.set_and_save(0);

            // note that we don't send multiple acks, as otherwise a
            // GCS that is doing a clear followed by a set may see
            // the additional ACKs as ACKs of the set operations
            mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, MAV_MISSION_ACCEPTED);
            break;
        }

    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
        {
            // decode
            mavlink_mission_set_current_t packet;
            mavlink_msg_mission_set_current_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // set current command
            change_command(packet.seq);

            mavlink_msg_mission_current_send(chan, g.command_index);
            break;
        }

    case MAVLINK_MSG_ID_MISSION_COUNT:
        {
            // decode
            mavlink_mission_count_t packet;
            mavlink_msg_mission_count_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // start waypoint receiving
            if (packet.count > MAX_WAYPOINTS) {
                packet.count = MAX_WAYPOINTS;
            }
            g.command_total.set_and_save(packet.count - 1);

            waypoint_timelast_receive = millis();
            waypoint_timelast_request = 0;
            waypoint_receiving   = true;
            waypoint_request_i   = 0;
            waypoint_request_last= g.command_total;
            break;
        }

    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
    {
        // decode
        mavlink_mission_write_partial_list_t packet;
        mavlink_msg_mission_write_partial_list_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // start waypoint receiving
        if (packet.start_index > g.command_total ||
            packet.end_index > g.command_total ||
            packet.end_index < packet.start_index) {
            send_text_P(SEVERITY_LOW,PSTR("flight plan update rejected"));
            break;
        }

        waypoint_timelast_receive = millis();
        waypoint_timelast_request = 0;
        waypoint_receiving   = true;
        waypoint_request_i   = packet.start_index;
        waypoint_request_last= packet.end_index;
        break;
    }

#ifdef MAVLINK_MSG_ID_SET_MAG_OFFSETS
    case MAVLINK_MSG_ID_SET_MAG_OFFSETS:
    {
        mavlink_set_mag_offsets_t packet;
        mavlink_msg_set_mag_offsets_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;
        compass.set_offsets(Vector3f(packet.mag_ofs_x, packet.mag_ofs_y, packet.mag_ofs_z));
        break;
    }
#endif

	// XXX receive a WP from GCS and store in EEPROM
    case MAVLINK_MSG_ID_MISSION_ITEM:
        {
            // decode
            mavlink_mission_item_t packet;
            uint8_t result = MAV_MISSION_ACCEPTED;

            mavlink_msg_mission_item_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // defaults
            tell_command.id = packet.command;

			switch (packet.frame)
			{
				case MAV_FRAME_MISSION:
				case MAV_FRAME_GLOBAL:
					{
						tell_command.lat = 1.0e7*packet.x; // in as DD converted to * t7
						tell_command.lng = 1.0e7*packet.y; // in as DD converted to * t7
						tell_command.alt = packet.z*1.0e2; // in as m converted to cm
						tell_command.options = 0; // absolute altitude
						break;
					}

#ifdef MAV_FRAME_LOCAL_NED
				case MAV_FRAME_LOCAL_NED: // local (relative to home position)
					{
						tell_command.lat = 1.0e7*ToDeg(packet.x/
						(radius_of_earth*cos(ToRad(home.lat/1.0e7)))) + home.lat;
						tell_command.lng = 1.0e7*ToDeg(packet.y/radius_of_earth) + home.lng;
						tell_command.alt = -packet.z*1.0e2;
						tell_command.options = MASK_OPTIONS_RELATIVE_ALT;
						break;
					}
#endif

#ifdef MAV_FRAME_LOCAL
				case MAV_FRAME_LOCAL: // local (relative to home position)
					{
						tell_command.lat = 1.0e7*ToDeg(packet.x/
						(radius_of_earth*cos(ToRad(home.lat/1.0e7)))) + home.lat;
						tell_command.lng = 1.0e7*ToDeg(packet.y/radius_of_earth) + home.lng;
						tell_command.alt = packet.z*1.0e2;
						tell_command.options = MASK_OPTIONS_RELATIVE_ALT;
						break;
					}
#endif

				case MAV_FRAME_GLOBAL_RELATIVE_ALT: // absolute lat/lng, relative altitude
					{
						tell_command.lat = 1.0e7 * packet.x; // in as DD converted to * t7
						tell_command.lng = 1.0e7 * packet.y; // in as DD converted to * t7
						tell_command.alt = packet.z * 1.0e2;
						tell_command.options = MASK_OPTIONS_RELATIVE_ALT; // store altitude relative!! Always!!
						break;
					}

            default:
                result = MAV_MISSION_UNSUPPORTED_FRAME;
                break;
			}

            
            if (result != MAV_MISSION_ACCEPTED) goto mission_failed;

            switch (tell_command.id) {                    // Switch to map APM command fields inot MAVLink command fields
            case MAV_CMD_NAV_WAYPOINT:
            case MAV_CMD_NAV_RETURN_TO_LAUNCH:
                break;

            case MAV_CMD_NAV_TAKEOFF:
            case MAV_CMD_DO_SET_HOME:
                tell_command.p1 = packet.param1;
                break;

            case MAV_CMD_CONDITION_CHANGE_ALT:
                tell_command.lat = packet.param1;
                break;

            case MAV_CMD_CONDITION_DELAY:
            case MAV_CMD_CONDITION_DISTANCE:
                tell_command.lat = packet.param1;
                break;

            case MAV_CMD_DO_JUMP:
                tell_command.lat = packet.param2;
                tell_command.p1 = packet.param1;
                break;

            case MAV_CMD_DO_REPEAT_SERVO:
                tell_command.lng = packet.param4;
            case MAV_CMD_DO_REPEAT_RELAY:
            case MAV_CMD_DO_CHANGE_SPEED:
                tell_command.lat = packet.param3;
                tell_command.alt = packet.param2;
                tell_command.p1 = packet.param1;
                break;

            case MAV_CMD_DO_SET_PARAMETER:
            case MAV_CMD_DO_SET_RELAY:
            case MAV_CMD_DO_SET_SERVO:
                tell_command.alt = packet.param2;
                tell_command.p1 = packet.param1;
                break;

            default:
                result = MAV_MISSION_UNSUPPORTED;
                break;
            }

            if (result != MAV_MISSION_ACCEPTED) goto mission_failed;

			if(packet.current == 2){ 				//current = 2 is a flag to tell us this is a "guided mode" waypoint and not for the mission
				guided_WP = tell_command;

				// add home alt if needed
				if (guided_WP.options & MASK_OPTIONS_RELATIVE_ALT){
					guided_WP.alt += home.alt;
				}

				set_mode(GUIDED);

				// make any new wp uploaded instant (in case we are already in Guided mode)
				set_guided_WP();

				// verify we recevied the command
				mavlink_msg_mission_ack_send(
						chan,
						msg->sysid,
						msg->compid,
						0);

			} else {
				// Check if receiving waypoints (mission upload expected)
				if (!waypoint_receiving) {
                    result = MAV_MISSION_ERROR;
                    goto mission_failed;
                }

				// check if this is the requested waypoint
				if (packet.seq != waypoint_request_i) {
                    result = MAV_MISSION_INVALID_SEQUENCE;
                    goto mission_failed;
                }

                set_cmd_with_index(tell_command, packet.seq);

				// update waypoint receiving state machine
				waypoint_timelast_receive = millis();
                waypoint_timelast_request = 0;
				waypoint_request_i++;

                if (waypoint_request_i > waypoint_request_last) {
					mavlink_msg_mission_ack_send(
						chan,
						msg->sysid,
						msg->compid,
						result);

					send_text_P(SEVERITY_LOW,PSTR("flight plan received"));
					waypoint_receiving = false;
					// XXX ignores waypoint radius for individual waypoints, can
					// only set WP_RADIUS parameter
				}
			}
            break;

        mission_failed:
            // we are rejecting the mission/waypoint
            mavlink_msg_mission_ack_send(
                chan,
                msg->sysid,
                msg->compid,
                result);
            break;
        }


    case MAVLINK_MSG_ID_PARAM_SET:
        {
            AP_Param                  *vp;
            enum ap_var_type        var_type;

            // decode
            mavlink_param_set_t packet;
            mavlink_msg_param_set_decode(msg, &packet);

			if (mavlink_check_target(packet.target_system, packet.target_component))
				break;

            // set parameter

            char key[AP_MAX_NAME_SIZE+1];
            strncpy(key, (char *)packet.param_id, AP_MAX_NAME_SIZE);
            key[AP_MAX_NAME_SIZE] = 0;

            // find the requested parameter
            vp = AP_Param::find(key, &var_type);
            if ((NULL != vp) &&                             // exists
                    !isnan(packet.param_value) &&               // not nan
                    !isinf(packet.param_value)) {               // not inf

                // add a small amount before casting parameter values
                // from float to integer to avoid truncating to the
                // next lower integer value.
				float rounding_addition = 0.01;

                // handle variables with standard type IDs
                if (var_type == AP_PARAM_FLOAT) {
                    ((AP_Float *)vp)->set_and_save(packet.param_value);
                } else if (var_type == AP_PARAM_INT32) {
                    if (packet.param_value < 0) rounding_addition = -rounding_addition;
                    float v = packet.param_value+rounding_addition;
                    v = constrain(v, -2147483648.0, 2147483647.0);
					((AP_Int32 *)vp)->set_and_save(v);
                } else if (var_type == AP_PARAM_INT16) {
                    if (packet.param_value < 0) rounding_addition = -rounding_addition;
                    float v = packet.param_value+rounding_addition;
                    v = constrain(v, -32768, 32767);
					((AP_Int16 *)vp)->set_and_save(v);
                } else if (var_type == AP_PARAM_INT8) {
                    if (packet.param_value < 0) rounding_addition = -rounding_addition;
                    float v = packet.param_value+rounding_addition;
                    v = constrain(v, -128, 127);
					((AP_Int8 *)vp)->set_and_save(v);
                } else {
                    // we don't support mavlink set on this parameter
                    break;
                }

                // Report back the new value if we accepted the change
                // we send the value we actually set, which could be
                // different from the value sent, in case someone sent
                // a fractional value to an integer type
                mavlink_msg_param_value_send(
                    chan,
                    key,
                    vp->cast_to_float(var_type),
                    mav_var_type(var_type),
                    _count_parameters(),
                    -1); // XXX we don't actually know what its index is...
            }

            break;
        } // end case

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
    {
        // allow override of RC channel values for HIL
        // or for complete GCS control of switch position
        // and RC PWM values.
        if(msg->sysid != g.sysid_my_gcs) break;                         // Only accept control from our gcs
        mavlink_rc_channels_override_t packet;
        int16_t v[8];
        mavlink_msg_rc_channels_override_decode(msg, &packet);

        if (mavlink_check_target(packet.target_system,packet.target_component))
            break;

        v[0] = packet.chan1_raw;
        v[1] = packet.chan2_raw;
        v[2] = packet.chan3_raw;
        v[3] = packet.chan4_raw;
        v[4] = packet.chan5_raw;
        v[5] = packet.chan6_raw;
        v[6] = packet.chan7_raw;
        v[7] = packet.chan8_raw;

        hal.rcin->set_overrides(v, 8);

        rc_override_fs_timer = millis();
        break;
    }

    case MAVLINK_MSG_ID_HEARTBEAT:
        {
            // We keep track of the last time we received a heartbeat from our GCS for failsafe purposes
			if(msg->sysid != g.sysid_my_gcs) break;
            last_heartbeat_ms = rc_override_fs_timer = millis();
			pmTest1++;
            break;
        }

#if HIL_MODE != HIL_MODE_DISABLED
	case MAVLINK_MSG_ID_HIL_STATE:
		{
			mavlink_hil_state_t packet;
			mavlink_msg_hil_state_decode(msg, &packet);
			
			float vel = sqrt((packet.vx * (float)packet.vx) + (packet.vy * (float)packet.vy));
            float cog = wrap_360_cd(ToDeg(atan2(packet.vy, packet.vx)) * 100);
			
            // set gps hil sensor
            g_gps->setHIL(packet.time_usec/1000,
                          packet.lat*1.0e-7, packet.lon*1.0e-7, packet.alt*1.0e-3,
                          vel*1.0e-2, cog*1.0e-2, 0, 10);
			
			// rad/sec
            Vector3f gyros;
            gyros.x = packet.rollspeed;
            gyros.y = packet.pitchspeed;
            gyros.z = packet.yawspeed;

            // m/s/s
            Vector3f accels;
            accels.x = packet.xacc * (gravity/1000.0);
            accels.y = packet.yacc * (gravity/1000.0);
            accels.z = packet.zacc * (gravity/1000.0);
            
            ins.set_gyro(gyros);

            ins.set_accel(accels);
			
 #if HIL_MODE == HIL_MODE_ATTITUDE
			// set AHRS hil sensor
            ahrs.setHil(packet.roll,packet.pitch,packet.yaw,packet.rollspeed,
            packet.pitchspeed,packet.yawspeed);
 #endif

			break;
		}
#endif // HIL_MODE

#if MOUNT == ENABLED
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:
		{
			camera_mount.configure_msg(msg);
			break;
		}

    case MAVLINK_MSG_ID_MOUNT_CONTROL:
		{
			camera_mount.control_msg(msg);
			break;
		}

    case MAVLINK_MSG_ID_MOUNT_STATUS:
		{
			camera_mount.status_msg(msg);
			break;
		}
#endif // MOUNT == ENABLED

    case MAVLINK_MSG_ID_RADIO:
        {
            mavlink_radio_t packet;
            mavlink_msg_radio_decode(msg, &packet);
            // use the state of the transmit buffer in the radio to
            // control the stream rate, giving us adaptive software
            // flow control
            if (packet.txbuf < 20 && stream_slowdown < 100) {
                // we are very low on space - slow down a lot
                stream_slowdown += 3;
            } else if (packet.txbuf < 50 && stream_slowdown < 100) {
                // we are a bit low on space, slow down slightly
                stream_slowdown += 1;
            } else if (packet.txbuf > 95 && stream_slowdown > 10) {
                // the buffer has plenty of space, speed up a lot
                stream_slowdown -= 2;
            } else if (packet.txbuf > 90 && stream_slowdown != 0) {
                // the buffer has enough space, speed up a bit
                stream_slowdown--;
            }
            break;
        }

    default:
        // forward unknown messages to the other link if there is one
        if ((chan == MAVLINK_COMM_1 && gcs0.initialised) ||
            (chan == MAVLINK_COMM_0 && gcs3.initialised)) {
            mavlink_channel_t out_chan = (mavlink_channel_t)(((uint8_t)chan)^1);
            // only forward if it would fit in our transmit buffer
            if (comm_get_txspace(out_chan) > ((uint16_t)msg->len) + MAVLINK_NUM_NON_PAYLOAD_BYTES) {
                _mavlink_resend_uart(out_chan, msg);
            }
        }
        break;

    } // end switch
} // end handle mavlink

uint16_t
GCS_MAVLINK::_count_parameters()
{
    // if we haven't cached the parameter count yet...
    if (0 == _parameter_count) {
        AP_Param  *vp;
        AP_Param::ParamToken token;

        vp = AP_Param::first(&token, NULL);
        do {
            _parameter_count++;
        } while (NULL != (vp = AP_Param::next_scalar(&token, NULL)));
    }
    return _parameter_count;
}

/**
* @brief Send the next pending parameter, called from deferred message
* handling code
*/
void
GCS_MAVLINK::queued_param_send()
{
    if (_queued_parameter == NULL) {
        return;
    }

    uint16_t bytes_allowed;
    uint8_t count;
    uint32_t tnow = millis();

    // use at most 30% of bandwidth on parameters. The constant 26 is
    // 1/(1000 * 1/8 * 0.001 * 0.3)
    bytes_allowed = g.serial3_baud * (tnow - _queued_parameter_send_time_ms) * 26;
    if (bytes_allowed > comm_get_txspace(chan)) {
        bytes_allowed = comm_get_txspace(chan);
    }
    count = bytes_allowed / (MAVLINK_MSG_ID_PARAM_VALUE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);

    while (_queued_parameter != NULL && count--) {
    AP_Param      *vp;
    float       value;

    // copy the current parameter and prepare to move to the next
    vp = _queued_parameter;

    // if the parameter can be cast to float, report it here and break out of the loop
    value = vp->cast_to_float(_queued_parameter_type);

    char param_name[AP_MAX_NAME_SIZE];
    vp->copy_name(param_name, sizeof(param_name), true);

    mavlink_msg_param_value_send(
        chan,
        param_name,
        value,
        mav_var_type(_queued_parameter_type),
        _queued_parameter_count,
        _queued_parameter_index);

    _queued_parameter = AP_Param::next_scalar(&_queued_parameter_token, &_queued_parameter_type);
    _queued_parameter_index++;
}
    _queued_parameter_send_time_ms = tnow;
}

/**
* @brief Send the next pending waypoint, called from deferred message
* handling code
*/
void
GCS_MAVLINK::queued_waypoint_send()
{
    if (waypoint_receiving &&
        waypoint_request_i <= waypoint_request_last) {
        mavlink_msg_mission_request_send(
            chan,
            waypoint_dest_sysid,
            waypoint_dest_compid,
            waypoint_request_i);
    }
}

void GCS_MAVLINK::reset_cli_timeout() {
      _cli_timeout = millis();
}
/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
static void mavlink_delay_cb()
{
    static uint32_t last_1hz, last_50hz, last_5s;
    if (!gcs0.initialised) return;

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
    }
    if (tnow - last_5s > 5000) {
        last_5s = tnow;
        gcs_send_text_P(SEVERITY_LOW, PSTR("Initialising APM..."));
    }
#if USB_MUX_PIN > 0
    check_usb_mux();
#endif

    in_mavlink_delay = false;
}

/*
 *  send a message on both GCS links
 */
static void gcs_send_message(enum ap_message id)
{
    gcs0.send_message(id);
    if (gcs3.initialised) {
        gcs3.send_message(id);
    }
}

/*
 *  send data streams in the given rate range on both links
 */
static void gcs_data_stream_send(void)
{
    gcs0.data_stream_send();
    if (gcs3.initialised) {
        gcs3.data_stream_send();
    }
}

/*
 *  look for incoming commands on the GCS links
 */
static void gcs_update(void)
{
	gcs0.update();
    if (gcs3.initialised) {
        gcs3.update();
    }
}

static void gcs_send_text_P(gcs_severity severity, const prog_char_t *str)
{
    gcs0.send_text_P(severity, str);
    if (gcs3.initialised) {
        gcs3.send_text_P(severity, str);
    }
}

/*
 *  send a low priority formatted message to the GCS
 *  only one fits in the queue, so if you send more than one before the
 *  last one gets into the serial buffer then the old one will be lost
 */
void gcs_send_text_fmt(const prog_char_t *fmt, ...)
{
    char fmtstr[40];
    va_list arg_list;
    uint8_t i;
    for (i=0; i<sizeof(fmtstr)-1; i++) {
        fmtstr[i] = pgm_read_byte((const prog_char *)(fmt++));
        if (fmtstr[i] == 0) break;
    }
    fmtstr[i] = 0;
    gcs0.pending_status.severity = (uint8_t)SEVERITY_LOW;
    va_start(arg_list, fmt);
    vsnprintf((char *)gcs0.pending_status.text, sizeof(gcs0.pending_status.text), fmtstr, arg_list);
    va_end(arg_list);
    gcs3.pending_status = gcs0.pending_status;
    mavlink_send_message(MAVLINK_COMM_0, MSG_STATUSTEXT, 0);
    if (gcs3.initialised) {
        mavlink_send_message(MAVLINK_COMM_1, MSG_STATUSTEXT, 0);
    }
}

