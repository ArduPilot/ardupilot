// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// default sensors are present and healthy: gyro, accelerometer, barometer, rate_control, attitude_stabilization, yaw_position, altitude control, x/y position control, motor_control
#define MAVLINK_SENSOR_PRESENT_DEFAULT (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_YAW_POSITION | MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS)

// use this to prevent recursion during sensor init
static bool in_mavlink_delay;

// true when we have received at least 1 MAVLink packet
static bool mavlink_active;

// check if a message will fit in the payload space available
#define CHECK_PAYLOAD_SIZE(id) if (payload_space < MAVLINK_MSG_ID_ ## id ## _LEN) return false

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
    mavlink_msg_heartbeat_send(
        chan,
        MAV_TYPE_ANTENNA_TRACKER,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        0,
        0,
        0);
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
    if (g_gps->status() >= GPS::GPS_OK_FIX_2D) {
        fix_time = g_gps->last_fix_time;
    } else {
        fix_time = hal.scheduler->millis();
    }

    Location loc;
    ahrs.get_position(loc);

    mavlink_msg_global_position_int_send(
        chan,
        fix_time,
        loc.lat,                // in 1E7 degrees
        loc.lng,                // in 1E7 degrees
        g_gps->altitude_cm * 10,        // millimeters above sea level
        0,
        g_gps->velocity_north() * 100,  // X speed cm/s (+ve North)
        g_gps->velocity_east()  * 100,  // Y speed cm/s (+ve East)
        g_gps->velocity_down()  * -100, // Z speed cm/s (+ve up)
        ahrs.yaw_sensor);
}

static void NOINLINE send_gps_raw(mavlink_channel_t chan)
{
    mavlink_msg_gps_raw_int_send(
        chan,
        g_gps->last_fix_time*(uint64_t)1000,
        g_gps->status(),
        g_gps->latitude,      // in 1E7 degrees
        g_gps->longitude,     // in 1E7 degrees
        g_gps->altitude_cm * 10, // in mm
        g_gps->hdop,
        65535,
        g_gps->ground_speed_cm,  // cm/s
        g_gps->ground_course_cd, // 1/100 degrees,
        g_gps->num_sats);
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

static void NOINLINE send_raw_imu1(mavlink_channel_t chan)
{
    Vector3f accel = ins.get_accel();
    Vector3f gyro = ins.get_gyro();

    mavlink_msg_raw_imu_send(
        chan,
        hal.scheduler->micros(),
        accel.x * 1000.0 / GRAVITY_MSS,
        accel.y * 1000.0 / GRAVITY_MSS,
        accel.z * 1000.0 / GRAVITY_MSS,
        gyro.x * 1000.0,
        gyro.y * 1000.0,
        gyro.z * 1000.0,
        compass.mag_x,
        compass.mag_y,
        compass.mag_z);
}

static void NOINLINE send_raw_imu2(mavlink_channel_t chan)
{
    float pressure = barometer.get_pressure();
    mavlink_msg_scaled_pressure_send(
        chan,
        hal.scheduler->millis(),
        pressure*0.01f, // hectopascal
        (pressure - barometer.get_ground_pressure())*0.01f, // hectopascal
        barometer.get_temperature()*100); // 0.01 degrees C
}

static void NOINLINE send_raw_imu3(mavlink_channel_t chan)
{
    // run this message at a much lower rate - otherwise it
    // pointlessly wastes quite a lot of bandwidth
    static uint8_t counter;
    if (counter++ < 10) {
        return;
    }
    counter = 0;

    Vector3f mag_offsets = compass.get_offsets();
    Vector3f accel_offsets = ins.get_accel_offsets();
    Vector3f gyro_offsets = ins.get_gyro_offsets();

    mavlink_msg_sensor_offsets_send(chan,
                                    mag_offsets.x,
                                    mag_offsets.y,
                                    mag_offsets.z,
                                    compass.get_declination(),
                                    barometer.get_pressure(),
                                    barometer.get_temperature()*100,
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

static void NOINLINE send_hwstatus(mavlink_channel_t chan)
{
    mavlink_msg_hwstatus_send(
        chan,
        0,
        hal.i2c->lockup_count());
}

static void NOINLINE send_statustext(mavlink_channel_t chan)
{
    mavlink_statustext_t *s = (chan == MAVLINK_COMM_0?&gcs0.pending_status:&gcs3.pending_status);
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
        0,
        0,
        0);
}


// try to send a message, return false if it won't fit in the serial tx buffer
static bool mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    int16_t payload_space = comm_get_txspace(chan) - MAVLINK_NUM_NON_PAYLOAD_BYTES;

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
        send_gps_raw(chan);
        break;

    case MSG_RADIO_OUT:
        CHECK_PAYLOAD_SIZE(SERVO_OUTPUT_RAW);
        send_radio_out(chan);
        break;

    case MSG_RAW_IMU1:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
        send_raw_imu1(chan);
        break;

    case MSG_RAW_IMU2:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE);
        send_raw_imu2(chan);
        break;

    case MSG_RAW_IMU3:
        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
        send_raw_imu3(chan);
        break;

    case MSG_NEXT_PARAM:
        CHECK_PAYLOAD_SIZE(PARAM_VALUE);
        if (chan == MAVLINK_COMM_0) {
            gcs0.queued_param_send();
        } else if (gcs3.initialised) {
            gcs3.queued_param_send();
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

    case MSG_HWSTATUS:
        CHECK_PAYLOAD_SIZE(HWSTATUS);
        send_hwstatus(chan);
        break;

    case MSG_SERVO_OUT:
    case MSG_EXTENDED_STATUS1:
    case MSG_EXTENDED_STATUS2:
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


GCS_MAVLINK::GCS_MAVLINK() :
    packet_drops(0)
{
    AP_Param::setup_object_defaults(this, var_info);
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
}

void
GCS_MAVLINK::update(void)
{
    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;
    status.packet_rx_drop_count = 0;

    // process received bytes
    uint16_t nbytes = comm_get_available(chan);
    for (uint16_t i=0; i<nbytes; i++)
    {
        uint8_t c = comm_receive_ch(chan);

        // Try to get a new message
        if (mavlink_parse_char(chan, c, &msg, &status)) {
            // we exclude radio packets to make it possible to use the
            // CLI over the radio
            if (msg.msgid != MAVLINK_MSG_ID_RADIO && msg.msgid != MAVLINK_MSG_ID_RADIO_STATUS) {
                mavlink_active = true;
            }
            handleMessage(&msg);
        }
    }

    // Update packet drops counter
    packet_drops += status.packet_rx_drop_count;
}

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
        send_message(MSG_RADIO_OUT);
    }

    if (stream_trigger(STREAM_EXTRA1)) {
        send_message(MSG_ATTITUDE);
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
        if (m.text[i] == '\0') {
            break;
        }
    }
    if (i < sizeof(m.text)) m.text[i] = 0;
    mavlink_send_text(chan, severity, (const char *)m.text);
}

void GCS_MAVLINK::handleMessage(mavlink_message_t* msg)
{
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
            freq = 0;                     // stop sending
        else if (packet.start_stop == 1)
            freq = packet.req_message_rate;                     // start sending
        else
            break;

        switch (packet.req_stream_id) {
        case MAV_DATA_STREAM_ALL:
            // note that we don't set STREAM_PARAMS - that is internal only
            for (uint8_t i=0; i<STREAM_PARAMS; i++) {
                streamRates[i].set_and_save_ifchanged(freq);
            }
            break;
        case MAV_DATA_STREAM_RAW_SENSORS:
            streamRates[STREAM_RAW_SENSORS].set_and_save_ifchanged(freq);
            break;
        case MAV_DATA_STREAM_EXTENDED_STATUS:
            streamRates[STREAM_EXTENDED_STATUS].set_and_save_ifchanged(freq);
            break;
        case MAV_DATA_STREAM_RC_CHANNELS:
            streamRates[STREAM_RC_CHANNELS].set_and_save_ifchanged(freq);
            break;
        case MAV_DATA_STREAM_RAW_CONTROLLER:
            streamRates[STREAM_RAW_CONTROLLER].set_and_save_ifchanged(freq);
            break;
        case MAV_DATA_STREAM_POSITION:
            streamRates[STREAM_POSITION].set_and_save_ifchanged(freq);
            break;
        case MAV_DATA_STREAM_EXTRA1:
            streamRates[STREAM_EXTRA1].set_and_save_ifchanged(freq);
            break;
        case MAV_DATA_STREAM_EXTRA2:
            streamRates[STREAM_EXTRA2].set_and_save_ifchanged(freq);
            break;
        case MAV_DATA_STREAM_EXTRA3:
            streamRates[STREAM_EXTRA3].set_and_save_ifchanged(freq);
            break;
        }
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
        char param_name[AP_MAX_NAME_SIZE+1];
        if (packet.param_index != -1) {
            AP_Param::ParamToken token;
            vp = AP_Param::find_by_index(packet.param_index, &p_type, &token);
            if (vp == NULL) {
                gcs_send_text_fmt(PSTR("Unknown parameter index %d"), packet.param_index);
                break;
            }
            vp->copy_name_token(token, param_name, AP_MAX_NAME_SIZE, true);
            param_name[AP_MAX_NAME_SIZE] = 0;
        } else {
            strncpy(param_name, packet.param_id, AP_MAX_NAME_SIZE);
            param_name[AP_MAX_NAME_SIZE] = 0;
            vp = AP_Param::find(param_name, &p_type);
            if (vp == NULL) {
                gcs_send_text_fmt(PSTR("Unknown parameter %.16s"), packet.param_id);
                break;
            }
        }

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

    case MAVLINK_MSG_ID_PARAM_SET:
    {
        AP_Param                  *vp;
        enum ap_var_type var_type;

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
        if ((NULL != vp) &&                                 // exists
            !isnan(packet.param_value) &&                       // not nan
            !isinf(packet.param_value)) {                       // not inf

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
                v = constrain_float(v, -2147483648.0, 2147483647.0);
                ((AP_Int32 *)vp)->set_and_save(v);
            } else if (var_type == AP_PARAM_INT16) {
                if (packet.param_value < 0) rounding_addition = -rounding_addition;
                float v = packet.param_value+rounding_addition;
                v = constrain_float(v, -32768, 32767);
                ((AP_Int16 *)vp)->set_and_save(v);
            } else if (var_type == AP_PARAM_INT8) {
                if (packet.param_value < 0) rounding_addition = -rounding_addition;
                float v = packet.param_value+rounding_addition;
                v = constrain_float(v, -128, 127);
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
                -1);     // XXX we don't actually know what its index is...
#if LOGGING_ENABLED == ENABLED
            DataFlash.Log_Write_Parameter(key, vp->cast_to_float(var_type));
#endif
        }

        break;
    }     // end case

    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        if (msg->sysid != g.sysid_my_gcs) break;
        break;
    }

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        // decode
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(msg, &packet);
        tracking_update_position(packet);
        break;
    }

    default:
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
    uint32_t tnow = hal.scheduler->millis();

    // use at most 30% of bandwidth on parameters. The constant 26 is
    // 1/(1000 * 1/8 * 0.001 * 0.3)
    bytes_allowed = g.serial3_baud * (tnow - _queued_parameter_send_time_ms) * 26;
    if (bytes_allowed > comm_get_txspace(chan)) {
        bytes_allowed = comm_get_txspace(chan);
    }
    count = bytes_allowed / (MAVLINK_MSG_ID_PARAM_VALUE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);

    while (_queued_parameter != NULL && count--) {
        AP_Param      *vp;
        float value;

        // copy the current parameter and prepare to move to the next
        vp = _queued_parameter;

        // if the parameter can be cast to float, report it here and break out of the loop
        value = vp->cast_to_float(_queued_parameter_type);

        char param_name[AP_MAX_NAME_SIZE];
        vp->copy_name_token(_queued_parameter_token, param_name, sizeof(param_name), true);

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
    gcs0.pending_status.severity = (uint8_t)SEVERITY_LOW;
    va_start(arg_list, fmt);
    hal.util->vsnprintf_P((char *)gcs0.pending_status.text,
            sizeof(gcs0.pending_status.text), fmt, arg_list);
    va_end(arg_list);
#if LOGGING_ENABLED == ENABLED
    DataFlash.Log_Write_Message(gcs0.pending_status.text);
#endif
    gcs3.pending_status = gcs0.pending_status;
    mavlink_send_message(MAVLINK_COMM_0, MSG_STATUSTEXT, 0);
    if (gcs3.initialised) {
        mavlink_send_message(MAVLINK_COMM_1, MSG_STATUSTEXT, 0);
    }
}

