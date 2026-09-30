/*
   GCS MAVLink functions related to parameter handling

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "GCS_config.h"

#if HAL_GCS_ENABLED

#include <AP_HAL/AP_HAL.h>

#include "GCS.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

// queue of pending parameter requests and replies
ObjectBuffer<GCS_MAVLINK::pending_param_request> GCS_MAVLINK::param_requests(20);
ObjectBuffer<GCS_MAVLINK::pending_param_reply> GCS_MAVLINK::param_replies(5);

bool GCS_MAVLINK::param_timer_registered;

/* Keep raw int32 bits out of float arguments, including signalling NaNs. */
static void param_value_send(mavlink_channel_t chan, const char *name, float value, MAV_PARAM_TYPE mtype, int32_t int_value, uint16_t count, uint16_t index, uint32_t supported_types)
{
    // A legacy request may have arrived while an IO reply was queued.
    if (mtype == MAV_PARAM_TYPE_BYTEWISE_INT32 &&
        (!(supported_types & MAV_PARAM_TYPES_SUPPORTED_BYTEWISE_INT32) ||
         gcs().option_is_enabled(GCS::Option::PARAM_NO_BYTEWISE) ||
         (mavlink_get_channel_status(chan)->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1))) {
        mtype = MAV_PARAM_TYPE_INT32;
        value = float(int_value);
    }
    mavlink_param_value_t packet{};
    strncpy_noterm(packet.param_id, name, sizeof(packet.param_id));
    packet.param_type = mtype;
    packet.param_count = count;
    packet.param_index = index;
    if (mtype == MAV_PARAM_TYPE_BYTEWISE_INT32) {
        memcpy(&packet.param_value, &int_value, sizeof(int_value));
    } else {
        packet.param_value = value;
    }
    // All ArduPilot targets are little-endian. Extension bytes remain zero.
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_VALUE, (const char *)&packet,
                                   MAVLINK_MSG_ID_PARAM_VALUE_MIN_LEN, MAVLINK_MSG_ID_PARAM_VALUE_LEN,
                                   MAVLINK_MSG_ID_PARAM_VALUE_CRC);
}

void GCS_MAVLINK::update_param_supported_types(const mavlink_message_t &msg, uint32_t supported_types)
{
    if (msg.magic == MAVLINK_STX_MAVLINK1) {
        supported_types = 0;
    }
    if (!_param_requester_seen) {
        _param_requester_seen = true;
        _param_requester_sysid = msg.sysid;
        _param_requester_compid = msg.compid;
        _param_supported_types = supported_types;
    } else if (!_param_multiple_requesters &&
               _param_requester_sysid == msg.sysid && _param_requester_compid == msg.compid) {
        // Also allows a reconnecting legacy client to clear an earlier opt-in.
        _param_supported_types = supported_types;
    } else {
        // PARAM_VALUE has no target: once clients share a channel, retain
        // only their common capabilities until the channel is reinitialised.
        _param_multiple_requesters = true;
        _param_supported_types &= supported_types;
    }
}

/**
 * @brief Send the next pending parameter, called from deferred message
 * handling code
 */
void
GCS_MAVLINK::queued_param_send()
{
    // send parameter async replies
    uint8_t async_replies_sent_count = send_parameter_async_replies();

    // now send the streaming parameters (from PARAM_REQUEST_LIST)
    if (_queued_parameter == nullptr) {
        // .... or not....
        return;
    }

    const uint32_t tnow = AP_HAL::millis();
    const uint32_t tstart = AP_HAL::micros();

    // use at most 30% of bandwidth on parameters
    const uint32_t link_bw = _port->bw_in_bytes_per_second();

    uint32_t bytes_allowed = link_bw * (tnow - _queued_parameter_send_time_ms) / 3333;
    // Bytewise int32 uses the original payload; all extension bytes are zero.
    const uint16_t size_for_one_param_value_msg = MAVLINK_MSG_ID_PARAM_VALUE_MIN_LEN + packet_overhead();
    if (bytes_allowed < size_for_one_param_value_msg) {
        bytes_allowed = size_for_one_param_value_msg;
    }
    if (bytes_allowed > txspace()) {
        bytes_allowed = txspace();
    }
    uint32_t count = bytes_allowed / size_for_one_param_value_msg;

    // when we don't have flow control we really need to keep the
    // param download very slow, or it tends to stall
    if (!have_flow_control() && count > 5) {
        count = 5;
    }
    if (async_replies_sent_count >= count) {
        return;
    }
    count -= async_replies_sent_count;

    while (count && _queued_parameter != nullptr && last_txbuf_is_greater(33)) {
        char param_name[AP_MAX_NAME_SIZE];
        _queued_parameter->copy_name_token(_queued_parameter_token, param_name, sizeof(param_name), true);

        float value = _queued_parameter->cast_to_float(_queued_parameter_type);
        int32_t int_value = 0;
        const MAV_PARAM_TYPE mtype = mav_param_send_encoding(chan, _queued_parameter, _queued_parameter_type, value, int_value, _param_supported_types);
        param_value_send(chan, param_name, value, mtype, int_value,
                         _queued_parameter_count,
                         _queued_parameter_index, _param_supported_types);

        _queued_parameter = AP_Param::next_scalar(&_queued_parameter_token, &_queued_parameter_type);
        _queued_parameter_index++;

        if (AP_HAL::micros() - tstart > 1000) {
            // don't use more than 1ms sending blocks of parameters
            break;
        }
        count--;
    }
    _queued_parameter_send_time_ms = tnow;
}

/*
  return true if a channel has flow control
 */
bool GCS_MAVLINK::have_flow_control(void)
{
    if (_port == nullptr) {
        return false;
    }

    if (_port->flow_control_enabled()) {
        return true;
    }

    if (chan == MAVLINK_COMM_0) {
        // assume USB console has flow control
        return hal.gpio->usb_connected();
    }

    return false;
}


/*
  handle a request to change stream rate. Note that copter passes in
  save==false so we don't want the save to happen when the user connects the
  ground station.
 */
void GCS_MAVLINK::handle_request_data_stream(const mavlink_message_t &msg)
{
    mavlink_request_data_stream_t packet;
    mavlink_msg_request_data_stream_decode(&msg, &packet);

    int16_t freq = 0;     // packet frequency

    if (packet.start_stop == 0)
        freq = 0;                     // stop sending
    else if (packet.start_stop == 1)
        freq = packet.req_message_rate;                     // start sending
    else
        return;

    // if stream_id is still NUM_STREAMS at the end of this switch
    // block then either we set stream rates for all streams, or we
    // were asked to set the streamrate for an unrecognised stream
    streams stream_id = NUM_STREAMS;
    switch (packet.req_stream_id) {
    case MAV_DATA_STREAM_ALL:
        for (uint8_t i=0; i<NUM_STREAMS; i++) {
            if (i == STREAM_PARAMS) {
                // don't touch parameter streaming rate; it is
                // considered "internal".
                continue;
            }
            if (persist_streamrates()) {
                streamRates[i].set_and_save_ifchanged(freq);
            } else {
                streamRates[i].set(freq);
            }
            initialise_message_intervals_for_stream((streams)i);
        }
        break;
    case MAV_DATA_STREAM_RAW_SENSORS:
        stream_id = STREAM_RAW_SENSORS;
        break;
    case MAV_DATA_STREAM_EXTENDED_STATUS:
        stream_id = STREAM_EXTENDED_STATUS;
        break;
    case MAV_DATA_STREAM_RC_CHANNELS:
        stream_id = STREAM_RC_CHANNELS;
        break;
    case MAV_DATA_STREAM_RAW_CONTROLLER:
        stream_id = STREAM_RAW_CONTROLLER;
        break;
    case MAV_DATA_STREAM_POSITION:
        stream_id = STREAM_POSITION;
        break;
    case MAV_DATA_STREAM_EXTRA1:
        stream_id = STREAM_EXTRA1;
        break;
    case MAV_DATA_STREAM_EXTRA2:
        stream_id = STREAM_EXTRA2;
        break;
    case MAV_DATA_STREAM_EXTRA3:
        stream_id = STREAM_EXTRA3;
        break;
    }

    if (stream_id == NUM_STREAMS) {
        // asked to set rate on unknown stream (or all were set already)
        return;
    }

    AP_Int16 *rate = &streamRates[stream_id];

    if (rate != nullptr) {
        if (persist_streamrates()) {
            rate->set_and_save_ifchanged(freq);
        } else {
            rate->set(freq);
        }
        initialise_message_intervals_for_stream(stream_id);
    }
}

void GCS_MAVLINK::handle_param_request_list(const mavlink_message_t &msg)
{
    if (!params_ready()) {
        return;
    }

    mavlink_param_request_list_t packet;
    mavlink_msg_param_request_list_decode(&msg, &packet);
    update_param_supported_types(msg, packet.supported_types);

    // requesting parameters is a convenient way to get extra information
    send_banner();

    // Start sending parameters - next call to ::update will kick the first one out
    _queued_parameter = AP_Param::first(&_queued_parameter_token, &_queued_parameter_type);
    _queued_parameter_index = 0;
    _queued_parameter_count = AP_Param::count_parameters();
    _queued_parameter_send_time_ms = AP_HAL::millis(); // avoid initial flooding
}

void GCS_MAVLINK::handle_param_request_read(const mavlink_message_t &msg)
{
    mavlink_param_request_read_t packet;
    mavlink_msg_param_request_read_decode(&msg, &packet);
    update_param_supported_types(msg, packet.supported_types);

    if (param_requests.space() == 0) {
        // Retain the advertisement even when the read queue is full.
        return;
    }

    /*
      we reserve some space for sending parameters if the client ever
      fails to get a parameter due to lack of space
     */
    uint32_t saved_reserve_param_space_start_ms = reserve_param_space_start_ms;
    reserve_param_space_start_ms = 0; // bypass packet_overhead_chan reservation checking
    if (!check_payload_size(MAVLINK_MSG_ID_PARAM_VALUE_MIN_LEN)) {
        reserve_param_space_start_ms = AP_HAL::millis();
    } else {
        reserve_param_space_start_ms = saved_reserve_param_space_start_ms;
    }

    struct pending_param_request req;
    req.supported_types = _param_supported_types;
    req.src_system_id = msg.sysid;
    req.src_component_id = msg.compid;
    req.chan = chan;
    req.param_index = packet.param_index;
    memcpy(req.param_name, packet.param_id, MIN(sizeof(packet.param_id), sizeof(req.param_name)));
    req.param_name[AP_MAX_NAME_SIZE] = 0;

    // queue it for processing by io timer
    param_requests.push(req);

    // speaking of which, we'd best make sure it is running:
    if (!param_timer_registered) {
        param_timer_registered = true;
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&GCS_MAVLINK::param_io_timer, void));
    }
}

void GCS_MAVLINK::handle_param_set(const mavlink_message_t &msg)
{
    mavlink_param_set_t packet;
    mavlink_msg_param_set_decode(&msg, &packet);
    enum ap_var_type var_type;

    // set parameter
    AP_Param *vp;
    char key[AP_MAX_NAME_SIZE+1];
    strncpy(key, (char *)packet.param_id, AP_MAX_NAME_SIZE);
    key[AP_MAX_NAME_SIZE] = 0;

    // find existing param so we can get the old value
    uint16_t parameter_flags = 0;
    vp = AP_Param::find(key, &var_type, &parameter_flags);
    if (vp == nullptr) {
        send_param_error(msg, packet, MAV_PARAM_ERROR_DOES_NOT_EXIST);
        return;
    }
    // AP_Param has no 64-bit or custom storage. IN_PROGRESS is a reply
    // status, never a value that can be written.
    if (packet.param_type == MAV_PARAM_TYPE_EXTENDED || packet.param_type == MAV_PARAM_TYPE_IN_PROGRESS) {
        send_param_error(msg, packet, MAV_PARAM_ERROR_TYPE_UNSUPPORTED);
        return;
    }
    const bool is_bytewise = packet.param_type == MAV_PARAM_TYPE_BYTEWISE_INT32 ||
                             packet.param_type == MAV_PARAM_TYPE_BYTEWISE_UINT32;
    int32_t int_value = 0;
    if (is_bytewise) {
        if (var_type != AP_PARAM_INT32) {
            send_param_error(msg, packet, MAV_PARAM_ERROR_TYPE_MISMATCH);
            return;
        }
        // param_value is the first four payload bytes, independent of the
        // variable-length MAVLink header. Do not evaluate it as a float.
        memcpy(&int_value, _MAV_PAYLOAD(&msg), sizeof(int_value));
        if (packet.param_type == MAV_PARAM_TYPE_BYTEWISE_UINT32 && int_value < 0) {
            send_param_error(msg, packet, MAV_PARAM_ERROR_VALUE_OUT_OF_RANGE);
            return;
        }
    } else if (isnan(packet.param_value) || isinf(packet.param_value)) {
        send_param_error(msg, packet, MAV_PARAM_ERROR_VALUE_OUT_OF_RANGE);
        return;
    }

    float old_value = vp->cast_to_float(var_type);

    if (!vp->allow_set_via_mavlink(parameter_flags)) {
        // don't warn the user about this failure if we are dropping
        // messages here.  This is on the assumption that scripting is
        // currently responsible for setting parameters and may set
        // the value instead of us.
        if (gcs().get_allow_param_set()) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Param write denied (%s)", key);
            send_param_error(msg, packet, MAV_PARAM_ERROR_PERMISSION_DENIED);
        }
        // send the readonly value
        send_parameter_value(key, var_type, old_value, vp);
        return;
    }

    /*
      we force the save if the value is not equal to the old
      value. This copes with the use of override values in
      constructors, such as PID elements. Otherwise a set to the
      default value which differs from the constructor value doesn't
      save the change
     */
    bool force_save;
    if (is_bytewise) {
        const int32_t old_int = ((AP_Int32 *)vp)->get();
        ((AP_Int32 *)vp)->set(int_value);
        force_save = (int_value != old_int);
    } else {
        vp->set_float(packet.param_value, var_type);
        force_save = !is_equal(packet.param_value, old_value);
    }

    // save the change
    vp->save(force_save);

    if (force_save && (parameter_flags & AP_PARAM_FLAG_ENABLE)) {
        AP_Param::invalidate_count();
    }

#if HAL_LOGGING_ENABLED
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger != nullptr) {
        logger->Write_Parameter(key, vp->cast_to_float(var_type));
    }
#endif
}

void GCS_MAVLINK::send_parameter_value(const char *param_name, ap_var_type param_type, float param_value, const AP_Param *vp)
{
    if (!check_payload_size(MAVLINK_MSG_ID_PARAM_VALUE_MIN_LEN)) {
        return;
    }
    int32_t int_value = 0;
    const MAV_PARAM_TYPE mtype = mav_param_send_encoding(chan, vp, param_type, param_value, int_value, _param_supported_types);
    param_value_send(chan, param_name, param_value, mtype, int_value,
                     AP_Param::count_parameters(),
                     -1, _param_supported_types);
}

/*
  send a parameter value message to all active MAVLink connections
 */
void GCS::send_parameter_value(const char *param_name, ap_var_type param_type, float param_value, const AP_Param *vp)
{
    mavlink_param_value_t packet{};
    const uint8_t to_copy = MIN(ARRAY_SIZE(packet.param_id), strlen(param_name));
    memcpy(packet.param_id, param_name, to_copy);
    packet.param_count = AP_Param::count_parameters();
    packet.param_index = -1;

    /*
      Select bytewise encoding only on channels whose requesters support it.
     */
    const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(MAVLINK_MSG_ID_PARAM_VALUE);
    if (entry == nullptr) {
        return;
    }
    // All parameter types stored by ArduPilot have zero extension fields.
    mavlink_msg_entry_t param_entry = *entry;
    param_entry.max_msg_len = MAVLINK_MSG_ID_PARAM_VALUE_MIN_LEN;
    for (uint8_t i=0; i<num_gcs(); i++) {
        GCS_MAVLINK &c = *chan(i);
        if (c.is_private()) {
            continue;
        }
        if (!c.is_active()) {
            continue;
        }
#if HAL_HIGH_LATENCY2_ENABLED
        if (c.is_high_latency_link) {
            continue;
        }
#endif
        float value = param_value;
        int32_t int_value = 0;
        packet.param_type = GCS_MAVLINK::mav_param_send_encoding(c.get_chan(), vp, param_type, value, int_value, c._param_supported_types);
        if (packet.param_type == MAV_PARAM_TYPE_BYTEWISE_INT32) {
            memcpy(&packet.param_value, &int_value, sizeof(int_value));
        } else {
            packet.param_value = value;
        }
        // size checks done by this method:
        c.send_message((const char *)&packet, &param_entry);
    }

#if HAL_LOGGING_ENABLED
    // also log to AP_Logger
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger != nullptr) {
        logger->Write_Parameter(param_name, param_value);
    }
#endif
}


/*
  timer callback for async parameter requests
 */
void GCS_MAVLINK::param_io_timer(void)
{
    struct pending_param_request req;

    // this is mostly a no-op, but doing this here means we won't
    // block the main thread counting parameters (~30ms on PH)
    AP_Param::count_parameters();

    if (param_replies.space() == 0) {
        // no room
        return;
    }
    
    if (!param_requests.pop(req)) {
        // nothing to do
        return;
    }

    struct pending_param_reply reply;
    AP_Param *vp;

    if (req.param_index != -1) {
        AP_Param::ParamToken token {};
        vp = AP_Param::find_by_index(req.param_index, &reply.p_type, &token);
        if (vp != nullptr) {
            vp->copy_name_token(token, reply.param_name, AP_MAX_NAME_SIZE, true);
        } else {
            memset(reply.param_name, '\0', sizeof(reply.param_name));
        }
    } else {
        strncpy(reply.param_name, req.param_name, AP_MAX_NAME_SIZE+1);
        vp = AP_Param::find(req.param_name, &reply.p_type);
    }

    reply.chan = req.chan;
    reply.src_system_id = req.src_system_id;
    reply.src_component_id = req.src_component_id;
    reply.param_name[AP_MAX_NAME_SIZE] = 0;
    reply.int_value = 0;
    if (vp != nullptr) {
        reply.value = vp->cast_to_float(reply.p_type);
        reply.mav_type = mav_param_send_encoding(reply.chan, vp, reply.p_type, reply.value, reply.int_value, req.supported_types);
        reply.param_error = MAV_PARAM_ERROR_NO_ERROR;
    } else {
        reply.value = NaNf;
        reply.mav_type = MAV_PARAM_TYPE_REAL32;
        reply.param_error = MAV_PARAM_ERROR_DOES_NOT_EXIST;
    }
    reply.param_index = req.param_index;
    reply.count = AP_Param::count_parameters();

    // queue for transmission
    param_replies.push(reply);
}

/*
  A variant of send_param_error which sends based off the received mavlink message information
 */
void GCS_MAVLINK::send_param_error(const mavlink_message_t &msg, const mavlink_param_set_t &param_set, MAV_PARAM_ERROR error)
{
    if (!HAVE_PAYLOAD_SPACE(chan, PARAM_ERROR)) {
        return;
    }
    char param_id[MAVLINK_MSG_PARAM_ERROR_FIELD_PARAM_ID_LEN] {};
    strncpy_noterm(param_id, param_set.param_id, ARRAY_SIZE(param_id));
    mavlink_msg_param_error_send(
        chan,
        msg.sysid,
        msg.compid,
        param_id,
        -1,
        error
        );
}

/*
  A variant of send_param_error which is sent based on what the
  parameter set thread returns
 */
void GCS_MAVLINK::send_param_error(const GCS_MAVLINK::pending_param_reply &reply, MAV_PARAM_ERROR error)
{
    if (!HAVE_PAYLOAD_SPACE(chan, PARAM_ERROR)) {
        return;
    }
    char param_id[MAVLINK_MSG_PARAM_ERROR_FIELD_PARAM_ID_LEN] {};
    strncpy_noterm(param_id, reply.param_name, ARRAY_SIZE(param_id));
    mavlink_msg_param_error_send(
        chan,
        reply.src_system_id,
        reply.src_component_id,
        param_id,
        reply.param_index,
        error
        );
}

/*
  send replies to PARAM_REQUEST_READ
 */
uint8_t GCS_MAVLINK::send_parameter_async_replies()
{
    uint8_t async_replies_sent_count = 0;

    while (async_replies_sent_count < 5) {
        struct pending_param_reply reply;
        if (!param_replies.peek(reply)) {
            return async_replies_sent_count;
        }

        uint16_t required_space;
        if (reply.param_error == MAV_PARAM_ERROR_NO_ERROR) {
            required_space = packet_overhead() + MAVLINK_MSG_ID_PARAM_VALUE_MIN_LEN;
        } else {
            required_space = PAYLOAD_SIZE(chan, PARAM_ERROR);
        }

        /*
          we reserve some space for sending parameters if the client ever
          fails to get a parameter due to lack of space
        */
        uint32_t saved_reserve_param_space_start_ms = reserve_param_space_start_ms;
        reserve_param_space_start_ms = 0; // bypass packet_overhead_chan reservation checking
        if (txspace() < required_space) {
            out_of_space_to_send();
            reserve_param_space_start_ms = AP_HAL::millis();
            return async_replies_sent_count;
        }
        reserve_param_space_start_ms = saved_reserve_param_space_start_ms;

        if (reply.param_error == MAV_PARAM_ERROR_NO_ERROR) {
            // Any channel can drain the shared queue. Use the destination's
            // current advertisement when checking for a pending downgrade.
            const GCS_MAVLINK *reply_link = gcs().chan(reply.chan);
            const uint32_t supported_types = reply_link == nullptr ? 0 : reply_link->_param_supported_types;
            param_value_send(
                reply.chan,
                reply.param_name,
                reply.value,
                reply.mav_type,
                reply.int_value,
                reply.count,
                reply.param_index, supported_types);
        } else {
            send_param_error(reply, reply.param_error);
        }

        _queued_parameter_send_time_ms = AP_HAL::millis();
        async_replies_sent_count++;

        if (!param_replies.pop()) {
            // internal error...
            return async_replies_sent_count;
        }
    }
    return async_replies_sent_count;
}

void GCS_MAVLINK::handle_common_param_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        handle_param_request_list(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_SET:
        handle_param_set(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        handle_param_request_read(msg);
        break;
    }
}

#endif  // HAL_GCS_ENABLED
