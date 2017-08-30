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
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_Common/AP_FWVersion.h"
#include "GCS.h"

extern const AP_HAL::HAL& hal;

// queue of pending parameter requests and replies
ObjectBuffer<GCS_MAVLINK::pending_param_request> GCS_MAVLINK::param_requests(20);
ObjectBuffer<GCS_MAVLINK::pending_param_reply> GCS_MAVLINK::param_replies(5);

bool GCS_MAVLINK::param_timer_registered;

/**
 * @brief Send the next pending parameter, called from deferred message
 * handling code
 */
void
GCS_MAVLINK::queued_param_send()
{
    if (!initialised) {
        return;
    }

    // send one parameter async reply if pending
    send_parameter_reply();

    if (_queued_parameter == nullptr) {
        return;
    }
    
    uint16_t bytes_allowed;
    uint8_t count;
    uint32_t tnow = AP_HAL::millis();
    uint32_t tstart = AP_HAL::micros();

    // use at most 30% of bandwidth on parameters. The constant 26 is
    // 1/(1000 * 1/8 * 0.001 * 0.3)
    bytes_allowed = 57 * (tnow - _queued_parameter_send_time_ms) * 26;
    if (bytes_allowed > comm_get_txspace(chan)) {
        bytes_allowed = comm_get_txspace(chan);
    }
    count = bytes_allowed / (MAVLINK_MSG_ID_PARAM_VALUE_LEN + packet_overhead());

    // when we don't have flow control we really need to keep the
    // param download very slow, or it tends to stall
    if (!have_flow_control() && count > 5) {
        count = 5;
    }

    while (_queued_parameter != nullptr && count--) {
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

        if (AP_HAL::micros() - tstart > 1000) {
            // don't use more than 1ms sending blocks of parameters
            break;
        }
    }
    _queued_parameter_send_time_ms = tnow;
}

/*
  return true if a channel has flow control
 */
bool GCS_MAVLINK::have_flow_control(void)
{
    if (!valid_channel(chan)) {
        return false;
    }

    if (mavlink_comm_port[chan] == nullptr) {
        return false;
    }

    if (chan == MAVLINK_COMM_0) {
        // assume USB console has flow control
        return hal.gpio->usb_connected() || mavlink_comm_port[chan]->get_flow_control() != AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE;
    } else {
        // all other channels
        return mavlink_comm_port[chan]->get_flow_control() != AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE;
    }
}


/*
  handle a request to change stream rate. Note that copter passes in
  save==false so we don't want the save to happen when the user connects the
  ground station.
 */
void GCS_MAVLINK::handle_request_data_stream(mavlink_message_t *msg, bool save)
{
    mavlink_request_data_stream_t packet;
    mavlink_msg_request_data_stream_decode(msg, &packet);

    int16_t freq = 0;     // packet frequency

    if (packet.start_stop == 0)
        freq = 0;                     // stop sending
    else if (packet.start_stop == 1)
        freq = packet.req_message_rate;                     // start sending
    else
        return;

    AP_Int16 *rate = nullptr;
    switch (packet.req_stream_id) {
    case MAV_DATA_STREAM_ALL:
        // note that we don't set STREAM_PARAMS - that is internal only
        for (uint8_t i=0; i<STREAM_PARAMS; i++) {
            if (save) {
                streamRates[i].set_and_save_ifchanged(freq);
            } else {
                streamRates[i].set(freq);
            }
        }
        break;
    case MAV_DATA_STREAM_RAW_SENSORS:
        rate = &streamRates[STREAM_RAW_SENSORS];
        break;
    case MAV_DATA_STREAM_EXTENDED_STATUS:
        rate = &streamRates[STREAM_EXTENDED_STATUS];
        break;
    case MAV_DATA_STREAM_RC_CHANNELS:
        rate = &streamRates[STREAM_RC_CHANNELS];
        break;
    case MAV_DATA_STREAM_RAW_CONTROLLER:
        rate = &streamRates[STREAM_RAW_CONTROLLER];
        break;
    case MAV_DATA_STREAM_POSITION:
        rate = &streamRates[STREAM_POSITION];
        break;
    case MAV_DATA_STREAM_EXTRA1:
        rate = &streamRates[STREAM_EXTRA1];
        break;
    case MAV_DATA_STREAM_EXTRA2:
        rate = &streamRates[STREAM_EXTRA2];
        break;
    case MAV_DATA_STREAM_EXTRA3:
        rate = &streamRates[STREAM_EXTRA3];
        break;
    }

    if (rate != nullptr) {
        if (save) {
            rate->set_and_save_ifchanged(freq);
        } else {
            rate->set(freq);
        }
    }
}

void GCS_MAVLINK::handle_param_request_list(mavlink_message_t *msg)
{
    if (!params_ready()) {
        return;
    }

    mavlink_param_request_list_t packet;
    mavlink_msg_param_request_list_decode(msg, &packet);

    // requesting parameters is a convenient way to get extra information
    send_banner();

    // Start sending parameters - next call to ::update will kick the first one out
    _queued_parameter = AP_Param::first(&_queued_parameter_token, &_queued_parameter_type);
    _queued_parameter_index = 0;
    _queued_parameter_count = AP_Param::count_parameters();
}

void GCS_MAVLINK::handle_param_request_read(mavlink_message_t *msg)
{
    if (param_requests.space() == 0) {
        // we can't process this right now, drop it
        return;
    }
    
    mavlink_param_request_read_t packet;
    mavlink_msg_param_request_read_decode(msg, &packet);

    /*
      we reserve some space for sending parameters if the client ever
      fails to get a parameter due to lack of space
     */
    uint32_t saved_reserve_param_space_start_ms = reserve_param_space_start_ms;
    reserve_param_space_start_ms = 0;
    if (!HAVE_PAYLOAD_SPACE(chan, PARAM_VALUE)) {
        reserve_param_space_start_ms = AP_HAL::millis();
        return;
    }
    reserve_param_space_start_ms = saved_reserve_param_space_start_ms;

    struct pending_param_request req;
    req.chan = chan;
    req.param_index = packet.param_index;
    memcpy(req.param_name, packet.param_id, sizeof(req.param_name));
    req.param_name[AP_MAX_NAME_SIZE] = 0;

    // queue it for processing by io timer
    param_requests.push(req);
}

void GCS_MAVLINK::handle_param_set(mavlink_message_t *msg)
{
    mavlink_param_set_t packet;
    mavlink_msg_param_set_decode(msg, &packet);
    enum ap_var_type var_type;

    // set parameter
    AP_Param *vp;
    char key[AP_MAX_NAME_SIZE+1];
    strncpy(key, (char *)packet.param_id, AP_MAX_NAME_SIZE);
    key[AP_MAX_NAME_SIZE] = 0;

    // find existing param so we can get the old value
    vp = AP_Param::find(key, &var_type);
    if (vp == nullptr) {
        return;
    }
    float old_value = vp->cast_to_float(var_type);

    // set the value
    vp->set_float(packet.param_value, var_type);

    /*
      we force the save if the value is not equal to the old
      value. This copes with the use of override values in
      constructors, such as PID elements. Otherwise a set to the
      default value which differs from the constructor value doesn't
      save the change
     */
    bool force_save = !is_equal(packet.param_value, old_value);

    // save the change
    vp->save(force_save);

    DataFlash_Class *DataFlash = DataFlash_Class::instance();
    if (DataFlash != nullptr) {
        DataFlash->Log_Write_Parameter(key, vp->cast_to_float(var_type));
    }
}

// see if we should send a stream now. Called at 50Hz
bool GCS_MAVLINK::stream_trigger(enum streams stream_num)
{
    if (stream_num >= NUM_STREAMS) {
        return false;
    }
    float rate = (uint8_t)streamRates[stream_num].get();

    rate *= adjust_rate_for_stream_trigger(stream_num);

    if (rate <= 0) {
        if (chan_is_streaming & (1U<<(chan-MAVLINK_COMM_0))) {
            // if currently streaming then check if all streams are disabled
            // to allow runtime detection of user disabling streaming
            bool is_streaming = false;
            for (uint8_t i=0; i<stream_num; i++) {
                if (streamRates[stream_num] > 0) {
                    is_streaming = true;
                }
            }
            if (!is_streaming) {
                // all streams have been turned off, clear the bit flag
                chan_is_streaming &= ~(1U<<(chan-MAVLINK_COMM_0));
            }
        }
        return false;
    } else {
        chan_is_streaming |= (1U<<(chan-MAVLINK_COMM_0));
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

/*
  send a parameter value message to all active MAVLink connections
 */
void GCS_MAVLINK::send_parameter_value_all(const char *param_name, ap_var_type param_type, float param_value)
{
    for (uint8_t i=0; i<MAVLINK_COMM_NUM_BUFFERS; i++) {
        if ((1U<<i) & mavlink_active) {
            mavlink_channel_t chan = (mavlink_channel_t)(MAVLINK_COMM_0+i);
            if (HAVE_PAYLOAD_SPACE(chan, PARAM_VALUE)) {
                mavlink_msg_param_value_send(
                    chan,
                    param_name,
                    param_value,
                    mav_var_type(param_type),
                    AP_Param::count_parameters(),
                    -1);
            }
        }
    }
    // also log to DataFlash
    DataFlash_Class *dataflash = DataFlash_Class::instance();
    if (dataflash != nullptr) {
        dataflash->Log_Write_Parameter(param_name, param_value);
    }
}

/*
  send queued parameters if needed
 */
void GCS_MAVLINK::send_queued_parameters(void)
{
    if (!param_timer_registered) {
        param_timer_registered = true;
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&GCS_MAVLINK::param_io_timer, void));
    }

    if (_queued_parameter == nullptr &&
        param_replies.empty()) {
        return;
    }
    if (streamRates[STREAM_PARAMS].get() <= 0) {
        streamRates[STREAM_PARAMS].set(10);
    }
    if (stream_trigger(STREAM_PARAMS)) {
        send_message(MSG_NEXT_PARAM);
    }
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
        AP_Param::ParamToken token;
        vp = AP_Param::find_by_index(req.param_index, &reply.p_type, &token);
        if (vp == nullptr) {
            return;
        }
        vp->copy_name_token(token, reply.param_name, AP_MAX_NAME_SIZE, true);
    } else {
        strncpy(reply.param_name, req.param_name, AP_MAX_NAME_SIZE+1);
        vp = AP_Param::find(req.param_name, &reply.p_type);
        if (vp == nullptr) {
            return;
        }
    }

    reply.chan = req.chan;
    reply.param_name[AP_MAX_NAME_SIZE] = 0;
    reply.value = vp->cast_to_float(reply.p_type);
    reply.param_index = req.param_index;
    reply.count = AP_Param::count_parameters();

    // queue for transmission
    param_replies.push(reply);
}

/*
  send a reply to a PARAM_REQUEST_READ
 */
void GCS_MAVLINK::send_parameter_reply(void)
{
    struct pending_param_reply reply;
    
    if (!param_replies.pop(reply)) {
        // nothing to do
        return;
    }
    
    mavlink_msg_param_value_send(
        reply.chan,
        reply.param_name,
        reply.value,
        mav_var_type(reply.p_type),
        reply.count,
        reply.param_index);
}

void GCS_MAVLINK::handle_common_param_message(mavlink_message_t *msg)
{
    switch (msg->msgid) {
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
