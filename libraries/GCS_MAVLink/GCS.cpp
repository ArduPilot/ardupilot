// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  Common GCS MAVLink functions for all vehicle types

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

#include <GCS.h>
#include <AP_AHRS.h>


extern const AP_HAL::HAL& hal;


GCS_MAVLINK::GCS_MAVLINK()
{
    AP_Param::setup_object_defaults(this, var_info);
}

/**
   handle all types of log download requests from the GCS
 */
void GCS_MAVLINK::handle_log_message(mavlink_message_t *msg, DataFlash_Class &dataflash)
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        handle_log_request_list(msg, dataflash);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
        handle_log_request_data(msg, dataflash);
        break;
    case MAVLINK_MSG_ID_LOG_ERASE:
        handle_log_request_erase(msg, dataflash);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        handle_log_request_end(msg, dataflash);
        break;
    }
}


/**
   handle all types of log download requests from the GCS
 */
void GCS_MAVLINK::handle_log_request_list(mavlink_message_t *msg, DataFlash_Class &dataflash)
{
    mavlink_log_request_list_t packet;
    mavlink_msg_log_request_list_decode(msg, &packet);

    _log_listing = false;
    _log_sending = false;

    _log_num_logs = dataflash.get_num_logs();
    if (_log_num_logs == 0) {
        _log_next_list_entry = 0;
        _log_last_list_entry = 0;        
    } else {
        uint16_t last_log_num = dataflash.find_last_log();

        _log_next_list_entry = packet.start;
        _log_last_list_entry = packet.end;

        if (_log_last_list_entry > last_log_num) {
            _log_last_list_entry = last_log_num;
        }
        if (_log_next_list_entry < last_log_num + 1 - _log_num_logs) {
            _log_next_list_entry = last_log_num + 1 - _log_num_logs;
        }
    }

    _log_listing = true;
    handle_log_send_listing(dataflash);
}


/**
   handle request for log data
 */
void GCS_MAVLINK::handle_log_request_data(mavlink_message_t *msg, DataFlash_Class &dataflash)
{
    mavlink_log_request_data_t packet;
    mavlink_msg_log_request_data_decode(msg, &packet);

    _log_listing = false;
    if (!_log_sending || _log_num_data != packet.id) {
        _log_sending = false;

        uint16_t num_logs = dataflash.get_num_logs();
        uint16_t last_log_num = dataflash.find_last_log();
        if (packet.id > last_log_num || packet.id < last_log_num + 1 - num_logs) {
            return;
        }

        uint32_t time_utc, size;
        dataflash.get_log_info(packet.id, size, time_utc);
        _log_num_data = packet.id;
        _log_data_size = size;

        uint16_t end;
        dataflash.get_log_boundaries(packet.id, _log_data_page, end);
    }

    _log_data_offset = packet.ofs;
    if (_log_data_offset >= _log_data_size) {
        _log_data_remaining = 0;
    } else {
        _log_data_remaining = _log_data_size - _log_data_offset;
    }
    if (_log_data_remaining > packet.count) {
        _log_data_remaining = packet.count;
    }
    _log_sending = true;

    handle_log_send(dataflash);
}

/**
   handle request to erase log data
 */
void GCS_MAVLINK::handle_log_request_erase(mavlink_message_t *msg, DataFlash_Class &dataflash)
{
    mavlink_log_erase_t packet;
    mavlink_msg_log_erase_decode(msg, &packet);

    dataflash.EraseAll();
}

/**
   handle request to stop transfer and resume normal logging
 */
void GCS_MAVLINK::handle_log_request_end(mavlink_message_t *msg, DataFlash_Class &dataflash)
{
    mavlink_log_request_end_t packet;
    mavlink_msg_log_request_end_decode(msg, &packet);
    _log_sending = false;
}

/**
   trigger sending of log messages if there are some pending
 */
void GCS_MAVLINK::handle_log_send(DataFlash_Class &dataflash)
{
    if (_log_listing) {
        handle_log_send_listing(dataflash);
    }
    if (!_log_sending) {
        return;
    }
    uint8_t num_sends = 1;
    if (chan == MAVLINK_COMM_0 && hal.gpio->usb_connected()) {
        // when on USB we can send a lot more data
        num_sends = 40;
    } else if (have_flow_control()) {
        num_sends = 10;        
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // assume USB speeds in SITL for the purposes of log download
    num_sends = 40;
#endif

    for (uint8_t i=0; i<num_sends; i++) {
        if (_log_sending) {
            if (!handle_log_send_data(dataflash)) break;
        }
    }
}

/**
   trigger sending of log messages if there are some pending
 */
void GCS_MAVLINK::handle_log_send_listing(DataFlash_Class &dataflash)
{
    uint16_t txspace = comm_get_txspace(chan);
    if (txspace < MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_LOG_ENTRY_LEN) {
        // no space
        return;
    }
    if (hal.scheduler->millis() - last_heartbeat_time > 3000) {
        // give a heartbeat a chance
        return;
    }

    uint32_t size, time_utc;
    if (_log_next_list_entry == 0) {
        size = 0;
        time_utc = 0;
    } else {
        dataflash.get_log_info(_log_next_list_entry, size, time_utc);
    }
    mavlink_msg_log_entry_send(chan, _log_next_list_entry, _log_num_logs, _log_last_list_entry, time_utc, size);
    if (_log_next_list_entry == _log_last_list_entry) {
        _log_listing = false;
    } else {
        _log_next_list_entry++;
    }
}

/**
   trigger sending of log data if there are some pending
 */
bool GCS_MAVLINK::handle_log_send_data(DataFlash_Class &dataflash)
{
    uint16_t txspace = comm_get_txspace(chan);
    if (txspace < MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_LOG_DATA_LEN) {
        // no space
        return false;
    }
    if (hal.scheduler->millis() - last_heartbeat_time > 3000) {
        // give a heartbeat a chance
        return false;
    }

    int16_t ret = 0;
    uint32_t len = _log_data_remaining;
  mavlink_log_data_t packet;

    if (len > 90) {
        len = 90;
    }
    ret = dataflash.get_log_data(_log_num_data, _log_data_page, _log_data_offset, len, packet.data);
    if (ret < 0) {
        // report as EOF on error
        ret = 0;
    }
    if (ret < 90) {
        memset(&packet.data[ret], 0, 90-ret);
    }

    packet.ofs = _log_data_offset;
    packet.id = _log_num_data;
    packet.count = ret;
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOG_DATA, (const char *)&packet, 
                                    MAVLINK_MSG_ID_LOG_DATA_LEN, MAVLINK_MSG_ID_LOG_DATA_CRC);

    _log_data_offset += len;
    _log_data_remaining -= len;
    if (ret < 90 || _log_data_remaining == 0) {
        _log_sending = false;
    }
    return true;
}

void GCS_MAVLINK::update(void (*run_cli)(AP_HAL::UARTDriver *)) {
    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;
    status.packet_rx_drop_count = 0;

    // process received bytes
    uint16_t nbytes = comm_get_available(chan);
    for (uint16_t i=0; i<nbytes; i++)
    {
        uint8_t c = comm_receive_ch(chan);

        if (run_cli != NULL) {
            /* allow CLI to be started by hitting enter 3 times, if no
             *  heartbeat packets have been received */
            if ((mavlink_active==0) && (hal.scheduler->millis() - _cli_timeout) < 20000 && 
                comm_is_idle(chan)) {
                if (c == '\n' || c == '\r') {
                    crlf_count++;
                } else {
                    crlf_count = 0;
                }
                if (crlf_count == 3) {
                    run_cli(_port);
                }
            }
        }

        // Try to get a new message
        if (mavlink_parse_char(chan, c, &msg, &status)) {
            // we exclude radio packets to make it possible to use the
            // CLI over the radio
            if (msg.msgid != MAVLINK_MSG_ID_RADIO && msg.msgid != MAVLINK_MSG_ID_RADIO_STATUS) {
                mavlink_active |= (1U<<(chan-MAVLINK_COMM_0));
            }
            // if a snoop handler has been setup then use it
            if (msg_snoop != NULL) {
                msg_snoop(&msg);
            }
            if (routing.check_and_forward(chan, &msg)) {
                handleMessage(&msg);
            }
        }
    }

    if (!waypoint_receiving) {
        return;
    }

    uint32_t tnow = hal.scheduler->millis();
    uint32_t wp_recv_time = 1000U + (stream_slowdown*20);

    if (waypoint_receiving &&
        waypoint_request_i <= waypoint_request_last &&
        tnow - waypoint_timelast_request > wp_recv_time) {
        waypoint_timelast_request = tnow;
        send_message(MSG_NEXT_WAYPOINT);
    }

    // stop waypoint receiving if timeout
    if (waypoint_receiving && (tnow - waypoint_timelast_receive) > wp_recv_time+waypoint_receive_timeout) {
        waypoint_receiving = false;
    }
}

/*
  handle a MISSION_REQUEST_LIST mavlink packet
 */
void GCS_MAVLINK::handle_mission_request_list(AP_Mission &mission, mavlink_message_t *msg)
{
    // decode
    mavlink_mission_request_list_t packet;
    mavlink_msg_mission_request_list_decode(msg, &packet);

    // reply with number of commands in the mission.  The GCS will then request each command separately
    mavlink_msg_mission_count_send(chan,msg->sysid, msg->compid, mission.num_commands());

    // set variables to help handle the expected sending of commands to the GCS
    waypoint_receiving = false;             // record that we are sending commands (i.e. not receiving)
    waypoint_dest_sysid = msg->sysid;       // record system id of GCS who has requested the commands
    waypoint_dest_compid = msg->compid;     // record component id of GCS who has requested the commands
}

/*
  handle a MISSION_REQUEST mavlink packet
 */
void GCS_MAVLINK::handle_mission_request(AP_Mission &mission, mavlink_message_t *msg)
{
    AP_Mission::Mission_Command cmd;
    // decode
    mavlink_mission_request_t packet;
    mavlink_msg_mission_request_decode(msg, &packet);

    // retrieve mission from eeprom
    if (!mission.read_cmd_from_storage(packet.seq, cmd)) {
        goto mission_item_send_failed;
    }

    // convert mission command to mavlink mission item packet
    mavlink_mission_item_t ret_packet;
    memset(&ret_packet, 0, sizeof(ret_packet));
    if (!AP_Mission::mission_cmd_to_mavlink(cmd, ret_packet)) {
        goto mission_item_send_failed;
    }

    // set packet's current field to 1 if this is the command being executed
    if (cmd.id == (uint16_t)mission.get_current_nav_cmd().index) {
        ret_packet.current = 1;
    } else {
        ret_packet.current = 0;
    }

    // set auto continue to 1
    ret_packet.autocontinue = 1;     // 1 (true), 0 (false)

    /*
      avoid the _send() function to save memory on APM2, as it avoids
      the stack usage of the _send() function by using the already
      declared ret_packet above
     */
    ret_packet.target_system = msg->sysid;
    ret_packet.target_component = msg->compid;
    ret_packet.seq = packet.seq;
    ret_packet.command = cmd.id;

    _mav_finalize_message_chan_send(chan, 
                                    MAVLINK_MSG_ID_MISSION_ITEM,
                                    (const char *)&ret_packet,
                                    MAVLINK_MSG_ID_MISSION_ITEM_LEN,
                                    MAVLINK_MSG_ID_MISSION_ITEM_CRC);
    return;

mission_item_send_failed:
    // send failure message
    mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, MAV_MISSION_ERROR);
}

/*
  handle a MISSION_SET_CURRENT mavlink packet
 */
void GCS_MAVLINK::handle_mission_set_current(AP_Mission &mission, mavlink_message_t *msg)
{
    // decode
    mavlink_mission_set_current_t packet;
    mavlink_msg_mission_set_current_decode(msg, &packet);

    // set current command
    if (mission.set_current_cmd(packet.seq)) {
        mavlink_msg_mission_current_send(chan, mission.get_current_nav_cmd().index);
    }
}

/*
  handle a MISSION_COUNT mavlink packet
 */
void GCS_MAVLINK::handle_mission_count(AP_Mission &mission, mavlink_message_t *msg)
{
    // decode
    mavlink_mission_count_t packet;
    mavlink_msg_mission_count_decode(msg, &packet);

    // start waypoint receiving
    if (packet.count > mission.num_commands_max()) {
        // send NAK
        mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, MAV_MISSION_NO_SPACE);
        return;
    }

    // new mission arriving, truncate mission to be the same length
    mission.truncate(packet.count);

    // set variables to help handle the expected receiving of commands from the GCS
    waypoint_timelast_receive = hal.scheduler->millis();    // set time we last received commands to now
    waypoint_receiving = true;              // record that we expect to receive commands
    waypoint_request_i = 0;                 // reset the next expected command number to zero
    waypoint_request_last = packet.count;   // record how many commands we expect to receive
    waypoint_timelast_request = 0;          // set time we last requested commands to zero
}

/*
  handle a MISSION_CLEAR_ALL mavlink packet
 */
void GCS_MAVLINK::handle_mission_clear_all(AP_Mission &mission, mavlink_message_t *msg)
{
    // decode
    mavlink_mission_clear_all_t packet;
    mavlink_msg_mission_clear_all_decode(msg, &packet);

    // clear all waypoints
    if (mission.clear()) {
        // send ack
        mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, MAV_RESULT_ACCEPTED);
    }else{
        // send nack
        mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, 1);
    }
}

/*
  handle a MISSION_WRITE_PARTIAL_LIST mavlink packet
 */
void GCS_MAVLINK::handle_mission_write_partial_list(AP_Mission &mission, mavlink_message_t *msg)
{
    // decode
    mavlink_mission_write_partial_list_t packet;
    mavlink_msg_mission_write_partial_list_decode(msg, &packet);

    // start waypoint receiving
    if ((unsigned)packet.start_index > mission.num_commands() ||
        (unsigned)packet.end_index > mission.num_commands() ||
        packet.end_index < packet.start_index) {
        send_text_P(SEVERITY_LOW,PSTR("flight plan update rejected"));
        return;
    }

    waypoint_timelast_receive = hal.scheduler->millis();
    waypoint_timelast_request = 0;
    waypoint_receiving   = true;
    waypoint_request_i   = packet.start_index;
    waypoint_request_last= packet.end_index;
}


/*
  handle a GIMBAL_REPORT mavlink packet
 */
void GCS_MAVLINK::handle_gimbal_report(AP_Mount &mount, mavlink_message_t *msg) const
{
    mount.handle_gimbal_report(chan, msg);
}


/*
  handle a request to change stream rate. Note that copter passes in
  save==false, as sending mavlink messages on copter on APM2 costs
  enough that it can cause flight issues, so we don't want the save to
  happen when the user connects the ground station.
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

    AP_Int16 *rate = NULL;
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

    if (rate != NULL) {
        if (save) {
            rate->set_and_save_ifchanged(freq);
        } else {
            rate->set(freq);
        }
    }
}

void GCS_MAVLINK::handle_param_request_list(mavlink_message_t *msg)
{
    mavlink_param_request_list_t packet;
    mavlink_msg_param_request_list_decode(msg, &packet);

#if CONFIG_HAL_BOARD != HAL_BOARD_APM1 && CONFIG_HAL_BOARD != HAL_BOARD_APM2
    // send system ID if we can
    char sysid[40];
    if (hal.util->get_system_id(sysid)) {
        send_text(SEVERITY_LOW, sysid);
    }
#endif

    // Start sending parameters - next call to ::update will kick the first one out
    _queued_parameter = AP_Param::first(&_queued_parameter_token, &_queued_parameter_type);
    _queued_parameter_index = 0;
    _queued_parameter_count = _count_parameters();
}

void GCS_MAVLINK::handle_param_request_read(mavlink_message_t *msg)
{
    mavlink_param_request_read_t packet;
    mavlink_msg_param_request_read_decode(msg, &packet);

    enum ap_var_type p_type;
    AP_Param *vp;
    char param_name[AP_MAX_NAME_SIZE+1];
    if (packet.param_index != -1) {
        AP_Param::ParamToken token;
        vp = AP_Param::find_by_index(packet.param_index, &p_type, &token);
        if (vp == NULL) {
            return;
        }
        vp->copy_name_token(token, param_name, AP_MAX_NAME_SIZE, true);
        param_name[AP_MAX_NAME_SIZE] = 0;
    } else {
        strncpy(param_name, packet.param_id, AP_MAX_NAME_SIZE);
        param_name[AP_MAX_NAME_SIZE] = 0;
        vp = AP_Param::find(param_name, &p_type);
        if (vp == NULL) {
            return;
        }
    }
    
    float value = vp->cast_to_float(p_type);
    mavlink_msg_param_value_send_buf(
        msg,
        chan,
        param_name,
        value,
        mav_var_type(p_type),
        _count_parameters(),
        packet.param_index);
}

void GCS_MAVLINK::handle_param_set(mavlink_message_t *msg, DataFlash_Class *DataFlash)
{
    mavlink_param_set_t packet;
    mavlink_msg_param_set_decode(msg, &packet);
    enum ap_var_type var_type;

    // set parameter
    AP_Param *vp;
    char key[AP_MAX_NAME_SIZE+1];
    strncpy(key, (char *)packet.param_id, AP_MAX_NAME_SIZE);
    key[AP_MAX_NAME_SIZE] = 0;

    vp = AP_Param::set_param_by_name(key, packet.param_value, &var_type);
    if (vp == NULL) {
        return;
    }

    // save the change
    vp->save();

    // Report back the new value if we accepted the change
    // we send the value we actually set, which could be
    // different from the value sent, in case someone sent
    // a fractional value to an integer type
    mavlink_msg_param_value_send_buf(
        msg,
        chan,
        key,
        vp->cast_to_float(var_type),
        mav_var_type(var_type),
        _count_parameters(),
        -1);     // XXX we don't actually know what its index is...

    if (DataFlash != NULL) {
        DataFlash->Log_Write_Parameter(key, vp->cast_to_float(var_type));
    }
}

void GCS_MAVLINK::handle_radio_status(mavlink_message_t *msg, DataFlash_Class &dataflash, bool log_radio)
{
    mavlink_radio_t packet;
    mavlink_msg_radio_decode(msg, &packet);

    // record if the GCS has been receiving radio messages from
    // the aircraft
    if (packet.remrssi != 0) {
        last_radio_status_remrssi_ms = hal.scheduler->millis();
    }

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

    //log rssi, noise, etc if logging Performance monitoring data
    if (log_radio) {
        dataflash.Log_Write_Radio(packet);
    }
}


/*
  handle an incoming mission item
  return true if this is the last mission item, otherwise false
 */
bool GCS_MAVLINK::handle_mission_item(mavlink_message_t *msg, AP_Mission &mission)
{
    mavlink_mission_item_t packet;
    MAV_MISSION_RESULT result = MAV_MISSION_ACCEPTED;
    struct AP_Mission::Mission_Command cmd = {};
    bool mission_is_complete = false;

    mavlink_msg_mission_item_decode(msg, &packet);

    // convert mavlink packet to mission command
    if (!AP_Mission::mavlink_to_mission_cmd(packet, cmd)) {
        result = MAV_MISSION_INVALID;
        goto mission_ack;
    }

    if (packet.current == 2) {                                               
        // current = 2 is a flag to tell us this is a "guided mode"
        // waypoint and not for the mission
        handle_guided_request(cmd);

        // verify we received the command
        result = MAV_MISSION_ACCEPTED;
        goto mission_ack;
    }

    if (packet.current == 3) {
        //current = 3 is a flag to tell us this is a alt change only
        // add home alt if needed
        handle_change_alt_request(cmd);

        // verify we recevied the command
        result = MAV_MISSION_ACCEPTED;
        goto mission_ack;
    }

    // Check if receiving waypoints (mission upload expected)
    if (!waypoint_receiving) {
        result = MAV_MISSION_ERROR;
        goto mission_ack;
    }

    // check if this is the requested waypoint
    if (packet.seq != waypoint_request_i) {
        result = MAV_MISSION_INVALID_SEQUENCE;
        goto mission_ack;
    }
    
    // if command index is within the existing list, replace the command
    if (packet.seq < mission.num_commands()) {
        if (mission.replace_cmd(packet.seq,cmd)) {
            result = MAV_MISSION_ACCEPTED;
        }else{
            result = MAV_MISSION_ERROR;
            goto mission_ack;
        }
        // if command is at the end of command list, add the command
    } else if (packet.seq == mission.num_commands()) {
        if (mission.add_cmd(cmd)) {
            result = MAV_MISSION_ACCEPTED;
        }else{
            result = MAV_MISSION_ERROR;
            goto mission_ack;
        }
        // if beyond the end of the command list, return an error
    } else {
        result = MAV_MISSION_ERROR;
        goto mission_ack;
    }
    
    // update waypoint receiving state machine
    waypoint_timelast_receive = hal.scheduler->millis();
    waypoint_request_i++;
    
    if (waypoint_request_i >= waypoint_request_last) {
        mavlink_msg_mission_ack_send_buf(
            msg,
            chan,
            msg->sysid,
            msg->compid,
            MAV_MISSION_ACCEPTED);
        
        send_text_P(SEVERITY_LOW,PSTR("flight plan received"));
        waypoint_receiving = false;
        mission_is_complete = true;
        // XXX ignores waypoint radius for individual waypoints, can
        // only set WP_RADIUS parameter
    } else {
        waypoint_timelast_request = hal.scheduler->millis();
        // if we have enough space, then send the next WP immediately
        if (comm_get_txspace(chan) >= 
            MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_MISSION_ITEM_LEN) {
            queued_waypoint_send();
        } else {
            send_message(MSG_NEXT_WAYPOINT);
        }
    }
    return mission_is_complete;

mission_ack:
    // we are rejecting the mission/waypoint
    mavlink_msg_mission_ack_send_buf(
        msg,
        chan,
        msg->sysid,
        msg->compid,
        result);

    return mission_is_complete;
}

void GCS_MAVLINK::handle_gps_inject(const mavlink_message_t *msg, AP_GPS &gps)
{
    mavlink_gps_inject_data_t packet;
    mavlink_msg_gps_inject_data_decode(msg, &packet);
    //TODO: check target

    gps.inject_data(packet.data, packet.len);

}

/*
  handle a SET_MODE MAVLink message
 */
void GCS_MAVLINK::handle_set_mode(mavlink_message_t* msg, bool (*set_mode)(uint8_t mode))
{
    uint8_t result = MAV_RESULT_FAILED;
    mavlink_set_mode_t packet;
    mavlink_msg_set_mode_decode(msg, &packet);

    // only accept custom modes because there is no easy mapping from Mavlink flight modes to AC flight modes
    if (packet.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
        if (set_mode(packet.custom_mode)) {
            result = MAV_RESULT_ACCEPTED;
        }
    } else if (packet.base_mode == MAV_MODE_FLAG_DECODE_POSITION_SAFETY) {
        // set the safety switch position. Must be in a command by itself
        if (packet.custom_mode == 0) {
            // turn safety off (pwm outputs flow to the motors)
            hal.rcout->force_safety_off();
            result = MAV_RESULT_ACCEPTED;
        } else if (packet.custom_mode == 1) {
            // turn safety on (no pwm outputs to the motors)
            if (hal.rcout->force_safety_on()) {
                result = MAV_RESULT_ACCEPTED;
            }
        }
    }

    // send ACK or NAK
    mavlink_msg_command_ack_send_buf(msg, chan, MAVLINK_MSG_ID_SET_MODE, result);
}
