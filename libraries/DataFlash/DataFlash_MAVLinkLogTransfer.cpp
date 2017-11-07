/*
  MAVLink logfile transfer functions
 */

/*
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


#include <AP_HAL/AP_HAL.h>
#include <DataFlash/DataFlash.h>
#include <GCS_MAVLink/GCS.h> // for LOG_ENTRY

extern const AP_HAL::HAL& hal;

// We avoid doing log messages when timing is critical:
bool DataFlash_Class::should_handle_log_message()
{
    if (!WritesEnabled()) {
        // this is currently used as a proxy for "in_mavlink_delay"
        return false;
    }
    if (vehicle_is_armed()) {
        return false;
    }
    return true;
}

/**
   handle all types of log download requests from the GCS
 */
void DataFlash_Class::handle_log_message(GCS_MAVLINK &link, mavlink_message_t *msg)
{
    if (!should_handle_log_message()) {
        return;
    }
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        handle_log_request_list(link, msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
        handle_log_request_data(link, msg);
        break;
    case MAVLINK_MSG_ID_LOG_ERASE:
        handle_log_request_erase(link, msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        handle_log_request_end(link, msg);
        break;
    }
}

/**
   handle all types of log download requests from the GCS
 */
void DataFlash_Class::handle_log_request_list(GCS_MAVLINK &link, mavlink_message_t *msg)
{
    if (_log_sending_chan >= 0) {
        link.send_text(MAV_SEVERITY_INFO, "Log download in progress");
        return;
    }

    mavlink_log_request_list_t packet;
    mavlink_msg_log_request_list_decode(msg, &packet);

    _log_listing = false;
    _log_sending = false;

    _log_num_logs = get_num_logs();
    if (_log_num_logs == 0) {
        _log_next_list_entry = 0;
        _log_last_list_entry = 0;        
    } else {
        _log_next_list_entry = packet.start;
        _log_last_list_entry = packet.end;

        if (_log_last_list_entry > _log_num_logs) {
            _log_last_list_entry = _log_num_logs;
        }
        if (_log_next_list_entry < 1) {
            _log_next_list_entry = 1;
        }
    }

    _log_listing = true;
    _log_sending_chan = link.get_chan();
    handle_log_send_listing(link);
}


/**
   handle request for log data
 */
void DataFlash_Class::handle_log_request_data(GCS_MAVLINK &link, mavlink_message_t *msg)
{
    if (_log_sending_chan >= 0) {
        // some GCS (e.g. MAVProxy) attempt to stream request_data
        // messages when they're filling gaps in the downloaded logs.
        // This channel check avoids complaining to them, at the cost
        // of silently dropping any repeated attempts to start logging
        if (_log_sending_chan != link.get_chan()) {
            link.send_text(MAV_SEVERITY_INFO, "Log download in progress");
        }
        return;
    }

    mavlink_log_request_data_t packet;
    mavlink_msg_log_request_data_decode(msg, &packet);

    _in_log_download = true;

    _log_listing = false;
    if (!_log_sending || _log_num_data != packet.id) {
        _log_sending = false;

        uint16_t num_logs = get_num_logs();
        if (packet.id > num_logs || packet.id < 1) {
            return;
        }

        uint32_t time_utc, size;
        get_log_info(packet.id, size, time_utc);
        _log_num_data = packet.id;
        _log_data_size = size;

        uint16_t end;
        get_log_boundaries(packet.id, _log_data_page, end);
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
    _log_sending_chan = link.get_chan();

    handle_log_send(link);
}

/**
   handle request to erase log data
 */
void DataFlash_Class::handle_log_request_erase(GCS_MAVLINK &link, mavlink_message_t *msg)
{
    // mavlink_log_erase_t packet;
    // mavlink_msg_log_erase_decode(msg, &packet);

    EraseAll();
}

/**
   handle request to stop transfer and resume normal logging
 */
void DataFlash_Class::handle_log_request_end(GCS_MAVLINK &link, mavlink_message_t *msg)
{
    mavlink_log_request_end_t packet;
    mavlink_msg_log_request_end_decode(msg, &packet);
    _in_log_download = false;
    _log_sending = false;
    _log_sending_chan = -1;
}

/**
   trigger sending of log messages if there are some pending
 */
void DataFlash_Class::handle_log_send(GCS_MAVLINK &link)
{
    if (_log_sending_chan != link.get_chan()) {
        return;
    }
    if (_log_listing) {
        handle_log_send_listing(link);
    }
    if (!_log_sending) {
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // assume USB speeds in SITL for the purposes of log download
    const uint8_t num_sends = 40;
#else
    uint8_t num_sends = 1;
    if (link.is_high_bandwidth() && hal.gpio->usb_connected()) {
        // when on USB we can send a lot more data
        num_sends = 250;
    } else if (link.have_flow_control()) {
    #if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
        num_sends = 80;
    #else
        num_sends = 10;
    #endif
    }
#endif

    for (uint8_t i=0; i<num_sends; i++) {
        if (_log_sending) {
            if (!handle_log_send_data(link)) break;
        }
    }
}

/**
   trigger sending of log messages if there are some pending
 */
void DataFlash_Class::handle_log_send_listing(GCS_MAVLINK &link)
{
    if (!HAVE_PAYLOAD_SPACE(link.get_chan(), LOG_ENTRY)) {
        // no space
        return;
    }
    if (AP_HAL::millis() - link.get_last_heartbeat_time() > 3000) {
        // give a heartbeat a chance
        return;
    }

    uint32_t size, time_utc;
    if (_log_next_list_entry == 0) {
        size = 0;
        time_utc = 0;
    } else {
        get_log_info(_log_next_list_entry, size, time_utc);
    }
    mavlink_msg_log_entry_send(link.get_chan(), _log_next_list_entry, _log_num_logs, _log_last_list_entry, time_utc, size);
    if (_log_next_list_entry == _log_last_list_entry) {
        _log_listing = false;
        _log_sending_chan = -1;
    } else {
        _log_next_list_entry++;
    }
}

/**
   trigger sending of log data if there are some pending
 */
bool DataFlash_Class::handle_log_send_data(GCS_MAVLINK &link)
{
    if (!HAVE_PAYLOAD_SPACE(link.get_chan(), LOG_DATA)) {
        // no space
        return false;
    }
    if (AP_HAL::millis() - link.get_last_heartbeat_time() > 3000) {
        // give a heartbeat a chance
        return false;
    }

    int16_t ret = 0;
    uint32_t len = _log_data_remaining;
	mavlink_log_data_t packet;

    if (len > 90) {
        len = 90;
    }
    ret = get_log_data(_log_num_data, _log_data_page, _log_data_offset, len, packet.data);
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
    _mav_finalize_message_chan_send(link.get_chan(), MAVLINK_MSG_ID_LOG_DATA, (const char *)&packet,
                                    MAVLINK_MSG_ID_LOG_DATA_MIN_LEN,
                                    MAVLINK_MSG_ID_LOG_DATA_LEN,
                                    MAVLINK_MSG_ID_LOG_DATA_CRC);

    _log_data_offset += len;
    _log_data_remaining -= len;
    if (ret < 90 || _log_data_remaining == 0) {
        _log_sending = false;
        _log_sending_chan = -1;
    }
    return true;
}
