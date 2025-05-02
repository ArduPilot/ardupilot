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

#include <GCS_MAVLink/GCS_config.h>
#include <AP_Logger/AP_Logger_config.h>

#if HAL_LOGGING_ENABLED && HAL_GCS_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h> // for LOG_ENTRY

extern const AP_HAL::HAL& hal;

/**
   handle all types of log download requests from the GCS.  This
   enqueues a request to the thread, which will send the mavlink
   responses when it is ready.
 */
void AP_Logger::handle_log_message(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    const bool synchronous = false;  // FIXME, these should all be async

    AP_LoggerThreadRequest *request = request_queue.claim_thread_request();
    if (request == nullptr) {
        return;
    }
    if (!synchronous) {
        request->free_after_processing = true;
    }
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST: {
        mavlink_log_request_list_t packet;
        mavlink_msg_log_request_list_decode(
            &msg,
            &packet);
        request->type = AP_LoggerThreadRequest::Type::HandleLogRequest_List;
        request->parameters.HandleLogRequest_List = {
            &link,
            packet.start,
            packet.end
        };
        break;
    }
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA: {
        mavlink_log_request_data_t packet;
        mavlink_msg_log_request_data_decode(
            &msg,
            &packet);
        request->type = AP_LoggerThreadRequest::Type::HandleLogRequest_Data;
        request->parameters.HandleLogRequest_Data = {
            &link,
            packet.ofs,
            packet.count,
            packet.id
        };
        break;
    }
    case MAVLINK_MSG_ID_LOG_ERASE:
        request->type = AP_LoggerThreadRequest::Type::HandleLogRequest_Erase;
        request->parameters.HandleLogRequest_Erase.link = &link;
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        request->type = AP_LoggerThreadRequest::Type::HandleLogRequest_End;
        request->parameters.HandleLogRequest_End.link = &link;
        break;
    default:
        return;
    }

    // synchronous completion:
    if (synchronous) {
        if (!request_queue.complete_thread_request(*request)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "%s failed", "HandleLogMessage");
        }
    } else {
        request->state = AP_LoggerThreadRequest::State::PENDING;
    }
}

void AP_LoggerThread::handle_log_request_message(AP_LoggerThreadRequest &request)
{
    // link is transported as a pointer to avoid having to create a
    // constructor on parameters.HandleLogMessage
    switch (request.type) {
    case AP_LoggerThreadRequest::Type::HandleLogRequest_List:
        handle_log_request_list(
            *(request.parameters.HandleLogRequest_List.link),
            request.parameters.HandleLogRequest_List.start,
            request.parameters.HandleLogRequest_List.end
            );
         break;
    case AP_LoggerThreadRequest::Type::HandleLogRequest_Data:
        handle_log_request_data(
            *request.parameters.HandleLogRequest_Data.link,
            request.parameters.HandleLogRequest_Data.id,
            request.parameters.HandleLogRequest_Data.ofs,
            request.parameters.HandleLogRequest_Data.count
            );
         break;
    case AP_LoggerThreadRequest::Type::HandleLogRequest_Erase:
        handle_log_request_erase(*request.parameters.HandleLogRequest_Erase.link);
         break;
    case AP_LoggerThreadRequest::Type::HandleLogRequest_End:
        handle_log_request_end(*request.parameters.HandleLogRequest_End.link);
        break;
    default:
        // hope caller did correct thing....
         break;
     }
 }

/**
   handle all types of log download requests from the GCS
 */
void AP_LoggerThread::handle_log_request_list(GCS_MAVLINK &link, uint16_t start, uint16_t end)
{
    WITH_SEMAPHORE(_log_send_sem);

    if (_log_sending_link != nullptr) {
        link.send_text(MAV_SEVERITY_INFO, "Log download in progress");
        return;
    }

    _log_num_logs = get_num_logs();

    if (_log_num_logs == 0) {
        _log_next_list_entry = 0;
        _log_last_list_entry = 0;        
    } else {
        _log_next_list_entry = start;
        _log_last_list_entry = end;

        if (_log_last_list_entry > _log_num_logs) {
            _log_last_list_entry = _log_num_logs;
        }
        if (_log_next_list_entry < 1) {
            _log_next_list_entry = 1;
        }
    }

    transfer_activity = TransferActivity::LISTING;
    _log_sending_link = &link;

    handle_log_send_listing();
}


/**
   handle request for log data
 */
void AP_LoggerThread::handle_log_request_data(GCS_MAVLINK &link, uint16_t id, uint32_t ofs, uint32_t count)
{
    WITH_SEMAPHORE(_log_send_sem);

    if (_log_sending_link != nullptr) {
        // some GCS (e.g. MAVProxy) attempt to stream request_data
        // messages when they're filling gaps in the downloaded logs.
        // This channel check avoids complaining to them, at the cost
        // of silently dropping any repeated attempts to start logging
        if (_log_sending_link->get_chan() != link.get_chan()) {
            link.send_text(MAV_SEVERITY_INFO, "Log download in progress");
        }
        return;
    }

    // consider opening or switching logs:
    if (transfer_activity != TransferActivity::SENDING || _log_num_data != id) {

        uint16_t num_logs = get_num_logs();
        if (id > num_logs || id < 1) {
            // request for an invalid log; cancel any current download
            end_log_transfer();
            return;
        }

        uint32_t time_utc, size;
        get_log_info(id, size, time_utc);
        _log_num_data = id;
        _log_data_size = size;

        uint32_t end;
        get_log_boundaries(id, _log_data_page, end);
    }

    _log_data_offset = ofs;
    if (_log_data_offset >= _log_data_size) {
        _log_data_remaining = 0;
    } else {
        _log_data_remaining = _log_data_size - _log_data_offset;
    }
    if (_log_data_remaining > count) {
        _log_data_remaining = count;
    }

    transfer_activity = TransferActivity::SENDING;
    _log_sending_link = &link;

    handle_log_send();
}

/**
   handle request to erase log data
 */
void AP_LoggerThread::handle_log_request_erase(GCS_MAVLINK &link)
{
    WITH_SEMAPHORE(_log_send_sem);

    transfer_activity = TransferActivity::IDLE;
    _log_sending_link = nullptr;

    erase();
}

/**
   handle request to stop transfer and resume normal logging
 */
void AP_LoggerThread::handle_log_request_end(GCS_MAVLINK &link)
{
    WITH_SEMAPHORE(_log_send_sem);
    // mavlink_log_request_end_t packet;
    // mavlink_msg_log_request_end_decode(&msg, &packet);
    end_log_transfer();
}

void AP_LoggerThread::end_log_transfer()
{
    transfer_activity = TransferActivity::IDLE;
    _log_sending_link = nullptr;
    backends[0]->end_log_transfer();
}

/**
   trigger sending of log messages if there are some pending
 */
void AP_LoggerThread::handle_log_send()
{
    WITH_SEMAPHORE(_log_send_sem);

    if (_log_sending_link == nullptr) {
        return;
    }
    if (hal.util->get_soft_armed()) {
        // might be flying
        return;
    }
    switch (transfer_activity) {
    case TransferActivity::IDLE:
        break;
    case TransferActivity::LISTING:
        handle_log_send_listing();
        break;
    case TransferActivity::SENDING:
        handle_log_sending();
        break;
    }
}

void AP_LoggerThread::handle_log_sending()
{
    WITH_SEMAPHORE(_log_send_sem);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // assume USB speeds in SITL for the purposes of log download
    const uint8_t num_sends = 40;
#else
    uint8_t num_sends = 1;
    if (_log_sending_link->is_high_bandwidth() && hal.gpio->usb_connected()) {
        // when on USB we can send a lot more data
        num_sends = 250;
    } else if (_log_sending_link->have_flow_control()) {
    #if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
        num_sends = 80;
    #else
        num_sends = 10;
    #endif
    }
#endif

    for (uint8_t i=0; i<num_sends; i++) {
        if (transfer_activity != TransferActivity::SENDING) {
            // may have completed sending data
            break;
        }
        if (!handle_log_send_data()) {
            break;
        }
    }
}

/**
   trigger sending of log messages if there are some pending
 */
void AP_LoggerThread::handle_log_send_listing()
{
    WITH_SEMAPHORE(_log_send_sem);

    if (!HAVE_PAYLOAD_SPACE(_log_sending_link->get_chan(), LOG_ENTRY)) {
        // no space
        return;
    }
    if (AP_HAL::millis() - _log_sending_link->get_last_heartbeat_time() > 3000) {
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
    mavlink_msg_log_entry_send(_log_sending_link->get_chan(),
                               _log_next_list_entry,
                               _log_num_logs,
                               _log_last_list_entry,
                               time_utc,
                               size);
    if (_log_next_list_entry == _log_last_list_entry) {
        transfer_activity = TransferActivity::IDLE;
        _log_sending_link = nullptr;
    } else {
        _log_next_list_entry++;
    }
}

/**
   trigger sending of log data if there are some pending
 */
bool AP_LoggerThread::handle_log_send_data()
{
    WITH_SEMAPHORE(_log_send_sem);

    if (!HAVE_PAYLOAD_SPACE(_log_sending_link->get_chan(), LOG_DATA)) {
        // no space
        return false;
    }
    if (AP_HAL::millis() - _log_sending_link->get_last_heartbeat_time() > 3000) {
        // give a heartbeat a chance
        return false;
    }

    int16_t nbytes = 0;
    uint32_t len = _log_data_remaining;
	mavlink_log_data_t packet;

    if (len > MAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN) {
        len = MAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN;
    }

    nbytes = get_log_data(_log_num_data, _log_data_page, _log_data_offset, len, packet.data);

    if (nbytes < 0) {
        // report as EOF on error
        nbytes = 0;
    }
    if (nbytes < MAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN) {
        memset(&packet.data[nbytes], 0, MAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN-nbytes);
    }

    packet.ofs = _log_data_offset;
    packet.id = _log_num_data;
    packet.count = nbytes;
    _mav_finalize_message_chan_send(_log_sending_link->get_chan(),
                                    MAVLINK_MSG_ID_LOG_DATA,
                                    (const char *)&packet,
                                    MAVLINK_MSG_ID_LOG_DATA_MIN_LEN,
                                    MAVLINK_MSG_ID_LOG_DATA_LEN,
                                    MAVLINK_MSG_ID_LOG_DATA_CRC);

    _log_data_offset += nbytes;
    _log_data_remaining -= nbytes;
    if (nbytes < MAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN || _log_data_remaining == 0) {
        end_log_transfer();
    }
    return true;
}

#endif  // HAL_LOGGING_ENABLED && HAL_GCS_ENABLED
