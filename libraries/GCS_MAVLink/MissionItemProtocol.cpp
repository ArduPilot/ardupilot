#include "GCS_config.h"

#if HAL_GCS_ENABLED

#include "MissionItemProtocol.h"

#include "GCS.h"

void MissionItemProtocol::init_send_requests(GCS_MAVLINK &_link,
                                             const mavlink_message_t &msg,
                                             const int16_t _request_first,
                                             const int16_t _request_last)
{
    // set variables to help handle the expected receiving of commands from the GCS
    timelast_receive_ms = AP_HAL::millis();    // set time we last received commands to now
    receiving = true;              // record that we expect to receive commands
    request_i = _request_first;                 // reset the next expected command number to zero
    request_last = _request_last;         // record how many commands we expect to receive

    dest_sysid = msg.sysid;       // record system id of GCS who wants to upload the mission
    dest_compid = msg.compid;     // record component id of GCS who wants to upload the mission

    link = &_link;

    timelast_request_ms = AP_HAL::millis();
    link->send_message(next_item_ap_message_id());

    mission_item_warning_sent = false;
    mission_request_warning_sent = false;
}

void MissionItemProtocol::handle_mission_clear_all(GCS_MAVLINK &_link,
                                                   const mavlink_message_t &msg)
{
    if (!cancel_upload(_link, msg)) {
        send_mission_ack(_link, msg, MAV_MISSION_ERROR);
        return;
    }
    if (!clear_all_items()) {
        send_mission_ack(_link, msg, MAV_MISSION_ERROR);
        return;
    }
    link = &_link;
    receiving = true;
    dest_sysid = msg.sysid;
    dest_compid = msg.compid;
    timelast_receive_ms = AP_HAL::millis();
    transfer_is_complete(_link, msg);
}

bool MissionItemProtocol::mavlink2_requirement_met(const GCS_MAVLINK &_link, const mavlink_message_t &msg) const
{
    // need mavlink2 to do mission types other than mission:
    if (mission_type() == MAV_MISSION_TYPE_MISSION) {
        return true;
    }
    if (!_link.sending_mavlink1()) {
        return true;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Need mavlink2 for item transfer");
    send_mission_ack(_link, msg, MAV_MISSION_UNSUPPORTED);
    return false;
}

// returns true if we are either not receiving, or we successfully
// cancelled an existing upload:
bool MissionItemProtocol::cancel_upload(const GCS_MAVLINK &_link, const mavlink_message_t &msg)
{
    if (receiving) {
        // someone is already uploading a mission.  If we are
        // receiving from someone then we will allow them to restart -
        // otherwise we deny.
        if (msg.sysid != dest_sysid || msg.compid != dest_compid) {
            // reject another upload until
            send_mission_ack(_link, msg, MAV_MISSION_DENIED);
            return false;
        }
        // the upload count may have changed; free resources and
        // allocate them again:
        free_upload_resources();
        receiving = false;
        link = nullptr;
    }

    return true;
}

void MissionItemProtocol::handle_mission_count(
    GCS_MAVLINK &_link,
    const mavlink_mission_count_t &packet,
    const mavlink_message_t &msg)
{
    if (!mavlink2_requirement_met(_link, msg)) {
        return;
    }

    if (!cancel_upload(_link, msg)) {
        return;
    }

    if (packet.count > max_items()) {
        // FIXME: different items take up different storage space!
        send_mission_ack(_link, msg, MAV_MISSION_NO_SPACE);
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Only %u items are supported", (unsigned)max_items());
        return;
    }

    MAV_MISSION_RESULT ret_alloc = allocate_receive_resources(packet.count);
    if (ret_alloc != MAV_MISSION_ACCEPTED) {
        send_mission_ack(_link, msg, ret_alloc);
        return;
    }

    truncate(packet);

    if (packet.count == 0) {
        // no requests to send...
        link = &_link;
        receiving = true;
        dest_sysid = msg.sysid;
        dest_compid = msg.compid;
        timelast_receive_ms = AP_HAL::millis();
        transfer_is_complete(_link, msg);
        return;
    }

    // start waypoint receiving
    init_send_requests(_link, msg, 0, packet.count-1);
}

void MissionItemProtocol::handle_mission_request_list(
    const GCS_MAVLINK &_link,
    const mavlink_mission_request_list_t &packet,
    const mavlink_message_t &msg)
{
    if (!mavlink2_requirement_met(_link, msg)) {
        return;
    }

    if (receiving) {
        // someone is uploading a mission; reject fetching of points
        // until done or timeout
        send_mission_ack(_link, msg, MAV_MISSION_DENIED);
        return;
    }

    // reply with number of commands in the mission.  The GCS will
    // then request each command separately
    CHECK_PAYLOAD_SIZE2_VOID(_link.get_chan(), MISSION_COUNT);
    uint32_t _opaque_id = 0;;
#if AP_MAVLINK_MISSION_OPAQUE_ID_ENABLED
    if (!opaque_id(_opaque_id)) {
        // we should probably defer!
    }
#endif  // AP_MAVLINK_MISSION_OPAQUE_ID_ENABLED
    mavlink_msg_mission_count_send(_link.get_chan(),
                                   msg.sysid,
                                   msg.compid,
                                   item_count(),
                                   mission_type(),
                                   _opaque_id
        );
}

void MissionItemProtocol::handle_mission_request_int(GCS_MAVLINK &_link,
                                                     const mavlink_mission_request_int_t &packet,
                                                     const mavlink_message_t &msg)
{
    if (!mavlink2_requirement_met(_link, msg)) {
        return;
    }

    if (receiving) {
        // someone is uploading a mission; reject fetching of points
        // until done or timeout
        send_mission_ack(_link, msg, MAV_MISSION_DENIED);
        return;
    }

    mavlink_mission_item_int_t ret_packet;
    const MAV_MISSION_RESULT result_code = get_item(packet.seq, ret_packet);
    if (result_code != MAV_MISSION_ACCEPTED) {
        if (result_code == MAV_MISSION_INVALID_SEQUENCE) {
            // try to educate the GCS on the actual size of the mission:
            const mavlink_channel_t chan = _link.get_chan();
            if (HAVE_PAYLOAD_SPACE(chan, MISSION_COUNT)) {
                uint32_t _opaque_id = 0;
#if AP_MAVLINK_MISSION_OPAQUE_ID_ENABLED
                if (!opaque_id(_opaque_id)) {
                    // we should probably defer!
                }
#endif  // AP_MAVLINK_MISSION_OPAQUE_ID_ENABLED
                mavlink_msg_mission_count_send(chan,
                                               msg.sysid,
                                               msg.compid,
                                               item_count(),
                                               mission_type(),
                                               _opaque_id);
            }
        }
        // send failure message
        send_mission_ack(_link, msg, result_code);
        return;
    }

    ret_packet.target_system = msg.sysid;
    ret_packet.target_component = msg.compid;

    _link.send_message(MAVLINK_MSG_ID_MISSION_ITEM_INT, (const char*)&ret_packet);
}

#if AP_MAVLINK_MSG_MISSION_REQUEST_ENABLED
void MissionItemProtocol::handle_mission_request(GCS_MAVLINK &_link,
                                                 const mavlink_mission_request_t &packet,
                                                 const mavlink_message_t &msg
)
{
    if (!mavlink2_requirement_met(_link, msg)) {
        return;
    }

    mavlink_mission_item_int_t item_int;
    MAV_MISSION_RESULT ret = get_item(packet.seq, item_int);
    if (ret != MAV_MISSION_ACCEPTED) {
        send_mission_ack(_link, msg, ret);
        return;
    }

    item_int.target_system = msg.sysid;
    item_int.target_component = msg.compid;

    mavlink_mission_item_t ret_packet{};
    ret = AP_Mission::convert_MISSION_ITEM_INT_to_MISSION_ITEM(item_int, ret_packet);
    if (ret != MAV_MISSION_ACCEPTED) {
        send_mission_ack(_link, msg, ret);
        return;
    }

    if (!mission_request_warning_sent) {
        mission_request_warning_sent = true;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "got MISSION_REQUEST; use MISSION_REQUEST_INT!");
    }

    // buffer space is checked by send_message
    _link.send_message(MAVLINK_MSG_ID_MISSION_ITEM, (const char*)&ret_packet);
}
#endif  // AP_MAVLINK_MSG_MISSION_REQUEST_ENABLED

void MissionItemProtocol::send_mission_item_warning()
{
    if (mission_item_warning_sent) {
        return;
    }
    mission_item_warning_sent = true;
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "got MISSION_ITEM; GCS should send MISSION_ITEM_INT");
}

void MissionItemProtocol::handle_mission_write_partial_list(GCS_MAVLINK &_link,
                                                            const mavlink_message_t &msg,
                                                            const mavlink_mission_write_partial_list_t &packet)
{
    if (!mavlink2_requirement_met(_link, msg)) {
        return;
    }

    if (receiving) {
        // someone is already uploading a mission.  Deny ability to
        // write a partial list here as they might be trying to
        // overwrite a subset of the waypoints which the current
        // transfer is uploading, and that may lead to storing a whole
        // bunch of empty items.
        send_mission_ack(_link, msg, MAV_MISSION_DENIED);
        return;
    }

    // start waypoint receiving
    if ((unsigned)packet.start_index > item_count() ||
        (unsigned)packet.end_index > item_count() ||
        packet.end_index < packet.start_index) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"Flight plan update rejected"); // FIXME: Remove this anytime after 2020-01-22
        send_mission_ack(_link, msg, MAV_MISSION_ERROR);
        return;
    }

    MAV_MISSION_RESULT ret_alloc = allocate_update_resources();
    if (ret_alloc != MAV_MISSION_ACCEPTED) {
        send_mission_ack(_link, msg, ret_alloc);
        return;
    }

    init_send_requests(_link, msg, packet.start_index, packet.end_index);
}

void MissionItemProtocol::handle_mission_item(const mavlink_message_t &msg, const mavlink_mission_item_int_t &cmd)
{
    if (link == nullptr) {
        INTERNAL_ERROR(AP_InternalError::error_t::gcs_bad_missionprotocol_link);
        return;
    }

    // check if this is the requested waypoint
    if (cmd.seq != request_i) {
        send_mission_ack(msg, MAV_MISSION_INVALID_SEQUENCE);
        return;
    }
    // make sure the item is coming from the system that initiated the upload
    if (msg.sysid != dest_sysid) {
        send_mission_ack(msg, MAV_MISSION_DENIED);
        return;
    }
    if (msg.compid != dest_compid) {
        send_mission_ack(msg, MAV_MISSION_DENIED);
        return;
    }

    const uint16_t _item_count = item_count();

    MAV_MISSION_RESULT result;
    if (cmd.seq < _item_count) {
        // command index is within the existing list, replace the command
        result = replace_item(cmd);
    } else if (cmd.seq == _item_count) {
        // command is at the end of command list, add the command
        result = append_item(cmd);
    } else {
        // beyond the end of the command list, return an error
        result = MAV_MISSION_ERROR;
    }
    if (result != MAV_MISSION_ACCEPTED) {
        send_mission_ack(msg, result);
        receiving = false;
        link = nullptr;
        free_upload_resources();
        return;
    }

    // update waypoint receiving state machine
    timelast_receive_ms = AP_HAL::millis();
    request_i++;

    if (request_i > request_last) {
        transfer_is_complete(*link, msg);
        return;
    }
    // if we have enough space, then send the next WP request immediately
    if (HAVE_PAYLOAD_SPACE(link->get_chan(), MISSION_REQUEST)) {
        queued_request_send();
    } else {
        link->send_message(next_item_ap_message_id());
    }
}

void MissionItemProtocol::transfer_is_complete(const GCS_MAVLINK &_link, const mavlink_message_t &msg)
{
    const MAV_MISSION_RESULT result = complete(_link);
    free_upload_resources();
    if (!receiving) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
#if AP_MAVLINK_MISSION_OPAQUE_ID_ENABLED
    uint32_t _opaque_id;
    if (!opaque_id(_opaque_id)) {
        // the opaque ID can't currently be calculated; we definitely
        // want to have it in the mission ack to avoid race
        // conditions.  Defer sending the mission ack until it is
        // available:
        deferred_mission_ack.ready = false;
        deferred_mission_ack.link = &_link;
        deferred_mission_ack.sysid = msg.sysid;
        deferred_mission_ack.compid = msg.compid;
        deferred_mission_ack.opaque_id = 0;
        return;
    }
#endif  // AP_MAVLINK_MISSION_OPAQUE_ID_ENABLED
    send_mission_ack(_link, msg, result);
    receiving = false;
    link = nullptr;
}

void MissionItemProtocol::send_mission_ack(const mavlink_message_t &msg,
                                           MAV_MISSION_RESULT result) const
{
    if (link == nullptr) {
        INTERNAL_ERROR(AP_InternalError::error_t::gcs_bad_missionprotocol_link);
        return;
    }
    send_mission_ack(*link, msg, result);
}
void MissionItemProtocol::send_mission_ack(const GCS_MAVLINK &_link,
                                           const mavlink_message_t &msg,
                                           MAV_MISSION_RESULT result) const
{
    send_mission_ack(_link, msg.sysid, msg.compid, result);
}
void MissionItemProtocol::send_mission_ack(const GCS_MAVLINK &_link,
                                           uint8_t sysid,
                                           uint8_t compid,
                                           MAV_MISSION_RESULT result) const
{
    CHECK_PAYLOAD_SIZE2_VOID(_link.get_chan(), MISSION_ACK);
    uint32_t _opaque_id = 0;
#if AP_MAVLINK_MISSION_OPAQUE_ID_ENABLED
    if (!opaque_id(_opaque_id)) {
    }
#endif  // AP_MAVLINK_MISSION_OPAQUE_ID_ENABLED
    mavlink_msg_mission_ack_send(_link.get_chan(),
                                 sysid,
                                 compid,
                                 result,
                                 mission_type(),
                                 _opaque_id);
}

/**
 * @brief Send the next pending waypoint, called from deferred message
 * handling code
 */
void MissionItemProtocol::queued_request_send()
{
    if (!receiving) {
        return;
    }
    if (request_i > request_last) {
        return;
    }
    if (link == nullptr) {
        INTERNAL_ERROR(AP_InternalError::error_t::gcs_bad_missionprotocol_link);
        return;
    }
    CHECK_PAYLOAD_SIZE2_VOID(link->get_chan(), MISSION_REQUEST);
    mavlink_msg_mission_request_send(
        link->get_chan(),
        dest_sysid,
        dest_compid,
        request_i,
        mission_type());
    timelast_request_ms = AP_HAL::millis();
}

void MissionItemProtocol::update()
{
#if AP_MAVLINK_MISSION_OPAQUE_ID_ENABLED
    update_checksum();
#endif  // AP_MAVLINK_MISSION_OPAQUE_ID_ENABLED

    if (!receiving) {
        // we don't need to do anything unless we're sending requests
        return;
    }
    if (link == nullptr) {
        INTERNAL_ERROR(AP_InternalError::error_t::gcs_bad_missionprotocol_link);
        return;
    }

    const mavlink_channel_t chan = link->get_chan();
    // stop waypoint receiving if timeout
    const uint32_t tnow = AP_HAL::millis();
    if (tnow - timelast_receive_ms > upload_timeout_ms) {
        receiving = false;
        timeout();
        if (HAVE_PAYLOAD_SPACE(chan, MISSION_ACK)) {
            uint32_t _opaque_id = 0;
#if AP_MAVLINK_MISSION_OPAQUE_ID_ENABLED
            if (!opaque_id(_opaque_id)) {
                // ...
            }
#endif  // AP_MAVLINK_MISSION_OPAQUE_ID_ENABLED
            mavlink_msg_mission_ack_send(chan,
                                         dest_sysid,
                                         dest_compid,
                                         MAV_MISSION_OPERATION_CANCELLED,
                                         mission_type(),
                                         _opaque_id);
        }
        link = nullptr;
        free_upload_resources();
        return;
    }
    // resend request if we haven't gotten one:
    const uint32_t wp_recv_timeout_ms = 1000U + link->get_stream_slowdown_ms();
    if (tnow - timelast_request_ms > wp_recv_timeout_ms) {
        timelast_request_ms = tnow;
        link->send_message(next_item_ap_message_id());
    }

    // send any deferred transfer acceptance (to allow for
    // asynchronous opaque-id calculation)
    if (HAVE_PAYLOAD_SPACE(chan, MISSION_ACK) &&
        deferred_mission_ack.link != nullptr &&
        deferred_mission_ack.ready) {
        send_mission_ack(*deferred_mission_ack.link,
                         deferred_mission_ack.sysid,
                         deferred_mission_ack.compid,
                         MAV_MISSION_ACCEPTED);
        deferred_mission_ack.link = nullptr;
        receiving = false;
        link = nullptr;
    }
}

#if AP_MAVLINK_MISSION_OPAQUE_ID_ENABLED
// returns a unique ID for this mission
bool MissionItemProtocol::opaque_id(uint32_t &checksum) const
{
    switch (checksum_state.state) {
    case ChecksumState::READY:
        checksum = checksum_state.checksum;
        // can't use zero as the field is an extension field in mavlink2:
        if (checksum == 0) {
            checksum = UINT32_MAX;
        }
        return last_items_change_time_ms() == checksum_state.items_change_time_ms;
    case ChecksumState::CALCULATING:
    case ChecksumState::ERROR:
        return false;
    }
    return false;
}

void MissionItemProtocol::update_checksum()
{
    const uint32_t items_last_change_time_ms = last_items_change_time_ms();

    if (items_last_change_time_ms == checksum_state.last_calculate_time_ms) {
        return;
    }

    // decide whether we need to start calculating the checksum from
    // the start; we may be partially through the calculation and need
    // to start again
    bool do_initialisation = false;
    switch (checksum_state.state) {
    case ChecksumState::READY:
        do_initialisation = true;
        checksum_state.state = ChecksumState::CALCULATING;
        // FALLTHROUGH
    case ChecksumState::CALCULATING:
        if (checksum_state.items_change_time_ms != items_last_change_time_ms) {
            // mission changed part-way through our calculations
            do_initialisation = true;
        }
        break;
    case ChecksumState::ERROR:
        do_initialisation = true;
        break;
    }

    if (do_initialisation) {
        checksum_state.checksum = 0;
        checksum_state.current_item = opaque_id_first_item();
        checksum_state.count = item_count();
        checksum_state.items_change_time_ms = last_items_change_time_ms();
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_items_change_time_ms() < 500) {
        // don't start to calculate unless the mission's been
        // unchanged for a while.
        return;
    }

    // AP: Took 2.178000ms to checksum 373 points (5.839142ms/1000 points
    for (uint16_t count = 0;
         count<16 && checksum_state.current_item<checksum_state.count;
         count++, checksum_state.current_item++) {
        mavlink_mission_item_int_t ret_packet;
        const MAV_MISSION_RESULT result_code = get_item(checksum_state.current_item, ret_packet);
        if (result_code != MAV_MISSION_ACCEPTED) {
            checksum_state.state = ChecksumState::ERROR;
            return;
        }
#define ADD_TO_CHECKSUM(field) checksum_state.checksum = crc_crc32(checksum_state.checksum, (uint8_t*)&ret_packet.field, sizeof(ret_packet.field));
        ADD_TO_CHECKSUM(frame);
        ADD_TO_CHECKSUM(command);
        ADD_TO_CHECKSUM(autocontinue);
        ADD_TO_CHECKSUM(param1);
        ADD_TO_CHECKSUM(param2);
        ADD_TO_CHECKSUM(param3);
        ADD_TO_CHECKSUM(param4);
        ADD_TO_CHECKSUM(x);
        ADD_TO_CHECKSUM(y);
        ADD_TO_CHECKSUM(z);
#undef ADD_TO_CHECKSUM
    }

    if (checksum_state.current_item < checksum_state.count) {
        return;
    }

    checksum_state.state = ChecksumState::READY;
    checksum_state.last_calculate_time_ms = items_last_change_time_ms;

    // poke the parent class to send the deferred ack, if any:
    deferred_mission_ack.opaque_id = checksum_state.checksum;
    deferred_mission_ack.ready = true;
}
#endif  // AP_MAVLINK_MISSION_OPAQUE_ID_ENABLED

#endif  // HAL_GCS_ENABLED
