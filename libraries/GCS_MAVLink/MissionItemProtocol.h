#pragma once

#include "GCS_MAVLink.h"

#include "ap_message.h"

#include <stdint.h>

// MissionItemProtocol objects are used for transfering missions from
// a GCS to ArduPilot and vice-versa.
//
// There exists one MissionItemProtocol instance for each of the types
// of item that might be transfered - e.g. MissionItemProtocol_Rally
// for rally point uploads.  These objects are static in GCS_MAVLINK
// and used by all of the backends.
//
// While prompting the GCS for items required to complete the mission,
// a link is stored to the link the MissionItemProtocol should send
// requests out on, and the "receiving" boolean is true.  In this
// state downloading of items (and the item count!) is blocked.
// Starting of uploads (for the same protocol) is also blocked -
// essentially the GCS uploading a set of items (e.g. a mission) has a
// mutex over the mission.
class MissionItemProtocol
{
public:

    // note that all of these methods are named after the packet they
    // are handling; the "mission" part just comes as part of that.
    void handle_mission_request_list(const class GCS_MAVLINK &link,
                                     const mavlink_mission_request_list_t &packet,
                                     const mavlink_message_t &msg);
    void handle_mission_request_int(const GCS_MAVLINK &link,
                                    const mavlink_mission_request_int_t &packet,
                                    const mavlink_message_t &msg);
    void handle_mission_request(const GCS_MAVLINK &link,
                                const mavlink_mission_request_t &packet,
                                const mavlink_message_t &msg);

    void handle_mission_count(class GCS_MAVLINK &link,
                              const mavlink_mission_count_t &packet,
                              const mavlink_message_t &msg);
    void handle_mission_write_partial_list(GCS_MAVLINK &link,
                                           const mavlink_message_t &msg,
                                           const mavlink_mission_write_partial_list_t &packet);

    // called on receipt of a MISSION_ITEM or MISSION_ITEM_INT packet;
    // the former is converted to the latter.
    void handle_mission_item(const mavlink_message_t &msg,
                             const mavlink_mission_item_int_t &cmd);

    void handle_mission_clear_all(const GCS_MAVLINK &link,
                                  const mavlink_message_t &msg);

    void queued_request_send();
    void update();

    bool active_link_is(const GCS_MAVLINK *_link) const { return _link == link; };

    virtual MAV_MISSION_TYPE mission_type() const = 0;

    bool receiving; // currently sending requests and expecting items

protected:

    GCS_MAVLINK *link; // link currently receiving waypoints on

    // return the ap_message which can be queued to be sent to send a
    // item request to the GCS:
    virtual ap_message next_item_ap_message_id() const = 0;

    virtual bool clear_all_items() = 0;

    uint16_t        request_last; // last request index

private:

    virtual void truncate(const mavlink_mission_count_t &packet) = 0;

    uint16_t        request_i; // request index

    // waypoints
    uint8_t         dest_sysid;  // where to send requests
    uint8_t         dest_compid; // "
    uint32_t        timelast_receive_ms;
    uint32_t        timelast_request_ms;
    const uint16_t  upload_timeout_ms = 8000;

    // support for GCS getting waypoints etc from us:
    virtual MAV_MISSION_RESULT get_item(const GCS_MAVLINK &_link,
                                        const mavlink_message_t &msg,
                                        const mavlink_mission_request_int_t &packet,
                                        mavlink_mission_item_int_t &ret_packet) = 0;

    void init_send_requests(GCS_MAVLINK &_link,
                            const mavlink_message_t &msg,
                            const int16_t _request_first,
                            const int16_t _request_last);

    void send_mission_ack(const mavlink_message_t &msg, MAV_MISSION_RESULT result) const;
    void send_mission_ack(const GCS_MAVLINK &link, const mavlink_message_t &msg, MAV_MISSION_RESULT result) const;

    virtual uint16_t item_count() const = 0;
    virtual uint16_t max_items() const = 0;

    virtual MAV_MISSION_RESULT replace_item(const mavlink_mission_item_int_t &mission_item_int) = 0;
    virtual MAV_MISSION_RESULT append_item(const mavlink_mission_item_int_t &mission_item_int) = 0;

    virtual void complete(const GCS_MAVLINK &_link) {};
    virtual void timeout() {};
};
