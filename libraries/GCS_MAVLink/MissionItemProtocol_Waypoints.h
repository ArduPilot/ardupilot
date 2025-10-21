#pragma once

#include "MissionItemProtocol.h"

class MissionItemProtocol_Waypoints : public MissionItemProtocol {
public:
    MissionItemProtocol_Waypoints(class AP_Mission &_mission) :
        mission(_mission) {}

    // mission_type returns the MAV_MISSION mavlink enumeration value
    // which this module is responsible for handling
    MAV_MISSION_TYPE mission_type() const override {
        return MAV_MISSION_TYPE_MISSION;
    }

    // complete() is called by the base class after all waypoints have
    // been received.  _link is the link which the last item was
    // transfered on.
    MAV_MISSION_RESULT complete(const GCS_MAVLINK &_link) override;
    // timeout() is called by the base class in the case that the GCS
    // does not transfer all waypoints to the vehicle.
    void timeout() override;
    // truncate() is called to set the absolute number of items.  It
    // must be less than or equal to the current number of items (you
    // can't truncate-to a longer list)
    void truncate(const mavlink_mission_count_t &packet) override;

protected:

    // clear_all_items() is called to clear all items on the vehicle
    bool clear_all_items() override WARN_IF_UNUSED;

    // next_item_ap_message_id returns an item from the ap_message
    // enumeration which (when acted upon by the GCS class) will send
    // a mavlink message to the GCS requesting it upload the next
    // required waypoint.
    ap_message next_item_ap_message_id() const override {
        return MSG_NEXT_MISSION_REQUEST_WAYPOINTS;
    }

private:
    AP_Mission &mission;

    // append_item() is called by the base class to add the supplied
    // item to the end of the list of stored items.
    MAV_MISSION_RESULT append_item(const mavlink_mission_item_int_t &) override WARN_IF_UNUSED;

    // get_item() fills in ret_packet based on packet; _link is the
    // link the request was received on, and msg is the undecoded
    // request.  Note that msg may not actually decode to a
    // request_int_t!
    MAV_MISSION_RESULT get_item(const GCS_MAVLINK &_link,
                                const mavlink_message_t &msg,
                                const mavlink_mission_request_int_t &packet,
                                mavlink_mission_item_int_t &ret_packet) override WARN_IF_UNUSED;

    // item_count() returns the number of stored items
    uint16_t item_count() const override;

    // item_count() returns the maximum number of items which could be
    // stored on-board
    uint16_t max_items() const override;

    // replace_item() replaces an item in the stored list
    MAV_MISSION_RESULT replace_item(const mavlink_mission_item_int_t &) override WARN_IF_UNUSED;

};

