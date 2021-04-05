#pragma once

#include "MissionItemProtocol.h"

class MissionItemProtocol_Rally : public MissionItemProtocol {
public:
    MissionItemProtocol_Rally(class AP_Rally &_rally) :
        rally(_rally) {}
    void truncate(const mavlink_mission_count_t &packet) override;
    MAV_MISSION_TYPE mission_type() const override { return MAV_MISSION_TYPE_RALLY; }

    MAV_MISSION_RESULT complete(const GCS_MAVLINK &_link) override;
    void timeout() override;

    /*
      static function to get rally item as mavlink_mission_item_int_t
    */
    static bool get_item_as_mission_item(uint16_t seq, mavlink_mission_item_int_t &ret_packet);
    
protected:

    ap_message next_item_ap_message_id() const override {
        return MSG_NEXT_MISSION_REQUEST_RALLY;
    }
    bool clear_all_items() override WARN_IF_UNUSED;

private:
    AP_Rally &rally;

    uint16_t item_count() const override;
    uint16_t max_items() const override;

    MAV_MISSION_RESULT replace_item(const mavlink_mission_item_int_t&) override WARN_IF_UNUSED;
    MAV_MISSION_RESULT append_item(const mavlink_mission_item_int_t&) override WARN_IF_UNUSED;

    MAV_MISSION_RESULT get_item(const GCS_MAVLINK &_link,
                                const mavlink_message_t &msg,
                                const mavlink_mission_request_int_t &packet,
                                mavlink_mission_item_int_t &ret_packet) override WARN_IF_UNUSED;

    static MAV_MISSION_RESULT convert_MISSION_ITEM_INT_to_RallyLocation(const mavlink_mission_item_int_t &cmd, class RallyLocation &ret) WARN_IF_UNUSED;

};
