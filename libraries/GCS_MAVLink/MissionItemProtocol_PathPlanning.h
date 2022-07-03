#pragma once

#include "MissionItemProtocol.h"

class MissionItemProtocol_PathPlanning : public MissionItemProtocol {
public:
    MissionItemProtocol_PathPlanning() {};

    MAV_MISSION_TYPE mission_type() const override { return MAV_MISSION_TYPE_PATH_PLANNING; }

protected:

    ap_message next_item_ap_message_id() const override;
    bool clear_all_items() override { return false; } WARN_IF_UNUSED;

private:

    bool read_only() const override { return true; }

    void truncate(const mavlink_mission_count_t &packet) override {};

    MAV_MISSION_RESULT get_item(const GCS_MAVLINK &_link,
                                const mavlink_message_t &msg,
                                const mavlink_mission_request_int_t &packet,
                                mavlink_mission_item_int_t &ret_packet) override WARN_IF_UNUSED;

    bool get_item_as_mission_item(uint16_t seq, mavlink_mission_item_int_t &ret_packet);

    uint16_t item_count() const override;
    uint16_t max_items() const override;

    MAV_MISSION_RESULT replace_item(const mavlink_mission_item_int_t&) override WARN_IF_UNUSED;
    MAV_MISSION_RESULT append_item(const mavlink_mission_item_int_t&) override WARN_IF_UNUSED;

    MAV_MISSION_RESULT complete(const GCS_MAVLINK &_link) override;

};
