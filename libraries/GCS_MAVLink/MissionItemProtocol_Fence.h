#pragma once

#include "MissionItemProtocol.h"

class AC_PolyFence_loader;

class MissionItemProtocol_Fence : public MissionItemProtocol {
public:
    MissionItemProtocol_Fence(class AC_Fence &fence) :
        _fence(fence) {}

    MAV_MISSION_TYPE mission_type() const override {
        return MAV_MISSION_TYPE_FENCE;
    }

    void truncate(const mavlink_mission_count_t &packet) override;
    MAV_MISSION_RESULT complete(const GCS_MAVLINK &_link) override;
    void timeout() override;

protected:

    ap_message next_item_ap_message_id() const override {
        return MSG_NEXT_MISSION_REQUEST_FENCE;
    }
    bool clear_all_items() override WARN_IF_UNUSED;

private:
    class AC_Fence &_fence;

    uint16_t item_count() const override;
    uint16_t max_items() const override;

    MAV_MISSION_RESULT replace_item(const mavlink_mission_item_int_t&) override WARN_IF_UNUSED;
    MAV_MISSION_RESULT append_item(const mavlink_mission_item_int_t&) override WARN_IF_UNUSED;

    MAV_MISSION_RESULT get_item(const GCS_MAVLINK &_link,
                                const mavlink_message_t &msg,
                                const mavlink_mission_request_int_t &packet,
                                mavlink_mission_item_int_t &ret_packet) override WARN_IF_UNUSED;

    void free_upload_resources() override;
    MAV_MISSION_RESULT allocate_receive_resources(const uint16_t count) override WARN_IF_UNUSED;
    MAV_MISSION_RESULT allocate_update_resources() override WARN_IF_UNUSED;

    class AC_PolyFenceItem *_new_items;
    uint16_t _new_items_count;
    uint8_t *_updated_mask;
};
