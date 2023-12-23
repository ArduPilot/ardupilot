#include "uxr/client/client.h"

enum class ServiceIndex: uint8_t {
    ARMING_MOTORS,
    MODE_SWITCH
};

static inline constexpr uint8_t to_underlying(const ServiceIndex index)
{
    static_assert(sizeof(index) == sizeof(uint8_t));
    return static_cast<uint8_t>(index);
}

constexpr struct AP_DDS_Client::Service_table AP_DDS_Client::services[] = {
    {
        .req_id = to_underlying(ServiceIndex::ARMING_MOTORS),
        .rep_id = to_underlying(ServiceIndex::ARMING_MOTORS),
        .req_profile_label = "",
        .rep_profile_label = "arm_motors__replier",
    },
    {
        .req_id = to_underlying(ServiceIndex::MODE_SWITCH),
        .rep_id = to_underlying(ServiceIndex::MODE_SWITCH),
        .req_profile_label = "",
        .rep_profile_label = "mode_switch__replier",
    },
};
