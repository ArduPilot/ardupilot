#include "uxr/client/client.h"

enum class ServiceIndex: uint8_t {
    ARMING_MOTORS
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
        .srv_profile_label = "ArmMotorsService",
        .req_profile_label = "",
        .rep_profile_label = "ArmMotors_Replier",
    }
};
