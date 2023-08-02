#include "builtin_interfaces/msg/Time.h"
#include "sensor_msgs/msg/NavSatFix.h"
#include "tf2_msgs/msg/TFMessage.h"
#include "sensor_msgs/msg/BatteryState.h"
#include "geographic_msgs/msg/GeoPoseStamped.h"

#include "uxr/client/client.h"

// Code generated table based on the enabled topics.
// Mavgen is using python, loops are not readable.
// Can use jinja to template (like Flask)

enum class TopicIndex: uint8_t {
    TIME_PUB,
    NAV_SAT_FIX_PUB,
    STATIC_TRANSFORMS_PUB,
    BATTERY_STATE_PUB,
    LOCAL_POSE_PUB,
    LOCAL_VELOCITY_PUB,
    GEOPOSE_PUB,
    CLOCK_PUB,
    JOY_SUB,
    DYNAMIC_TRANSFORMS_SUB,
    VELOCITY_CONTROL_SUB,
};

static inline constexpr uint8_t to_underlying(const TopicIndex index)
{
    static_assert(sizeof(index) == sizeof(uint8_t));
    return static_cast<uint8_t>(index);
}


constexpr struct AP_DDS_Client::Topic_table AP_DDS_Client::topics[] = {
    {
        .topic_id = to_underlying(TopicIndex::TIME_PUB),
        .pub_id = to_underlying(TopicIndex::TIME_PUB),
        .sub_id = to_underlying(TopicIndex::TIME_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::TIME_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::TIME_PUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "time__t",
        .dw_profile_label = "time__dw",
        .dr_profile_label = "",
    },
    {
        .topic_id = to_underlying(TopicIndex::NAV_SAT_FIX_PUB),
        .pub_id = to_underlying(TopicIndex::NAV_SAT_FIX_PUB),
        .sub_id = to_underlying(TopicIndex::NAV_SAT_FIX_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::NAV_SAT_FIX_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::NAV_SAT_FIX_PUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "navsatfix0__t",
        .dw_profile_label = "navsatfix0__dw",
        .dr_profile_label = "",
    },
    {
        .topic_id = to_underlying(TopicIndex::STATIC_TRANSFORMS_PUB),
        .pub_id = to_underlying(TopicIndex::STATIC_TRANSFORMS_PUB),
        .sub_id = to_underlying(TopicIndex::STATIC_TRANSFORMS_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::STATIC_TRANSFORMS_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::STATIC_TRANSFORMS_PUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "statictransforms__t",
        .dw_profile_label = "statictransforms__dw",
        .dr_profile_label = "",
    },
    {
        .topic_id = to_underlying(TopicIndex::BATTERY_STATE_PUB),
        .pub_id = to_underlying(TopicIndex::BATTERY_STATE_PUB),
        .sub_id = to_underlying(TopicIndex::BATTERY_STATE_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::BATTERY_STATE_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::BATTERY_STATE_PUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "batterystate0__t",
        .dw_profile_label = "batterystate0__dw",
        .dr_profile_label = "",
    },
    {
        .topic_id = to_underlying(TopicIndex::LOCAL_POSE_PUB),
        .pub_id = to_underlying(TopicIndex::LOCAL_POSE_PUB),
        .sub_id = to_underlying(TopicIndex::LOCAL_POSE_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::LOCAL_POSE_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::LOCAL_POSE_PUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "localpose__t",
        .dw_profile_label = "localpose__dw",
        .dr_profile_label = "",
    },
    {
        .topic_id = to_underlying(TopicIndex::LOCAL_VELOCITY_PUB),
        .pub_id = to_underlying(TopicIndex::LOCAL_VELOCITY_PUB),
        .sub_id = to_underlying(TopicIndex::LOCAL_VELOCITY_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::LOCAL_VELOCITY_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::LOCAL_VELOCITY_PUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "localvelocity__t",
        .dw_profile_label = "localvelocity__dw",
        .dr_profile_label = "",
    },
    {
        .topic_id = to_underlying(TopicIndex::GEOPOSE_PUB),
        .pub_id = to_underlying(TopicIndex::GEOPOSE_PUB),
        .sub_id = to_underlying(TopicIndex::GEOPOSE_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::GEOPOSE_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::GEOPOSE_PUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "geopose__t",
        .dw_profile_label = "geopose__dw",
        .dr_profile_label = "",
    },
    {
        .topic_id = to_underlying(TopicIndex::CLOCK_PUB),
        .pub_id = to_underlying(TopicIndex::CLOCK_PUB),
        .sub_id = to_underlying(TopicIndex::CLOCK_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::CLOCK_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::CLOCK_PUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "clock__t",
        .dw_profile_label = "clock__dw",
        .dr_profile_label = "",
    },
    {
        .topic_id = to_underlying(TopicIndex::JOY_SUB),
        .pub_id = to_underlying(TopicIndex::JOY_SUB),
        .sub_id = to_underlying(TopicIndex::JOY_SUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::JOY_SUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::JOY_SUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "joy__t",
        .dw_profile_label = "",
        .dr_profile_label = "joy__dr",
    },
    {
        .topic_id = to_underlying(TopicIndex::DYNAMIC_TRANSFORMS_SUB),
        .pub_id = to_underlying(TopicIndex::DYNAMIC_TRANSFORMS_SUB),
        .sub_id = to_underlying(TopicIndex::DYNAMIC_TRANSFORMS_SUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::DYNAMIC_TRANSFORMS_SUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::DYNAMIC_TRANSFORMS_SUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "dynamictf__t",
        .dw_profile_label = "",
        .dr_profile_label = "dynamictf__dr",
    },
    {
        .topic_id = to_underlying(TopicIndex::VELOCITY_CONTROL_SUB),
        .pub_id = to_underlying(TopicIndex::VELOCITY_CONTROL_SUB),
        .sub_id = to_underlying(TopicIndex::VELOCITY_CONTROL_SUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::VELOCITY_CONTROL_SUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::VELOCITY_CONTROL_SUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "velocitycontrol__t",
        .dw_profile_label = "",
        .dr_profile_label = "velocitycontrol__dr",
    },
};
