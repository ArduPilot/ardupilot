#include "builtin_interfaces/msg/Time.h"
#include "sensor_msgs/msg/NavSatFix.h"
#include "tf2_msgs/msg/TFMessage.h"
#include "sensor_msgs/msg/BatteryState.h"
#include "geographic_msgs/msg/GeoPoseStamped.h"
#include "sensor_msgs/msg/Imu.h"

#include "uxr/client/client.h"

// Code generated table based on the enabled topics.
// Mavgen is using python, loops are not readable.
// Can use jinja to template (like Flask)

enum class TopicIndex: uint8_t {
    TIME_PUB,
    NAV_SAT_FIX_PUB,
    STATIC_TRANSFORMS_PUB,
    BATTERY_STATE_PUB,
    IMU_PUB,
    LOCAL_POSE_PUB,
    LOCAL_VELOCITY_PUB,
    GEOPOSE_PUB,
    CLOCK_PUB,
    GPS_GLOBAL_ORIGIN_PUB,
    JOY_SUB,
    DYNAMIC_TRANSFORMS_SUB,
    VELOCITY_CONTROL_SUB,
    GLOBAL_POSITION_SUB,
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
        .topic_rw = Topic_rw::DataWriter,
        .topic_name = "rt/ap/time",
        .type_name = "builtin_interfaces::msg::dds_::Time_",
        .qos = {
            .durability = UXR_DURABILITY_VOLATILE,
            .reliability = UXR_RELIABILITY_RELIABLE,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 20,
        },
    },
    {
        .topic_id = to_underlying(TopicIndex::NAV_SAT_FIX_PUB),
        .pub_id = to_underlying(TopicIndex::NAV_SAT_FIX_PUB),
        .sub_id = to_underlying(TopicIndex::NAV_SAT_FIX_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::NAV_SAT_FIX_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::NAV_SAT_FIX_PUB), .type=UXR_DATAREADER_ID},
        .topic_rw = Topic_rw::DataWriter,
        .topic_name = "rt/ap/navsat/navsat0",
        .type_name = "sensor_msgs::msg::dds_::NavSatFix_",
        .qos = {
            .durability = UXR_DURABILITY_VOLATILE,
            .reliability = UXR_RELIABILITY_BEST_EFFORT,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 5,
        },
    },
    {
        .topic_id = to_underlying(TopicIndex::STATIC_TRANSFORMS_PUB),
        .pub_id = to_underlying(TopicIndex::STATIC_TRANSFORMS_PUB),
        .sub_id = to_underlying(TopicIndex::STATIC_TRANSFORMS_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::STATIC_TRANSFORMS_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::STATIC_TRANSFORMS_PUB), .type=UXR_DATAREADER_ID},
        .topic_rw = Topic_rw::DataWriter,
        .topic_name = "rt/ap/tf_static",
        .type_name = "tf2_msgs::msg::dds_::TFMessage_",
        .qos = {
            .durability = UXR_DURABILITY_TRANSIENT_LOCAL,
            .reliability = UXR_RELIABILITY_RELIABLE,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 1,
        },
    },
    {
        .topic_id = to_underlying(TopicIndex::BATTERY_STATE_PUB),
        .pub_id = to_underlying(TopicIndex::BATTERY_STATE_PUB),
        .sub_id = to_underlying(TopicIndex::BATTERY_STATE_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::BATTERY_STATE_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::BATTERY_STATE_PUB), .type=UXR_DATAREADER_ID},
        .topic_rw = Topic_rw::DataWriter,
        .topic_name = "rt/ap/battery/battery0",
        .type_name = "sensor_msgs::msg::dds_::BatteryState_",
        .qos = {
            .durability = UXR_DURABILITY_VOLATILE,
            .reliability = UXR_RELIABILITY_BEST_EFFORT,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 5,
        },
    },
    {
        .topic_id = to_underlying(TopicIndex::IMU_PUB),
        .pub_id = to_underlying(TopicIndex::IMU_PUB),
        .sub_id = to_underlying(TopicIndex::IMU_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::IMU_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::IMU_PUB), .type=UXR_DATAREADER_ID},
        .topic_rw = Topic_rw::DataWriter,
        .topic_name = "rt/ap/imu/experimental/data",
        .type_name = "sensor_msgs::msg::dds_::Imu_",
        .qos = {
            .durability = UXR_DURABILITY_VOLATILE,
            .reliability = UXR_RELIABILITY_BEST_EFFORT,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 5,
        },
    },
    {
        .topic_id = to_underlying(TopicIndex::LOCAL_POSE_PUB),
        .pub_id = to_underlying(TopicIndex::LOCAL_POSE_PUB),
        .sub_id = to_underlying(TopicIndex::LOCAL_POSE_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::LOCAL_POSE_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::LOCAL_POSE_PUB), .type=UXR_DATAREADER_ID},
        .topic_rw = Topic_rw::DataWriter,
        .topic_name = "rt/ap/pose/filtered",
        .type_name = "geometry_msgs::msg::dds_::PoseStamped_",
        .qos = {
            .durability = UXR_DURABILITY_VOLATILE,
            .reliability = UXR_RELIABILITY_BEST_EFFORT,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 5,
        },
    },
    {
        .topic_id = to_underlying(TopicIndex::LOCAL_VELOCITY_PUB),
        .pub_id = to_underlying(TopicIndex::LOCAL_VELOCITY_PUB),
        .sub_id = to_underlying(TopicIndex::LOCAL_VELOCITY_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::LOCAL_VELOCITY_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::LOCAL_VELOCITY_PUB), .type=UXR_DATAREADER_ID},
        .topic_rw = Topic_rw::DataWriter,
        .topic_name = "rt/ap/twist/filtered",
        .type_name = "geometry_msgs::msg::dds_::TwistStamped_",
        .qos = {
            .durability = UXR_DURABILITY_VOLATILE,
            .reliability = UXR_RELIABILITY_BEST_EFFORT,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 5,
        },
    },
    {
        .topic_id = to_underlying(TopicIndex::GEOPOSE_PUB),
        .pub_id = to_underlying(TopicIndex::GEOPOSE_PUB),
        .sub_id = to_underlying(TopicIndex::GEOPOSE_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::GEOPOSE_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::GEOPOSE_PUB), .type=UXR_DATAREADER_ID},
        .topic_rw = Topic_rw::DataWriter,
        .topic_name = "rt/ap/geopose/filtered",
        .type_name = "geographic_msgs::msg::dds_::GeoPoseStamped_",
        .qos = {
            .durability = UXR_DURABILITY_VOLATILE,
            .reliability = UXR_RELIABILITY_BEST_EFFORT,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 5,
        },
    },
    {
        .topic_id = to_underlying(TopicIndex::CLOCK_PUB),
        .pub_id = to_underlying(TopicIndex::CLOCK_PUB),
        .sub_id = to_underlying(TopicIndex::CLOCK_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::CLOCK_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::CLOCK_PUB), .type=UXR_DATAREADER_ID},
        .topic_rw = Topic_rw::DataWriter,
        .topic_name = "rt/ap/clock",
        .type_name = "rosgraph_msgs::msg::dds_::Clock_",
        .qos = {
            .durability = UXR_DURABILITY_VOLATILE,
            .reliability = UXR_RELIABILITY_RELIABLE,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 20,
        },
    },
    {
        .topic_id = to_underlying(TopicIndex::GPS_GLOBAL_ORIGIN_PUB),
        .pub_id = to_underlying(TopicIndex::GPS_GLOBAL_ORIGIN_PUB),
        .sub_id = to_underlying(TopicIndex::GPS_GLOBAL_ORIGIN_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::GPS_GLOBAL_ORIGIN_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::GPS_GLOBAL_ORIGIN_PUB), .type=UXR_DATAREADER_ID},
        .topic_rw = Topic_rw::DataWriter,
        .topic_name = "rt/ap/gps_global_origin/filtered",
        .type_name = "geographic_msgs::msg::dds_::GeoPointStamped_",
        .qos = {
            .durability = UXR_DURABILITY_VOLATILE,
            .reliability = UXR_RELIABILITY_BEST_EFFORT,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 5,
        },
    },
    {
        .topic_id = to_underlying(TopicIndex::JOY_SUB),
        .pub_id = to_underlying(TopicIndex::JOY_SUB),
        .sub_id = to_underlying(TopicIndex::JOY_SUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::JOY_SUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::JOY_SUB), .type=UXR_DATAREADER_ID},
        .topic_rw = Topic_rw::DataReader,
        .topic_name = "rt/ap/joy",
        .type_name = "sensor_msgs::msg::dds_::Joy_",
        .qos = {
            .durability = UXR_DURABILITY_VOLATILE,
            .reliability = UXR_RELIABILITY_BEST_EFFORT,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 5,
        },
    },
    {
        .topic_id = to_underlying(TopicIndex::DYNAMIC_TRANSFORMS_SUB),
        .pub_id = to_underlying(TopicIndex::DYNAMIC_TRANSFORMS_SUB),
        .sub_id = to_underlying(TopicIndex::DYNAMIC_TRANSFORMS_SUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::DYNAMIC_TRANSFORMS_SUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::DYNAMIC_TRANSFORMS_SUB), .type=UXR_DATAREADER_ID},
        .topic_rw = Topic_rw::DataReader,
        .topic_name = "rt/ap/tf",
        .type_name = "tf2_msgs::msg::dds_::TFMessage_",
        .qos = {
            .durability = UXR_DURABILITY_VOLATILE,
            .reliability = UXR_RELIABILITY_BEST_EFFORT,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 5,
        },
    },
    {
        .topic_id = to_underlying(TopicIndex::VELOCITY_CONTROL_SUB),
        .pub_id = to_underlying(TopicIndex::VELOCITY_CONTROL_SUB),
        .sub_id = to_underlying(TopicIndex::VELOCITY_CONTROL_SUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::VELOCITY_CONTROL_SUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::VELOCITY_CONTROL_SUB), .type=UXR_DATAREADER_ID},
        .topic_rw = Topic_rw::DataReader,
        .topic_name = "rt/ap/cmd_vel",
        .type_name = "geometry_msgs::msg::dds_::TwistStamped_",
        .qos = {
            .durability = UXR_DURABILITY_VOLATILE,
            .reliability = UXR_RELIABILITY_BEST_EFFORT,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 5,
        },
    },
    {
        .topic_id = to_underlying(TopicIndex::GLOBAL_POSITION_SUB),
        .pub_id = to_underlying(TopicIndex::GLOBAL_POSITION_SUB),
        .sub_id = to_underlying(TopicIndex::GLOBAL_POSITION_SUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::GLOBAL_POSITION_SUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::GLOBAL_POSITION_SUB), .type=UXR_DATAREADER_ID},
        .topic_rw = Topic_rw::DataReader,
        .topic_name = "rt/ap/cmd_gps_pose",
        .type_name = "ardupilot_msgs::msg::dds_::GlobalPosition_",
        .qos = {
            .durability = UXR_DURABILITY_VOLATILE,
            .reliability = UXR_RELIABILITY_BEST_EFFORT,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 5,
        },
    },
};
