#include "builtin_interfaces/msg/Time.h"
#include "sensor_msgs/msg/NavSatFix.h"
#include "tf2_msgs/msg/TFMessage.h"
#include "sensor_msgs/msg/BatteryState.h"
#include "geographic_msgs/msg/GeoPoseStamped.h"
#include "geometry_msgs/msg/Vector3Stamped.h"
#if AP_DDS_IMU_PUB_ENABLED
#include "sensor_msgs/msg/Imu.h"
#endif //AP_DDS_IMU_PUB_ENABLED

#include "uxr/client/client.h"

// Code generated table based on the enabled topics.
// Mavgen is using python, loops are not readable.
// Can use jinja to template (like Flask)

enum class TopicIndex: uint8_t {
#if AP_DDS_TIME_PUB_ENABLED
    TIME_PUB,
#endif // AP_DDS_TIME_PUB_ENABLED
#if AP_DDS_NAVSATFIX_PUB_ENABLED
    NAV_SAT_FIX_PUB,
#endif // AP_DDS_NAVSATFIX_PUB_ENABLED
#if AP_DDS_STATIC_TF_PUB_ENABLED
    STATIC_TRANSFORMS_PUB,
#endif // AP_DDS_STATIC_TF_PUB_ENABLED
#if AP_DDS_BATTERY_STATE_PUB_ENABLED
    BATTERY_STATE_PUB,
#endif // AP_DDS_BATTERY_STATE_PUB_ENABLED
#if AP_DDS_IMU_PUB_ENABLED
    IMU_PUB,
#endif //AP_DDS_IMU_PUB_ENABLED
#if AP_DDS_LOCAL_POSE_PUB_ENABLED
    LOCAL_POSE_PUB,
#endif // AP_DDS_LOCAL_POSE_PUB_ENABLED
#if AP_DDS_LOCAL_VEL_PUB_ENABLED
    LOCAL_VELOCITY_PUB,
#endif // AP_DDS_LOCAL_VEL_PUB_ENABLED
#if AP_DDS_AIRSPEED_PUB_ENABLED
    LOCAL_AIRSPEED_PUB,
#endif // AP_DDS_AIRSPEED_PUB_ENABLED
#if AP_DDS_RC_PUB_ENABLED
    LOCAL_RC_PUB,
#endif // AP_DDS_RC_PUB_ENABLED
#if AP_DDS_GEOPOSE_PUB_ENABLED
    GEOPOSE_PUB,
#endif // AP_DDS_GEOPOSE_PUB_ENABLED
#if AP_DDS_GOAL_PUB_ENABLED
    GOAL_PUB,
#endif // AP_DDS_GOAL_PUB_ENABLED
#if AP_DDS_CLOCK_PUB_ENABLED
    CLOCK_PUB,
#endif // AP_DDS_CLOCK_PUB_ENABLED
#if AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
    GPS_GLOBAL_ORIGIN_PUB,
#endif // AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
#if AP_DDS_STATUS_PUB_ENABLED
    STATUS_PUB,
#endif // AP_DDS_STATUS_PUB_ENABLED
#if AP_DDS_JOY_SUB_ENABLED
    JOY_SUB,
#endif // AP_DDS_JOY_SUB_ENABLED
#if AP_DDS_DYNAMIC_TF_SUB_ENABLED
    DYNAMIC_TRANSFORMS_SUB,
#endif // AP_DDS_DYNAMIC_TF_SUB_ENABLED
#if AP_DDS_VEL_CTRL_ENABLED
    VELOCITY_CONTROL_SUB,
#endif // AP_DDS_VEL_CTRL_ENABLED
#if AP_DDS_GLOBAL_POS_CTRL_ENABLED
    GLOBAL_POSITION_SUB,
#endif // AP_DDS_GLOBAL_POS_CTRL_ENABLED
};

static inline constexpr uint8_t to_underlying(const TopicIndex index)
{
    static_assert(sizeof(index) == sizeof(uint8_t));
    return static_cast<uint8_t>(index);
}


constexpr struct AP_DDS_Client::Topic_table AP_DDS_Client::topics[] = {
#if AP_DDS_TIME_PUB_ENABLED
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
#endif // AP_DDS_TIME_PUB_ENABLED
#if AP_DDS_NAVSATFIX_PUB_ENABLED
    {
        .topic_id = to_underlying(TopicIndex::NAV_SAT_FIX_PUB),
        .pub_id = to_underlying(TopicIndex::NAV_SAT_FIX_PUB),
        .sub_id = to_underlying(TopicIndex::NAV_SAT_FIX_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::NAV_SAT_FIX_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::NAV_SAT_FIX_PUB), .type=UXR_DATAREADER_ID},
        .topic_rw = Topic_rw::DataWriter,
        .topic_name = "rt/ap/navsat",
        .type_name = "sensor_msgs::msg::dds_::NavSatFix_",
        .qos = {
            .durability = UXR_DURABILITY_VOLATILE,
            .reliability = UXR_RELIABILITY_BEST_EFFORT,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 5,
        },
    },
#endif // AP_DDS_NAVSATFIX_PUB_ENABLED
#if AP_DDS_STATIC_TF_PUB_ENABLED
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
#endif // AP_DDS_STATIC_TF_PUB_ENABLED
#if AP_DDS_BATTERY_STATE_PUB_ENABLED
    {
        .topic_id = to_underlying(TopicIndex::BATTERY_STATE_PUB),
        .pub_id = to_underlying(TopicIndex::BATTERY_STATE_PUB),
        .sub_id = to_underlying(TopicIndex::BATTERY_STATE_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::BATTERY_STATE_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::BATTERY_STATE_PUB), .type=UXR_DATAREADER_ID},
        .topic_rw = Topic_rw::DataWriter,
        .topic_name = "rt/ap/battery",
        .type_name = "sensor_msgs::msg::dds_::BatteryState_",
        .qos = {
            .durability = UXR_DURABILITY_VOLATILE,
            .reliability = UXR_RELIABILITY_BEST_EFFORT,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 5,
        },
    },
#endif // AP_DDS_BATTERY_STATE_PUB_ENABLED
#if AP_DDS_IMU_PUB_ENABLED
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
#endif //AP_DDS_IMU_PUB_ENABLED
#if AP_DDS_LOCAL_POSE_PUB_ENABLED
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
#endif // AP_DDS_LOCAL_POSE_PUB_ENABLED
#if AP_DDS_LOCAL_VEL_PUB_ENABLED
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
#endif // AP_DDS_LOCAL_VEL_PUB_ENABLED
#if AP_DDS_AIRSPEED_PUB_ENABLED
    {
        .topic_id = to_underlying(TopicIndex::LOCAL_AIRSPEED_PUB),
        .pub_id = to_underlying(TopicIndex::LOCAL_AIRSPEED_PUB),
        .sub_id = to_underlying(TopicIndex::LOCAL_AIRSPEED_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::LOCAL_AIRSPEED_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::LOCAL_AIRSPEED_PUB), .type=UXR_DATAREADER_ID},
        .topic_rw = Topic_rw::DataWriter,
        .topic_name = "rt/ap/airspeed",
        .type_name = "ardupilot_msgs::msg::dds_::Airspeed_",
        .qos = {
            .durability = UXR_DURABILITY_VOLATILE,
            .reliability = UXR_RELIABILITY_BEST_EFFORT,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 5,
        },
    },
#endif // AP_DDS_AIRSPEED_PUB_ENABLED
#if AP_DDS_RC_PUB_ENABLED
    {
        .topic_id = to_underlying(TopicIndex::LOCAL_RC_PUB),
        .pub_id = to_underlying(TopicIndex::LOCAL_RC_PUB),
        .sub_id = to_underlying(TopicIndex::LOCAL_RC_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::LOCAL_RC_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::LOCAL_RC_PUB), .type=UXR_DATAREADER_ID},
        .topic_rw = Topic_rw::DataWriter,
        .topic_name = "rt/ap/rc",
        .type_name = "ardupilot_msgs::msg::dds_::Rc_",
        .qos = {
            .durability = UXR_DURABILITY_VOLATILE,
            .reliability = UXR_RELIABILITY_BEST_EFFORT,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 1,
        },
    },
#endif // AP_DDS_RC_PUB_ENABLED
#if AP_DDS_GEOPOSE_PUB_ENABLED
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
#endif // AP_DDS_GEOPOSE_PUB_ENABLED
#if AP_DDS_GOAL_PUB_ENABLED
    {
        .topic_id = to_underlying(TopicIndex::GOAL_PUB),
        .pub_id = to_underlying(TopicIndex::GOAL_PUB),
        .sub_id = to_underlying(TopicIndex::GOAL_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::GOAL_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::GOAL_PUB), .type=UXR_DATAREADER_ID},
        .topic_rw = Topic_rw::DataWriter,
        .topic_name = "rt/ap/goal_lla",
        .type_name = "geographic_msgs::msg::dds_::GeoPointStamped_",
        .qos = {
            .durability = UXR_DURABILITY_TRANSIENT_LOCAL,
            .reliability = UXR_RELIABILITY_RELIABLE,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 1,
        },
    },
#endif // AP_DDS_GOAL_PUB_ENABLED
#if AP_DDS_CLOCK_PUB_ENABLED
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
#endif // AP_DDS_CLOCK_PUB_ENABLED
#if AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
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
#endif // AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
#if AP_DDS_STATUS_PUB_ENABLED
    {
        .topic_id = to_underlying(TopicIndex::STATUS_PUB),
        .pub_id = to_underlying(TopicIndex::STATUS_PUB),
        .sub_id = to_underlying(TopicIndex::STATUS_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::STATUS_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::STATUS_PUB), .type=UXR_DATAREADER_ID},
        .topic_rw = Topic_rw::DataWriter,
        .topic_name = "rt/ap/status",
        .type_name = "ardupilot_msgs::msg::dds_::Status_",
        .qos = {
            .durability = UXR_DURABILITY_TRANSIENT_LOCAL,
            .reliability = UXR_RELIABILITY_RELIABLE,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 1,
        },
    },
#endif // AP_DDS_STATUS_PUB_ENABLED
#if AP_DDS_JOY_SUB_ENABLED
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
#endif // AP_DDS_JOY_SUB_ENABLED
#if AP_DDS_DYNAMIC_TF_SUB_ENABLED
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
#endif // AP_DDS_DYNAMIC_TF_SUB_ENABLED
#if AP_DDS_VEL_CTRL_ENABLED
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
#endif // AP_DDS_VEL_CTRL_ENABLED
#if AP_DDS_GLOBAL_POS_CTRL_ENABLED
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
#endif // AP_DDS_GLOBAL_POS_CTRL_ENABLED
};
