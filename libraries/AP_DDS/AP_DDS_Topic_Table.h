#include "builtin_interfaces/msg/Time.h"
#include "sensor_msgs/msg/NavSatFix.h"
#include "tf2_msgs/msg/TFMessage.h"
#include "sensor_msgs/msg/BatteryState.h"
#include "geographic_msgs/msg/GeoPoseStamped.h"

#include "AP_DDS_Generic_Fn_T.h"
#include "uxr/client/client.h"

// Code generated table based on the enabled topics.
// Mavgen is using python, loops are not readable.
// Can use jinja to template (like Flask)


const struct AP_DDS_Client::Topic_table AP_DDS_Client::topics[] = {
    {
        .topic_id = 0x01,
        .pub_id = 0x01,
        .dw_id = uxrObjectId{.id=0x01, .type=UXR_DATAWRITER_ID},
        .topic_profile_label = "time__t",
        .dw_profile_label = "time__dw",
        .serialize = Generic_serialize_topic_fn_t(&builtin_interfaces_msg_Time_serialize_topic),
        .deserialize = Generic_deserialize_topic_fn_t(&builtin_interfaces_msg_Time_deserialize_topic),
        .size_of = Generic_size_of_topic_fn_t(&builtin_interfaces_msg_Time_size_of_topic),
    },
    {
        .topic_id = 0x02,
        .pub_id = 0x02,
        .dw_id = uxrObjectId{.id=0x02, .type=UXR_DATAWRITER_ID},
        .topic_profile_label = "navsatfix0__t",
        .dw_profile_label = "navsatfix0__dw",
        .serialize = Generic_serialize_topic_fn_t(&sensor_msgs_msg_NavSatFix_serialize_topic),
        .deserialize = Generic_deserialize_topic_fn_t(&sensor_msgs_msg_NavSatFix_deserialize_topic),
        .size_of = Generic_size_of_topic_fn_t(&sensor_msgs_msg_NavSatFix_size_of_topic),
    },
    {
        .topic_id = 0x03,
        .pub_id = 0x03,
        .dw_id = uxrObjectId{.id=0x03, .type=UXR_DATAWRITER_ID},
        .topic_profile_label = "statictransforms__t",
        .dw_profile_label = "statictransforms__dw",
        .serialize = Generic_serialize_topic_fn_t(&tf2_msgs_msg_TFMessage_serialize_topic),
        .deserialize = Generic_deserialize_topic_fn_t(&tf2_msgs_msg_TFMessage_deserialize_topic),
        .size_of = Generic_size_of_topic_fn_t(&tf2_msgs_msg_TFMessage_size_of_topic),
    },
    {
        .topic_id = 0x04,
        .pub_id = 0x04,
        .dw_id = uxrObjectId{.id=0x04, .type=UXR_DATAWRITER_ID},
        .topic_profile_label = "batterystate0__t",
        .dw_profile_label = "batterystate0__dw",
        .serialize = Generic_serialize_topic_fn_t(&sensor_msgs_msg_BatteryState_serialize_topic),
        .deserialize = Generic_deserialize_topic_fn_t(&sensor_msgs_msg_BatteryState_deserialize_topic),
        .size_of = Generic_size_of_topic_fn_t(&sensor_msgs_msg_BatteryState_size_of_topic),
    },
    {
        .topic_id = 0x05,
        .pub_id = 0x05,
        .dw_id = uxrObjectId{.id=0x05, .type=UXR_DATAWRITER_ID},
        .topic_profile_label = "localpose__t",
        .dw_profile_label = "localpose__dw",
        .serialize = Generic_serialize_topic_fn_t(&geometry_msgs_msg_PoseStamped_serialize_topic),
        .deserialize = Generic_deserialize_topic_fn_t(&geometry_msgs_msg_PoseStamped_deserialize_topic),
        .size_of = Generic_size_of_topic_fn_t(&geometry_msgs_msg_PoseStamped_size_of_topic),
    },
    {
        .topic_id = 0x06,
        .pub_id = 0x06,
        .dw_id = uxrObjectId{.id=0x06, .type=UXR_DATAWRITER_ID},
        .topic_profile_label = "localvelocity__t",
        .dw_profile_label = "localvelocity__dw",
        .serialize = Generic_serialize_topic_fn_t(&geometry_msgs_msg_TwistStamped_serialize_topic),
        .deserialize = Generic_deserialize_topic_fn_t(&geometry_msgs_msg_TwistStamped_deserialize_topic),
        .size_of = Generic_size_of_topic_fn_t(&geometry_msgs_msg_TwistStamped_size_of_topic),
    },
    {
        .topic_id = 0x07,
        .pub_id = 0x07,
        .dw_id = uxrObjectId{.id=0x07, .type=UXR_DATAWRITER_ID},
        .topic_profile_label = "geopose__t",
        .dw_profile_label = "geopose__dw",
        .serialize = Generic_serialize_topic_fn_t(&geographic_msgs_msg_GeoPoseStamped_serialize_topic),
        .deserialize = Generic_deserialize_topic_fn_t(&geographic_msgs_msg_GeoPoseStamped_deserialize_topic),
        .size_of = Generic_size_of_topic_fn_t(&geographic_msgs_msg_GeoPoseStamped_size_of_topic),
    },
    {
        .topic_id = 0x08,
        .pub_id = 0x08,
        .dw_id = uxrObjectId{.id=0x08, .type=UXR_DATAWRITER_ID},
        .topic_profile_label = "clock__t",
        .dw_profile_label = "clock__dw",
        .serialize = Generic_serialize_topic_fn_t(&rosgraph_msgs_msg_Clock_serialize_topic),
        .deserialize = Generic_deserialize_topic_fn_t(&rosgraph_msgs_msg_Clock_deserialize_topic),
        .size_of = Generic_size_of_topic_fn_t(&rosgraph_msgs_msg_Clock_size_of_topic),
    },
};
