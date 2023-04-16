#include "builtin_interfaces/msg/Time.h"
#include "sensor_msgs/msg/NavSatFix.h"
#include "tf2_msgs/msg/TFMessage.h"
#include "sensor_msgs/msg/BatteryState.h"

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
};
