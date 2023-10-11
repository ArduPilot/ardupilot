#include "AP_DDS_Type_Conversions.h"
#if AP_DDS_ENABLED

uint64_t AP_DDS_Type_Conversions::time_u64_micros(const builtin_interfaces_msg_Time& ros_time)
{
    return AP_ROS_TypeConversions::time_u64_micros(ros_time);
}

// string specialisations
template <>
const char* string_data(const char* str) {
    return str;
}

template <>
char* mutable_string_data(char* str) {
    return str;
}

// transform specialisations
template <>
typename transforms_size_type<tf2_msgs_msg_TFMessage>::type
transforms_size(const tf2_msgs_msg_TFMessage& msg) {
    return msg.transforms_size;
}

template <>
typename mutable_transforms_size_type<tf2_msgs_msg_TFMessage>::type
mutable_transforms_size(tf2_msgs_msg_TFMessage& msg) {
    return msg.transforms_size;
}

template <>
typename transforms_type<tf2_msgs_msg_TFMessage>::type
transforms_data(const tf2_msgs_msg_TFMessage& msg) {
    return msg.transforms;
}

template <>
typename mutable_transforms_type<tf2_msgs_msg_TFMessage>::type
mutable_transforms_data(tf2_msgs_msg_TFMessage& msg) {
    return msg.transforms;
}

// cell_voltage specialisations
template <>
typename mutable_cell_voltage_type<sensor_msgs_msg_BatteryState>::type
mutable_cell_voltage_data(sensor_msgs_msg_BatteryState& msg) {
    return msg.cell_voltage;
}

#endif // AP_DDS_ENABLED
