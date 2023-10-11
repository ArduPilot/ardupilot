// Class for handling type conversions for DDS.

#pragma once

#if AP_DDS_ENABLED

#include <AP_ROS/AP_ROS_TypeConversions.h>

#include "builtin_interfaces/msg/Time.h"
#include "geometry_msgs/msg/TransformStamped.h"
#include "sensor_msgs/msg/BatteryState.h"
#include "tf2_msgs/msg/TFMessage.h"

class AP_DDS_Type_Conversions
{
public:

    // Convert ROS time to a uint64_t [μS]
    static uint64_t time_u64_micros(const builtin_interfaces_msg_Time& ros_time);
};

// string specialisations
template <>
const char* string_data(const char* str);

template <>
char* mutable_string_data(char* str);

// transform specialisations
template <>
struct transforms_size_type<tf2_msgs_msg_TFMessage> {
    typedef uint32_t type;
};

template <>
struct mutable_transforms_size_type<tf2_msgs_msg_TFMessage> {
    typedef uint32_t& type;
};

template <>
typename transforms_size_type<tf2_msgs_msg_TFMessage>::type
transforms_size(const tf2_msgs_msg_TFMessage& msg);

template <>
typename mutable_transforms_size_type<tf2_msgs_msg_TFMessage>::type
mutable_transforms_size(tf2_msgs_msg_TFMessage& msg);

template <>
struct transforms_type<tf2_msgs_msg_TFMessage> {
    typedef const geometry_msgs_msg_TransformStamped* type;
};

template <>
struct mutable_transforms_type<tf2_msgs_msg_TFMessage> {
    typedef geometry_msgs_msg_TransformStamped* type;
};

template <>
typename transforms_type<tf2_msgs_msg_TFMessage>::type
transforms_data(const tf2_msgs_msg_TFMessage& msg);

template <>
typename mutable_transforms_type<tf2_msgs_msg_TFMessage>::type
mutable_transforms_data(tf2_msgs_msg_TFMessage& msg);

// cell_voltage specialisations
template <>
struct mutable_cell_voltage_type<sensor_msgs_msg_BatteryState> {
    typedef float* type;
};

template <>
typename mutable_cell_voltage_type<sensor_msgs_msg_BatteryState>::type
mutable_cell_voltage_data(sensor_msgs_msg_BatteryState& msg);

#endif // AP_DDS_ENABLED
