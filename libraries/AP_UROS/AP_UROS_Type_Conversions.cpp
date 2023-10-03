#include "AP_UROS_Type_Conversions.h"
#if AP_UROS_ENABLED

uint64_t AP_UROS_Type_Conversions::time_u64_micros(const builtin_interfaces__msg__Time& ros_time)
{
    return AP_ROS_TypeConversions::time_u64_micros(ros_time);
}

// string specialisations
template <>
const char* string_data(const rosidl_runtime_c__String& str) {
    return str.data;
}

template <>
char* mutable_string_data(rosidl_runtime_c__String& str) {
    return str.data;
}

// transform specialisations
template <>
typename transforms_size_type<tf2_msgs__msg__TFMessage>::type
transforms_size(const tf2_msgs__msg__TFMessage& msg) {
    return msg.transforms.size;
}

template <>
typename mutable_transforms_size_type<tf2_msgs__msg__TFMessage>::type
mutable_transforms_size(tf2_msgs__msg__TFMessage& msg) {
    return msg.transforms.size;
}

template <>
typename transforms_type<tf2_msgs__msg__TFMessage>::type
transforms_data(const tf2_msgs__msg__TFMessage& msg) {
    return msg.transforms.data;
}

template <>
typename mutable_transforms_type<tf2_msgs__msg__TFMessage>::type
mutable_transforms_data(tf2_msgs__msg__TFMessage& msg) {
    return msg.transforms.data;
}

// cell_voltage specialisations
template <>
typename mutable_cell_voltage_type<sensor_msgs__msg__BatteryState>::type
mutable_cell_voltage_data(sensor_msgs__msg__BatteryState& msg) {
    return msg.cell_voltage.data;
}

#if 0
//! @todo(srmainwaring) move to test / examples sub-folder.
// tests for the accessor templates
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

template <typename TFMessage>
void test_read(const TFMessage& msg) {
    // read msg.transforms.size
    hal.console->printf("%u", (uint32_t)transforms_size<TFMessage>(msg));

    // read msg.transforms[i]
    double sum_x = 0.0;
    for (size_t i = 0; i < transforms_size<TFMessage>(msg); ++i) {
        const auto& transform_stamped = transforms_data<TFMessage>(msg)[i];
        sum_x += transform_stamped.transform.translation.x;
    }

    hal.console->printf("%f", sum_x);
}

template <typename TFMessage>
void test_write(TFMessage& msg) {
    // write msg.transforms.size
    mutable_transforms_size<TFMessage>(msg) = 0;
    for (size_t i = 0; i < 4; ++i) {
        mutable_transforms_size<TFMessage>(msg)++;
    }
    hal.console->printf("%u", (uint32_t)transforms_size<TFMessage>(msg));

    // write msg.transforms[i]
    auto* transform_stamped = mutable_transforms_data<TFMessage>(msg);
    transform_stamped[0].transform.translation.x = 10.0;

    hal.console->printf("%f",
        transforms_data<TFMessage>(msg)[0].transform.translation.x);
}

void test_read(const tf2_msgs__msg__TFMessage& msg) {
    test_read<tf2_msgs__msg__TFMessage>(msg);
}

void test_write(tf2_msgs__msg__TFMessage& msg) {
    test_write<tf2_msgs__msg__TFMessage>(msg);
}

void test_write2(tf2_msgs__msg__TFMessage& msg) {
    test_write<tf2_msgs__msg__TFMessage>(msg);
}
#endif

#endif // AP_UROS_ENABLED
