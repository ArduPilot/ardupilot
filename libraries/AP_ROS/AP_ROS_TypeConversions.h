// Class for handling type conversions for ROS.

#pragma once

#include <stdint.h>

class AP_ROS_TypeConversions
{
public:

    // Convert ROS time to a uint64_t [μS]
    template <typename TTime>
    static uint64_t time_u64_micros(const TTime& ros_time);
};

template <typename TTime>
uint64_t AP_ROS_TypeConversions::time_u64_micros(const TTime& ros_time)
{
    return (uint64_t(ros_time.sec) * 1000000ULL) + (ros_time.nanosec / 1000ULL);
}

// string accessor templates
template <typename S>
const char* string_data(const S* str);

template <typename S>
char* mutable_string_data(S* str);

template <typename S>
const char* string_data(const S& str);

template <typename S>
char* mutable_string_data(S& str);

// sequence accessor templates
// see: https://stackoverflow.com/questions/15911890/overriding-return-type-in-function-template-specialization

template <typename T>
struct transforms_size_type { typedef uint32_t type; };

template <typename T>
struct mutable_transforms_size_type { typedef uint32_t& type; };

template <typename T>
typename transforms_size_type<T>::type transforms_size(const T& msg);

template <typename T>
typename mutable_transforms_size_type<T>::type mutable_transforms_size(T& msg);

template <typename T>
struct transforms_type { typedef void* type; };

template <typename T>
struct mutable_transforms_type { typedef void* type; };

template <typename T>
typename transforms_type<T>::type transforms_data(const T& msg);

template <typename T>
typename mutable_transforms_type<T>::type mutable_transforms_data(T& msg);

// battery state : cell_voltage

template <typename T>
struct mutable_cell_voltage_type { typedef void* type; };

template <typename T>
typename mutable_cell_voltage_type<T>::type mutable_cell_voltage_data(T& msg);
