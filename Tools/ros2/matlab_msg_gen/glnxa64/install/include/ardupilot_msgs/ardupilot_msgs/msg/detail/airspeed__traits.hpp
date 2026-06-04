// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ardupilot_msgs:msg/Airspeed.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__MSG__DETAIL__AIRSPEED__TRAITS_HPP_
#define ARDUPILOT_MSGS__MSG__DETAIL__AIRSPEED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ardupilot_msgs/msg/detail/airspeed__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'true_airspeed'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace ardupilot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Airspeed & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: true_airspeed
  {
    out << "true_airspeed: ";
    to_flow_style_yaml(msg.true_airspeed, out);
    out << ", ";
  }

  // member: eas_2_tas
  {
    out << "eas_2_tas: ";
    rosidl_generator_traits::value_to_yaml(msg.eas_2_tas, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Airspeed & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: true_airspeed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "true_airspeed:\n";
    to_block_style_yaml(msg.true_airspeed, out, indentation + 2);
  }

  // member: eas_2_tas
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "eas_2_tas: ";
    rosidl_generator_traits::value_to_yaml(msg.eas_2_tas, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Airspeed & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ardupilot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ardupilot_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ardupilot_msgs::msg::Airspeed & msg,
  std::ostream & out, size_t indentation = 0)
{
  ardupilot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ardupilot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ardupilot_msgs::msg::Airspeed & msg)
{
  return ardupilot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ardupilot_msgs::msg::Airspeed>()
{
  return "ardupilot_msgs::msg::Airspeed";
}

template<>
inline const char * name<ardupilot_msgs::msg::Airspeed>()
{
  return "ardupilot_msgs/msg/Airspeed";
}

template<>
struct has_fixed_size<ardupilot_msgs::msg::Airspeed>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<ardupilot_msgs::msg::Airspeed>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ardupilot_msgs::msg::Airspeed>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARDUPILOT_MSGS__MSG__DETAIL__AIRSPEED__TRAITS_HPP_
