// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ardupilot_msgs:msg/GlobalPosition.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__MSG__DETAIL__GLOBAL_POSITION__TRAITS_HPP_
#define ARDUPILOT_MSGS__MSG__DETAIL__GLOBAL_POSITION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ardupilot_msgs/msg/detail/global_position__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'velocity'
// Member 'acceleration_or_force'
#include "geometry_msgs/msg/detail/twist__traits.hpp"

namespace ardupilot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GlobalPosition & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: coordinate_frame
  {
    out << "coordinate_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.coordinate_frame, out);
    out << ", ";
  }

  // member: type_mask
  {
    out << "type_mask: ";
    rosidl_generator_traits::value_to_yaml(msg.type_mask, out);
    out << ", ";
  }

  // member: latitude
  {
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << ", ";
  }

  // member: longitude
  {
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << ", ";
  }

  // member: altitude
  {
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    to_flow_style_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: acceleration_or_force
  {
    out << "acceleration_or_force: ";
    to_flow_style_yaml(msg.acceleration_or_force, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GlobalPosition & msg,
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

  // member: coordinate_frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "coordinate_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.coordinate_frame, out);
    out << "\n";
  }

  // member: type_mask
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type_mask: ";
    rosidl_generator_traits::value_to_yaml(msg.type_mask, out);
    out << "\n";
  }

  // member: latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << "\n";
  }

  // member: longitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << "\n";
  }

  // member: altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << "\n";
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity:\n";
    to_block_style_yaml(msg.velocity, out, indentation + 2);
  }

  // member: acceleration_or_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acceleration_or_force:\n";
    to_block_style_yaml(msg.acceleration_or_force, out, indentation + 2);
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GlobalPosition & msg, bool use_flow_style = false)
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
  const ardupilot_msgs::msg::GlobalPosition & msg,
  std::ostream & out, size_t indentation = 0)
{
  ardupilot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ardupilot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ardupilot_msgs::msg::GlobalPosition & msg)
{
  return ardupilot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ardupilot_msgs::msg::GlobalPosition>()
{
  return "ardupilot_msgs::msg::GlobalPosition";
}

template<>
inline const char * name<ardupilot_msgs::msg::GlobalPosition>()
{
  return "ardupilot_msgs/msg/GlobalPosition";
}

template<>
struct has_fixed_size<ardupilot_msgs::msg::GlobalPosition>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Twist>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<ardupilot_msgs::msg::GlobalPosition>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Twist>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ardupilot_msgs::msg::GlobalPosition>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARDUPILOT_MSGS__MSG__DETAIL__GLOBAL_POSITION__TRAITS_HPP_
