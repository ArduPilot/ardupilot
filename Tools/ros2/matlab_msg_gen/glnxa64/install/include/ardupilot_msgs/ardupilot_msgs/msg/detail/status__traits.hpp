// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ardupilot_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__MSG__DETAIL__STATUS__TRAITS_HPP_
#define ARDUPILOT_MSGS__MSG__DETAIL__STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ardupilot_msgs/msg/detail/status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace ardupilot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Status & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: vehicle_type
  {
    out << "vehicle_type: ";
    rosidl_generator_traits::value_to_yaml(msg.vehicle_type, out);
    out << ", ";
  }

  // member: armed
  {
    out << "armed: ";
    rosidl_generator_traits::value_to_yaml(msg.armed, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: flying
  {
    out << "flying: ";
    rosidl_generator_traits::value_to_yaml(msg.flying, out);
    out << ", ";
  }

  // member: external_control
  {
    out << "external_control: ";
    rosidl_generator_traits::value_to_yaml(msg.external_control, out);
    out << ", ";
  }

  // member: failsafe
  {
    if (msg.failsafe.size() == 0) {
      out << "failsafe: []";
    } else {
      out << "failsafe: [";
      size_t pending_items = msg.failsafe.size();
      for (auto item : msg.failsafe) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Status & msg,
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

  // member: vehicle_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vehicle_type: ";
    rosidl_generator_traits::value_to_yaml(msg.vehicle_type, out);
    out << "\n";
  }

  // member: armed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "armed: ";
    rosidl_generator_traits::value_to_yaml(msg.armed, out);
    out << "\n";
  }

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: flying
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flying: ";
    rosidl_generator_traits::value_to_yaml(msg.flying, out);
    out << "\n";
  }

  // member: external_control
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "external_control: ";
    rosidl_generator_traits::value_to_yaml(msg.external_control, out);
    out << "\n";
  }

  // member: failsafe
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.failsafe.size() == 0) {
      out << "failsafe: []\n";
    } else {
      out << "failsafe:\n";
      for (auto item : msg.failsafe) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Status & msg, bool use_flow_style = false)
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
  const ardupilot_msgs::msg::Status & msg,
  std::ostream & out, size_t indentation = 0)
{
  ardupilot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ardupilot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ardupilot_msgs::msg::Status & msg)
{
  return ardupilot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ardupilot_msgs::msg::Status>()
{
  return "ardupilot_msgs::msg::Status";
}

template<>
inline const char * name<ardupilot_msgs::msg::Status>()
{
  return "ardupilot_msgs/msg/Status";
}

template<>
struct has_fixed_size<ardupilot_msgs::msg::Status>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ardupilot_msgs::msg::Status>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ardupilot_msgs::msg::Status>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARDUPILOT_MSGS__MSG__DETAIL__STATUS__TRAITS_HPP_
