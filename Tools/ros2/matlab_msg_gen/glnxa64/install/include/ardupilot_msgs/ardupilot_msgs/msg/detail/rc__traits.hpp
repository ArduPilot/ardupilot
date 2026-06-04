// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ardupilot_msgs:msg/Rc.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__MSG__DETAIL__RC__TRAITS_HPP_
#define ARDUPILOT_MSGS__MSG__DETAIL__RC__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ardupilot_msgs/msg/detail/rc__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace ardupilot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Rc & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: is_connected
  {
    out << "is_connected: ";
    rosidl_generator_traits::value_to_yaml(msg.is_connected, out);
    out << ", ";
  }

  // member: receiver_rssi
  {
    out << "receiver_rssi: ";
    rosidl_generator_traits::value_to_yaml(msg.receiver_rssi, out);
    out << ", ";
  }

  // member: channels
  {
    if (msg.channels.size() == 0) {
      out << "channels: []";
    } else {
      out << "channels: [";
      size_t pending_items = msg.channels.size();
      for (auto item : msg.channels) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: active_overrides
  {
    if (msg.active_overrides.size() == 0) {
      out << "active_overrides: []";
    } else {
      out << "active_overrides: [";
      size_t pending_items = msg.active_overrides.size();
      for (auto item : msg.active_overrides) {
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
  const Rc & msg,
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

  // member: is_connected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_connected: ";
    rosidl_generator_traits::value_to_yaml(msg.is_connected, out);
    out << "\n";
  }

  // member: receiver_rssi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "receiver_rssi: ";
    rosidl_generator_traits::value_to_yaml(msg.receiver_rssi, out);
    out << "\n";
  }

  // member: channels
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.channels.size() == 0) {
      out << "channels: []\n";
    } else {
      out << "channels:\n";
      for (auto item : msg.channels) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: active_overrides
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.active_overrides.size() == 0) {
      out << "active_overrides: []\n";
    } else {
      out << "active_overrides:\n";
      for (auto item : msg.active_overrides) {
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

inline std::string to_yaml(const Rc & msg, bool use_flow_style = false)
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
  const ardupilot_msgs::msg::Rc & msg,
  std::ostream & out, size_t indentation = 0)
{
  ardupilot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ardupilot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ardupilot_msgs::msg::Rc & msg)
{
  return ardupilot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ardupilot_msgs::msg::Rc>()
{
  return "ardupilot_msgs::msg::Rc";
}

template<>
inline const char * name<ardupilot_msgs::msg::Rc>()
{
  return "ardupilot_msgs/msg/Rc";
}

template<>
struct has_fixed_size<ardupilot_msgs::msg::Rc>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ardupilot_msgs::msg::Rc>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ardupilot_msgs::msg::Rc>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARDUPILOT_MSGS__MSG__DETAIL__RC__TRAITS_HPP_
