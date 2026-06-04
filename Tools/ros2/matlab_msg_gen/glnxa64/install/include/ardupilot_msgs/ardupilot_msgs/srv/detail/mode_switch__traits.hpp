// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ardupilot_msgs:srv/ModeSwitch.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__SRV__DETAIL__MODE_SWITCH__TRAITS_HPP_
#define ARDUPILOT_MSGS__SRV__DETAIL__MODE_SWITCH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ardupilot_msgs/srv/detail/mode_switch__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ardupilot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ModeSwitch_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ModeSwitch_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ModeSwitch_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ardupilot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ardupilot_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ardupilot_msgs::srv::ModeSwitch_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ardupilot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ardupilot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const ardupilot_msgs::srv::ModeSwitch_Request & msg)
{
  return ardupilot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ardupilot_msgs::srv::ModeSwitch_Request>()
{
  return "ardupilot_msgs::srv::ModeSwitch_Request";
}

template<>
inline const char * name<ardupilot_msgs::srv::ModeSwitch_Request>()
{
  return "ardupilot_msgs/srv/ModeSwitch_Request";
}

template<>
struct has_fixed_size<ardupilot_msgs::srv::ModeSwitch_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ardupilot_msgs::srv::ModeSwitch_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ardupilot_msgs::srv::ModeSwitch_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ardupilot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ModeSwitch_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: curr_mode
  {
    out << "curr_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.curr_mode, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ModeSwitch_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: curr_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "curr_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.curr_mode, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ModeSwitch_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ardupilot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ardupilot_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ardupilot_msgs::srv::ModeSwitch_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ardupilot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ardupilot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const ardupilot_msgs::srv::ModeSwitch_Response & msg)
{
  return ardupilot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ardupilot_msgs::srv::ModeSwitch_Response>()
{
  return "ardupilot_msgs::srv::ModeSwitch_Response";
}

template<>
inline const char * name<ardupilot_msgs::srv::ModeSwitch_Response>()
{
  return "ardupilot_msgs/srv/ModeSwitch_Response";
}

template<>
struct has_fixed_size<ardupilot_msgs::srv::ModeSwitch_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ardupilot_msgs::srv::ModeSwitch_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ardupilot_msgs::srv::ModeSwitch_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ardupilot_msgs::srv::ModeSwitch>()
{
  return "ardupilot_msgs::srv::ModeSwitch";
}

template<>
inline const char * name<ardupilot_msgs::srv::ModeSwitch>()
{
  return "ardupilot_msgs/srv/ModeSwitch";
}

template<>
struct has_fixed_size<ardupilot_msgs::srv::ModeSwitch>
  : std::integral_constant<
    bool,
    has_fixed_size<ardupilot_msgs::srv::ModeSwitch_Request>::value &&
    has_fixed_size<ardupilot_msgs::srv::ModeSwitch_Response>::value
  >
{
};

template<>
struct has_bounded_size<ardupilot_msgs::srv::ModeSwitch>
  : std::integral_constant<
    bool,
    has_bounded_size<ardupilot_msgs::srv::ModeSwitch_Request>::value &&
    has_bounded_size<ardupilot_msgs::srv::ModeSwitch_Response>::value
  >
{
};

template<>
struct is_service<ardupilot_msgs::srv::ModeSwitch>
  : std::true_type
{
};

template<>
struct is_service_request<ardupilot_msgs::srv::ModeSwitch_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ardupilot_msgs::srv::ModeSwitch_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ARDUPILOT_MSGS__SRV__DETAIL__MODE_SWITCH__TRAITS_HPP_
