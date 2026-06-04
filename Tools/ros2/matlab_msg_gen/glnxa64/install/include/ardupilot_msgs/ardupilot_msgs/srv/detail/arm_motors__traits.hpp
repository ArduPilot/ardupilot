// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ardupilot_msgs:srv/ArmMotors.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__SRV__DETAIL__ARM_MOTORS__TRAITS_HPP_
#define ARDUPILOT_MSGS__SRV__DETAIL__ARM_MOTORS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ardupilot_msgs/srv/detail/arm_motors__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ardupilot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ArmMotors_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: arm
  {
    out << "arm: ";
    rosidl_generator_traits::value_to_yaml(msg.arm, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ArmMotors_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: arm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "arm: ";
    rosidl_generator_traits::value_to_yaml(msg.arm, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ArmMotors_Request & msg, bool use_flow_style = false)
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
  const ardupilot_msgs::srv::ArmMotors_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ardupilot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ardupilot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const ardupilot_msgs::srv::ArmMotors_Request & msg)
{
  return ardupilot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ardupilot_msgs::srv::ArmMotors_Request>()
{
  return "ardupilot_msgs::srv::ArmMotors_Request";
}

template<>
inline const char * name<ardupilot_msgs::srv::ArmMotors_Request>()
{
  return "ardupilot_msgs/srv/ArmMotors_Request";
}

template<>
struct has_fixed_size<ardupilot_msgs::srv::ArmMotors_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ardupilot_msgs::srv::ArmMotors_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ardupilot_msgs::srv::ArmMotors_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ardupilot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ArmMotors_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: result
  {
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ArmMotors_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ArmMotors_Response & msg, bool use_flow_style = false)
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
  const ardupilot_msgs::srv::ArmMotors_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ardupilot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ardupilot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const ardupilot_msgs::srv::ArmMotors_Response & msg)
{
  return ardupilot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ardupilot_msgs::srv::ArmMotors_Response>()
{
  return "ardupilot_msgs::srv::ArmMotors_Response";
}

template<>
inline const char * name<ardupilot_msgs::srv::ArmMotors_Response>()
{
  return "ardupilot_msgs/srv/ArmMotors_Response";
}

template<>
struct has_fixed_size<ardupilot_msgs::srv::ArmMotors_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ardupilot_msgs::srv::ArmMotors_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ardupilot_msgs::srv::ArmMotors_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ardupilot_msgs::srv::ArmMotors>()
{
  return "ardupilot_msgs::srv::ArmMotors";
}

template<>
inline const char * name<ardupilot_msgs::srv::ArmMotors>()
{
  return "ardupilot_msgs/srv/ArmMotors";
}

template<>
struct has_fixed_size<ardupilot_msgs::srv::ArmMotors>
  : std::integral_constant<
    bool,
    has_fixed_size<ardupilot_msgs::srv::ArmMotors_Request>::value &&
    has_fixed_size<ardupilot_msgs::srv::ArmMotors_Response>::value
  >
{
};

template<>
struct has_bounded_size<ardupilot_msgs::srv::ArmMotors>
  : std::integral_constant<
    bool,
    has_bounded_size<ardupilot_msgs::srv::ArmMotors_Request>::value &&
    has_bounded_size<ardupilot_msgs::srv::ArmMotors_Response>::value
  >
{
};

template<>
struct is_service<ardupilot_msgs::srv::ArmMotors>
  : std::true_type
{
};

template<>
struct is_service_request<ardupilot_msgs::srv::ArmMotors_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ardupilot_msgs::srv::ArmMotors_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ARDUPILOT_MSGS__SRV__DETAIL__ARM_MOTORS__TRAITS_HPP_
