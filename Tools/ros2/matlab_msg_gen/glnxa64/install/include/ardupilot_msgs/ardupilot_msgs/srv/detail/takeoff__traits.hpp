// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ardupilot_msgs:srv/Takeoff.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__SRV__DETAIL__TAKEOFF__TRAITS_HPP_
#define ARDUPILOT_MSGS__SRV__DETAIL__TAKEOFF__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ardupilot_msgs/srv/detail/takeoff__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ardupilot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Takeoff_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: alt
  {
    out << "alt: ";
    rosidl_generator_traits::value_to_yaml(msg.alt, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Takeoff_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: alt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alt: ";
    rosidl_generator_traits::value_to_yaml(msg.alt, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Takeoff_Request & msg, bool use_flow_style = false)
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
  const ardupilot_msgs::srv::Takeoff_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ardupilot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ardupilot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const ardupilot_msgs::srv::Takeoff_Request & msg)
{
  return ardupilot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ardupilot_msgs::srv::Takeoff_Request>()
{
  return "ardupilot_msgs::srv::Takeoff_Request";
}

template<>
inline const char * name<ardupilot_msgs::srv::Takeoff_Request>()
{
  return "ardupilot_msgs/srv/Takeoff_Request";
}

template<>
struct has_fixed_size<ardupilot_msgs::srv::Takeoff_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ardupilot_msgs::srv::Takeoff_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ardupilot_msgs::srv::Takeoff_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ardupilot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Takeoff_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Takeoff_Response & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Takeoff_Response & msg, bool use_flow_style = false)
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
  const ardupilot_msgs::srv::Takeoff_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ardupilot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ardupilot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const ardupilot_msgs::srv::Takeoff_Response & msg)
{
  return ardupilot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ardupilot_msgs::srv::Takeoff_Response>()
{
  return "ardupilot_msgs::srv::Takeoff_Response";
}

template<>
inline const char * name<ardupilot_msgs::srv::Takeoff_Response>()
{
  return "ardupilot_msgs/srv/Takeoff_Response";
}

template<>
struct has_fixed_size<ardupilot_msgs::srv::Takeoff_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ardupilot_msgs::srv::Takeoff_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ardupilot_msgs::srv::Takeoff_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ardupilot_msgs::srv::Takeoff>()
{
  return "ardupilot_msgs::srv::Takeoff";
}

template<>
inline const char * name<ardupilot_msgs::srv::Takeoff>()
{
  return "ardupilot_msgs/srv/Takeoff";
}

template<>
struct has_fixed_size<ardupilot_msgs::srv::Takeoff>
  : std::integral_constant<
    bool,
    has_fixed_size<ardupilot_msgs::srv::Takeoff_Request>::value &&
    has_fixed_size<ardupilot_msgs::srv::Takeoff_Response>::value
  >
{
};

template<>
struct has_bounded_size<ardupilot_msgs::srv::Takeoff>
  : std::integral_constant<
    bool,
    has_bounded_size<ardupilot_msgs::srv::Takeoff_Request>::value &&
    has_bounded_size<ardupilot_msgs::srv::Takeoff_Response>::value
  >
{
};

template<>
struct is_service<ardupilot_msgs::srv::Takeoff>
  : std::true_type
{
};

template<>
struct is_service_request<ardupilot_msgs::srv::Takeoff_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ardupilot_msgs::srv::Takeoff_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ARDUPILOT_MSGS__SRV__DETAIL__TAKEOFF__TRAITS_HPP_
