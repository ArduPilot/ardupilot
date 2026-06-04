// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ardupilot_msgs:srv/Takeoff.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__SRV__DETAIL__TAKEOFF__BUILDER_HPP_
#define ARDUPILOT_MSGS__SRV__DETAIL__TAKEOFF__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ardupilot_msgs/srv/detail/takeoff__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ardupilot_msgs
{

namespace srv
{

namespace builder
{

class Init_Takeoff_Request_alt
{
public:
  Init_Takeoff_Request_alt()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ardupilot_msgs::srv::Takeoff_Request alt(::ardupilot_msgs::srv::Takeoff_Request::_alt_type arg)
  {
    msg_.alt = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ardupilot_msgs::srv::Takeoff_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ardupilot_msgs::srv::Takeoff_Request>()
{
  return ardupilot_msgs::srv::builder::Init_Takeoff_Request_alt();
}

}  // namespace ardupilot_msgs


namespace ardupilot_msgs
{

namespace srv
{

namespace builder
{

class Init_Takeoff_Response_status
{
public:
  Init_Takeoff_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ardupilot_msgs::srv::Takeoff_Response status(::ardupilot_msgs::srv::Takeoff_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ardupilot_msgs::srv::Takeoff_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ardupilot_msgs::srv::Takeoff_Response>()
{
  return ardupilot_msgs::srv::builder::Init_Takeoff_Response_status();
}

}  // namespace ardupilot_msgs

#endif  // ARDUPILOT_MSGS__SRV__DETAIL__TAKEOFF__BUILDER_HPP_
