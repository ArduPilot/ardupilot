// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ardupilot_msgs:srv/ModeSwitch.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__SRV__DETAIL__MODE_SWITCH__BUILDER_HPP_
#define ARDUPILOT_MSGS__SRV__DETAIL__MODE_SWITCH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ardupilot_msgs/srv/detail/mode_switch__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ardupilot_msgs
{

namespace srv
{

namespace builder
{

class Init_ModeSwitch_Request_mode
{
public:
  Init_ModeSwitch_Request_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ardupilot_msgs::srv::ModeSwitch_Request mode(::ardupilot_msgs::srv::ModeSwitch_Request::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ardupilot_msgs::srv::ModeSwitch_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ardupilot_msgs::srv::ModeSwitch_Request>()
{
  return ardupilot_msgs::srv::builder::Init_ModeSwitch_Request_mode();
}

}  // namespace ardupilot_msgs


namespace ardupilot_msgs
{

namespace srv
{

namespace builder
{

class Init_ModeSwitch_Response_curr_mode
{
public:
  explicit Init_ModeSwitch_Response_curr_mode(::ardupilot_msgs::srv::ModeSwitch_Response & msg)
  : msg_(msg)
  {}
  ::ardupilot_msgs::srv::ModeSwitch_Response curr_mode(::ardupilot_msgs::srv::ModeSwitch_Response::_curr_mode_type arg)
  {
    msg_.curr_mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ardupilot_msgs::srv::ModeSwitch_Response msg_;
};

class Init_ModeSwitch_Response_status
{
public:
  Init_ModeSwitch_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ModeSwitch_Response_curr_mode status(::ardupilot_msgs::srv::ModeSwitch_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_ModeSwitch_Response_curr_mode(msg_);
  }

private:
  ::ardupilot_msgs::srv::ModeSwitch_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ardupilot_msgs::srv::ModeSwitch_Response>()
{
  return ardupilot_msgs::srv::builder::Init_ModeSwitch_Response_status();
}

}  // namespace ardupilot_msgs

#endif  // ARDUPILOT_MSGS__SRV__DETAIL__MODE_SWITCH__BUILDER_HPP_
