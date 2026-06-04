// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ardupilot_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__MSG__DETAIL__STATUS__BUILDER_HPP_
#define ARDUPILOT_MSGS__MSG__DETAIL__STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ardupilot_msgs/msg/detail/status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ardupilot_msgs
{

namespace msg
{

namespace builder
{

class Init_Status_failsafe
{
public:
  explicit Init_Status_failsafe(::ardupilot_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  ::ardupilot_msgs::msg::Status failsafe(::ardupilot_msgs::msg::Status::_failsafe_type arg)
  {
    msg_.failsafe = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ardupilot_msgs::msg::Status msg_;
};

class Init_Status_external_control
{
public:
  explicit Init_Status_external_control(::ardupilot_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_failsafe external_control(::ardupilot_msgs::msg::Status::_external_control_type arg)
  {
    msg_.external_control = std::move(arg);
    return Init_Status_failsafe(msg_);
  }

private:
  ::ardupilot_msgs::msg::Status msg_;
};

class Init_Status_flying
{
public:
  explicit Init_Status_flying(::ardupilot_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_external_control flying(::ardupilot_msgs::msg::Status::_flying_type arg)
  {
    msg_.flying = std::move(arg);
    return Init_Status_external_control(msg_);
  }

private:
  ::ardupilot_msgs::msg::Status msg_;
};

class Init_Status_mode
{
public:
  explicit Init_Status_mode(::ardupilot_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_flying mode(::ardupilot_msgs::msg::Status::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_Status_flying(msg_);
  }

private:
  ::ardupilot_msgs::msg::Status msg_;
};

class Init_Status_armed
{
public:
  explicit Init_Status_armed(::ardupilot_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_mode armed(::ardupilot_msgs::msg::Status::_armed_type arg)
  {
    msg_.armed = std::move(arg);
    return Init_Status_mode(msg_);
  }

private:
  ::ardupilot_msgs::msg::Status msg_;
};

class Init_Status_vehicle_type
{
public:
  explicit Init_Status_vehicle_type(::ardupilot_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_armed vehicle_type(::ardupilot_msgs::msg::Status::_vehicle_type_type arg)
  {
    msg_.vehicle_type = std::move(arg);
    return Init_Status_armed(msg_);
  }

private:
  ::ardupilot_msgs::msg::Status msg_;
};

class Init_Status_header
{
public:
  Init_Status_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Status_vehicle_type header(::ardupilot_msgs::msg::Status::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Status_vehicle_type(msg_);
  }

private:
  ::ardupilot_msgs::msg::Status msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ardupilot_msgs::msg::Status>()
{
  return ardupilot_msgs::msg::builder::Init_Status_header();
}

}  // namespace ardupilot_msgs

#endif  // ARDUPILOT_MSGS__MSG__DETAIL__STATUS__BUILDER_HPP_
