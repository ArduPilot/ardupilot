// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ardupilot_msgs:srv/ArmMotors.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__SRV__DETAIL__ARM_MOTORS__BUILDER_HPP_
#define ARDUPILOT_MSGS__SRV__DETAIL__ARM_MOTORS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ardupilot_msgs/srv/detail/arm_motors__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ardupilot_msgs
{

namespace srv
{

namespace builder
{

class Init_ArmMotors_Request_arm
{
public:
  Init_ArmMotors_Request_arm()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ardupilot_msgs::srv::ArmMotors_Request arm(::ardupilot_msgs::srv::ArmMotors_Request::_arm_type arg)
  {
    msg_.arm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ardupilot_msgs::srv::ArmMotors_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ardupilot_msgs::srv::ArmMotors_Request>()
{
  return ardupilot_msgs::srv::builder::Init_ArmMotors_Request_arm();
}

}  // namespace ardupilot_msgs


namespace ardupilot_msgs
{

namespace srv
{

namespace builder
{

class Init_ArmMotors_Response_result
{
public:
  Init_ArmMotors_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ardupilot_msgs::srv::ArmMotors_Response result(::ardupilot_msgs::srv::ArmMotors_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ardupilot_msgs::srv::ArmMotors_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ardupilot_msgs::srv::ArmMotors_Response>()
{
  return ardupilot_msgs::srv::builder::Init_ArmMotors_Response_result();
}

}  // namespace ardupilot_msgs

#endif  // ARDUPILOT_MSGS__SRV__DETAIL__ARM_MOTORS__BUILDER_HPP_
