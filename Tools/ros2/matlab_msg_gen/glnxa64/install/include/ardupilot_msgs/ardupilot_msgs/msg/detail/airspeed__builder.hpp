// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ardupilot_msgs:msg/Airspeed.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__MSG__DETAIL__AIRSPEED__BUILDER_HPP_
#define ARDUPILOT_MSGS__MSG__DETAIL__AIRSPEED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ardupilot_msgs/msg/detail/airspeed__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ardupilot_msgs
{

namespace msg
{

namespace builder
{

class Init_Airspeed_eas_2_tas
{
public:
  explicit Init_Airspeed_eas_2_tas(::ardupilot_msgs::msg::Airspeed & msg)
  : msg_(msg)
  {}
  ::ardupilot_msgs::msg::Airspeed eas_2_tas(::ardupilot_msgs::msg::Airspeed::_eas_2_tas_type arg)
  {
    msg_.eas_2_tas = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ardupilot_msgs::msg::Airspeed msg_;
};

class Init_Airspeed_true_airspeed
{
public:
  explicit Init_Airspeed_true_airspeed(::ardupilot_msgs::msg::Airspeed & msg)
  : msg_(msg)
  {}
  Init_Airspeed_eas_2_tas true_airspeed(::ardupilot_msgs::msg::Airspeed::_true_airspeed_type arg)
  {
    msg_.true_airspeed = std::move(arg);
    return Init_Airspeed_eas_2_tas(msg_);
  }

private:
  ::ardupilot_msgs::msg::Airspeed msg_;
};

class Init_Airspeed_header
{
public:
  Init_Airspeed_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Airspeed_true_airspeed header(::ardupilot_msgs::msg::Airspeed::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Airspeed_true_airspeed(msg_);
  }

private:
  ::ardupilot_msgs::msg::Airspeed msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ardupilot_msgs::msg::Airspeed>()
{
  return ardupilot_msgs::msg::builder::Init_Airspeed_header();
}

}  // namespace ardupilot_msgs

#endif  // ARDUPILOT_MSGS__MSG__DETAIL__AIRSPEED__BUILDER_HPP_
