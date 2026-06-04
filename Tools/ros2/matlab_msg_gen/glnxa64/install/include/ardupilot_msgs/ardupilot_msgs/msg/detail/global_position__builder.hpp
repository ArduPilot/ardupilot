// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ardupilot_msgs:msg/GlobalPosition.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__MSG__DETAIL__GLOBAL_POSITION__BUILDER_HPP_
#define ARDUPILOT_MSGS__MSG__DETAIL__GLOBAL_POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ardupilot_msgs/msg/detail/global_position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ardupilot_msgs
{

namespace msg
{

namespace builder
{

class Init_GlobalPosition_yaw
{
public:
  explicit Init_GlobalPosition_yaw(::ardupilot_msgs::msg::GlobalPosition & msg)
  : msg_(msg)
  {}
  ::ardupilot_msgs::msg::GlobalPosition yaw(::ardupilot_msgs::msg::GlobalPosition::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ardupilot_msgs::msg::GlobalPosition msg_;
};

class Init_GlobalPosition_acceleration_or_force
{
public:
  explicit Init_GlobalPosition_acceleration_or_force(::ardupilot_msgs::msg::GlobalPosition & msg)
  : msg_(msg)
  {}
  Init_GlobalPosition_yaw acceleration_or_force(::ardupilot_msgs::msg::GlobalPosition::_acceleration_or_force_type arg)
  {
    msg_.acceleration_or_force = std::move(arg);
    return Init_GlobalPosition_yaw(msg_);
  }

private:
  ::ardupilot_msgs::msg::GlobalPosition msg_;
};

class Init_GlobalPosition_velocity
{
public:
  explicit Init_GlobalPosition_velocity(::ardupilot_msgs::msg::GlobalPosition & msg)
  : msg_(msg)
  {}
  Init_GlobalPosition_acceleration_or_force velocity(::ardupilot_msgs::msg::GlobalPosition::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_GlobalPosition_acceleration_or_force(msg_);
  }

private:
  ::ardupilot_msgs::msg::GlobalPosition msg_;
};

class Init_GlobalPosition_altitude
{
public:
  explicit Init_GlobalPosition_altitude(::ardupilot_msgs::msg::GlobalPosition & msg)
  : msg_(msg)
  {}
  Init_GlobalPosition_velocity altitude(::ardupilot_msgs::msg::GlobalPosition::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_GlobalPosition_velocity(msg_);
  }

private:
  ::ardupilot_msgs::msg::GlobalPosition msg_;
};

class Init_GlobalPosition_longitude
{
public:
  explicit Init_GlobalPosition_longitude(::ardupilot_msgs::msg::GlobalPosition & msg)
  : msg_(msg)
  {}
  Init_GlobalPosition_altitude longitude(::ardupilot_msgs::msg::GlobalPosition::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_GlobalPosition_altitude(msg_);
  }

private:
  ::ardupilot_msgs::msg::GlobalPosition msg_;
};

class Init_GlobalPosition_latitude
{
public:
  explicit Init_GlobalPosition_latitude(::ardupilot_msgs::msg::GlobalPosition & msg)
  : msg_(msg)
  {}
  Init_GlobalPosition_longitude latitude(::ardupilot_msgs::msg::GlobalPosition::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_GlobalPosition_longitude(msg_);
  }

private:
  ::ardupilot_msgs::msg::GlobalPosition msg_;
};

class Init_GlobalPosition_type_mask
{
public:
  explicit Init_GlobalPosition_type_mask(::ardupilot_msgs::msg::GlobalPosition & msg)
  : msg_(msg)
  {}
  Init_GlobalPosition_latitude type_mask(::ardupilot_msgs::msg::GlobalPosition::_type_mask_type arg)
  {
    msg_.type_mask = std::move(arg);
    return Init_GlobalPosition_latitude(msg_);
  }

private:
  ::ardupilot_msgs::msg::GlobalPosition msg_;
};

class Init_GlobalPosition_coordinate_frame
{
public:
  explicit Init_GlobalPosition_coordinate_frame(::ardupilot_msgs::msg::GlobalPosition & msg)
  : msg_(msg)
  {}
  Init_GlobalPosition_type_mask coordinate_frame(::ardupilot_msgs::msg::GlobalPosition::_coordinate_frame_type arg)
  {
    msg_.coordinate_frame = std::move(arg);
    return Init_GlobalPosition_type_mask(msg_);
  }

private:
  ::ardupilot_msgs::msg::GlobalPosition msg_;
};

class Init_GlobalPosition_header
{
public:
  Init_GlobalPosition_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GlobalPosition_coordinate_frame header(::ardupilot_msgs::msg::GlobalPosition::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GlobalPosition_coordinate_frame(msg_);
  }

private:
  ::ardupilot_msgs::msg::GlobalPosition msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ardupilot_msgs::msg::GlobalPosition>()
{
  return ardupilot_msgs::msg::builder::Init_GlobalPosition_header();
}

}  // namespace ardupilot_msgs

#endif  // ARDUPILOT_MSGS__MSG__DETAIL__GLOBAL_POSITION__BUILDER_HPP_
