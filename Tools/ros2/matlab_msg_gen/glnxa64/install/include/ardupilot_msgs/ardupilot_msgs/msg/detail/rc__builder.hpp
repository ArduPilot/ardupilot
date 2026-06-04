// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ardupilot_msgs:msg/Rc.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__MSG__DETAIL__RC__BUILDER_HPP_
#define ARDUPILOT_MSGS__MSG__DETAIL__RC__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ardupilot_msgs/msg/detail/rc__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ardupilot_msgs
{

namespace msg
{

namespace builder
{

class Init_Rc_active_overrides
{
public:
  explicit Init_Rc_active_overrides(::ardupilot_msgs::msg::Rc & msg)
  : msg_(msg)
  {}
  ::ardupilot_msgs::msg::Rc active_overrides(::ardupilot_msgs::msg::Rc::_active_overrides_type arg)
  {
    msg_.active_overrides = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ardupilot_msgs::msg::Rc msg_;
};

class Init_Rc_channels
{
public:
  explicit Init_Rc_channels(::ardupilot_msgs::msg::Rc & msg)
  : msg_(msg)
  {}
  Init_Rc_active_overrides channels(::ardupilot_msgs::msg::Rc::_channels_type arg)
  {
    msg_.channels = std::move(arg);
    return Init_Rc_active_overrides(msg_);
  }

private:
  ::ardupilot_msgs::msg::Rc msg_;
};

class Init_Rc_receiver_rssi
{
public:
  explicit Init_Rc_receiver_rssi(::ardupilot_msgs::msg::Rc & msg)
  : msg_(msg)
  {}
  Init_Rc_channels receiver_rssi(::ardupilot_msgs::msg::Rc::_receiver_rssi_type arg)
  {
    msg_.receiver_rssi = std::move(arg);
    return Init_Rc_channels(msg_);
  }

private:
  ::ardupilot_msgs::msg::Rc msg_;
};

class Init_Rc_is_connected
{
public:
  explicit Init_Rc_is_connected(::ardupilot_msgs::msg::Rc & msg)
  : msg_(msg)
  {}
  Init_Rc_receiver_rssi is_connected(::ardupilot_msgs::msg::Rc::_is_connected_type arg)
  {
    msg_.is_connected = std::move(arg);
    return Init_Rc_receiver_rssi(msg_);
  }

private:
  ::ardupilot_msgs::msg::Rc msg_;
};

class Init_Rc_header
{
public:
  Init_Rc_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Rc_is_connected header(::ardupilot_msgs::msg::Rc::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Rc_is_connected(msg_);
  }

private:
  ::ardupilot_msgs::msg::Rc msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ardupilot_msgs::msg::Rc>()
{
  return ardupilot_msgs::msg::builder::Init_Rc_header();
}

}  // namespace ardupilot_msgs

#endif  // ARDUPILOT_MSGS__MSG__DETAIL__RC__BUILDER_HPP_
