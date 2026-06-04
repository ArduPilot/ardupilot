// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ardupilot_msgs:msg/Rc.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__MSG__DETAIL__RC__STRUCT_HPP_
#define ARDUPILOT_MSGS__MSG__DETAIL__RC__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ardupilot_msgs__msg__Rc __attribute__((deprecated))
#else
# define DEPRECATED__ardupilot_msgs__msg__Rc __declspec(deprecated)
#endif

namespace ardupilot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Rc_
{
  using Type = Rc_<ContainerAllocator>;

  explicit Rc_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_connected = false;
      this->receiver_rssi = 0;
    }
  }

  explicit Rc_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_connected = false;
      this->receiver_rssi = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _is_connected_type =
    bool;
  _is_connected_type is_connected;
  using _receiver_rssi_type =
    uint8_t;
  _receiver_rssi_type receiver_rssi;
  using _channels_type =
    rosidl_runtime_cpp::BoundedVector<int16_t, 32, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int16_t>>;
  _channels_type channels;
  using _active_overrides_type =
    rosidl_runtime_cpp::BoundedVector<bool, 32, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _active_overrides_type active_overrides;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__is_connected(
    const bool & _arg)
  {
    this->is_connected = _arg;
    return *this;
  }
  Type & set__receiver_rssi(
    const uint8_t & _arg)
  {
    this->receiver_rssi = _arg;
    return *this;
  }
  Type & set__channels(
    const rosidl_runtime_cpp::BoundedVector<int16_t, 32, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int16_t>> & _arg)
  {
    this->channels = _arg;
    return *this;
  }
  Type & set__active_overrides(
    const rosidl_runtime_cpp::BoundedVector<bool, 32, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->active_overrides = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ardupilot_msgs::msg::Rc_<ContainerAllocator> *;
  using ConstRawPtr =
    const ardupilot_msgs::msg::Rc_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ardupilot_msgs::msg::Rc_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ardupilot_msgs::msg::Rc_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ardupilot_msgs::msg::Rc_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ardupilot_msgs::msg::Rc_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ardupilot_msgs::msg::Rc_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ardupilot_msgs::msg::Rc_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ardupilot_msgs::msg::Rc_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ardupilot_msgs::msg::Rc_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ardupilot_msgs__msg__Rc
    std::shared_ptr<ardupilot_msgs::msg::Rc_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ardupilot_msgs__msg__Rc
    std::shared_ptr<ardupilot_msgs::msg::Rc_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Rc_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->is_connected != other.is_connected) {
      return false;
    }
    if (this->receiver_rssi != other.receiver_rssi) {
      return false;
    }
    if (this->channels != other.channels) {
      return false;
    }
    if (this->active_overrides != other.active_overrides) {
      return false;
    }
    return true;
  }
  bool operator!=(const Rc_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Rc_

// alias to use template instance with default allocator
using Rc =
  ardupilot_msgs::msg::Rc_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ardupilot_msgs

#endif  // ARDUPILOT_MSGS__MSG__DETAIL__RC__STRUCT_HPP_
