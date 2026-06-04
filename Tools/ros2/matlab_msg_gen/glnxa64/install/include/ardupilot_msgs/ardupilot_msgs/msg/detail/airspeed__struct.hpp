// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ardupilot_msgs:msg/Airspeed.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__MSG__DETAIL__AIRSPEED__STRUCT_HPP_
#define ARDUPILOT_MSGS__MSG__DETAIL__AIRSPEED__STRUCT_HPP_

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
// Member 'true_airspeed'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ardupilot_msgs__msg__Airspeed __attribute__((deprecated))
#else
# define DEPRECATED__ardupilot_msgs__msg__Airspeed __declspec(deprecated)
#endif

namespace ardupilot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Airspeed_
{
  using Type = Airspeed_<ContainerAllocator>;

  explicit Airspeed_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    true_airspeed(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->eas_2_tas = 0.0f;
    }
  }

  explicit Airspeed_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    true_airspeed(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->eas_2_tas = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _true_airspeed_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _true_airspeed_type true_airspeed;
  using _eas_2_tas_type =
    float;
  _eas_2_tas_type eas_2_tas;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__true_airspeed(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->true_airspeed = _arg;
    return *this;
  }
  Type & set__eas_2_tas(
    const float & _arg)
  {
    this->eas_2_tas = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ardupilot_msgs::msg::Airspeed_<ContainerAllocator> *;
  using ConstRawPtr =
    const ardupilot_msgs::msg::Airspeed_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ardupilot_msgs::msg::Airspeed_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ardupilot_msgs::msg::Airspeed_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ardupilot_msgs::msg::Airspeed_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ardupilot_msgs::msg::Airspeed_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ardupilot_msgs::msg::Airspeed_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ardupilot_msgs::msg::Airspeed_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ardupilot_msgs::msg::Airspeed_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ardupilot_msgs::msg::Airspeed_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ardupilot_msgs__msg__Airspeed
    std::shared_ptr<ardupilot_msgs::msg::Airspeed_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ardupilot_msgs__msg__Airspeed
    std::shared_ptr<ardupilot_msgs::msg::Airspeed_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Airspeed_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->true_airspeed != other.true_airspeed) {
      return false;
    }
    if (this->eas_2_tas != other.eas_2_tas) {
      return false;
    }
    return true;
  }
  bool operator!=(const Airspeed_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Airspeed_

// alias to use template instance with default allocator
using Airspeed =
  ardupilot_msgs::msg::Airspeed_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ardupilot_msgs

#endif  // ARDUPILOT_MSGS__MSG__DETAIL__AIRSPEED__STRUCT_HPP_
