// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ardupilot_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__MSG__DETAIL__STATUS__STRUCT_HPP_
#define ARDUPILOT_MSGS__MSG__DETAIL__STATUS__STRUCT_HPP_

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
# define DEPRECATED__ardupilot_msgs__msg__Status __attribute__((deprecated))
#else
# define DEPRECATED__ardupilot_msgs__msg__Status __declspec(deprecated)
#endif

namespace ardupilot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Status_
{
  using Type = Status_<ContainerAllocator>;

  explicit Status_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vehicle_type = 0;
      this->armed = false;
      this->mode = 0;
      this->flying = false;
      this->external_control = false;
    }
  }

  explicit Status_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vehicle_type = 0;
      this->armed = false;
      this->mode = 0;
      this->flying = false;
      this->external_control = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _vehicle_type_type =
    uint8_t;
  _vehicle_type_type vehicle_type;
  using _armed_type =
    bool;
  _armed_type armed;
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _flying_type =
    bool;
  _flying_type flying;
  using _external_control_type =
    bool;
  _external_control_type external_control;
  using _failsafe_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _failsafe_type failsafe;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__vehicle_type(
    const uint8_t & _arg)
  {
    this->vehicle_type = _arg;
    return *this;
  }
  Type & set__armed(
    const bool & _arg)
  {
    this->armed = _arg;
    return *this;
  }
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__flying(
    const bool & _arg)
  {
    this->flying = _arg;
    return *this;
  }
  Type & set__external_control(
    const bool & _arg)
  {
    this->external_control = _arg;
    return *this;
  }
  Type & set__failsafe(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->failsafe = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t APM_ROVER =
    1u;
  static constexpr uint8_t APM_ARDUCOPTER =
    2u;
  static constexpr uint8_t APM_ARDUPLANE =
    3u;
  static constexpr uint8_t APM_ANTENNATRACKER =
    4u;
  static constexpr uint8_t APM_UNKNOWN =
    5u;
  static constexpr uint8_t APM_REPLAY =
    6u;
  static constexpr uint8_t APM_ARDUSUB =
    7u;
  static constexpr uint8_t APM_IOFIRMWARE =
    8u;
  static constexpr uint8_t APM_AP_PERIPH =
    9u;
  static constexpr uint8_t APM_AP_DAL_STANDALONE =
    10u;
  static constexpr uint8_t APM_AP_BOOTLOADER =
    11u;
  static constexpr uint8_t APM_BLIMP =
    12u;
  static constexpr uint8_t APM_HELI =
    13u;
  static constexpr uint8_t FS_RADIO =
    21u;
  static constexpr uint8_t FS_BATTERY =
    22u;
  static constexpr uint8_t FS_GCS =
    23u;
  static constexpr uint8_t FS_EKF =
    24u;

  // pointer types
  using RawPtr =
    ardupilot_msgs::msg::Status_<ContainerAllocator> *;
  using ConstRawPtr =
    const ardupilot_msgs::msg::Status_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ardupilot_msgs::msg::Status_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ardupilot_msgs::msg::Status_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ardupilot_msgs::msg::Status_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ardupilot_msgs::msg::Status_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ardupilot_msgs::msg::Status_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ardupilot_msgs::msg::Status_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ardupilot_msgs::msg::Status_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ardupilot_msgs::msg::Status_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ardupilot_msgs__msg__Status
    std::shared_ptr<ardupilot_msgs::msg::Status_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ardupilot_msgs__msg__Status
    std::shared_ptr<ardupilot_msgs::msg::Status_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Status_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->vehicle_type != other.vehicle_type) {
      return false;
    }
    if (this->armed != other.armed) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->flying != other.flying) {
      return false;
    }
    if (this->external_control != other.external_control) {
      return false;
    }
    if (this->failsafe != other.failsafe) {
      return false;
    }
    return true;
  }
  bool operator!=(const Status_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Status_

// alias to use template instance with default allocator
using Status =
  ardupilot_msgs::msg::Status_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::APM_ROVER;
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::APM_ARDUCOPTER;
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::APM_ARDUPLANE;
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::APM_ANTENNATRACKER;
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::APM_UNKNOWN;
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::APM_REPLAY;
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::APM_ARDUSUB;
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::APM_IOFIRMWARE;
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::APM_AP_PERIPH;
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::APM_AP_DAL_STANDALONE;
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::APM_AP_BOOTLOADER;
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::APM_BLIMP;
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::APM_HELI;
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::FS_RADIO;
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::FS_BATTERY;
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::FS_GCS;
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::FS_EKF;

}  // namespace msg

}  // namespace ardupilot_msgs

#endif  // ARDUPILOT_MSGS__MSG__DETAIL__STATUS__STRUCT_HPP_
