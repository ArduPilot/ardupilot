// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ardupilot_msgs:msg/GlobalPosition.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__MSG__DETAIL__GLOBAL_POSITION__STRUCT_HPP_
#define ARDUPILOT_MSGS__MSG__DETAIL__GLOBAL_POSITION__STRUCT_HPP_

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
// Member 'velocity'
// Member 'acceleration_or_force'
#include "geometry_msgs/msg/detail/twist__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ardupilot_msgs__msg__GlobalPosition __attribute__((deprecated))
#else
# define DEPRECATED__ardupilot_msgs__msg__GlobalPosition __declspec(deprecated)
#endif

namespace ardupilot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GlobalPosition_
{
  using Type = GlobalPosition_<ContainerAllocator>;

  explicit GlobalPosition_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    velocity(_init),
    acceleration_or_force(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->coordinate_frame = 0;
      this->type_mask = 0;
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0f;
      this->yaw = 0.0f;
    }
  }

  explicit GlobalPosition_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    velocity(_alloc, _init),
    acceleration_or_force(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->coordinate_frame = 0;
      this->type_mask = 0;
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0f;
      this->yaw = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _coordinate_frame_type =
    uint8_t;
  _coordinate_frame_type coordinate_frame;
  using _type_mask_type =
    uint16_t;
  _type_mask_type type_mask;
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;
  using _altitude_type =
    float;
  _altitude_type altitude;
  using _velocity_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _velocity_type velocity;
  using _acceleration_or_force_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _acceleration_or_force_type acceleration_or_force;
  using _yaw_type =
    float;
  _yaw_type yaw;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__coordinate_frame(
    const uint8_t & _arg)
  {
    this->coordinate_frame = _arg;
    return *this;
  }
  Type & set__type_mask(
    const uint16_t & _arg)
  {
    this->type_mask = _arg;
    return *this;
  }
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__altitude(
    const float & _arg)
  {
    this->altitude = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__acceleration_or_force(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->acceleration_or_force = _arg;
    return *this;
  }
  Type & set__yaw(
    const float & _arg)
  {
    this->yaw = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t FRAME_GLOBAL_INT =
    5u;
  static constexpr uint8_t FRAME_GLOBAL_REL_ALT =
    6u;
  static constexpr uint8_t FRAME_GLOBAL_TERRAIN_ALT =
    11u;
  static constexpr uint16_t IGNORE_LATITUDE =
    1u;
  static constexpr uint16_t IGNORE_LONGITUDE =
    2u;
  static constexpr uint16_t IGNORE_ALTITUDE =
    4u;
  static constexpr uint16_t IGNORE_VX =
    8u;
  static constexpr uint16_t IGNORE_VY =
    16u;
  static constexpr uint16_t IGNORE_VZ =
    32u;
  static constexpr uint16_t IGNORE_AFX =
    64u;
  static constexpr uint16_t IGNORE_AFY =
    128u;
  static constexpr uint16_t IGNORE_AFZ =
    256u;
  static constexpr uint16_t FORCE =
    512u;
  static constexpr uint16_t IGNORE_YAW =
    1024u;
  static constexpr uint16_t IGNORE_YAW_RATE =
    2048u;

  // pointer types
  using RawPtr =
    ardupilot_msgs::msg::GlobalPosition_<ContainerAllocator> *;
  using ConstRawPtr =
    const ardupilot_msgs::msg::GlobalPosition_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ardupilot_msgs::msg::GlobalPosition_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ardupilot_msgs::msg::GlobalPosition_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ardupilot_msgs::msg::GlobalPosition_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ardupilot_msgs::msg::GlobalPosition_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ardupilot_msgs::msg::GlobalPosition_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ardupilot_msgs::msg::GlobalPosition_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ardupilot_msgs::msg::GlobalPosition_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ardupilot_msgs::msg::GlobalPosition_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ardupilot_msgs__msg__GlobalPosition
    std::shared_ptr<ardupilot_msgs::msg::GlobalPosition_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ardupilot_msgs__msg__GlobalPosition
    std::shared_ptr<ardupilot_msgs::msg::GlobalPosition_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GlobalPosition_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->coordinate_frame != other.coordinate_frame) {
      return false;
    }
    if (this->type_mask != other.type_mask) {
      return false;
    }
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->acceleration_or_force != other.acceleration_or_force) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const GlobalPosition_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GlobalPosition_

// alias to use template instance with default allocator
using GlobalPosition =
  ardupilot_msgs::msg::GlobalPosition_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t GlobalPosition_<ContainerAllocator>::FRAME_GLOBAL_INT;
template<typename ContainerAllocator>
constexpr uint8_t GlobalPosition_<ContainerAllocator>::FRAME_GLOBAL_REL_ALT;
template<typename ContainerAllocator>
constexpr uint8_t GlobalPosition_<ContainerAllocator>::FRAME_GLOBAL_TERRAIN_ALT;
template<typename ContainerAllocator>
constexpr uint16_t GlobalPosition_<ContainerAllocator>::IGNORE_LATITUDE;
template<typename ContainerAllocator>
constexpr uint16_t GlobalPosition_<ContainerAllocator>::IGNORE_LONGITUDE;
template<typename ContainerAllocator>
constexpr uint16_t GlobalPosition_<ContainerAllocator>::IGNORE_ALTITUDE;
template<typename ContainerAllocator>
constexpr uint16_t GlobalPosition_<ContainerAllocator>::IGNORE_VX;
template<typename ContainerAllocator>
constexpr uint16_t GlobalPosition_<ContainerAllocator>::IGNORE_VY;
template<typename ContainerAllocator>
constexpr uint16_t GlobalPosition_<ContainerAllocator>::IGNORE_VZ;
template<typename ContainerAllocator>
constexpr uint16_t GlobalPosition_<ContainerAllocator>::IGNORE_AFX;
template<typename ContainerAllocator>
constexpr uint16_t GlobalPosition_<ContainerAllocator>::IGNORE_AFY;
template<typename ContainerAllocator>
constexpr uint16_t GlobalPosition_<ContainerAllocator>::IGNORE_AFZ;
template<typename ContainerAllocator>
constexpr uint16_t GlobalPosition_<ContainerAllocator>::FORCE;
template<typename ContainerAllocator>
constexpr uint16_t GlobalPosition_<ContainerAllocator>::IGNORE_YAW;
template<typename ContainerAllocator>
constexpr uint16_t GlobalPosition_<ContainerAllocator>::IGNORE_YAW_RATE;

}  // namespace msg

}  // namespace ardupilot_msgs

#endif  // ARDUPILOT_MSGS__MSG__DETAIL__GLOBAL_POSITION__STRUCT_HPP_
