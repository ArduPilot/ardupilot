// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ardupilot_msgs:srv/ModeSwitch.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__SRV__DETAIL__MODE_SWITCH__STRUCT_HPP_
#define ARDUPILOT_MSGS__SRV__DETAIL__MODE_SWITCH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ardupilot_msgs__srv__ModeSwitch_Request __attribute__((deprecated))
#else
# define DEPRECATED__ardupilot_msgs__srv__ModeSwitch_Request __declspec(deprecated)
#endif

namespace ardupilot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ModeSwitch_Request_
{
  using Type = ModeSwitch_Request_<ContainerAllocator>;

  explicit ModeSwitch_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
    }
  }

  explicit ModeSwitch_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
    }
  }

  // field types and members
  using _mode_type =
    uint8_t;
  _mode_type mode;

  // setters for named parameter idiom
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ardupilot_msgs::srv::ModeSwitch_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ardupilot_msgs::srv::ModeSwitch_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ardupilot_msgs::srv::ModeSwitch_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ardupilot_msgs::srv::ModeSwitch_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ardupilot_msgs::srv::ModeSwitch_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ardupilot_msgs::srv::ModeSwitch_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ardupilot_msgs::srv::ModeSwitch_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ardupilot_msgs::srv::ModeSwitch_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ardupilot_msgs::srv::ModeSwitch_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ardupilot_msgs::srv::ModeSwitch_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ardupilot_msgs__srv__ModeSwitch_Request
    std::shared_ptr<ardupilot_msgs::srv::ModeSwitch_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ardupilot_msgs__srv__ModeSwitch_Request
    std::shared_ptr<ardupilot_msgs::srv::ModeSwitch_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ModeSwitch_Request_ & other) const
  {
    if (this->mode != other.mode) {
      return false;
    }
    return true;
  }
  bool operator!=(const ModeSwitch_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ModeSwitch_Request_

// alias to use template instance with default allocator
using ModeSwitch_Request =
  ardupilot_msgs::srv::ModeSwitch_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ardupilot_msgs


#ifndef _WIN32
# define DEPRECATED__ardupilot_msgs__srv__ModeSwitch_Response __attribute__((deprecated))
#else
# define DEPRECATED__ardupilot_msgs__srv__ModeSwitch_Response __declspec(deprecated)
#endif

namespace ardupilot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ModeSwitch_Response_
{
  using Type = ModeSwitch_Response_<ContainerAllocator>;

  explicit ModeSwitch_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = false;
      this->curr_mode = 0;
    }
  }

  explicit ModeSwitch_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = false;
      this->curr_mode = 0;
    }
  }

  // field types and members
  using _status_type =
    bool;
  _status_type status;
  using _curr_mode_type =
    uint8_t;
  _curr_mode_type curr_mode;

  // setters for named parameter idiom
  Type & set__status(
    const bool & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__curr_mode(
    const uint8_t & _arg)
  {
    this->curr_mode = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ardupilot_msgs::srv::ModeSwitch_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ardupilot_msgs::srv::ModeSwitch_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ardupilot_msgs::srv::ModeSwitch_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ardupilot_msgs::srv::ModeSwitch_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ardupilot_msgs::srv::ModeSwitch_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ardupilot_msgs::srv::ModeSwitch_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ardupilot_msgs::srv::ModeSwitch_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ardupilot_msgs::srv::ModeSwitch_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ardupilot_msgs::srv::ModeSwitch_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ardupilot_msgs::srv::ModeSwitch_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ardupilot_msgs__srv__ModeSwitch_Response
    std::shared_ptr<ardupilot_msgs::srv::ModeSwitch_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ardupilot_msgs__srv__ModeSwitch_Response
    std::shared_ptr<ardupilot_msgs::srv::ModeSwitch_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ModeSwitch_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->curr_mode != other.curr_mode) {
      return false;
    }
    return true;
  }
  bool operator!=(const ModeSwitch_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ModeSwitch_Response_

// alias to use template instance with default allocator
using ModeSwitch_Response =
  ardupilot_msgs::srv::ModeSwitch_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ardupilot_msgs

namespace ardupilot_msgs
{

namespace srv
{

struct ModeSwitch
{
  using Request = ardupilot_msgs::srv::ModeSwitch_Request;
  using Response = ardupilot_msgs::srv::ModeSwitch_Response;
};

}  // namespace srv

}  // namespace ardupilot_msgs

#endif  // ARDUPILOT_MSGS__SRV__DETAIL__MODE_SWITCH__STRUCT_HPP_
