// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ardupilot_msgs:srv/ArmMotors.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__SRV__DETAIL__ARM_MOTORS__STRUCT_HPP_
#define ARDUPILOT_MSGS__SRV__DETAIL__ARM_MOTORS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ardupilot_msgs__srv__ArmMotors_Request __attribute__((deprecated))
#else
# define DEPRECATED__ardupilot_msgs__srv__ArmMotors_Request __declspec(deprecated)
#endif

namespace ardupilot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ArmMotors_Request_
{
  using Type = ArmMotors_Request_<ContainerAllocator>;

  explicit ArmMotors_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->arm = false;
    }
  }

  explicit ArmMotors_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->arm = false;
    }
  }

  // field types and members
  using _arm_type =
    bool;
  _arm_type arm;

  // setters for named parameter idiom
  Type & set__arm(
    const bool & _arg)
  {
    this->arm = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ardupilot_msgs::srv::ArmMotors_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ardupilot_msgs::srv::ArmMotors_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ardupilot_msgs::srv::ArmMotors_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ardupilot_msgs::srv::ArmMotors_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ardupilot_msgs::srv::ArmMotors_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ardupilot_msgs::srv::ArmMotors_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ardupilot_msgs::srv::ArmMotors_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ardupilot_msgs::srv::ArmMotors_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ardupilot_msgs::srv::ArmMotors_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ardupilot_msgs::srv::ArmMotors_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ardupilot_msgs__srv__ArmMotors_Request
    std::shared_ptr<ardupilot_msgs::srv::ArmMotors_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ardupilot_msgs__srv__ArmMotors_Request
    std::shared_ptr<ardupilot_msgs::srv::ArmMotors_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ArmMotors_Request_ & other) const
  {
    if (this->arm != other.arm) {
      return false;
    }
    return true;
  }
  bool operator!=(const ArmMotors_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ArmMotors_Request_

// alias to use template instance with default allocator
using ArmMotors_Request =
  ardupilot_msgs::srv::ArmMotors_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ardupilot_msgs


#ifndef _WIN32
# define DEPRECATED__ardupilot_msgs__srv__ArmMotors_Response __attribute__((deprecated))
#else
# define DEPRECATED__ardupilot_msgs__srv__ArmMotors_Response __declspec(deprecated)
#endif

namespace ardupilot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ArmMotors_Response_
{
  using Type = ArmMotors_Response_<ContainerAllocator>;

  explicit ArmMotors_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = false;
    }
  }

  explicit ArmMotors_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = false;
    }
  }

  // field types and members
  using _result_type =
    bool;
  _result_type result;

  // setters for named parameter idiom
  Type & set__result(
    const bool & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ardupilot_msgs::srv::ArmMotors_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ardupilot_msgs::srv::ArmMotors_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ardupilot_msgs::srv::ArmMotors_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ardupilot_msgs::srv::ArmMotors_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ardupilot_msgs::srv::ArmMotors_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ardupilot_msgs::srv::ArmMotors_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ardupilot_msgs::srv::ArmMotors_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ardupilot_msgs::srv::ArmMotors_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ardupilot_msgs::srv::ArmMotors_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ardupilot_msgs::srv::ArmMotors_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ardupilot_msgs__srv__ArmMotors_Response
    std::shared_ptr<ardupilot_msgs::srv::ArmMotors_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ardupilot_msgs__srv__ArmMotors_Response
    std::shared_ptr<ardupilot_msgs::srv::ArmMotors_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ArmMotors_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const ArmMotors_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ArmMotors_Response_

// alias to use template instance with default allocator
using ArmMotors_Response =
  ardupilot_msgs::srv::ArmMotors_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ardupilot_msgs

namespace ardupilot_msgs
{

namespace srv
{

struct ArmMotors
{
  using Request = ardupilot_msgs::srv::ArmMotors_Request;
  using Response = ardupilot_msgs::srv::ArmMotors_Response;
};

}  // namespace srv

}  // namespace ardupilot_msgs

#endif  // ARDUPILOT_MSGS__SRV__DETAIL__ARM_MOTORS__STRUCT_HPP_
