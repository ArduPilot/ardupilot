// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ardupilot_msgs:srv/Takeoff.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__SRV__DETAIL__TAKEOFF__STRUCT_HPP_
#define ARDUPILOT_MSGS__SRV__DETAIL__TAKEOFF__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ardupilot_msgs__srv__Takeoff_Request __attribute__((deprecated))
#else
# define DEPRECATED__ardupilot_msgs__srv__Takeoff_Request __declspec(deprecated)
#endif

namespace ardupilot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Takeoff_Request_
{
  using Type = Takeoff_Request_<ContainerAllocator>;

  explicit Takeoff_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->alt = 0.0f;
    }
  }

  explicit Takeoff_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->alt = 0.0f;
    }
  }

  // field types and members
  using _alt_type =
    float;
  _alt_type alt;

  // setters for named parameter idiom
  Type & set__alt(
    const float & _arg)
  {
    this->alt = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ardupilot_msgs::srv::Takeoff_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ardupilot_msgs::srv::Takeoff_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ardupilot_msgs::srv::Takeoff_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ardupilot_msgs::srv::Takeoff_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ardupilot_msgs::srv::Takeoff_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ardupilot_msgs::srv::Takeoff_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ardupilot_msgs::srv::Takeoff_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ardupilot_msgs::srv::Takeoff_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ardupilot_msgs::srv::Takeoff_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ardupilot_msgs::srv::Takeoff_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ardupilot_msgs__srv__Takeoff_Request
    std::shared_ptr<ardupilot_msgs::srv::Takeoff_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ardupilot_msgs__srv__Takeoff_Request
    std::shared_ptr<ardupilot_msgs::srv::Takeoff_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Takeoff_Request_ & other) const
  {
    if (this->alt != other.alt) {
      return false;
    }
    return true;
  }
  bool operator!=(const Takeoff_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Takeoff_Request_

// alias to use template instance with default allocator
using Takeoff_Request =
  ardupilot_msgs::srv::Takeoff_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ardupilot_msgs


#ifndef _WIN32
# define DEPRECATED__ardupilot_msgs__srv__Takeoff_Response __attribute__((deprecated))
#else
# define DEPRECATED__ardupilot_msgs__srv__Takeoff_Response __declspec(deprecated)
#endif

namespace ardupilot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Takeoff_Response_
{
  using Type = Takeoff_Response_<ContainerAllocator>;

  explicit Takeoff_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = false;
    }
  }

  explicit Takeoff_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = false;
    }
  }

  // field types and members
  using _status_type =
    bool;
  _status_type status;

  // setters for named parameter idiom
  Type & set__status(
    const bool & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ardupilot_msgs::srv::Takeoff_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ardupilot_msgs::srv::Takeoff_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ardupilot_msgs::srv::Takeoff_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ardupilot_msgs::srv::Takeoff_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ardupilot_msgs::srv::Takeoff_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ardupilot_msgs::srv::Takeoff_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ardupilot_msgs::srv::Takeoff_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ardupilot_msgs::srv::Takeoff_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ardupilot_msgs::srv::Takeoff_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ardupilot_msgs::srv::Takeoff_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ardupilot_msgs__srv__Takeoff_Response
    std::shared_ptr<ardupilot_msgs::srv::Takeoff_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ardupilot_msgs__srv__Takeoff_Response
    std::shared_ptr<ardupilot_msgs::srv::Takeoff_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Takeoff_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const Takeoff_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Takeoff_Response_

// alias to use template instance with default allocator
using Takeoff_Response =
  ardupilot_msgs::srv::Takeoff_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ardupilot_msgs

namespace ardupilot_msgs
{

namespace srv
{

struct Takeoff
{
  using Request = ardupilot_msgs::srv::Takeoff_Request;
  using Response = ardupilot_msgs::srv::Takeoff_Response;
};

}  // namespace srv

}  // namespace ardupilot_msgs

#endif  // ARDUPILOT_MSGS__SRV__DETAIL__TAKEOFF__STRUCT_HPP_
