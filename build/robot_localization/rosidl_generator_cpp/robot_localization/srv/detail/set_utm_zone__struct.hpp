// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_localization:srv/SetUTMZone.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__SET_UTM_ZONE__STRUCT_HPP_
#define ROBOT_LOCALIZATION__SRV__DETAIL__SET_UTM_ZONE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_localization__srv__SetUTMZone_Request __attribute__((deprecated))
#else
# define DEPRECATED__robot_localization__srv__SetUTMZone_Request __declspec(deprecated)
#endif

namespace robot_localization
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetUTMZone_Request_
{
  using Type = SetUTMZone_Request_<ContainerAllocator>;

  explicit SetUTMZone_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->utm_zone = "";
    }
  }

  explicit SetUTMZone_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : utm_zone(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->utm_zone = "";
    }
  }

  // field types and members
  using _utm_zone_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _utm_zone_type utm_zone;

  // setters for named parameter idiom
  Type & set__utm_zone(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->utm_zone = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_localization::srv::SetUTMZone_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_localization::srv::SetUTMZone_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_localization::srv::SetUTMZone_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_localization::srv::SetUTMZone_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_localization::srv::SetUTMZone_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_localization::srv::SetUTMZone_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_localization::srv::SetUTMZone_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_localization::srv::SetUTMZone_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_localization::srv::SetUTMZone_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_localization::srv::SetUTMZone_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_localization__srv__SetUTMZone_Request
    std::shared_ptr<robot_localization::srv::SetUTMZone_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_localization__srv__SetUTMZone_Request
    std::shared_ptr<robot_localization::srv::SetUTMZone_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetUTMZone_Request_ & other) const
  {
    if (this->utm_zone != other.utm_zone) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetUTMZone_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetUTMZone_Request_

// alias to use template instance with default allocator
using SetUTMZone_Request =
  robot_localization::srv::SetUTMZone_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robot_localization


#ifndef _WIN32
# define DEPRECATED__robot_localization__srv__SetUTMZone_Response __attribute__((deprecated))
#else
# define DEPRECATED__robot_localization__srv__SetUTMZone_Response __declspec(deprecated)
#endif

namespace robot_localization
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetUTMZone_Response_
{
  using Type = SetUTMZone_Response_<ContainerAllocator>;

  explicit SetUTMZone_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit SetUTMZone_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    robot_localization::srv::SetUTMZone_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_localization::srv::SetUTMZone_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_localization::srv::SetUTMZone_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_localization::srv::SetUTMZone_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_localization::srv::SetUTMZone_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_localization::srv::SetUTMZone_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_localization::srv::SetUTMZone_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_localization::srv::SetUTMZone_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_localization::srv::SetUTMZone_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_localization::srv::SetUTMZone_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_localization__srv__SetUTMZone_Response
    std::shared_ptr<robot_localization::srv::SetUTMZone_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_localization__srv__SetUTMZone_Response
    std::shared_ptr<robot_localization::srv::SetUTMZone_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetUTMZone_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetUTMZone_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetUTMZone_Response_

// alias to use template instance with default allocator
using SetUTMZone_Response =
  robot_localization::srv::SetUTMZone_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robot_localization

namespace robot_localization
{

namespace srv
{

struct SetUTMZone
{
  using Request = robot_localization::srv::SetUTMZone_Request;
  using Response = robot_localization::srv::SetUTMZone_Response;
};

}  // namespace srv

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__SET_UTM_ZONE__STRUCT_HPP_
