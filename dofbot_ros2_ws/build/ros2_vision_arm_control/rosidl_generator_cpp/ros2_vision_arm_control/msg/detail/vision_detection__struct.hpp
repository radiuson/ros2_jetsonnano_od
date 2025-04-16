// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2_vision_arm_control:msg/VisionDetection.idl
// generated code does not contain a copyright notice

#ifndef ROS2_VISION_ARM_CONTROL__MSG__DETAIL__VISION_DETECTION__STRUCT_HPP_
#define ROS2_VISION_ARM_CONTROL__MSG__DETAIL__VISION_DETECTION__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'rgb_image'
// Member 'depth_image'
#include "sensor_msgs/msg/detail/image__struct.hpp"
// Member 'boxes'
#include "ros2_vision_arm_control/msg/detail/bounding_box__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros2_vision_arm_control__msg__VisionDetection __attribute__((deprecated))
#else
# define DEPRECATED__ros2_vision_arm_control__msg__VisionDetection __declspec(deprecated)
#endif

namespace ros2_vision_arm_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VisionDetection_
{
  using Type = VisionDetection_<ContainerAllocator>;

  explicit VisionDetection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : rgb_image(_init),
    depth_image(_init)
  {
    (void)_init;
  }

  explicit VisionDetection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : rgb_image(_alloc, _init),
    depth_image(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _rgb_image_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _rgb_image_type rgb_image;
  using _depth_image_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _depth_image_type depth_image;
  using _boxes_type =
    std::vector<ros2_vision_arm_control::msg::BoundingBox_<ContainerAllocator>, typename ContainerAllocator::template rebind<ros2_vision_arm_control::msg::BoundingBox_<ContainerAllocator>>::other>;
  _boxes_type boxes;

  // setters for named parameter idiom
  Type & set__rgb_image(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->rgb_image = _arg;
    return *this;
  }
  Type & set__depth_image(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->depth_image = _arg;
    return *this;
  }
  Type & set__boxes(
    const std::vector<ros2_vision_arm_control::msg::BoundingBox_<ContainerAllocator>, typename ContainerAllocator::template rebind<ros2_vision_arm_control::msg::BoundingBox_<ContainerAllocator>>::other> & _arg)
  {
    this->boxes = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2_vision_arm_control::msg::VisionDetection_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_vision_arm_control::msg::VisionDetection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_vision_arm_control::msg::VisionDetection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_vision_arm_control::msg::VisionDetection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_vision_arm_control::msg::VisionDetection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_vision_arm_control::msg::VisionDetection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_vision_arm_control::msg::VisionDetection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_vision_arm_control::msg::VisionDetection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_vision_arm_control::msg::VisionDetection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_vision_arm_control::msg::VisionDetection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_vision_arm_control__msg__VisionDetection
    std::shared_ptr<ros2_vision_arm_control::msg::VisionDetection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_vision_arm_control__msg__VisionDetection
    std::shared_ptr<ros2_vision_arm_control::msg::VisionDetection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VisionDetection_ & other) const
  {
    if (this->rgb_image != other.rgb_image) {
      return false;
    }
    if (this->depth_image != other.depth_image) {
      return false;
    }
    if (this->boxes != other.boxes) {
      return false;
    }
    return true;
  }
  bool operator!=(const VisionDetection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VisionDetection_

// alias to use template instance with default allocator
using VisionDetection =
  ros2_vision_arm_control::msg::VisionDetection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros2_vision_arm_control

#endif  // ROS2_VISION_ARM_CONTROL__MSG__DETAIL__VISION_DETECTION__STRUCT_HPP_
