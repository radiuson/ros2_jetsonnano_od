// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_vision_arm_control:msg/VisionDetection.idl
// generated code does not contain a copyright notice

#ifndef ROS2_VISION_ARM_CONTROL__MSG__DETAIL__VISION_DETECTION__TRAITS_HPP_
#define ROS2_VISION_ARM_CONTROL__MSG__DETAIL__VISION_DETECTION__TRAITS_HPP_

#include "ros2_vision_arm_control/msg/detail/vision_detection__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'rgb_image'
// Member 'depth_image'
#include "sensor_msgs/msg/detail/image__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2_vision_arm_control::msg::VisionDetection>()
{
  return "ros2_vision_arm_control::msg::VisionDetection";
}

template<>
inline const char * name<ros2_vision_arm_control::msg::VisionDetection>()
{
  return "ros2_vision_arm_control/msg/VisionDetection";
}

template<>
struct has_fixed_size<ros2_vision_arm_control::msg::VisionDetection>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2_vision_arm_control::msg::VisionDetection>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2_vision_arm_control::msg::VisionDetection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS2_VISION_ARM_CONTROL__MSG__DETAIL__VISION_DETECTION__TRAITS_HPP_
