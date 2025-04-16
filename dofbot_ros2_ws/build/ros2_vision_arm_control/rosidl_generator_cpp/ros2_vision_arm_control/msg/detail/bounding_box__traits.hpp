// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_vision_arm_control:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef ROS2_VISION_ARM_CONTROL__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_
#define ROS2_VISION_ARM_CONTROL__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_

#include "ros2_vision_arm_control/msg/detail/bounding_box__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2_vision_arm_control::msg::BoundingBox>()
{
  return "ros2_vision_arm_control::msg::BoundingBox";
}

template<>
inline const char * name<ros2_vision_arm_control::msg::BoundingBox>()
{
  return "ros2_vision_arm_control/msg/BoundingBox";
}

template<>
struct has_fixed_size<ros2_vision_arm_control::msg::BoundingBox>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros2_vision_arm_control::msg::BoundingBox>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros2_vision_arm_control::msg::BoundingBox>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS2_VISION_ARM_CONTROL__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_
