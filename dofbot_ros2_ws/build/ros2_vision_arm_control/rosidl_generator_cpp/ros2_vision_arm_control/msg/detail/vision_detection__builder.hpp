// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_vision_arm_control:msg/VisionDetection.idl
// generated code does not contain a copyright notice

#ifndef ROS2_VISION_ARM_CONTROL__MSG__DETAIL__VISION_DETECTION__BUILDER_HPP_
#define ROS2_VISION_ARM_CONTROL__MSG__DETAIL__VISION_DETECTION__BUILDER_HPP_

#include "ros2_vision_arm_control/msg/detail/vision_detection__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ros2_vision_arm_control
{

namespace msg
{

namespace builder
{

class Init_VisionDetection_boxes
{
public:
  explicit Init_VisionDetection_boxes(::ros2_vision_arm_control::msg::VisionDetection & msg)
  : msg_(msg)
  {}
  ::ros2_vision_arm_control::msg::VisionDetection boxes(::ros2_vision_arm_control::msg::VisionDetection::_boxes_type arg)
  {
    msg_.boxes = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_vision_arm_control::msg::VisionDetection msg_;
};

class Init_VisionDetection_depth_image
{
public:
  explicit Init_VisionDetection_depth_image(::ros2_vision_arm_control::msg::VisionDetection & msg)
  : msg_(msg)
  {}
  Init_VisionDetection_boxes depth_image(::ros2_vision_arm_control::msg::VisionDetection::_depth_image_type arg)
  {
    msg_.depth_image = std::move(arg);
    return Init_VisionDetection_boxes(msg_);
  }

private:
  ::ros2_vision_arm_control::msg::VisionDetection msg_;
};

class Init_VisionDetection_rgb_image
{
public:
  Init_VisionDetection_rgb_image()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VisionDetection_depth_image rgb_image(::ros2_vision_arm_control::msg::VisionDetection::_rgb_image_type arg)
  {
    msg_.rgb_image = std::move(arg);
    return Init_VisionDetection_depth_image(msg_);
  }

private:
  ::ros2_vision_arm_control::msg::VisionDetection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_vision_arm_control::msg::VisionDetection>()
{
  return ros2_vision_arm_control::msg::builder::Init_VisionDetection_rgb_image();
}

}  // namespace ros2_vision_arm_control

#endif  // ROS2_VISION_ARM_CONTROL__MSG__DETAIL__VISION_DETECTION__BUILDER_HPP_
