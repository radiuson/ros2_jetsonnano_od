// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_vision_arm_control:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef ROS2_VISION_ARM_CONTROL__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_
#define ROS2_VISION_ARM_CONTROL__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_

#include "ros2_vision_arm_control/msg/detail/bounding_box__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ros2_vision_arm_control
{

namespace msg
{

namespace builder
{

class Init_BoundingBox_class_id
{
public:
  explicit Init_BoundingBox_class_id(::ros2_vision_arm_control::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  ::ros2_vision_arm_control::msg::BoundingBox class_id(::ros2_vision_arm_control::msg::BoundingBox::_class_id_type arg)
  {
    msg_.class_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_vision_arm_control::msg::BoundingBox msg_;
};

class Init_BoundingBox_confidence
{
public:
  explicit Init_BoundingBox_confidence(::ros2_vision_arm_control::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_class_id confidence(::ros2_vision_arm_control::msg::BoundingBox::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_BoundingBox_class_id(msg_);
  }

private:
  ::ros2_vision_arm_control::msg::BoundingBox msg_;
};

class Init_BoundingBox_ymax
{
public:
  explicit Init_BoundingBox_ymax(::ros2_vision_arm_control::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_confidence ymax(::ros2_vision_arm_control::msg::BoundingBox::_ymax_type arg)
  {
    msg_.ymax = std::move(arg);
    return Init_BoundingBox_confidence(msg_);
  }

private:
  ::ros2_vision_arm_control::msg::BoundingBox msg_;
};

class Init_BoundingBox_xmax
{
public:
  explicit Init_BoundingBox_xmax(::ros2_vision_arm_control::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_ymax xmax(::ros2_vision_arm_control::msg::BoundingBox::_xmax_type arg)
  {
    msg_.xmax = std::move(arg);
    return Init_BoundingBox_ymax(msg_);
  }

private:
  ::ros2_vision_arm_control::msg::BoundingBox msg_;
};

class Init_BoundingBox_ymin
{
public:
  explicit Init_BoundingBox_ymin(::ros2_vision_arm_control::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_xmax ymin(::ros2_vision_arm_control::msg::BoundingBox::_ymin_type arg)
  {
    msg_.ymin = std::move(arg);
    return Init_BoundingBox_xmax(msg_);
  }

private:
  ::ros2_vision_arm_control::msg::BoundingBox msg_;
};

class Init_BoundingBox_xmin
{
public:
  Init_BoundingBox_xmin()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoundingBox_ymin xmin(::ros2_vision_arm_control::msg::BoundingBox::_xmin_type arg)
  {
    msg_.xmin = std::move(arg);
    return Init_BoundingBox_ymin(msg_);
  }

private:
  ::ros2_vision_arm_control::msg::BoundingBox msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_vision_arm_control::msg::BoundingBox>()
{
  return ros2_vision_arm_control::msg::builder::Init_BoundingBox_xmin();
}

}  // namespace ros2_vision_arm_control

#endif  // ROS2_VISION_ARM_CONTROL__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_
