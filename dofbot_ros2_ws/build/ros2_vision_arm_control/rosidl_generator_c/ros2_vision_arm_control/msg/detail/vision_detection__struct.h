// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_vision_arm_control:msg/VisionDetection.idl
// generated code does not contain a copyright notice

#ifndef ROS2_VISION_ARM_CONTROL__MSG__DETAIL__VISION_DETECTION__STRUCT_H_
#define ROS2_VISION_ARM_CONTROL__MSG__DETAIL__VISION_DETECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'rgb_image'
// Member 'depth_image'
#include "sensor_msgs/msg/detail/image__struct.h"
// Member 'boxes'
#include "ros2_vision_arm_control/msg/detail/bounding_box__struct.h"

// Struct defined in msg/VisionDetection in the package ros2_vision_arm_control.
typedef struct ros2_vision_arm_control__msg__VisionDetection
{
  sensor_msgs__msg__Image rgb_image;
  sensor_msgs__msg__Image depth_image;
  ros2_vision_arm_control__msg__BoundingBox__Sequence boxes;
} ros2_vision_arm_control__msg__VisionDetection;

// Struct for a sequence of ros2_vision_arm_control__msg__VisionDetection.
typedef struct ros2_vision_arm_control__msg__VisionDetection__Sequence
{
  ros2_vision_arm_control__msg__VisionDetection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_vision_arm_control__msg__VisionDetection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_VISION_ARM_CONTROL__MSG__DETAIL__VISION_DETECTION__STRUCT_H_
