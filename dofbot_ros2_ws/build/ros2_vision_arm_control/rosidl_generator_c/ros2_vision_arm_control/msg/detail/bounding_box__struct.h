// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_vision_arm_control:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef ROS2_VISION_ARM_CONTROL__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_
#define ROS2_VISION_ARM_CONTROL__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'class_name'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/BoundingBox in the package ros2_vision_arm_control.
typedef struct ros2_vision_arm_control__msg__BoundingBox
{
  int32_t xmin;
  int32_t ymin;
  int32_t xmax;
  int32_t ymax;
  float confidence;
  int32_t class_id;
  rosidl_runtime_c__String class_name;
} ros2_vision_arm_control__msg__BoundingBox;

// Struct for a sequence of ros2_vision_arm_control__msg__BoundingBox.
typedef struct ros2_vision_arm_control__msg__BoundingBox__Sequence
{
  ros2_vision_arm_control__msg__BoundingBox * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_vision_arm_control__msg__BoundingBox__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_VISION_ARM_CONTROL__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_
