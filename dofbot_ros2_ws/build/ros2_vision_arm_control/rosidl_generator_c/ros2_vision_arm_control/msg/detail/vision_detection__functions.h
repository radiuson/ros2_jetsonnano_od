// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ros2_vision_arm_control:msg/VisionDetection.idl
// generated code does not contain a copyright notice

#ifndef ROS2_VISION_ARM_CONTROL__MSG__DETAIL__VISION_DETECTION__FUNCTIONS_H_
#define ROS2_VISION_ARM_CONTROL__MSG__DETAIL__VISION_DETECTION__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ros2_vision_arm_control/msg/rosidl_generator_c__visibility_control.h"

#include "ros2_vision_arm_control/msg/detail/vision_detection__struct.h"

/// Initialize msg/VisionDetection message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ros2_vision_arm_control__msg__VisionDetection
 * )) before or use
 * ros2_vision_arm_control__msg__VisionDetection__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_vision_arm_control
bool
ros2_vision_arm_control__msg__VisionDetection__init(ros2_vision_arm_control__msg__VisionDetection * msg);

/// Finalize msg/VisionDetection message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_vision_arm_control
void
ros2_vision_arm_control__msg__VisionDetection__fini(ros2_vision_arm_control__msg__VisionDetection * msg);

/// Create msg/VisionDetection message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ros2_vision_arm_control__msg__VisionDetection__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_vision_arm_control
ros2_vision_arm_control__msg__VisionDetection *
ros2_vision_arm_control__msg__VisionDetection__create();

/// Destroy msg/VisionDetection message.
/**
 * It calls
 * ros2_vision_arm_control__msg__VisionDetection__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_vision_arm_control
void
ros2_vision_arm_control__msg__VisionDetection__destroy(ros2_vision_arm_control__msg__VisionDetection * msg);

/// Check for msg/VisionDetection message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_vision_arm_control
bool
ros2_vision_arm_control__msg__VisionDetection__are_equal(const ros2_vision_arm_control__msg__VisionDetection * lhs, const ros2_vision_arm_control__msg__VisionDetection * rhs);

/// Copy a msg/VisionDetection message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_vision_arm_control
bool
ros2_vision_arm_control__msg__VisionDetection__copy(
  const ros2_vision_arm_control__msg__VisionDetection * input,
  ros2_vision_arm_control__msg__VisionDetection * output);

/// Initialize array of msg/VisionDetection messages.
/**
 * It allocates the memory for the number of elements and calls
 * ros2_vision_arm_control__msg__VisionDetection__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_vision_arm_control
bool
ros2_vision_arm_control__msg__VisionDetection__Sequence__init(ros2_vision_arm_control__msg__VisionDetection__Sequence * array, size_t size);

/// Finalize array of msg/VisionDetection messages.
/**
 * It calls
 * ros2_vision_arm_control__msg__VisionDetection__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_vision_arm_control
void
ros2_vision_arm_control__msg__VisionDetection__Sequence__fini(ros2_vision_arm_control__msg__VisionDetection__Sequence * array);

/// Create array of msg/VisionDetection messages.
/**
 * It allocates the memory for the array and calls
 * ros2_vision_arm_control__msg__VisionDetection__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_vision_arm_control
ros2_vision_arm_control__msg__VisionDetection__Sequence *
ros2_vision_arm_control__msg__VisionDetection__Sequence__create(size_t size);

/// Destroy array of msg/VisionDetection messages.
/**
 * It calls
 * ros2_vision_arm_control__msg__VisionDetection__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_vision_arm_control
void
ros2_vision_arm_control__msg__VisionDetection__Sequence__destroy(ros2_vision_arm_control__msg__VisionDetection__Sequence * array);

/// Check for msg/VisionDetection message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_vision_arm_control
bool
ros2_vision_arm_control__msg__VisionDetection__Sequence__are_equal(const ros2_vision_arm_control__msg__VisionDetection__Sequence * lhs, const ros2_vision_arm_control__msg__VisionDetection__Sequence * rhs);

/// Copy an array of msg/VisionDetection messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_vision_arm_control
bool
ros2_vision_arm_control__msg__VisionDetection__Sequence__copy(
  const ros2_vision_arm_control__msg__VisionDetection__Sequence * input,
  ros2_vision_arm_control__msg__VisionDetection__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROS2_VISION_ARM_CONTROL__MSG__DETAIL__VISION_DETECTION__FUNCTIONS_H_
