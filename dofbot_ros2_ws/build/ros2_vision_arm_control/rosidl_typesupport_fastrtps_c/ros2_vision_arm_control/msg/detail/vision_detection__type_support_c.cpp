// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ros2_vision_arm_control:msg/VisionDetection.idl
// generated code does not contain a copyright notice
#include "ros2_vision_arm_control/msg/detail/vision_detection__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ros2_vision_arm_control/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ros2_vision_arm_control/msg/detail/vision_detection__struct.h"
#include "ros2_vision_arm_control/msg/detail/vision_detection__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "ros2_vision_arm_control/msg/detail/bounding_box__functions.h"  // boxes
#include "sensor_msgs/msg/detail/image__functions.h"  // depth_image, rgb_image

// forward declare type support functions
size_t get_serialized_size_ros2_vision_arm_control__msg__BoundingBox(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ros2_vision_arm_control__msg__BoundingBox(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2_vision_arm_control, msg, BoundingBox)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2_vision_arm_control
size_t get_serialized_size_sensor_msgs__msg__Image(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2_vision_arm_control
size_t max_serialized_size_sensor_msgs__msg__Image(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2_vision_arm_control
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sensor_msgs, msg, Image)();


using _VisionDetection__ros_msg_type = ros2_vision_arm_control__msg__VisionDetection;

static bool _VisionDetection__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _VisionDetection__ros_msg_type * ros_message = static_cast<const _VisionDetection__ros_msg_type *>(untyped_ros_message);
  // Field name: rgb_image
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, Image
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->rgb_image, cdr))
    {
      return false;
    }
  }

  // Field name: depth_image
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, Image
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->depth_image, cdr))
    {
      return false;
    }
  }

  // Field name: boxes
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ros2_vision_arm_control, msg, BoundingBox
      )()->data);
    size_t size = ros_message->boxes.size;
    auto array_ptr = ros_message->boxes.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  return true;
}

static bool _VisionDetection__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _VisionDetection__ros_msg_type * ros_message = static_cast<_VisionDetection__ros_msg_type *>(untyped_ros_message);
  // Field name: rgb_image
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, Image
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->rgb_image))
    {
      return false;
    }
  }

  // Field name: depth_image
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, Image
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->depth_image))
    {
      return false;
    }
  }

  // Field name: boxes
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ros2_vision_arm_control, msg, BoundingBox
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->boxes.data) {
      ros2_vision_arm_control__msg__BoundingBox__Sequence__fini(&ros_message->boxes);
    }
    if (!ros2_vision_arm_control__msg__BoundingBox__Sequence__init(&ros_message->boxes, size)) {
      return "failed to create array for field 'boxes'";
    }
    auto array_ptr = ros_message->boxes.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2_vision_arm_control
size_t get_serialized_size_ros2_vision_arm_control__msg__VisionDetection(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _VisionDetection__ros_msg_type * ros_message = static_cast<const _VisionDetection__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name rgb_image

  current_alignment += get_serialized_size_sensor_msgs__msg__Image(
    &(ros_message->rgb_image), current_alignment);
  // field.name depth_image

  current_alignment += get_serialized_size_sensor_msgs__msg__Image(
    &(ros_message->depth_image), current_alignment);
  // field.name boxes
  {
    size_t array_size = ros_message->boxes.size;
    auto array_ptr = ros_message->boxes.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_ros2_vision_arm_control__msg__BoundingBox(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static uint32_t _VisionDetection__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros2_vision_arm_control__msg__VisionDetection(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2_vision_arm_control
size_t max_serialized_size_ros2_vision_arm_control__msg__VisionDetection(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: rgb_image
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_sensor_msgs__msg__Image(
        full_bounded, current_alignment);
    }
  }
  // member: depth_image
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_sensor_msgs__msg__Image(
        full_bounded, current_alignment);
    }
  }
  // member: boxes
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_ros2_vision_arm_control__msg__BoundingBox(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _VisionDetection__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_ros2_vision_arm_control__msg__VisionDetection(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_VisionDetection = {
  "ros2_vision_arm_control::msg",
  "VisionDetection",
  _VisionDetection__cdr_serialize,
  _VisionDetection__cdr_deserialize,
  _VisionDetection__get_serialized_size,
  _VisionDetection__max_serialized_size
};

static rosidl_message_type_support_t _VisionDetection__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_VisionDetection,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2_vision_arm_control, msg, VisionDetection)() {
  return &_VisionDetection__type_support;
}

#if defined(__cplusplus)
}
#endif
