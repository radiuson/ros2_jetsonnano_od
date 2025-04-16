// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros2_vision_arm_control:msg/VisionDetection.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros2_vision_arm_control/msg/detail/vision_detection__rosidl_typesupport_introspection_c.h"
#include "ros2_vision_arm_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros2_vision_arm_control/msg/detail/vision_detection__functions.h"
#include "ros2_vision_arm_control/msg/detail/vision_detection__struct.h"


// Include directives for member types
// Member `rgb_image`
// Member `depth_image`
#include "sensor_msgs/msg/image.h"
// Member `rgb_image`
// Member `depth_image`
#include "sensor_msgs/msg/detail/image__rosidl_typesupport_introspection_c.h"
// Member `boxes`
#include "ros2_vision_arm_control/msg/bounding_box.h"
// Member `boxes`
#include "ros2_vision_arm_control/msg/detail/bounding_box__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros2_vision_arm_control__msg__VisionDetection__init(message_memory);
}

void VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_fini_function(void * message_memory)
{
  ros2_vision_arm_control__msg__VisionDetection__fini(message_memory);
}

size_t VisionDetection__rosidl_typesupport_introspection_c__size_function__BoundingBox__boxes(
  const void * untyped_member)
{
  const ros2_vision_arm_control__msg__BoundingBox__Sequence * member =
    (const ros2_vision_arm_control__msg__BoundingBox__Sequence *)(untyped_member);
  return member->size;
}

const void * VisionDetection__rosidl_typesupport_introspection_c__get_const_function__BoundingBox__boxes(
  const void * untyped_member, size_t index)
{
  const ros2_vision_arm_control__msg__BoundingBox__Sequence * member =
    (const ros2_vision_arm_control__msg__BoundingBox__Sequence *)(untyped_member);
  return &member->data[index];
}

void * VisionDetection__rosidl_typesupport_introspection_c__get_function__BoundingBox__boxes(
  void * untyped_member, size_t index)
{
  ros2_vision_arm_control__msg__BoundingBox__Sequence * member =
    (ros2_vision_arm_control__msg__BoundingBox__Sequence *)(untyped_member);
  return &member->data[index];
}

bool VisionDetection__rosidl_typesupport_introspection_c__resize_function__BoundingBox__boxes(
  void * untyped_member, size_t size)
{
  ros2_vision_arm_control__msg__BoundingBox__Sequence * member =
    (ros2_vision_arm_control__msg__BoundingBox__Sequence *)(untyped_member);
  ros2_vision_arm_control__msg__BoundingBox__Sequence__fini(member);
  return ros2_vision_arm_control__msg__BoundingBox__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_message_member_array[3] = {
  {
    "rgb_image",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_vision_arm_control__msg__VisionDetection, rgb_image),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "depth_image",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_vision_arm_control__msg__VisionDetection, depth_image),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "boxes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_vision_arm_control__msg__VisionDetection, boxes),  // bytes offset in struct
    NULL,  // default value
    VisionDetection__rosidl_typesupport_introspection_c__size_function__BoundingBox__boxes,  // size() function pointer
    VisionDetection__rosidl_typesupport_introspection_c__get_const_function__BoundingBox__boxes,  // get_const(index) function pointer
    VisionDetection__rosidl_typesupport_introspection_c__get_function__BoundingBox__boxes,  // get(index) function pointer
    VisionDetection__rosidl_typesupport_introspection_c__resize_function__BoundingBox__boxes  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_message_members = {
  "ros2_vision_arm_control__msg",  // message namespace
  "VisionDetection",  // message name
  3,  // number of fields
  sizeof(ros2_vision_arm_control__msg__VisionDetection),
  VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_message_member_array,  // message members
  VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_init_function,  // function to initialize message memory (memory has to be allocated)
  VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_message_type_support_handle = {
  0,
  &VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2_vision_arm_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_vision_arm_control, msg, VisionDetection)() {
  VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_vision_arm_control, msg, BoundingBox)();
  if (!VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_message_type_support_handle.typesupport_identifier) {
    VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &VisionDetection__rosidl_typesupport_introspection_c__VisionDetection_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
