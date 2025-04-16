// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ros2_vision_arm_control:msg/VisionDetection.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ros2_vision_arm_control/msg/detail/vision_detection__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ros2_vision_arm_control
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void VisionDetection_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ros2_vision_arm_control::msg::VisionDetection(_init);
}

void VisionDetection_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ros2_vision_arm_control::msg::VisionDetection *>(message_memory);
  typed_message->~VisionDetection();
}

size_t size_function__VisionDetection__boxes(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<ros2_vision_arm_control::msg::BoundingBox> *>(untyped_member);
  return member->size();
}

const void * get_const_function__VisionDetection__boxes(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<ros2_vision_arm_control::msg::BoundingBox> *>(untyped_member);
  return &member[index];
}

void * get_function__VisionDetection__boxes(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<ros2_vision_arm_control::msg::BoundingBox> *>(untyped_member);
  return &member[index];
}

void resize_function__VisionDetection__boxes(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<ros2_vision_arm_control::msg::BoundingBox> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember VisionDetection_message_member_array[3] = {
  {
    "rgb_image",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sensor_msgs::msg::Image>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_vision_arm_control::msg::VisionDetection, rgb_image),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "depth_image",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sensor_msgs::msg::Image>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_vision_arm_control::msg::VisionDetection, depth_image),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "boxes",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ros2_vision_arm_control::msg::BoundingBox>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_vision_arm_control::msg::VisionDetection, boxes),  // bytes offset in struct
    nullptr,  // default value
    size_function__VisionDetection__boxes,  // size() function pointer
    get_const_function__VisionDetection__boxes,  // get_const(index) function pointer
    get_function__VisionDetection__boxes,  // get(index) function pointer
    resize_function__VisionDetection__boxes  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers VisionDetection_message_members = {
  "ros2_vision_arm_control::msg",  // message namespace
  "VisionDetection",  // message name
  3,  // number of fields
  sizeof(ros2_vision_arm_control::msg::VisionDetection),
  VisionDetection_message_member_array,  // message members
  VisionDetection_init_function,  // function to initialize message memory (memory has to be allocated)
  VisionDetection_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t VisionDetection_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &VisionDetection_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace ros2_vision_arm_control


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros2_vision_arm_control::msg::VisionDetection>()
{
  return &::ros2_vision_arm_control::msg::rosidl_typesupport_introspection_cpp::VisionDetection_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros2_vision_arm_control, msg, VisionDetection)() {
  return &::ros2_vision_arm_control::msg::rosidl_typesupport_introspection_cpp::VisionDetection_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
