// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ros2_vision_arm_control:msg/VisionDetection.idl
// generated code does not contain a copyright notice
#include "ros2_vision_arm_control/msg/detail/vision_detection__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ros2_vision_arm_control/msg/detail/vision_detection__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace sensor_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const sensor_msgs::msg::Image &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  sensor_msgs::msg::Image &);
size_t get_serialized_size(
  const sensor_msgs::msg::Image &,
  size_t current_alignment);
size_t
max_serialized_size_Image(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace sensor_msgs

namespace sensor_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const sensor_msgs::msg::Image &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  sensor_msgs::msg::Image &);
size_t get_serialized_size(
  const sensor_msgs::msg::Image &,
  size_t current_alignment);
size_t
max_serialized_size_Image(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace sensor_msgs

namespace ros2_vision_arm_control
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const ros2_vision_arm_control::msg::BoundingBox &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  ros2_vision_arm_control::msg::BoundingBox &);
size_t get_serialized_size(
  const ros2_vision_arm_control::msg::BoundingBox &,
  size_t current_alignment);
size_t
max_serialized_size_BoundingBox(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace ros2_vision_arm_control


namespace ros2_vision_arm_control
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_vision_arm_control
cdr_serialize(
  const ros2_vision_arm_control::msg::VisionDetection & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: rgb_image
  sensor_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.rgb_image,
    cdr);
  // Member: depth_image
  sensor_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.depth_image,
    cdr);
  // Member: boxes
  {
    size_t size = ros_message.boxes.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      ros2_vision_arm_control::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.boxes[i],
        cdr);
    }
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_vision_arm_control
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ros2_vision_arm_control::msg::VisionDetection & ros_message)
{
  // Member: rgb_image
  sensor_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.rgb_image);

  // Member: depth_image
  sensor_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.depth_image);

  // Member: boxes
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.boxes.resize(size);
    for (size_t i = 0; i < size; i++) {
      ros2_vision_arm_control::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.boxes[i]);
    }
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_vision_arm_control
get_serialized_size(
  const ros2_vision_arm_control::msg::VisionDetection & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: rgb_image

  current_alignment +=
    sensor_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.rgb_image, current_alignment);
  // Member: depth_image

  current_alignment +=
    sensor_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.depth_image, current_alignment);
  // Member: boxes
  {
    size_t array_size = ros_message.boxes.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        ros2_vision_arm_control::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.boxes[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_vision_arm_control
max_serialized_size_VisionDetection(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: rgb_image
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        sensor_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Image(
        full_bounded, current_alignment);
    }
  }

  // Member: depth_image
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        sensor_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Image(
        full_bounded, current_alignment);
    }
  }

  // Member: boxes
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        ros2_vision_arm_control::msg::typesupport_fastrtps_cpp::max_serialized_size_BoundingBox(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _VisionDetection__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ros2_vision_arm_control::msg::VisionDetection *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _VisionDetection__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ros2_vision_arm_control::msg::VisionDetection *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _VisionDetection__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ros2_vision_arm_control::msg::VisionDetection *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _VisionDetection__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_VisionDetection(full_bounded, 0);
}

static message_type_support_callbacks_t _VisionDetection__callbacks = {
  "ros2_vision_arm_control::msg",
  "VisionDetection",
  _VisionDetection__cdr_serialize,
  _VisionDetection__cdr_deserialize,
  _VisionDetection__get_serialized_size,
  _VisionDetection__max_serialized_size
};

static rosidl_message_type_support_t _VisionDetection__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_VisionDetection__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ros2_vision_arm_control

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_ros2_vision_arm_control
const rosidl_message_type_support_t *
get_message_type_support_handle<ros2_vision_arm_control::msg::VisionDetection>()
{
  return &ros2_vision_arm_control::msg::typesupport_fastrtps_cpp::_VisionDetection__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros2_vision_arm_control, msg, VisionDetection)() {
  return &ros2_vision_arm_control::msg::typesupport_fastrtps_cpp::_VisionDetection__handle;
}

#ifdef __cplusplus
}
#endif
