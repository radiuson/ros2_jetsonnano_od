// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros2_vision_arm_control:msg/VisionDetection.idl
// generated code does not contain a copyright notice
#include "ros2_vision_arm_control/msg/detail/vision_detection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `rgb_image`
// Member `depth_image`
#include "sensor_msgs/msg/detail/image__functions.h"
// Member `boxes`
#include "ros2_vision_arm_control/msg/detail/bounding_box__functions.h"

bool
ros2_vision_arm_control__msg__VisionDetection__init(ros2_vision_arm_control__msg__VisionDetection * msg)
{
  if (!msg) {
    return false;
  }
  // rgb_image
  if (!sensor_msgs__msg__Image__init(&msg->rgb_image)) {
    ros2_vision_arm_control__msg__VisionDetection__fini(msg);
    return false;
  }
  // depth_image
  if (!sensor_msgs__msg__Image__init(&msg->depth_image)) {
    ros2_vision_arm_control__msg__VisionDetection__fini(msg);
    return false;
  }
  // boxes
  if (!ros2_vision_arm_control__msg__BoundingBox__Sequence__init(&msg->boxes, 0)) {
    ros2_vision_arm_control__msg__VisionDetection__fini(msg);
    return false;
  }
  return true;
}

void
ros2_vision_arm_control__msg__VisionDetection__fini(ros2_vision_arm_control__msg__VisionDetection * msg)
{
  if (!msg) {
    return;
  }
  // rgb_image
  sensor_msgs__msg__Image__fini(&msg->rgb_image);
  // depth_image
  sensor_msgs__msg__Image__fini(&msg->depth_image);
  // boxes
  ros2_vision_arm_control__msg__BoundingBox__Sequence__fini(&msg->boxes);
}

bool
ros2_vision_arm_control__msg__VisionDetection__are_equal(const ros2_vision_arm_control__msg__VisionDetection * lhs, const ros2_vision_arm_control__msg__VisionDetection * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // rgb_image
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->rgb_image), &(rhs->rgb_image)))
  {
    return false;
  }
  // depth_image
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->depth_image), &(rhs->depth_image)))
  {
    return false;
  }
  // boxes
  if (!ros2_vision_arm_control__msg__BoundingBox__Sequence__are_equal(
      &(lhs->boxes), &(rhs->boxes)))
  {
    return false;
  }
  return true;
}

bool
ros2_vision_arm_control__msg__VisionDetection__copy(
  const ros2_vision_arm_control__msg__VisionDetection * input,
  ros2_vision_arm_control__msg__VisionDetection * output)
{
  if (!input || !output) {
    return false;
  }
  // rgb_image
  if (!sensor_msgs__msg__Image__copy(
      &(input->rgb_image), &(output->rgb_image)))
  {
    return false;
  }
  // depth_image
  if (!sensor_msgs__msg__Image__copy(
      &(input->depth_image), &(output->depth_image)))
  {
    return false;
  }
  // boxes
  if (!ros2_vision_arm_control__msg__BoundingBox__Sequence__copy(
      &(input->boxes), &(output->boxes)))
  {
    return false;
  }
  return true;
}

ros2_vision_arm_control__msg__VisionDetection *
ros2_vision_arm_control__msg__VisionDetection__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_vision_arm_control__msg__VisionDetection * msg = (ros2_vision_arm_control__msg__VisionDetection *)allocator.allocate(sizeof(ros2_vision_arm_control__msg__VisionDetection), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2_vision_arm_control__msg__VisionDetection));
  bool success = ros2_vision_arm_control__msg__VisionDetection__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2_vision_arm_control__msg__VisionDetection__destroy(ros2_vision_arm_control__msg__VisionDetection * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2_vision_arm_control__msg__VisionDetection__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2_vision_arm_control__msg__VisionDetection__Sequence__init(ros2_vision_arm_control__msg__VisionDetection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_vision_arm_control__msg__VisionDetection * data = NULL;

  if (size) {
    data = (ros2_vision_arm_control__msg__VisionDetection *)allocator.zero_allocate(size, sizeof(ros2_vision_arm_control__msg__VisionDetection), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2_vision_arm_control__msg__VisionDetection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2_vision_arm_control__msg__VisionDetection__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ros2_vision_arm_control__msg__VisionDetection__Sequence__fini(ros2_vision_arm_control__msg__VisionDetection__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ros2_vision_arm_control__msg__VisionDetection__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ros2_vision_arm_control__msg__VisionDetection__Sequence *
ros2_vision_arm_control__msg__VisionDetection__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_vision_arm_control__msg__VisionDetection__Sequence * array = (ros2_vision_arm_control__msg__VisionDetection__Sequence *)allocator.allocate(sizeof(ros2_vision_arm_control__msg__VisionDetection__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2_vision_arm_control__msg__VisionDetection__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2_vision_arm_control__msg__VisionDetection__Sequence__destroy(ros2_vision_arm_control__msg__VisionDetection__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2_vision_arm_control__msg__VisionDetection__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2_vision_arm_control__msg__VisionDetection__Sequence__are_equal(const ros2_vision_arm_control__msg__VisionDetection__Sequence * lhs, const ros2_vision_arm_control__msg__VisionDetection__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2_vision_arm_control__msg__VisionDetection__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2_vision_arm_control__msg__VisionDetection__Sequence__copy(
  const ros2_vision_arm_control__msg__VisionDetection__Sequence * input,
  ros2_vision_arm_control__msg__VisionDetection__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2_vision_arm_control__msg__VisionDetection);
    ros2_vision_arm_control__msg__VisionDetection * data =
      (ros2_vision_arm_control__msg__VisionDetection *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2_vision_arm_control__msg__VisionDetection__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          ros2_vision_arm_control__msg__VisionDetection__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2_vision_arm_control__msg__VisionDetection__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
