// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ros2_vision_arm_control:msg/VisionDetection.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "ros2_vision_arm_control/msg/detail/vision_detection__struct.h"
#include "ros2_vision_arm_control/msg/detail/vision_detection__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "ros2_vision_arm_control/msg/detail/bounding_box__functions.h"
// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool sensor_msgs__msg__image__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * sensor_msgs__msg__image__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool sensor_msgs__msg__image__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * sensor_msgs__msg__image__convert_to_py(void * raw_ros_message);
bool ros2_vision_arm_control__msg__bounding_box__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ros2_vision_arm_control__msg__bounding_box__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ros2_vision_arm_control__msg__vision_detection__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[62];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("ros2_vision_arm_control.msg._vision_detection.VisionDetection", full_classname_dest, 61) == 0);
  }
  ros2_vision_arm_control__msg__VisionDetection * ros_message = _ros_message;
  {  // rgb_image
    PyObject * field = PyObject_GetAttrString(_pymsg, "rgb_image");
    if (!field) {
      return false;
    }
    if (!sensor_msgs__msg__image__convert_from_py(field, &ros_message->rgb_image)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // depth_image
    PyObject * field = PyObject_GetAttrString(_pymsg, "depth_image");
    if (!field) {
      return false;
    }
    if (!sensor_msgs__msg__image__convert_from_py(field, &ros_message->depth_image)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // boxes
    PyObject * field = PyObject_GetAttrString(_pymsg, "boxes");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'boxes'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!ros2_vision_arm_control__msg__BoundingBox__Sequence__init(&(ros_message->boxes), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create ros2_vision_arm_control__msg__BoundingBox__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    ros2_vision_arm_control__msg__BoundingBox * dest = ros_message->boxes.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!ros2_vision_arm_control__msg__bounding_box__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ros2_vision_arm_control__msg__vision_detection__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of VisionDetection */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ros2_vision_arm_control.msg._vision_detection");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "VisionDetection");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ros2_vision_arm_control__msg__VisionDetection * ros_message = (ros2_vision_arm_control__msg__VisionDetection *)raw_ros_message;
  {  // rgb_image
    PyObject * field = NULL;
    field = sensor_msgs__msg__image__convert_to_py(&ros_message->rgb_image);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "rgb_image", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // depth_image
    PyObject * field = NULL;
    field = sensor_msgs__msg__image__convert_to_py(&ros_message->depth_image);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "depth_image", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // boxes
    PyObject * field = NULL;
    size_t size = ros_message->boxes.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    ros2_vision_arm_control__msg__BoundingBox * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->boxes.data[i]);
      PyObject * pyitem = ros2_vision_arm_control__msg__bounding_box__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "boxes", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
