// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:srv/MyCustomServiceMessage.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__MY_CUSTOM_SERVICE_MESSAGE__STRUCT_H_
#define CUSTOM_INTERFACES__SRV__DETAIL__MY_CUSTOM_SERVICE_MESSAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'move'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MyCustomServiceMessage in the package custom_interfaces.
typedef struct custom_interfaces__srv__MyCustomServiceMessage_Request
{
  /// Signal to define the movement
  /// "Turn right" to make the robot turn in the right direction.
  /// "Turn left" to make the robot turn in the left direction.
  /// "Stop" to make the robot stop the movement.
  rosidl_runtime_c__String move;
} custom_interfaces__srv__MyCustomServiceMessage_Request;

// Struct for a sequence of custom_interfaces__srv__MyCustomServiceMessage_Request.
typedef struct custom_interfaces__srv__MyCustomServiceMessage_Request__Sequence
{
  custom_interfaces__srv__MyCustomServiceMessage_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__srv__MyCustomServiceMessage_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/MyCustomServiceMessage in the package custom_interfaces.
typedef struct custom_interfaces__srv__MyCustomServiceMessage_Response
{
  /// Did it achieve it?
  bool success;
} custom_interfaces__srv__MyCustomServiceMessage_Response;

// Struct for a sequence of custom_interfaces__srv__MyCustomServiceMessage_Response.
typedef struct custom_interfaces__srv__MyCustomServiceMessage_Response__Sequence
{
  custom_interfaces__srv__MyCustomServiceMessage_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__srv__MyCustomServiceMessage_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__MY_CUSTOM_SERVICE_MESSAGE__STRUCT_H_
