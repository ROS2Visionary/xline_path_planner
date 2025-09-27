// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from xline_path_planner:action/PlanPath.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "xline_path_planner/action/detail/plan_path__rosidl_typesupport_introspection_c.h"
#include "xline_path_planner/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "xline_path_planner/action/detail/plan_path__functions.h"
#include "xline_path_planner/action/detail/plan_path__struct.h"


// Include directives for member types
// Member `request_id`
// Member `cad_json`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void xline_path_planner__action__PlanPath_Goal__rosidl_typesupport_introspection_c__PlanPath_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  xline_path_planner__action__PlanPath_Goal__init(message_memory);
}

void xline_path_planner__action__PlanPath_Goal__rosidl_typesupport_introspection_c__PlanPath_Goal_fini_function(void * message_memory)
{
  xline_path_planner__action__PlanPath_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember xline_path_planner__action__PlanPath_Goal__rosidl_typesupport_introspection_c__PlanPath_Goal_message_member_array[2] = {
  {
    "request_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xline_path_planner__action__PlanPath_Goal, request_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cad_json",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xline_path_planner__action__PlanPath_Goal, cad_json),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers xline_path_planner__action__PlanPath_Goal__rosidl_typesupport_introspection_c__PlanPath_Goal_message_members = {
  "xline_path_planner__action",  // message namespace
  "PlanPath_Goal",  // message name
  2,  // number of fields
  sizeof(xline_path_planner__action__PlanPath_Goal),
  xline_path_planner__action__PlanPath_Goal__rosidl_typesupport_introspection_c__PlanPath_Goal_message_member_array,  // message members
  xline_path_planner__action__PlanPath_Goal__rosidl_typesupport_introspection_c__PlanPath_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  xline_path_planner__action__PlanPath_Goal__rosidl_typesupport_introspection_c__PlanPath_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t xline_path_planner__action__PlanPath_Goal__rosidl_typesupport_introspection_c__PlanPath_Goal_message_type_support_handle = {
  0,
  &xline_path_planner__action__PlanPath_Goal__rosidl_typesupport_introspection_c__PlanPath_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xline_path_planner
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_Goal)() {
  if (!xline_path_planner__action__PlanPath_Goal__rosidl_typesupport_introspection_c__PlanPath_Goal_message_type_support_handle.typesupport_identifier) {
    xline_path_planner__action__PlanPath_Goal__rosidl_typesupport_introspection_c__PlanPath_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &xline_path_planner__action__PlanPath_Goal__rosidl_typesupport_introspection_c__PlanPath_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "xline_path_planner/action/detail/plan_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "xline_path_planner/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__functions.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.h"


// Include directives for member types
// Member `planned_json`
// Member `error`
// Member `warnings`
// Member `metrics_json`
// Member `planner_version`
// Member `config_hash`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void xline_path_planner__action__PlanPath_Result__rosidl_typesupport_introspection_c__PlanPath_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  xline_path_planner__action__PlanPath_Result__init(message_memory);
}

void xline_path_planner__action__PlanPath_Result__rosidl_typesupport_introspection_c__PlanPath_Result_fini_function(void * message_memory)
{
  xline_path_planner__action__PlanPath_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember xline_path_planner__action__PlanPath_Result__rosidl_typesupport_introspection_c__PlanPath_Result_message_member_array[7] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xline_path_planner__action__PlanPath_Result, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "planned_json",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xline_path_planner__action__PlanPath_Result, planned_json),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xline_path_planner__action__PlanPath_Result, error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "warnings",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xline_path_planner__action__PlanPath_Result, warnings),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "metrics_json",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xline_path_planner__action__PlanPath_Result, metrics_json),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "planner_version",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xline_path_planner__action__PlanPath_Result, planner_version),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "config_hash",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xline_path_planner__action__PlanPath_Result, config_hash),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers xline_path_planner__action__PlanPath_Result__rosidl_typesupport_introspection_c__PlanPath_Result_message_members = {
  "xline_path_planner__action",  // message namespace
  "PlanPath_Result",  // message name
  7,  // number of fields
  sizeof(xline_path_planner__action__PlanPath_Result),
  xline_path_planner__action__PlanPath_Result__rosidl_typesupport_introspection_c__PlanPath_Result_message_member_array,  // message members
  xline_path_planner__action__PlanPath_Result__rosidl_typesupport_introspection_c__PlanPath_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  xline_path_planner__action__PlanPath_Result__rosidl_typesupport_introspection_c__PlanPath_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t xline_path_planner__action__PlanPath_Result__rosidl_typesupport_introspection_c__PlanPath_Result_message_type_support_handle = {
  0,
  &xline_path_planner__action__PlanPath_Result__rosidl_typesupport_introspection_c__PlanPath_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xline_path_planner
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_Result)() {
  if (!xline_path_planner__action__PlanPath_Result__rosidl_typesupport_introspection_c__PlanPath_Result_message_type_support_handle.typesupport_identifier) {
    xline_path_planner__action__PlanPath_Result__rosidl_typesupport_introspection_c__PlanPath_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &xline_path_planner__action__PlanPath_Result__rosidl_typesupport_introspection_c__PlanPath_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "xline_path_planner/action/detail/plan_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "xline_path_planner/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__functions.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.h"


// Include directives for member types
// Member `stage`
// Member `info`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void xline_path_planner__action__PlanPath_Feedback__rosidl_typesupport_introspection_c__PlanPath_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  xline_path_planner__action__PlanPath_Feedback__init(message_memory);
}

void xline_path_planner__action__PlanPath_Feedback__rosidl_typesupport_introspection_c__PlanPath_Feedback_fini_function(void * message_memory)
{
  xline_path_planner__action__PlanPath_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember xline_path_planner__action__PlanPath_Feedback__rosidl_typesupport_introspection_c__PlanPath_Feedback_message_member_array[3] = {
  {
    "progress",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xline_path_planner__action__PlanPath_Feedback, progress),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xline_path_planner__action__PlanPath_Feedback, stage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xline_path_planner__action__PlanPath_Feedback, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers xline_path_planner__action__PlanPath_Feedback__rosidl_typesupport_introspection_c__PlanPath_Feedback_message_members = {
  "xline_path_planner__action",  // message namespace
  "PlanPath_Feedback",  // message name
  3,  // number of fields
  sizeof(xline_path_planner__action__PlanPath_Feedback),
  xline_path_planner__action__PlanPath_Feedback__rosidl_typesupport_introspection_c__PlanPath_Feedback_message_member_array,  // message members
  xline_path_planner__action__PlanPath_Feedback__rosidl_typesupport_introspection_c__PlanPath_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  xline_path_planner__action__PlanPath_Feedback__rosidl_typesupport_introspection_c__PlanPath_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t xline_path_planner__action__PlanPath_Feedback__rosidl_typesupport_introspection_c__PlanPath_Feedback_message_type_support_handle = {
  0,
  &xline_path_planner__action__PlanPath_Feedback__rosidl_typesupport_introspection_c__PlanPath_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xline_path_planner
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_Feedback)() {
  if (!xline_path_planner__action__PlanPath_Feedback__rosidl_typesupport_introspection_c__PlanPath_Feedback_message_type_support_handle.typesupport_identifier) {
    xline_path_planner__action__PlanPath_Feedback__rosidl_typesupport_introspection_c__PlanPath_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &xline_path_planner__action__PlanPath_Feedback__rosidl_typesupport_introspection_c__PlanPath_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "xline_path_planner/action/detail/plan_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "xline_path_planner/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__functions.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "xline_path_planner/action/plan_path.h"
// Member `goal`
// already included above
// #include "xline_path_planner/action/detail/plan_path__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void xline_path_planner__action__PlanPath_SendGoal_Request__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  xline_path_planner__action__PlanPath_SendGoal_Request__init(message_memory);
}

void xline_path_planner__action__PlanPath_SendGoal_Request__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Request_fini_function(void * message_memory)
{
  xline_path_planner__action__PlanPath_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember xline_path_planner__action__PlanPath_SendGoal_Request__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xline_path_planner__action__PlanPath_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xline_path_planner__action__PlanPath_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers xline_path_planner__action__PlanPath_SendGoal_Request__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Request_message_members = {
  "xline_path_planner__action",  // message namespace
  "PlanPath_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(xline_path_planner__action__PlanPath_SendGoal_Request),
  xline_path_planner__action__PlanPath_SendGoal_Request__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Request_message_member_array,  // message members
  xline_path_planner__action__PlanPath_SendGoal_Request__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  xline_path_planner__action__PlanPath_SendGoal_Request__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t xline_path_planner__action__PlanPath_SendGoal_Request__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Request_message_type_support_handle = {
  0,
  &xline_path_planner__action__PlanPath_SendGoal_Request__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xline_path_planner
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_SendGoal_Request)() {
  xline_path_planner__action__PlanPath_SendGoal_Request__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  xline_path_planner__action__PlanPath_SendGoal_Request__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_Goal)();
  if (!xline_path_planner__action__PlanPath_SendGoal_Request__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    xline_path_planner__action__PlanPath_SendGoal_Request__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &xline_path_planner__action__PlanPath_SendGoal_Request__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "xline_path_planner/action/detail/plan_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "xline_path_planner/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__functions.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void xline_path_planner__action__PlanPath_SendGoal_Response__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  xline_path_planner__action__PlanPath_SendGoal_Response__init(message_memory);
}

void xline_path_planner__action__PlanPath_SendGoal_Response__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Response_fini_function(void * message_memory)
{
  xline_path_planner__action__PlanPath_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember xline_path_planner__action__PlanPath_SendGoal_Response__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xline_path_planner__action__PlanPath_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xline_path_planner__action__PlanPath_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers xline_path_planner__action__PlanPath_SendGoal_Response__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Response_message_members = {
  "xline_path_planner__action",  // message namespace
  "PlanPath_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(xline_path_planner__action__PlanPath_SendGoal_Response),
  xline_path_planner__action__PlanPath_SendGoal_Response__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Response_message_member_array,  // message members
  xline_path_planner__action__PlanPath_SendGoal_Response__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  xline_path_planner__action__PlanPath_SendGoal_Response__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t xline_path_planner__action__PlanPath_SendGoal_Response__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Response_message_type_support_handle = {
  0,
  &xline_path_planner__action__PlanPath_SendGoal_Response__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xline_path_planner
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_SendGoal_Response)() {
  xline_path_planner__action__PlanPath_SendGoal_Response__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!xline_path_planner__action__PlanPath_SendGoal_Response__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    xline_path_planner__action__PlanPath_SendGoal_Response__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &xline_path_planner__action__PlanPath_SendGoal_Response__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "xline_path_planner/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers xline_path_planner__action__detail__plan_path__rosidl_typesupport_introspection_c__PlanPath_SendGoal_service_members = {
  "xline_path_planner__action",  // service namespace
  "PlanPath_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // xline_path_planner__action__detail__plan_path__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // xline_path_planner__action__detail__plan_path__rosidl_typesupport_introspection_c__PlanPath_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t xline_path_planner__action__detail__plan_path__rosidl_typesupport_introspection_c__PlanPath_SendGoal_service_type_support_handle = {
  0,
  &xline_path_planner__action__detail__plan_path__rosidl_typesupport_introspection_c__PlanPath_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xline_path_planner
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_SendGoal)() {
  if (!xline_path_planner__action__detail__plan_path__rosidl_typesupport_introspection_c__PlanPath_SendGoal_service_type_support_handle.typesupport_identifier) {
    xline_path_planner__action__detail__plan_path__rosidl_typesupport_introspection_c__PlanPath_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)xline_path_planner__action__detail__plan_path__rosidl_typesupport_introspection_c__PlanPath_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_SendGoal_Response)()->data;
  }

  return &xline_path_planner__action__detail__plan_path__rosidl_typesupport_introspection_c__PlanPath_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "xline_path_planner/action/detail/plan_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "xline_path_planner/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__functions.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void xline_path_planner__action__PlanPath_GetResult_Request__rosidl_typesupport_introspection_c__PlanPath_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  xline_path_planner__action__PlanPath_GetResult_Request__init(message_memory);
}

void xline_path_planner__action__PlanPath_GetResult_Request__rosidl_typesupport_introspection_c__PlanPath_GetResult_Request_fini_function(void * message_memory)
{
  xline_path_planner__action__PlanPath_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember xline_path_planner__action__PlanPath_GetResult_Request__rosidl_typesupport_introspection_c__PlanPath_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xline_path_planner__action__PlanPath_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers xline_path_planner__action__PlanPath_GetResult_Request__rosidl_typesupport_introspection_c__PlanPath_GetResult_Request_message_members = {
  "xline_path_planner__action",  // message namespace
  "PlanPath_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(xline_path_planner__action__PlanPath_GetResult_Request),
  xline_path_planner__action__PlanPath_GetResult_Request__rosidl_typesupport_introspection_c__PlanPath_GetResult_Request_message_member_array,  // message members
  xline_path_planner__action__PlanPath_GetResult_Request__rosidl_typesupport_introspection_c__PlanPath_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  xline_path_planner__action__PlanPath_GetResult_Request__rosidl_typesupport_introspection_c__PlanPath_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t xline_path_planner__action__PlanPath_GetResult_Request__rosidl_typesupport_introspection_c__PlanPath_GetResult_Request_message_type_support_handle = {
  0,
  &xline_path_planner__action__PlanPath_GetResult_Request__rosidl_typesupport_introspection_c__PlanPath_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xline_path_planner
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_GetResult_Request)() {
  xline_path_planner__action__PlanPath_GetResult_Request__rosidl_typesupport_introspection_c__PlanPath_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!xline_path_planner__action__PlanPath_GetResult_Request__rosidl_typesupport_introspection_c__PlanPath_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    xline_path_planner__action__PlanPath_GetResult_Request__rosidl_typesupport_introspection_c__PlanPath_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &xline_path_planner__action__PlanPath_GetResult_Request__rosidl_typesupport_introspection_c__PlanPath_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "xline_path_planner/action/detail/plan_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "xline_path_planner/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__functions.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "xline_path_planner/action/plan_path.h"
// Member `result`
// already included above
// #include "xline_path_planner/action/detail/plan_path__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void xline_path_planner__action__PlanPath_GetResult_Response__rosidl_typesupport_introspection_c__PlanPath_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  xline_path_planner__action__PlanPath_GetResult_Response__init(message_memory);
}

void xline_path_planner__action__PlanPath_GetResult_Response__rosidl_typesupport_introspection_c__PlanPath_GetResult_Response_fini_function(void * message_memory)
{
  xline_path_planner__action__PlanPath_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember xline_path_planner__action__PlanPath_GetResult_Response__rosidl_typesupport_introspection_c__PlanPath_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xline_path_planner__action__PlanPath_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xline_path_planner__action__PlanPath_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers xline_path_planner__action__PlanPath_GetResult_Response__rosidl_typesupport_introspection_c__PlanPath_GetResult_Response_message_members = {
  "xline_path_planner__action",  // message namespace
  "PlanPath_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(xline_path_planner__action__PlanPath_GetResult_Response),
  xline_path_planner__action__PlanPath_GetResult_Response__rosidl_typesupport_introspection_c__PlanPath_GetResult_Response_message_member_array,  // message members
  xline_path_planner__action__PlanPath_GetResult_Response__rosidl_typesupport_introspection_c__PlanPath_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  xline_path_planner__action__PlanPath_GetResult_Response__rosidl_typesupport_introspection_c__PlanPath_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t xline_path_planner__action__PlanPath_GetResult_Response__rosidl_typesupport_introspection_c__PlanPath_GetResult_Response_message_type_support_handle = {
  0,
  &xline_path_planner__action__PlanPath_GetResult_Response__rosidl_typesupport_introspection_c__PlanPath_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xline_path_planner
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_GetResult_Response)() {
  xline_path_planner__action__PlanPath_GetResult_Response__rosidl_typesupport_introspection_c__PlanPath_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_Result)();
  if (!xline_path_planner__action__PlanPath_GetResult_Response__rosidl_typesupport_introspection_c__PlanPath_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    xline_path_planner__action__PlanPath_GetResult_Response__rosidl_typesupport_introspection_c__PlanPath_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &xline_path_planner__action__PlanPath_GetResult_Response__rosidl_typesupport_introspection_c__PlanPath_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "xline_path_planner/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers xline_path_planner__action__detail__plan_path__rosidl_typesupport_introspection_c__PlanPath_GetResult_service_members = {
  "xline_path_planner__action",  // service namespace
  "PlanPath_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // xline_path_planner__action__detail__plan_path__rosidl_typesupport_introspection_c__PlanPath_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // xline_path_planner__action__detail__plan_path__rosidl_typesupport_introspection_c__PlanPath_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t xline_path_planner__action__detail__plan_path__rosidl_typesupport_introspection_c__PlanPath_GetResult_service_type_support_handle = {
  0,
  &xline_path_planner__action__detail__plan_path__rosidl_typesupport_introspection_c__PlanPath_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xline_path_planner
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_GetResult)() {
  if (!xline_path_planner__action__detail__plan_path__rosidl_typesupport_introspection_c__PlanPath_GetResult_service_type_support_handle.typesupport_identifier) {
    xline_path_planner__action__detail__plan_path__rosidl_typesupport_introspection_c__PlanPath_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)xline_path_planner__action__detail__plan_path__rosidl_typesupport_introspection_c__PlanPath_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_GetResult_Response)()->data;
  }

  return &xline_path_planner__action__detail__plan_path__rosidl_typesupport_introspection_c__PlanPath_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "xline_path_planner/action/detail/plan_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "xline_path_planner/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__functions.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "xline_path_planner/action/plan_path.h"
// Member `feedback`
// already included above
// #include "xline_path_planner/action/detail/plan_path__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void xline_path_planner__action__PlanPath_FeedbackMessage__rosidl_typesupport_introspection_c__PlanPath_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  xline_path_planner__action__PlanPath_FeedbackMessage__init(message_memory);
}

void xline_path_planner__action__PlanPath_FeedbackMessage__rosidl_typesupport_introspection_c__PlanPath_FeedbackMessage_fini_function(void * message_memory)
{
  xline_path_planner__action__PlanPath_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember xline_path_planner__action__PlanPath_FeedbackMessage__rosidl_typesupport_introspection_c__PlanPath_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xline_path_planner__action__PlanPath_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xline_path_planner__action__PlanPath_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers xline_path_planner__action__PlanPath_FeedbackMessage__rosidl_typesupport_introspection_c__PlanPath_FeedbackMessage_message_members = {
  "xline_path_planner__action",  // message namespace
  "PlanPath_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(xline_path_planner__action__PlanPath_FeedbackMessage),
  xline_path_planner__action__PlanPath_FeedbackMessage__rosidl_typesupport_introspection_c__PlanPath_FeedbackMessage_message_member_array,  // message members
  xline_path_planner__action__PlanPath_FeedbackMessage__rosidl_typesupport_introspection_c__PlanPath_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  xline_path_planner__action__PlanPath_FeedbackMessage__rosidl_typesupport_introspection_c__PlanPath_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t xline_path_planner__action__PlanPath_FeedbackMessage__rosidl_typesupport_introspection_c__PlanPath_FeedbackMessage_message_type_support_handle = {
  0,
  &xline_path_planner__action__PlanPath_FeedbackMessage__rosidl_typesupport_introspection_c__PlanPath_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xline_path_planner
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_FeedbackMessage)() {
  xline_path_planner__action__PlanPath_FeedbackMessage__rosidl_typesupport_introspection_c__PlanPath_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  xline_path_planner__action__PlanPath_FeedbackMessage__rosidl_typesupport_introspection_c__PlanPath_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_Feedback)();
  if (!xline_path_planner__action__PlanPath_FeedbackMessage__rosidl_typesupport_introspection_c__PlanPath_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    xline_path_planner__action__PlanPath_FeedbackMessage__rosidl_typesupport_introspection_c__PlanPath_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &xline_path_planner__action__PlanPath_FeedbackMessage__rosidl_typesupport_introspection_c__PlanPath_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
