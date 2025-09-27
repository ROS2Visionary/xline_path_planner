// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from xline_path_planner:action/PlanPath.idl
// generated code does not contain a copyright notice

#ifndef XLINE_PATH_PLANNER__ACTION__DETAIL__PLAN_PATH__STRUCT_H_
#define XLINE_PATH_PLANNER__ACTION__DETAIL__PLAN_PATH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'request_id'
// Member 'cad_json'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/PlanPath in the package xline_path_planner.
typedef struct xline_path_planner__action__PlanPath_Goal
{
  rosidl_runtime_c__String request_id;
  rosidl_runtime_c__String cad_json;
} xline_path_planner__action__PlanPath_Goal;

// Struct for a sequence of xline_path_planner__action__PlanPath_Goal.
typedef struct xline_path_planner__action__PlanPath_Goal__Sequence
{
  xline_path_planner__action__PlanPath_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xline_path_planner__action__PlanPath_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'planned_json'
// Member 'error'
// Member 'warnings'
// Member 'metrics_json'
// Member 'planner_version'
// Member 'config_hash'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/PlanPath in the package xline_path_planner.
typedef struct xline_path_planner__action__PlanPath_Result
{
  bool success;
  rosidl_runtime_c__String planned_json;
  rosidl_runtime_c__String error;
  rosidl_runtime_c__String warnings;
  rosidl_runtime_c__String metrics_json;
  rosidl_runtime_c__String planner_version;
  rosidl_runtime_c__String config_hash;
} xline_path_planner__action__PlanPath_Result;

// Struct for a sequence of xline_path_planner__action__PlanPath_Result.
typedef struct xline_path_planner__action__PlanPath_Result__Sequence
{
  xline_path_planner__action__PlanPath_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xline_path_planner__action__PlanPath_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stage'
// Member 'info'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/PlanPath in the package xline_path_planner.
typedef struct xline_path_planner__action__PlanPath_Feedback
{
  uint8_t progress;
  rosidl_runtime_c__String stage;
  rosidl_runtime_c__String info;
} xline_path_planner__action__PlanPath_Feedback;

// Struct for a sequence of xline_path_planner__action__PlanPath_Feedback.
typedef struct xline_path_planner__action__PlanPath_Feedback__Sequence
{
  xline_path_planner__action__PlanPath_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xline_path_planner__action__PlanPath_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "xline_path_planner/action/detail/plan_path__struct.h"

/// Struct defined in action/PlanPath in the package xline_path_planner.
typedef struct xline_path_planner__action__PlanPath_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  xline_path_planner__action__PlanPath_Goal goal;
} xline_path_planner__action__PlanPath_SendGoal_Request;

// Struct for a sequence of xline_path_planner__action__PlanPath_SendGoal_Request.
typedef struct xline_path_planner__action__PlanPath_SendGoal_Request__Sequence
{
  xline_path_planner__action__PlanPath_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xline_path_planner__action__PlanPath_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/PlanPath in the package xline_path_planner.
typedef struct xline_path_planner__action__PlanPath_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} xline_path_planner__action__PlanPath_SendGoal_Response;

// Struct for a sequence of xline_path_planner__action__PlanPath_SendGoal_Response.
typedef struct xline_path_planner__action__PlanPath_SendGoal_Response__Sequence
{
  xline_path_planner__action__PlanPath_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xline_path_planner__action__PlanPath_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/PlanPath in the package xline_path_planner.
typedef struct xline_path_planner__action__PlanPath_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} xline_path_planner__action__PlanPath_GetResult_Request;

// Struct for a sequence of xline_path_planner__action__PlanPath_GetResult_Request.
typedef struct xline_path_planner__action__PlanPath_GetResult_Request__Sequence
{
  xline_path_planner__action__PlanPath_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xline_path_planner__action__PlanPath_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.h"

/// Struct defined in action/PlanPath in the package xline_path_planner.
typedef struct xline_path_planner__action__PlanPath_GetResult_Response
{
  int8_t status;
  xline_path_planner__action__PlanPath_Result result;
} xline_path_planner__action__PlanPath_GetResult_Response;

// Struct for a sequence of xline_path_planner__action__PlanPath_GetResult_Response.
typedef struct xline_path_planner__action__PlanPath_GetResult_Response__Sequence
{
  xline_path_planner__action__PlanPath_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xline_path_planner__action__PlanPath_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.h"

/// Struct defined in action/PlanPath in the package xline_path_planner.
typedef struct xline_path_planner__action__PlanPath_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  xline_path_planner__action__PlanPath_Feedback feedback;
} xline_path_planner__action__PlanPath_FeedbackMessage;

// Struct for a sequence of xline_path_planner__action__PlanPath_FeedbackMessage.
typedef struct xline_path_planner__action__PlanPath_FeedbackMessage__Sequence
{
  xline_path_planner__action__PlanPath_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xline_path_planner__action__PlanPath_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // XLINE_PATH_PLANNER__ACTION__DETAIL__PLAN_PATH__STRUCT_H_
