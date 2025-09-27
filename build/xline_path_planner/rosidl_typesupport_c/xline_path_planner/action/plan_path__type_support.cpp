// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from xline_path_planner:action/PlanPath.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "xline_path_planner/action/detail/plan_path__struct.h"
#include "xline_path_planner/action/detail/plan_path__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace xline_path_planner
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _PlanPath_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_Goal_type_support_ids_t;

static const _PlanPath_Goal_type_support_ids_t _PlanPath_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PlanPath_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanPath_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanPath_Goal_type_support_symbol_names_t _PlanPath_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xline_path_planner, action, PlanPath_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_Goal)),
  }
};

typedef struct _PlanPath_Goal_type_support_data_t
{
  void * data[2];
} _PlanPath_Goal_type_support_data_t;

static _PlanPath_Goal_type_support_data_t _PlanPath_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanPath_Goal_message_typesupport_map = {
  2,
  "xline_path_planner",
  &_PlanPath_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_PlanPath_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_PlanPath_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlanPath_Goal_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_Goal_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace xline_path_planner

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, xline_path_planner, action, PlanPath_Goal)() {
  return &::xline_path_planner::action::rosidl_typesupport_c::PlanPath_Goal_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xline_path_planner
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _PlanPath_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_Result_type_support_ids_t;

static const _PlanPath_Result_type_support_ids_t _PlanPath_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PlanPath_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanPath_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanPath_Result_type_support_symbol_names_t _PlanPath_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xline_path_planner, action, PlanPath_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_Result)),
  }
};

typedef struct _PlanPath_Result_type_support_data_t
{
  void * data[2];
} _PlanPath_Result_type_support_data_t;

static _PlanPath_Result_type_support_data_t _PlanPath_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanPath_Result_message_typesupport_map = {
  2,
  "xline_path_planner",
  &_PlanPath_Result_message_typesupport_ids.typesupport_identifier[0],
  &_PlanPath_Result_message_typesupport_symbol_names.symbol_name[0],
  &_PlanPath_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlanPath_Result_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_Result_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace xline_path_planner

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, xline_path_planner, action, PlanPath_Result)() {
  return &::xline_path_planner::action::rosidl_typesupport_c::PlanPath_Result_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xline_path_planner
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _PlanPath_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_Feedback_type_support_ids_t;

static const _PlanPath_Feedback_type_support_ids_t _PlanPath_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PlanPath_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanPath_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanPath_Feedback_type_support_symbol_names_t _PlanPath_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xline_path_planner, action, PlanPath_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_Feedback)),
  }
};

typedef struct _PlanPath_Feedback_type_support_data_t
{
  void * data[2];
} _PlanPath_Feedback_type_support_data_t;

static _PlanPath_Feedback_type_support_data_t _PlanPath_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanPath_Feedback_message_typesupport_map = {
  2,
  "xline_path_planner",
  &_PlanPath_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_PlanPath_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_PlanPath_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlanPath_Feedback_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_Feedback_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace xline_path_planner

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, xline_path_planner, action, PlanPath_Feedback)() {
  return &::xline_path_planner::action::rosidl_typesupport_c::PlanPath_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xline_path_planner
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _PlanPath_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_SendGoal_Request_type_support_ids_t;

static const _PlanPath_SendGoal_Request_type_support_ids_t _PlanPath_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PlanPath_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanPath_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanPath_SendGoal_Request_type_support_symbol_names_t _PlanPath_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xline_path_planner, action, PlanPath_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_SendGoal_Request)),
  }
};

typedef struct _PlanPath_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _PlanPath_SendGoal_Request_type_support_data_t;

static _PlanPath_SendGoal_Request_type_support_data_t _PlanPath_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanPath_SendGoal_Request_message_typesupport_map = {
  2,
  "xline_path_planner",
  &_PlanPath_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_PlanPath_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_PlanPath_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlanPath_SendGoal_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_SendGoal_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace xline_path_planner

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, xline_path_planner, action, PlanPath_SendGoal_Request)() {
  return &::xline_path_planner::action::rosidl_typesupport_c::PlanPath_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xline_path_planner
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _PlanPath_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_SendGoal_Response_type_support_ids_t;

static const _PlanPath_SendGoal_Response_type_support_ids_t _PlanPath_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PlanPath_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanPath_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanPath_SendGoal_Response_type_support_symbol_names_t _PlanPath_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xline_path_planner, action, PlanPath_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_SendGoal_Response)),
  }
};

typedef struct _PlanPath_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _PlanPath_SendGoal_Response_type_support_data_t;

static _PlanPath_SendGoal_Response_type_support_data_t _PlanPath_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanPath_SendGoal_Response_message_typesupport_map = {
  2,
  "xline_path_planner",
  &_PlanPath_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_PlanPath_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_PlanPath_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlanPath_SendGoal_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_SendGoal_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace xline_path_planner

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, xline_path_planner, action, PlanPath_SendGoal_Response)() {
  return &::xline_path_planner::action::rosidl_typesupport_c::PlanPath_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xline_path_planner
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _PlanPath_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_SendGoal_type_support_ids_t;

static const _PlanPath_SendGoal_type_support_ids_t _PlanPath_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PlanPath_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanPath_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanPath_SendGoal_type_support_symbol_names_t _PlanPath_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xline_path_planner, action, PlanPath_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_SendGoal)),
  }
};

typedef struct _PlanPath_SendGoal_type_support_data_t
{
  void * data[2];
} _PlanPath_SendGoal_type_support_data_t;

static _PlanPath_SendGoal_type_support_data_t _PlanPath_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanPath_SendGoal_service_typesupport_map = {
  2,
  "xline_path_planner",
  &_PlanPath_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_PlanPath_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_PlanPath_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t PlanPath_SendGoal_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_SendGoal_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace xline_path_planner

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, xline_path_planner, action, PlanPath_SendGoal)() {
  return &::xline_path_planner::action::rosidl_typesupport_c::PlanPath_SendGoal_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xline_path_planner
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _PlanPath_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_GetResult_Request_type_support_ids_t;

static const _PlanPath_GetResult_Request_type_support_ids_t _PlanPath_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PlanPath_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanPath_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanPath_GetResult_Request_type_support_symbol_names_t _PlanPath_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xline_path_planner, action, PlanPath_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_GetResult_Request)),
  }
};

typedef struct _PlanPath_GetResult_Request_type_support_data_t
{
  void * data[2];
} _PlanPath_GetResult_Request_type_support_data_t;

static _PlanPath_GetResult_Request_type_support_data_t _PlanPath_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanPath_GetResult_Request_message_typesupport_map = {
  2,
  "xline_path_planner",
  &_PlanPath_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_PlanPath_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_PlanPath_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlanPath_GetResult_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_GetResult_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace xline_path_planner

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, xline_path_planner, action, PlanPath_GetResult_Request)() {
  return &::xline_path_planner::action::rosidl_typesupport_c::PlanPath_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xline_path_planner
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _PlanPath_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_GetResult_Response_type_support_ids_t;

static const _PlanPath_GetResult_Response_type_support_ids_t _PlanPath_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PlanPath_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanPath_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanPath_GetResult_Response_type_support_symbol_names_t _PlanPath_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xline_path_planner, action, PlanPath_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_GetResult_Response)),
  }
};

typedef struct _PlanPath_GetResult_Response_type_support_data_t
{
  void * data[2];
} _PlanPath_GetResult_Response_type_support_data_t;

static _PlanPath_GetResult_Response_type_support_data_t _PlanPath_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanPath_GetResult_Response_message_typesupport_map = {
  2,
  "xline_path_planner",
  &_PlanPath_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_PlanPath_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_PlanPath_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlanPath_GetResult_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_GetResult_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace xline_path_planner

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, xline_path_planner, action, PlanPath_GetResult_Response)() {
  return &::xline_path_planner::action::rosidl_typesupport_c::PlanPath_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xline_path_planner
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _PlanPath_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_GetResult_type_support_ids_t;

static const _PlanPath_GetResult_type_support_ids_t _PlanPath_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PlanPath_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanPath_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanPath_GetResult_type_support_symbol_names_t _PlanPath_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xline_path_planner, action, PlanPath_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_GetResult)),
  }
};

typedef struct _PlanPath_GetResult_type_support_data_t
{
  void * data[2];
} _PlanPath_GetResult_type_support_data_t;

static _PlanPath_GetResult_type_support_data_t _PlanPath_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanPath_GetResult_service_typesupport_map = {
  2,
  "xline_path_planner",
  &_PlanPath_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_PlanPath_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_PlanPath_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t PlanPath_GetResult_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_GetResult_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace xline_path_planner

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, xline_path_planner, action, PlanPath_GetResult)() {
  return &::xline_path_planner::action::rosidl_typesupport_c::PlanPath_GetResult_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xline_path_planner
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _PlanPath_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_FeedbackMessage_type_support_ids_t;

static const _PlanPath_FeedbackMessage_type_support_ids_t _PlanPath_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PlanPath_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanPath_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanPath_FeedbackMessage_type_support_symbol_names_t _PlanPath_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xline_path_planner, action, PlanPath_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xline_path_planner, action, PlanPath_FeedbackMessage)),
  }
};

typedef struct _PlanPath_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _PlanPath_FeedbackMessage_type_support_data_t;

static _PlanPath_FeedbackMessage_type_support_data_t _PlanPath_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanPath_FeedbackMessage_message_typesupport_map = {
  2,
  "xline_path_planner",
  &_PlanPath_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_PlanPath_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_PlanPath_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlanPath_FeedbackMessage_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_FeedbackMessage_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace xline_path_planner

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, xline_path_planner, action, PlanPath_FeedbackMessage)() {
  return &::xline_path_planner::action::rosidl_typesupport_c::PlanPath_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "action_msgs/msg/goal_status_array.h"
#include "action_msgs/srv/cancel_goal.h"
#include "xline_path_planner/action/plan_path.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__type_support.h"

static rosidl_action_type_support_t _xline_path_planner__action__PlanPath__typesupport_c;

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(
  rosidl_typesupport_c, xline_path_planner, action, PlanPath)()
{
  // Thread-safe by always writing the same values to the static struct
  _xline_path_planner__action__PlanPath__typesupport_c.goal_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, xline_path_planner, action, PlanPath_SendGoal)();
  _xline_path_planner__action__PlanPath__typesupport_c.result_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, xline_path_planner, action, PlanPath_GetResult)();
  _xline_path_planner__action__PlanPath__typesupport_c.cancel_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, srv, CancelGoal)();
  _xline_path_planner__action__PlanPath__typesupport_c.feedback_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, xline_path_planner, action, PlanPath_FeedbackMessage)();
  _xline_path_planner__action__PlanPath__typesupport_c.status_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, msg, GoalStatusArray)();

  return &_xline_path_planner__action__PlanPath__typesupport_c;
}

#ifdef __cplusplus
}
#endif
