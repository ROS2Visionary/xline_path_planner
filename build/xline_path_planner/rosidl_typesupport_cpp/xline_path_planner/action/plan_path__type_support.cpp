// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from xline_path_planner:action/PlanPath.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "xline_path_planner/action/detail/plan_path__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace xline_path_planner
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanPath_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_Goal_type_support_ids_t;

static const _PlanPath_Goal_type_support_ids_t _PlanPath_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xline_path_planner, action, PlanPath_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, xline_path_planner, action, PlanPath_Goal)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_Goal_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace xline_path_planner

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<xline_path_planner::action::PlanPath_Goal>()
{
  return &::xline_path_planner::action::rosidl_typesupport_cpp::PlanPath_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, xline_path_planner, action, PlanPath_Goal)() {
  return get_message_type_support_handle<xline_path_planner::action::PlanPath_Goal>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xline_path_planner
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanPath_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_Result_type_support_ids_t;

static const _PlanPath_Result_type_support_ids_t _PlanPath_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xline_path_planner, action, PlanPath_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, xline_path_planner, action, PlanPath_Result)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_Result_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace xline_path_planner

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<xline_path_planner::action::PlanPath_Result>()
{
  return &::xline_path_planner::action::rosidl_typesupport_cpp::PlanPath_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, xline_path_planner, action, PlanPath_Result)() {
  return get_message_type_support_handle<xline_path_planner::action::PlanPath_Result>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xline_path_planner
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanPath_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_Feedback_type_support_ids_t;

static const _PlanPath_Feedback_type_support_ids_t _PlanPath_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xline_path_planner, action, PlanPath_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, xline_path_planner, action, PlanPath_Feedback)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_Feedback_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace xline_path_planner

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<xline_path_planner::action::PlanPath_Feedback>()
{
  return &::xline_path_planner::action::rosidl_typesupport_cpp::PlanPath_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, xline_path_planner, action, PlanPath_Feedback)() {
  return get_message_type_support_handle<xline_path_planner::action::PlanPath_Feedback>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xline_path_planner
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanPath_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_SendGoal_Request_type_support_ids_t;

static const _PlanPath_SendGoal_Request_type_support_ids_t _PlanPath_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xline_path_planner, action, PlanPath_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, xline_path_planner, action, PlanPath_SendGoal_Request)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_SendGoal_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace xline_path_planner

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<xline_path_planner::action::PlanPath_SendGoal_Request>()
{
  return &::xline_path_planner::action::rosidl_typesupport_cpp::PlanPath_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, xline_path_planner, action, PlanPath_SendGoal_Request)() {
  return get_message_type_support_handle<xline_path_planner::action::PlanPath_SendGoal_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xline_path_planner
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanPath_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_SendGoal_Response_type_support_ids_t;

static const _PlanPath_SendGoal_Response_type_support_ids_t _PlanPath_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xline_path_planner, action, PlanPath_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, xline_path_planner, action, PlanPath_SendGoal_Response)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_SendGoal_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace xline_path_planner

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<xline_path_planner::action::PlanPath_SendGoal_Response>()
{
  return &::xline_path_planner::action::rosidl_typesupport_cpp::PlanPath_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, xline_path_planner, action, PlanPath_SendGoal_Response)() {
  return get_message_type_support_handle<xline_path_planner::action::PlanPath_SendGoal_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xline_path_planner
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanPath_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_SendGoal_type_support_ids_t;

static const _PlanPath_SendGoal_type_support_ids_t _PlanPath_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xline_path_planner, action, PlanPath_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, xline_path_planner, action, PlanPath_SendGoal)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_SendGoal_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace xline_path_planner

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<xline_path_planner::action::PlanPath_SendGoal>()
{
  return &::xline_path_planner::action::rosidl_typesupport_cpp::PlanPath_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, xline_path_planner, action, PlanPath_SendGoal)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<xline_path_planner::action::PlanPath_SendGoal>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xline_path_planner
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanPath_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_GetResult_Request_type_support_ids_t;

static const _PlanPath_GetResult_Request_type_support_ids_t _PlanPath_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xline_path_planner, action, PlanPath_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, xline_path_planner, action, PlanPath_GetResult_Request)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_GetResult_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace xline_path_planner

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<xline_path_planner::action::PlanPath_GetResult_Request>()
{
  return &::xline_path_planner::action::rosidl_typesupport_cpp::PlanPath_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, xline_path_planner, action, PlanPath_GetResult_Request)() {
  return get_message_type_support_handle<xline_path_planner::action::PlanPath_GetResult_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xline_path_planner
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanPath_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_GetResult_Response_type_support_ids_t;

static const _PlanPath_GetResult_Response_type_support_ids_t _PlanPath_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xline_path_planner, action, PlanPath_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, xline_path_planner, action, PlanPath_GetResult_Response)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_GetResult_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace xline_path_planner

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<xline_path_planner::action::PlanPath_GetResult_Response>()
{
  return &::xline_path_planner::action::rosidl_typesupport_cpp::PlanPath_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, xline_path_planner, action, PlanPath_GetResult_Response)() {
  return get_message_type_support_handle<xline_path_planner::action::PlanPath_GetResult_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xline_path_planner
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanPath_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_GetResult_type_support_ids_t;

static const _PlanPath_GetResult_type_support_ids_t _PlanPath_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xline_path_planner, action, PlanPath_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, xline_path_planner, action, PlanPath_GetResult)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_GetResult_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace xline_path_planner

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<xline_path_planner::action::PlanPath_GetResult>()
{
  return &::xline_path_planner::action::rosidl_typesupport_cpp::PlanPath_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, xline_path_planner, action, PlanPath_GetResult)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<xline_path_planner::action::PlanPath_GetResult>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xline_path_planner
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanPath_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_FeedbackMessage_type_support_ids_t;

static const _PlanPath_FeedbackMessage_type_support_ids_t _PlanPath_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xline_path_planner, action, PlanPath_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, xline_path_planner, action, PlanPath_FeedbackMessage)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_FeedbackMessage_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace xline_path_planner

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<xline_path_planner::action::PlanPath_FeedbackMessage>()
{
  return &::xline_path_planner::action::rosidl_typesupport_cpp::PlanPath_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, xline_path_planner, action, PlanPath_FeedbackMessage)() {
  return get_message_type_support_handle<xline_path_planner::action::PlanPath_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"

namespace xline_path_planner
{

namespace action
{

namespace rosidl_typesupport_cpp
{

static rosidl_action_type_support_t PlanPath_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace xline_path_planner

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<xline_path_planner::action::PlanPath>()
{
  using ::xline_path_planner::action::rosidl_typesupport_cpp::PlanPath_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  PlanPath_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::xline_path_planner::action::PlanPath::Impl::SendGoalService>();
  PlanPath_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::xline_path_planner::action::PlanPath::Impl::GetResultService>();
  PlanPath_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::xline_path_planner::action::PlanPath::Impl::CancelGoalService>();
  PlanPath_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::xline_path_planner::action::PlanPath::Impl::FeedbackMessage>();
  PlanPath_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::xline_path_planner::action::PlanPath::Impl::GoalStatusMessage>();
  return &PlanPath_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(rosidl_typesupport_cpp, xline_path_planner, action, PlanPath)() {
  return ::rosidl_typesupport_cpp::get_action_type_support_handle<xline_path_planner::action::PlanPath>();
}

#ifdef __cplusplus
}
#endif
