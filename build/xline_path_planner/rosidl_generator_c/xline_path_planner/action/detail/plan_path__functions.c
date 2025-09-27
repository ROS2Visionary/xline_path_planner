// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from xline_path_planner:action/PlanPath.idl
// generated code does not contain a copyright notice
#include "xline_path_planner/action/detail/plan_path__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `request_id`
// Member `cad_json`
#include "rosidl_runtime_c/string_functions.h"

bool
xline_path_planner__action__PlanPath_Goal__init(xline_path_planner__action__PlanPath_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // request_id
  if (!rosidl_runtime_c__String__init(&msg->request_id)) {
    xline_path_planner__action__PlanPath_Goal__fini(msg);
    return false;
  }
  // cad_json
  if (!rosidl_runtime_c__String__init(&msg->cad_json)) {
    xline_path_planner__action__PlanPath_Goal__fini(msg);
    return false;
  }
  return true;
}

void
xline_path_planner__action__PlanPath_Goal__fini(xline_path_planner__action__PlanPath_Goal * msg)
{
  if (!msg) {
    return;
  }
  // request_id
  rosidl_runtime_c__String__fini(&msg->request_id);
  // cad_json
  rosidl_runtime_c__String__fini(&msg->cad_json);
}

bool
xline_path_planner__action__PlanPath_Goal__are_equal(const xline_path_planner__action__PlanPath_Goal * lhs, const xline_path_planner__action__PlanPath_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // request_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->request_id), &(rhs->request_id)))
  {
    return false;
  }
  // cad_json
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->cad_json), &(rhs->cad_json)))
  {
    return false;
  }
  return true;
}

bool
xline_path_planner__action__PlanPath_Goal__copy(
  const xline_path_planner__action__PlanPath_Goal * input,
  xline_path_planner__action__PlanPath_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // request_id
  if (!rosidl_runtime_c__String__copy(
      &(input->request_id), &(output->request_id)))
  {
    return false;
  }
  // cad_json
  if (!rosidl_runtime_c__String__copy(
      &(input->cad_json), &(output->cad_json)))
  {
    return false;
  }
  return true;
}

xline_path_planner__action__PlanPath_Goal *
xline_path_planner__action__PlanPath_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_Goal * msg = (xline_path_planner__action__PlanPath_Goal *)allocator.allocate(sizeof(xline_path_planner__action__PlanPath_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xline_path_planner__action__PlanPath_Goal));
  bool success = xline_path_planner__action__PlanPath_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xline_path_planner__action__PlanPath_Goal__destroy(xline_path_planner__action__PlanPath_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xline_path_planner__action__PlanPath_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xline_path_planner__action__PlanPath_Goal__Sequence__init(xline_path_planner__action__PlanPath_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_Goal * data = NULL;

  if (size) {
    data = (xline_path_planner__action__PlanPath_Goal *)allocator.zero_allocate(size, sizeof(xline_path_planner__action__PlanPath_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xline_path_planner__action__PlanPath_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xline_path_planner__action__PlanPath_Goal__fini(&data[i - 1]);
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
xline_path_planner__action__PlanPath_Goal__Sequence__fini(xline_path_planner__action__PlanPath_Goal__Sequence * array)
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
      xline_path_planner__action__PlanPath_Goal__fini(&array->data[i]);
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

xline_path_planner__action__PlanPath_Goal__Sequence *
xline_path_planner__action__PlanPath_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_Goal__Sequence * array = (xline_path_planner__action__PlanPath_Goal__Sequence *)allocator.allocate(sizeof(xline_path_planner__action__PlanPath_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xline_path_planner__action__PlanPath_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xline_path_planner__action__PlanPath_Goal__Sequence__destroy(xline_path_planner__action__PlanPath_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xline_path_planner__action__PlanPath_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xline_path_planner__action__PlanPath_Goal__Sequence__are_equal(const xline_path_planner__action__PlanPath_Goal__Sequence * lhs, const xline_path_planner__action__PlanPath_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xline_path_planner__action__PlanPath_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xline_path_planner__action__PlanPath_Goal__Sequence__copy(
  const xline_path_planner__action__PlanPath_Goal__Sequence * input,
  xline_path_planner__action__PlanPath_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xline_path_planner__action__PlanPath_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xline_path_planner__action__PlanPath_Goal * data =
      (xline_path_planner__action__PlanPath_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xline_path_planner__action__PlanPath_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xline_path_planner__action__PlanPath_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xline_path_planner__action__PlanPath_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `planned_json`
// Member `error`
// Member `warnings`
// Member `metrics_json`
// Member `planner_version`
// Member `config_hash`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
xline_path_planner__action__PlanPath_Result__init(xline_path_planner__action__PlanPath_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // planned_json
  if (!rosidl_runtime_c__String__init(&msg->planned_json)) {
    xline_path_planner__action__PlanPath_Result__fini(msg);
    return false;
  }
  // error
  if (!rosidl_runtime_c__String__init(&msg->error)) {
    xline_path_planner__action__PlanPath_Result__fini(msg);
    return false;
  }
  // warnings
  if (!rosidl_runtime_c__String__init(&msg->warnings)) {
    xline_path_planner__action__PlanPath_Result__fini(msg);
    return false;
  }
  // metrics_json
  if (!rosidl_runtime_c__String__init(&msg->metrics_json)) {
    xline_path_planner__action__PlanPath_Result__fini(msg);
    return false;
  }
  // planner_version
  if (!rosidl_runtime_c__String__init(&msg->planner_version)) {
    xline_path_planner__action__PlanPath_Result__fini(msg);
    return false;
  }
  // config_hash
  if (!rosidl_runtime_c__String__init(&msg->config_hash)) {
    xline_path_planner__action__PlanPath_Result__fini(msg);
    return false;
  }
  return true;
}

void
xline_path_planner__action__PlanPath_Result__fini(xline_path_planner__action__PlanPath_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
  // planned_json
  rosidl_runtime_c__String__fini(&msg->planned_json);
  // error
  rosidl_runtime_c__String__fini(&msg->error);
  // warnings
  rosidl_runtime_c__String__fini(&msg->warnings);
  // metrics_json
  rosidl_runtime_c__String__fini(&msg->metrics_json);
  // planner_version
  rosidl_runtime_c__String__fini(&msg->planner_version);
  // config_hash
  rosidl_runtime_c__String__fini(&msg->config_hash);
}

bool
xline_path_planner__action__PlanPath_Result__are_equal(const xline_path_planner__action__PlanPath_Result * lhs, const xline_path_planner__action__PlanPath_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // planned_json
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->planned_json), &(rhs->planned_json)))
  {
    return false;
  }
  // error
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->error), &(rhs->error)))
  {
    return false;
  }
  // warnings
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->warnings), &(rhs->warnings)))
  {
    return false;
  }
  // metrics_json
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->metrics_json), &(rhs->metrics_json)))
  {
    return false;
  }
  // planner_version
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->planner_version), &(rhs->planner_version)))
  {
    return false;
  }
  // config_hash
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->config_hash), &(rhs->config_hash)))
  {
    return false;
  }
  return true;
}

bool
xline_path_planner__action__PlanPath_Result__copy(
  const xline_path_planner__action__PlanPath_Result * input,
  xline_path_planner__action__PlanPath_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // planned_json
  if (!rosidl_runtime_c__String__copy(
      &(input->planned_json), &(output->planned_json)))
  {
    return false;
  }
  // error
  if (!rosidl_runtime_c__String__copy(
      &(input->error), &(output->error)))
  {
    return false;
  }
  // warnings
  if (!rosidl_runtime_c__String__copy(
      &(input->warnings), &(output->warnings)))
  {
    return false;
  }
  // metrics_json
  if (!rosidl_runtime_c__String__copy(
      &(input->metrics_json), &(output->metrics_json)))
  {
    return false;
  }
  // planner_version
  if (!rosidl_runtime_c__String__copy(
      &(input->planner_version), &(output->planner_version)))
  {
    return false;
  }
  // config_hash
  if (!rosidl_runtime_c__String__copy(
      &(input->config_hash), &(output->config_hash)))
  {
    return false;
  }
  return true;
}

xline_path_planner__action__PlanPath_Result *
xline_path_planner__action__PlanPath_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_Result * msg = (xline_path_planner__action__PlanPath_Result *)allocator.allocate(sizeof(xline_path_planner__action__PlanPath_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xline_path_planner__action__PlanPath_Result));
  bool success = xline_path_planner__action__PlanPath_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xline_path_planner__action__PlanPath_Result__destroy(xline_path_planner__action__PlanPath_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xline_path_planner__action__PlanPath_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xline_path_planner__action__PlanPath_Result__Sequence__init(xline_path_planner__action__PlanPath_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_Result * data = NULL;

  if (size) {
    data = (xline_path_planner__action__PlanPath_Result *)allocator.zero_allocate(size, sizeof(xline_path_planner__action__PlanPath_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xline_path_planner__action__PlanPath_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xline_path_planner__action__PlanPath_Result__fini(&data[i - 1]);
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
xline_path_planner__action__PlanPath_Result__Sequence__fini(xline_path_planner__action__PlanPath_Result__Sequence * array)
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
      xline_path_planner__action__PlanPath_Result__fini(&array->data[i]);
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

xline_path_planner__action__PlanPath_Result__Sequence *
xline_path_planner__action__PlanPath_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_Result__Sequence * array = (xline_path_planner__action__PlanPath_Result__Sequence *)allocator.allocate(sizeof(xline_path_planner__action__PlanPath_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xline_path_planner__action__PlanPath_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xline_path_planner__action__PlanPath_Result__Sequence__destroy(xline_path_planner__action__PlanPath_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xline_path_planner__action__PlanPath_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xline_path_planner__action__PlanPath_Result__Sequence__are_equal(const xline_path_planner__action__PlanPath_Result__Sequence * lhs, const xline_path_planner__action__PlanPath_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xline_path_planner__action__PlanPath_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xline_path_planner__action__PlanPath_Result__Sequence__copy(
  const xline_path_planner__action__PlanPath_Result__Sequence * input,
  xline_path_planner__action__PlanPath_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xline_path_planner__action__PlanPath_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xline_path_planner__action__PlanPath_Result * data =
      (xline_path_planner__action__PlanPath_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xline_path_planner__action__PlanPath_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xline_path_planner__action__PlanPath_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xline_path_planner__action__PlanPath_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stage`
// Member `info`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
xline_path_planner__action__PlanPath_Feedback__init(xline_path_planner__action__PlanPath_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // progress
  // stage
  if (!rosidl_runtime_c__String__init(&msg->stage)) {
    xline_path_planner__action__PlanPath_Feedback__fini(msg);
    return false;
  }
  // info
  if (!rosidl_runtime_c__String__init(&msg->info)) {
    xline_path_planner__action__PlanPath_Feedback__fini(msg);
    return false;
  }
  return true;
}

void
xline_path_planner__action__PlanPath_Feedback__fini(xline_path_planner__action__PlanPath_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // progress
  // stage
  rosidl_runtime_c__String__fini(&msg->stage);
  // info
  rosidl_runtime_c__String__fini(&msg->info);
}

bool
xline_path_planner__action__PlanPath_Feedback__are_equal(const xline_path_planner__action__PlanPath_Feedback * lhs, const xline_path_planner__action__PlanPath_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // progress
  if (lhs->progress != rhs->progress) {
    return false;
  }
  // stage
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->stage), &(rhs->stage)))
  {
    return false;
  }
  // info
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  return true;
}

bool
xline_path_planner__action__PlanPath_Feedback__copy(
  const xline_path_planner__action__PlanPath_Feedback * input,
  xline_path_planner__action__PlanPath_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // progress
  output->progress = input->progress;
  // stage
  if (!rosidl_runtime_c__String__copy(
      &(input->stage), &(output->stage)))
  {
    return false;
  }
  // info
  if (!rosidl_runtime_c__String__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  return true;
}

xline_path_planner__action__PlanPath_Feedback *
xline_path_planner__action__PlanPath_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_Feedback * msg = (xline_path_planner__action__PlanPath_Feedback *)allocator.allocate(sizeof(xline_path_planner__action__PlanPath_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xline_path_planner__action__PlanPath_Feedback));
  bool success = xline_path_planner__action__PlanPath_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xline_path_planner__action__PlanPath_Feedback__destroy(xline_path_planner__action__PlanPath_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xline_path_planner__action__PlanPath_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xline_path_planner__action__PlanPath_Feedback__Sequence__init(xline_path_planner__action__PlanPath_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_Feedback * data = NULL;

  if (size) {
    data = (xline_path_planner__action__PlanPath_Feedback *)allocator.zero_allocate(size, sizeof(xline_path_planner__action__PlanPath_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xline_path_planner__action__PlanPath_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xline_path_planner__action__PlanPath_Feedback__fini(&data[i - 1]);
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
xline_path_planner__action__PlanPath_Feedback__Sequence__fini(xline_path_planner__action__PlanPath_Feedback__Sequence * array)
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
      xline_path_planner__action__PlanPath_Feedback__fini(&array->data[i]);
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

xline_path_planner__action__PlanPath_Feedback__Sequence *
xline_path_planner__action__PlanPath_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_Feedback__Sequence * array = (xline_path_planner__action__PlanPath_Feedback__Sequence *)allocator.allocate(sizeof(xline_path_planner__action__PlanPath_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xline_path_planner__action__PlanPath_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xline_path_planner__action__PlanPath_Feedback__Sequence__destroy(xline_path_planner__action__PlanPath_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xline_path_planner__action__PlanPath_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xline_path_planner__action__PlanPath_Feedback__Sequence__are_equal(const xline_path_planner__action__PlanPath_Feedback__Sequence * lhs, const xline_path_planner__action__PlanPath_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xline_path_planner__action__PlanPath_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xline_path_planner__action__PlanPath_Feedback__Sequence__copy(
  const xline_path_planner__action__PlanPath_Feedback__Sequence * input,
  xline_path_planner__action__PlanPath_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xline_path_planner__action__PlanPath_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xline_path_planner__action__PlanPath_Feedback * data =
      (xline_path_planner__action__PlanPath_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xline_path_planner__action__PlanPath_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xline_path_planner__action__PlanPath_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xline_path_planner__action__PlanPath_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "xline_path_planner/action/detail/plan_path__functions.h"

bool
xline_path_planner__action__PlanPath_SendGoal_Request__init(xline_path_planner__action__PlanPath_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    xline_path_planner__action__PlanPath_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!xline_path_planner__action__PlanPath_Goal__init(&msg->goal)) {
    xline_path_planner__action__PlanPath_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
xline_path_planner__action__PlanPath_SendGoal_Request__fini(xline_path_planner__action__PlanPath_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  xline_path_planner__action__PlanPath_Goal__fini(&msg->goal);
}

bool
xline_path_planner__action__PlanPath_SendGoal_Request__are_equal(const xline_path_planner__action__PlanPath_SendGoal_Request * lhs, const xline_path_planner__action__PlanPath_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!xline_path_planner__action__PlanPath_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
xline_path_planner__action__PlanPath_SendGoal_Request__copy(
  const xline_path_planner__action__PlanPath_SendGoal_Request * input,
  xline_path_planner__action__PlanPath_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!xline_path_planner__action__PlanPath_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

xline_path_planner__action__PlanPath_SendGoal_Request *
xline_path_planner__action__PlanPath_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_SendGoal_Request * msg = (xline_path_planner__action__PlanPath_SendGoal_Request *)allocator.allocate(sizeof(xline_path_planner__action__PlanPath_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xline_path_planner__action__PlanPath_SendGoal_Request));
  bool success = xline_path_planner__action__PlanPath_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xline_path_planner__action__PlanPath_SendGoal_Request__destroy(xline_path_planner__action__PlanPath_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xline_path_planner__action__PlanPath_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xline_path_planner__action__PlanPath_SendGoal_Request__Sequence__init(xline_path_planner__action__PlanPath_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_SendGoal_Request * data = NULL;

  if (size) {
    data = (xline_path_planner__action__PlanPath_SendGoal_Request *)allocator.zero_allocate(size, sizeof(xline_path_planner__action__PlanPath_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xline_path_planner__action__PlanPath_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xline_path_planner__action__PlanPath_SendGoal_Request__fini(&data[i - 1]);
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
xline_path_planner__action__PlanPath_SendGoal_Request__Sequence__fini(xline_path_planner__action__PlanPath_SendGoal_Request__Sequence * array)
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
      xline_path_planner__action__PlanPath_SendGoal_Request__fini(&array->data[i]);
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

xline_path_planner__action__PlanPath_SendGoal_Request__Sequence *
xline_path_planner__action__PlanPath_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_SendGoal_Request__Sequence * array = (xline_path_planner__action__PlanPath_SendGoal_Request__Sequence *)allocator.allocate(sizeof(xline_path_planner__action__PlanPath_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xline_path_planner__action__PlanPath_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xline_path_planner__action__PlanPath_SendGoal_Request__Sequence__destroy(xline_path_planner__action__PlanPath_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xline_path_planner__action__PlanPath_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xline_path_planner__action__PlanPath_SendGoal_Request__Sequence__are_equal(const xline_path_planner__action__PlanPath_SendGoal_Request__Sequence * lhs, const xline_path_planner__action__PlanPath_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xline_path_planner__action__PlanPath_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xline_path_planner__action__PlanPath_SendGoal_Request__Sequence__copy(
  const xline_path_planner__action__PlanPath_SendGoal_Request__Sequence * input,
  xline_path_planner__action__PlanPath_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xline_path_planner__action__PlanPath_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xline_path_planner__action__PlanPath_SendGoal_Request * data =
      (xline_path_planner__action__PlanPath_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xline_path_planner__action__PlanPath_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xline_path_planner__action__PlanPath_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xline_path_planner__action__PlanPath_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
xline_path_planner__action__PlanPath_SendGoal_Response__init(xline_path_planner__action__PlanPath_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    xline_path_planner__action__PlanPath_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
xline_path_planner__action__PlanPath_SendGoal_Response__fini(xline_path_planner__action__PlanPath_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
xline_path_planner__action__PlanPath_SendGoal_Response__are_equal(const xline_path_planner__action__PlanPath_SendGoal_Response * lhs, const xline_path_planner__action__PlanPath_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
xline_path_planner__action__PlanPath_SendGoal_Response__copy(
  const xline_path_planner__action__PlanPath_SendGoal_Response * input,
  xline_path_planner__action__PlanPath_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

xline_path_planner__action__PlanPath_SendGoal_Response *
xline_path_planner__action__PlanPath_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_SendGoal_Response * msg = (xline_path_planner__action__PlanPath_SendGoal_Response *)allocator.allocate(sizeof(xline_path_planner__action__PlanPath_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xline_path_planner__action__PlanPath_SendGoal_Response));
  bool success = xline_path_planner__action__PlanPath_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xline_path_planner__action__PlanPath_SendGoal_Response__destroy(xline_path_planner__action__PlanPath_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xline_path_planner__action__PlanPath_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xline_path_planner__action__PlanPath_SendGoal_Response__Sequence__init(xline_path_planner__action__PlanPath_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_SendGoal_Response * data = NULL;

  if (size) {
    data = (xline_path_planner__action__PlanPath_SendGoal_Response *)allocator.zero_allocate(size, sizeof(xline_path_planner__action__PlanPath_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xline_path_planner__action__PlanPath_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xline_path_planner__action__PlanPath_SendGoal_Response__fini(&data[i - 1]);
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
xline_path_planner__action__PlanPath_SendGoal_Response__Sequence__fini(xline_path_planner__action__PlanPath_SendGoal_Response__Sequence * array)
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
      xline_path_planner__action__PlanPath_SendGoal_Response__fini(&array->data[i]);
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

xline_path_planner__action__PlanPath_SendGoal_Response__Sequence *
xline_path_planner__action__PlanPath_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_SendGoal_Response__Sequence * array = (xline_path_planner__action__PlanPath_SendGoal_Response__Sequence *)allocator.allocate(sizeof(xline_path_planner__action__PlanPath_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xline_path_planner__action__PlanPath_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xline_path_planner__action__PlanPath_SendGoal_Response__Sequence__destroy(xline_path_planner__action__PlanPath_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xline_path_planner__action__PlanPath_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xline_path_planner__action__PlanPath_SendGoal_Response__Sequence__are_equal(const xline_path_planner__action__PlanPath_SendGoal_Response__Sequence * lhs, const xline_path_planner__action__PlanPath_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xline_path_planner__action__PlanPath_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xline_path_planner__action__PlanPath_SendGoal_Response__Sequence__copy(
  const xline_path_planner__action__PlanPath_SendGoal_Response__Sequence * input,
  xline_path_planner__action__PlanPath_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xline_path_planner__action__PlanPath_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xline_path_planner__action__PlanPath_SendGoal_Response * data =
      (xline_path_planner__action__PlanPath_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xline_path_planner__action__PlanPath_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xline_path_planner__action__PlanPath_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xline_path_planner__action__PlanPath_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
xline_path_planner__action__PlanPath_GetResult_Request__init(xline_path_planner__action__PlanPath_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    xline_path_planner__action__PlanPath_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
xline_path_planner__action__PlanPath_GetResult_Request__fini(xline_path_planner__action__PlanPath_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
xline_path_planner__action__PlanPath_GetResult_Request__are_equal(const xline_path_planner__action__PlanPath_GetResult_Request * lhs, const xline_path_planner__action__PlanPath_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
xline_path_planner__action__PlanPath_GetResult_Request__copy(
  const xline_path_planner__action__PlanPath_GetResult_Request * input,
  xline_path_planner__action__PlanPath_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

xline_path_planner__action__PlanPath_GetResult_Request *
xline_path_planner__action__PlanPath_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_GetResult_Request * msg = (xline_path_planner__action__PlanPath_GetResult_Request *)allocator.allocate(sizeof(xline_path_planner__action__PlanPath_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xline_path_planner__action__PlanPath_GetResult_Request));
  bool success = xline_path_planner__action__PlanPath_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xline_path_planner__action__PlanPath_GetResult_Request__destroy(xline_path_planner__action__PlanPath_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xline_path_planner__action__PlanPath_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xline_path_planner__action__PlanPath_GetResult_Request__Sequence__init(xline_path_planner__action__PlanPath_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_GetResult_Request * data = NULL;

  if (size) {
    data = (xline_path_planner__action__PlanPath_GetResult_Request *)allocator.zero_allocate(size, sizeof(xline_path_planner__action__PlanPath_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xline_path_planner__action__PlanPath_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xline_path_planner__action__PlanPath_GetResult_Request__fini(&data[i - 1]);
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
xline_path_planner__action__PlanPath_GetResult_Request__Sequence__fini(xline_path_planner__action__PlanPath_GetResult_Request__Sequence * array)
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
      xline_path_planner__action__PlanPath_GetResult_Request__fini(&array->data[i]);
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

xline_path_planner__action__PlanPath_GetResult_Request__Sequence *
xline_path_planner__action__PlanPath_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_GetResult_Request__Sequence * array = (xline_path_planner__action__PlanPath_GetResult_Request__Sequence *)allocator.allocate(sizeof(xline_path_planner__action__PlanPath_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xline_path_planner__action__PlanPath_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xline_path_planner__action__PlanPath_GetResult_Request__Sequence__destroy(xline_path_planner__action__PlanPath_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xline_path_planner__action__PlanPath_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xline_path_planner__action__PlanPath_GetResult_Request__Sequence__are_equal(const xline_path_planner__action__PlanPath_GetResult_Request__Sequence * lhs, const xline_path_planner__action__PlanPath_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xline_path_planner__action__PlanPath_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xline_path_planner__action__PlanPath_GetResult_Request__Sequence__copy(
  const xline_path_planner__action__PlanPath_GetResult_Request__Sequence * input,
  xline_path_planner__action__PlanPath_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xline_path_planner__action__PlanPath_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xline_path_planner__action__PlanPath_GetResult_Request * data =
      (xline_path_planner__action__PlanPath_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xline_path_planner__action__PlanPath_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xline_path_planner__action__PlanPath_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xline_path_planner__action__PlanPath_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "xline_path_planner/action/detail/plan_path__functions.h"

bool
xline_path_planner__action__PlanPath_GetResult_Response__init(xline_path_planner__action__PlanPath_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!xline_path_planner__action__PlanPath_Result__init(&msg->result)) {
    xline_path_planner__action__PlanPath_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
xline_path_planner__action__PlanPath_GetResult_Response__fini(xline_path_planner__action__PlanPath_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  xline_path_planner__action__PlanPath_Result__fini(&msg->result);
}

bool
xline_path_planner__action__PlanPath_GetResult_Response__are_equal(const xline_path_planner__action__PlanPath_GetResult_Response * lhs, const xline_path_planner__action__PlanPath_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!xline_path_planner__action__PlanPath_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
xline_path_planner__action__PlanPath_GetResult_Response__copy(
  const xline_path_planner__action__PlanPath_GetResult_Response * input,
  xline_path_planner__action__PlanPath_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!xline_path_planner__action__PlanPath_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

xline_path_planner__action__PlanPath_GetResult_Response *
xline_path_planner__action__PlanPath_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_GetResult_Response * msg = (xline_path_planner__action__PlanPath_GetResult_Response *)allocator.allocate(sizeof(xline_path_planner__action__PlanPath_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xline_path_planner__action__PlanPath_GetResult_Response));
  bool success = xline_path_planner__action__PlanPath_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xline_path_planner__action__PlanPath_GetResult_Response__destroy(xline_path_planner__action__PlanPath_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xline_path_planner__action__PlanPath_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xline_path_planner__action__PlanPath_GetResult_Response__Sequence__init(xline_path_planner__action__PlanPath_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_GetResult_Response * data = NULL;

  if (size) {
    data = (xline_path_planner__action__PlanPath_GetResult_Response *)allocator.zero_allocate(size, sizeof(xline_path_planner__action__PlanPath_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xline_path_planner__action__PlanPath_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xline_path_planner__action__PlanPath_GetResult_Response__fini(&data[i - 1]);
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
xline_path_planner__action__PlanPath_GetResult_Response__Sequence__fini(xline_path_planner__action__PlanPath_GetResult_Response__Sequence * array)
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
      xline_path_planner__action__PlanPath_GetResult_Response__fini(&array->data[i]);
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

xline_path_planner__action__PlanPath_GetResult_Response__Sequence *
xline_path_planner__action__PlanPath_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_GetResult_Response__Sequence * array = (xline_path_planner__action__PlanPath_GetResult_Response__Sequence *)allocator.allocate(sizeof(xline_path_planner__action__PlanPath_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xline_path_planner__action__PlanPath_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xline_path_planner__action__PlanPath_GetResult_Response__Sequence__destroy(xline_path_planner__action__PlanPath_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xline_path_planner__action__PlanPath_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xline_path_planner__action__PlanPath_GetResult_Response__Sequence__are_equal(const xline_path_planner__action__PlanPath_GetResult_Response__Sequence * lhs, const xline_path_planner__action__PlanPath_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xline_path_planner__action__PlanPath_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xline_path_planner__action__PlanPath_GetResult_Response__Sequence__copy(
  const xline_path_planner__action__PlanPath_GetResult_Response__Sequence * input,
  xline_path_planner__action__PlanPath_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xline_path_planner__action__PlanPath_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xline_path_planner__action__PlanPath_GetResult_Response * data =
      (xline_path_planner__action__PlanPath_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xline_path_planner__action__PlanPath_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xline_path_planner__action__PlanPath_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xline_path_planner__action__PlanPath_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "xline_path_planner/action/detail/plan_path__functions.h"

bool
xline_path_planner__action__PlanPath_FeedbackMessage__init(xline_path_planner__action__PlanPath_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    xline_path_planner__action__PlanPath_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!xline_path_planner__action__PlanPath_Feedback__init(&msg->feedback)) {
    xline_path_planner__action__PlanPath_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
xline_path_planner__action__PlanPath_FeedbackMessage__fini(xline_path_planner__action__PlanPath_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  xline_path_planner__action__PlanPath_Feedback__fini(&msg->feedback);
}

bool
xline_path_planner__action__PlanPath_FeedbackMessage__are_equal(const xline_path_planner__action__PlanPath_FeedbackMessage * lhs, const xline_path_planner__action__PlanPath_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!xline_path_planner__action__PlanPath_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
xline_path_planner__action__PlanPath_FeedbackMessage__copy(
  const xline_path_planner__action__PlanPath_FeedbackMessage * input,
  xline_path_planner__action__PlanPath_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!xline_path_planner__action__PlanPath_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

xline_path_planner__action__PlanPath_FeedbackMessage *
xline_path_planner__action__PlanPath_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_FeedbackMessage * msg = (xline_path_planner__action__PlanPath_FeedbackMessage *)allocator.allocate(sizeof(xline_path_planner__action__PlanPath_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xline_path_planner__action__PlanPath_FeedbackMessage));
  bool success = xline_path_planner__action__PlanPath_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xline_path_planner__action__PlanPath_FeedbackMessage__destroy(xline_path_planner__action__PlanPath_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xline_path_planner__action__PlanPath_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xline_path_planner__action__PlanPath_FeedbackMessage__Sequence__init(xline_path_planner__action__PlanPath_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_FeedbackMessage * data = NULL;

  if (size) {
    data = (xline_path_planner__action__PlanPath_FeedbackMessage *)allocator.zero_allocate(size, sizeof(xline_path_planner__action__PlanPath_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xline_path_planner__action__PlanPath_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xline_path_planner__action__PlanPath_FeedbackMessage__fini(&data[i - 1]);
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
xline_path_planner__action__PlanPath_FeedbackMessage__Sequence__fini(xline_path_planner__action__PlanPath_FeedbackMessage__Sequence * array)
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
      xline_path_planner__action__PlanPath_FeedbackMessage__fini(&array->data[i]);
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

xline_path_planner__action__PlanPath_FeedbackMessage__Sequence *
xline_path_planner__action__PlanPath_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xline_path_planner__action__PlanPath_FeedbackMessage__Sequence * array = (xline_path_planner__action__PlanPath_FeedbackMessage__Sequence *)allocator.allocate(sizeof(xline_path_planner__action__PlanPath_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xline_path_planner__action__PlanPath_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xline_path_planner__action__PlanPath_FeedbackMessage__Sequence__destroy(xline_path_planner__action__PlanPath_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xline_path_planner__action__PlanPath_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xline_path_planner__action__PlanPath_FeedbackMessage__Sequence__are_equal(const xline_path_planner__action__PlanPath_FeedbackMessage__Sequence * lhs, const xline_path_planner__action__PlanPath_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xline_path_planner__action__PlanPath_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xline_path_planner__action__PlanPath_FeedbackMessage__Sequence__copy(
  const xline_path_planner__action__PlanPath_FeedbackMessage__Sequence * input,
  xline_path_planner__action__PlanPath_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xline_path_planner__action__PlanPath_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xline_path_planner__action__PlanPath_FeedbackMessage * data =
      (xline_path_planner__action__PlanPath_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xline_path_planner__action__PlanPath_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xline_path_planner__action__PlanPath_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xline_path_planner__action__PlanPath_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
