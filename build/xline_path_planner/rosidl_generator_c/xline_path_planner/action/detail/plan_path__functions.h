// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from xline_path_planner:action/PlanPath.idl
// generated code does not contain a copyright notice

#ifndef XLINE_PATH_PLANNER__ACTION__DETAIL__PLAN_PATH__FUNCTIONS_H_
#define XLINE_PATH_PLANNER__ACTION__DETAIL__PLAN_PATH__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "xline_path_planner/msg/rosidl_generator_c__visibility_control.h"

#include "xline_path_planner/action/detail/plan_path__struct.h"

/// Initialize action/PlanPath message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * xline_path_planner__action__PlanPath_Goal
 * )) before or use
 * xline_path_planner__action__PlanPath_Goal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_Goal__init(xline_path_planner__action__PlanPath_Goal * msg);

/// Finalize action/PlanPath message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_Goal__fini(xline_path_planner__action__PlanPath_Goal * msg);

/// Create action/PlanPath message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * xline_path_planner__action__PlanPath_Goal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
xline_path_planner__action__PlanPath_Goal *
xline_path_planner__action__PlanPath_Goal__create();

/// Destroy action/PlanPath message.
/**
 * It calls
 * xline_path_planner__action__PlanPath_Goal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_Goal__destroy(xline_path_planner__action__PlanPath_Goal * msg);

/// Check for action/PlanPath message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_Goal__are_equal(const xline_path_planner__action__PlanPath_Goal * lhs, const xline_path_planner__action__PlanPath_Goal * rhs);

/// Copy a action/PlanPath message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_Goal__copy(
  const xline_path_planner__action__PlanPath_Goal * input,
  xline_path_planner__action__PlanPath_Goal * output);

/// Initialize array of action/PlanPath messages.
/**
 * It allocates the memory for the number of elements and calls
 * xline_path_planner__action__PlanPath_Goal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_Goal__Sequence__init(xline_path_planner__action__PlanPath_Goal__Sequence * array, size_t size);

/// Finalize array of action/PlanPath messages.
/**
 * It calls
 * xline_path_planner__action__PlanPath_Goal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_Goal__Sequence__fini(xline_path_planner__action__PlanPath_Goal__Sequence * array);

/// Create array of action/PlanPath messages.
/**
 * It allocates the memory for the array and calls
 * xline_path_planner__action__PlanPath_Goal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
xline_path_planner__action__PlanPath_Goal__Sequence *
xline_path_planner__action__PlanPath_Goal__Sequence__create(size_t size);

/// Destroy array of action/PlanPath messages.
/**
 * It calls
 * xline_path_planner__action__PlanPath_Goal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_Goal__Sequence__destroy(xline_path_planner__action__PlanPath_Goal__Sequence * array);

/// Check for action/PlanPath message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_Goal__Sequence__are_equal(const xline_path_planner__action__PlanPath_Goal__Sequence * lhs, const xline_path_planner__action__PlanPath_Goal__Sequence * rhs);

/// Copy an array of action/PlanPath messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_Goal__Sequence__copy(
  const xline_path_planner__action__PlanPath_Goal__Sequence * input,
  xline_path_planner__action__PlanPath_Goal__Sequence * output);

/// Initialize action/PlanPath message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * xline_path_planner__action__PlanPath_Result
 * )) before or use
 * xline_path_planner__action__PlanPath_Result__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_Result__init(xline_path_planner__action__PlanPath_Result * msg);

/// Finalize action/PlanPath message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_Result__fini(xline_path_planner__action__PlanPath_Result * msg);

/// Create action/PlanPath message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * xline_path_planner__action__PlanPath_Result__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
xline_path_planner__action__PlanPath_Result *
xline_path_planner__action__PlanPath_Result__create();

/// Destroy action/PlanPath message.
/**
 * It calls
 * xline_path_planner__action__PlanPath_Result__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_Result__destroy(xline_path_planner__action__PlanPath_Result * msg);

/// Check for action/PlanPath message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_Result__are_equal(const xline_path_planner__action__PlanPath_Result * lhs, const xline_path_planner__action__PlanPath_Result * rhs);

/// Copy a action/PlanPath message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_Result__copy(
  const xline_path_planner__action__PlanPath_Result * input,
  xline_path_planner__action__PlanPath_Result * output);

/// Initialize array of action/PlanPath messages.
/**
 * It allocates the memory for the number of elements and calls
 * xline_path_planner__action__PlanPath_Result__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_Result__Sequence__init(xline_path_planner__action__PlanPath_Result__Sequence * array, size_t size);

/// Finalize array of action/PlanPath messages.
/**
 * It calls
 * xline_path_planner__action__PlanPath_Result__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_Result__Sequence__fini(xline_path_planner__action__PlanPath_Result__Sequence * array);

/// Create array of action/PlanPath messages.
/**
 * It allocates the memory for the array and calls
 * xline_path_planner__action__PlanPath_Result__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
xline_path_planner__action__PlanPath_Result__Sequence *
xline_path_planner__action__PlanPath_Result__Sequence__create(size_t size);

/// Destroy array of action/PlanPath messages.
/**
 * It calls
 * xline_path_planner__action__PlanPath_Result__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_Result__Sequence__destroy(xline_path_planner__action__PlanPath_Result__Sequence * array);

/// Check for action/PlanPath message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_Result__Sequence__are_equal(const xline_path_planner__action__PlanPath_Result__Sequence * lhs, const xline_path_planner__action__PlanPath_Result__Sequence * rhs);

/// Copy an array of action/PlanPath messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_Result__Sequence__copy(
  const xline_path_planner__action__PlanPath_Result__Sequence * input,
  xline_path_planner__action__PlanPath_Result__Sequence * output);

/// Initialize action/PlanPath message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * xline_path_planner__action__PlanPath_Feedback
 * )) before or use
 * xline_path_planner__action__PlanPath_Feedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_Feedback__init(xline_path_planner__action__PlanPath_Feedback * msg);

/// Finalize action/PlanPath message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_Feedback__fini(xline_path_planner__action__PlanPath_Feedback * msg);

/// Create action/PlanPath message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * xline_path_planner__action__PlanPath_Feedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
xline_path_planner__action__PlanPath_Feedback *
xline_path_planner__action__PlanPath_Feedback__create();

/// Destroy action/PlanPath message.
/**
 * It calls
 * xline_path_planner__action__PlanPath_Feedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_Feedback__destroy(xline_path_planner__action__PlanPath_Feedback * msg);

/// Check for action/PlanPath message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_Feedback__are_equal(const xline_path_planner__action__PlanPath_Feedback * lhs, const xline_path_planner__action__PlanPath_Feedback * rhs);

/// Copy a action/PlanPath message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_Feedback__copy(
  const xline_path_planner__action__PlanPath_Feedback * input,
  xline_path_planner__action__PlanPath_Feedback * output);

/// Initialize array of action/PlanPath messages.
/**
 * It allocates the memory for the number of elements and calls
 * xline_path_planner__action__PlanPath_Feedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_Feedback__Sequence__init(xline_path_planner__action__PlanPath_Feedback__Sequence * array, size_t size);

/// Finalize array of action/PlanPath messages.
/**
 * It calls
 * xline_path_planner__action__PlanPath_Feedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_Feedback__Sequence__fini(xline_path_planner__action__PlanPath_Feedback__Sequence * array);

/// Create array of action/PlanPath messages.
/**
 * It allocates the memory for the array and calls
 * xline_path_planner__action__PlanPath_Feedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
xline_path_planner__action__PlanPath_Feedback__Sequence *
xline_path_planner__action__PlanPath_Feedback__Sequence__create(size_t size);

/// Destroy array of action/PlanPath messages.
/**
 * It calls
 * xline_path_planner__action__PlanPath_Feedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_Feedback__Sequence__destroy(xline_path_planner__action__PlanPath_Feedback__Sequence * array);

/// Check for action/PlanPath message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_Feedback__Sequence__are_equal(const xline_path_planner__action__PlanPath_Feedback__Sequence * lhs, const xline_path_planner__action__PlanPath_Feedback__Sequence * rhs);

/// Copy an array of action/PlanPath messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_Feedback__Sequence__copy(
  const xline_path_planner__action__PlanPath_Feedback__Sequence * input,
  xline_path_planner__action__PlanPath_Feedback__Sequence * output);

/// Initialize action/PlanPath message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * xline_path_planner__action__PlanPath_SendGoal_Request
 * )) before or use
 * xline_path_planner__action__PlanPath_SendGoal_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_SendGoal_Request__init(xline_path_planner__action__PlanPath_SendGoal_Request * msg);

/// Finalize action/PlanPath message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_SendGoal_Request__fini(xline_path_planner__action__PlanPath_SendGoal_Request * msg);

/// Create action/PlanPath message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * xline_path_planner__action__PlanPath_SendGoal_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
xline_path_planner__action__PlanPath_SendGoal_Request *
xline_path_planner__action__PlanPath_SendGoal_Request__create();

/// Destroy action/PlanPath message.
/**
 * It calls
 * xline_path_planner__action__PlanPath_SendGoal_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_SendGoal_Request__destroy(xline_path_planner__action__PlanPath_SendGoal_Request * msg);

/// Check for action/PlanPath message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_SendGoal_Request__are_equal(const xline_path_planner__action__PlanPath_SendGoal_Request * lhs, const xline_path_planner__action__PlanPath_SendGoal_Request * rhs);

/// Copy a action/PlanPath message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_SendGoal_Request__copy(
  const xline_path_planner__action__PlanPath_SendGoal_Request * input,
  xline_path_planner__action__PlanPath_SendGoal_Request * output);

/// Initialize array of action/PlanPath messages.
/**
 * It allocates the memory for the number of elements and calls
 * xline_path_planner__action__PlanPath_SendGoal_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_SendGoal_Request__Sequence__init(xline_path_planner__action__PlanPath_SendGoal_Request__Sequence * array, size_t size);

/// Finalize array of action/PlanPath messages.
/**
 * It calls
 * xline_path_planner__action__PlanPath_SendGoal_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_SendGoal_Request__Sequence__fini(xline_path_planner__action__PlanPath_SendGoal_Request__Sequence * array);

/// Create array of action/PlanPath messages.
/**
 * It allocates the memory for the array and calls
 * xline_path_planner__action__PlanPath_SendGoal_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
xline_path_planner__action__PlanPath_SendGoal_Request__Sequence *
xline_path_planner__action__PlanPath_SendGoal_Request__Sequence__create(size_t size);

/// Destroy array of action/PlanPath messages.
/**
 * It calls
 * xline_path_planner__action__PlanPath_SendGoal_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_SendGoal_Request__Sequence__destroy(xline_path_planner__action__PlanPath_SendGoal_Request__Sequence * array);

/// Check for action/PlanPath message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_SendGoal_Request__Sequence__are_equal(const xline_path_planner__action__PlanPath_SendGoal_Request__Sequence * lhs, const xline_path_planner__action__PlanPath_SendGoal_Request__Sequence * rhs);

/// Copy an array of action/PlanPath messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_SendGoal_Request__Sequence__copy(
  const xline_path_planner__action__PlanPath_SendGoal_Request__Sequence * input,
  xline_path_planner__action__PlanPath_SendGoal_Request__Sequence * output);

/// Initialize action/PlanPath message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * xline_path_planner__action__PlanPath_SendGoal_Response
 * )) before or use
 * xline_path_planner__action__PlanPath_SendGoal_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_SendGoal_Response__init(xline_path_planner__action__PlanPath_SendGoal_Response * msg);

/// Finalize action/PlanPath message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_SendGoal_Response__fini(xline_path_planner__action__PlanPath_SendGoal_Response * msg);

/// Create action/PlanPath message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * xline_path_planner__action__PlanPath_SendGoal_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
xline_path_planner__action__PlanPath_SendGoal_Response *
xline_path_planner__action__PlanPath_SendGoal_Response__create();

/// Destroy action/PlanPath message.
/**
 * It calls
 * xline_path_planner__action__PlanPath_SendGoal_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_SendGoal_Response__destroy(xline_path_planner__action__PlanPath_SendGoal_Response * msg);

/// Check for action/PlanPath message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_SendGoal_Response__are_equal(const xline_path_planner__action__PlanPath_SendGoal_Response * lhs, const xline_path_planner__action__PlanPath_SendGoal_Response * rhs);

/// Copy a action/PlanPath message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_SendGoal_Response__copy(
  const xline_path_planner__action__PlanPath_SendGoal_Response * input,
  xline_path_planner__action__PlanPath_SendGoal_Response * output);

/// Initialize array of action/PlanPath messages.
/**
 * It allocates the memory for the number of elements and calls
 * xline_path_planner__action__PlanPath_SendGoal_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_SendGoal_Response__Sequence__init(xline_path_planner__action__PlanPath_SendGoal_Response__Sequence * array, size_t size);

/// Finalize array of action/PlanPath messages.
/**
 * It calls
 * xline_path_planner__action__PlanPath_SendGoal_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_SendGoal_Response__Sequence__fini(xline_path_planner__action__PlanPath_SendGoal_Response__Sequence * array);

/// Create array of action/PlanPath messages.
/**
 * It allocates the memory for the array and calls
 * xline_path_planner__action__PlanPath_SendGoal_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
xline_path_planner__action__PlanPath_SendGoal_Response__Sequence *
xline_path_planner__action__PlanPath_SendGoal_Response__Sequence__create(size_t size);

/// Destroy array of action/PlanPath messages.
/**
 * It calls
 * xline_path_planner__action__PlanPath_SendGoal_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_SendGoal_Response__Sequence__destroy(xline_path_planner__action__PlanPath_SendGoal_Response__Sequence * array);

/// Check for action/PlanPath message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_SendGoal_Response__Sequence__are_equal(const xline_path_planner__action__PlanPath_SendGoal_Response__Sequence * lhs, const xline_path_planner__action__PlanPath_SendGoal_Response__Sequence * rhs);

/// Copy an array of action/PlanPath messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_SendGoal_Response__Sequence__copy(
  const xline_path_planner__action__PlanPath_SendGoal_Response__Sequence * input,
  xline_path_planner__action__PlanPath_SendGoal_Response__Sequence * output);

/// Initialize action/PlanPath message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * xline_path_planner__action__PlanPath_GetResult_Request
 * )) before or use
 * xline_path_planner__action__PlanPath_GetResult_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_GetResult_Request__init(xline_path_planner__action__PlanPath_GetResult_Request * msg);

/// Finalize action/PlanPath message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_GetResult_Request__fini(xline_path_planner__action__PlanPath_GetResult_Request * msg);

/// Create action/PlanPath message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * xline_path_planner__action__PlanPath_GetResult_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
xline_path_planner__action__PlanPath_GetResult_Request *
xline_path_planner__action__PlanPath_GetResult_Request__create();

/// Destroy action/PlanPath message.
/**
 * It calls
 * xline_path_planner__action__PlanPath_GetResult_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_GetResult_Request__destroy(xline_path_planner__action__PlanPath_GetResult_Request * msg);

/// Check for action/PlanPath message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_GetResult_Request__are_equal(const xline_path_planner__action__PlanPath_GetResult_Request * lhs, const xline_path_planner__action__PlanPath_GetResult_Request * rhs);

/// Copy a action/PlanPath message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_GetResult_Request__copy(
  const xline_path_planner__action__PlanPath_GetResult_Request * input,
  xline_path_planner__action__PlanPath_GetResult_Request * output);

/// Initialize array of action/PlanPath messages.
/**
 * It allocates the memory for the number of elements and calls
 * xline_path_planner__action__PlanPath_GetResult_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_GetResult_Request__Sequence__init(xline_path_planner__action__PlanPath_GetResult_Request__Sequence * array, size_t size);

/// Finalize array of action/PlanPath messages.
/**
 * It calls
 * xline_path_planner__action__PlanPath_GetResult_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_GetResult_Request__Sequence__fini(xline_path_planner__action__PlanPath_GetResult_Request__Sequence * array);

/// Create array of action/PlanPath messages.
/**
 * It allocates the memory for the array and calls
 * xline_path_planner__action__PlanPath_GetResult_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
xline_path_planner__action__PlanPath_GetResult_Request__Sequence *
xline_path_planner__action__PlanPath_GetResult_Request__Sequence__create(size_t size);

/// Destroy array of action/PlanPath messages.
/**
 * It calls
 * xline_path_planner__action__PlanPath_GetResult_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_GetResult_Request__Sequence__destroy(xline_path_planner__action__PlanPath_GetResult_Request__Sequence * array);

/// Check for action/PlanPath message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_GetResult_Request__Sequence__are_equal(const xline_path_planner__action__PlanPath_GetResult_Request__Sequence * lhs, const xline_path_planner__action__PlanPath_GetResult_Request__Sequence * rhs);

/// Copy an array of action/PlanPath messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_GetResult_Request__Sequence__copy(
  const xline_path_planner__action__PlanPath_GetResult_Request__Sequence * input,
  xline_path_planner__action__PlanPath_GetResult_Request__Sequence * output);

/// Initialize action/PlanPath message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * xline_path_planner__action__PlanPath_GetResult_Response
 * )) before or use
 * xline_path_planner__action__PlanPath_GetResult_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_GetResult_Response__init(xline_path_planner__action__PlanPath_GetResult_Response * msg);

/// Finalize action/PlanPath message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_GetResult_Response__fini(xline_path_planner__action__PlanPath_GetResult_Response * msg);

/// Create action/PlanPath message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * xline_path_planner__action__PlanPath_GetResult_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
xline_path_planner__action__PlanPath_GetResult_Response *
xline_path_planner__action__PlanPath_GetResult_Response__create();

/// Destroy action/PlanPath message.
/**
 * It calls
 * xline_path_planner__action__PlanPath_GetResult_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_GetResult_Response__destroy(xline_path_planner__action__PlanPath_GetResult_Response * msg);

/// Check for action/PlanPath message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_GetResult_Response__are_equal(const xline_path_planner__action__PlanPath_GetResult_Response * lhs, const xline_path_planner__action__PlanPath_GetResult_Response * rhs);

/// Copy a action/PlanPath message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_GetResult_Response__copy(
  const xline_path_planner__action__PlanPath_GetResult_Response * input,
  xline_path_planner__action__PlanPath_GetResult_Response * output);

/// Initialize array of action/PlanPath messages.
/**
 * It allocates the memory for the number of elements and calls
 * xline_path_planner__action__PlanPath_GetResult_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_GetResult_Response__Sequence__init(xline_path_planner__action__PlanPath_GetResult_Response__Sequence * array, size_t size);

/// Finalize array of action/PlanPath messages.
/**
 * It calls
 * xline_path_planner__action__PlanPath_GetResult_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_GetResult_Response__Sequence__fini(xline_path_planner__action__PlanPath_GetResult_Response__Sequence * array);

/// Create array of action/PlanPath messages.
/**
 * It allocates the memory for the array and calls
 * xline_path_planner__action__PlanPath_GetResult_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
xline_path_planner__action__PlanPath_GetResult_Response__Sequence *
xline_path_planner__action__PlanPath_GetResult_Response__Sequence__create(size_t size);

/// Destroy array of action/PlanPath messages.
/**
 * It calls
 * xline_path_planner__action__PlanPath_GetResult_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_GetResult_Response__Sequence__destroy(xline_path_planner__action__PlanPath_GetResult_Response__Sequence * array);

/// Check for action/PlanPath message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_GetResult_Response__Sequence__are_equal(const xline_path_planner__action__PlanPath_GetResult_Response__Sequence * lhs, const xline_path_planner__action__PlanPath_GetResult_Response__Sequence * rhs);

/// Copy an array of action/PlanPath messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_GetResult_Response__Sequence__copy(
  const xline_path_planner__action__PlanPath_GetResult_Response__Sequence * input,
  xline_path_planner__action__PlanPath_GetResult_Response__Sequence * output);

/// Initialize action/PlanPath message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * xline_path_planner__action__PlanPath_FeedbackMessage
 * )) before or use
 * xline_path_planner__action__PlanPath_FeedbackMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_FeedbackMessage__init(xline_path_planner__action__PlanPath_FeedbackMessage * msg);

/// Finalize action/PlanPath message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_FeedbackMessage__fini(xline_path_planner__action__PlanPath_FeedbackMessage * msg);

/// Create action/PlanPath message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * xline_path_planner__action__PlanPath_FeedbackMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
xline_path_planner__action__PlanPath_FeedbackMessage *
xline_path_planner__action__PlanPath_FeedbackMessage__create();

/// Destroy action/PlanPath message.
/**
 * It calls
 * xline_path_planner__action__PlanPath_FeedbackMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_FeedbackMessage__destroy(xline_path_planner__action__PlanPath_FeedbackMessage * msg);

/// Check for action/PlanPath message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_FeedbackMessage__are_equal(const xline_path_planner__action__PlanPath_FeedbackMessage * lhs, const xline_path_planner__action__PlanPath_FeedbackMessage * rhs);

/// Copy a action/PlanPath message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_FeedbackMessage__copy(
  const xline_path_planner__action__PlanPath_FeedbackMessage * input,
  xline_path_planner__action__PlanPath_FeedbackMessage * output);

/// Initialize array of action/PlanPath messages.
/**
 * It allocates the memory for the number of elements and calls
 * xline_path_planner__action__PlanPath_FeedbackMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_FeedbackMessage__Sequence__init(xline_path_planner__action__PlanPath_FeedbackMessage__Sequence * array, size_t size);

/// Finalize array of action/PlanPath messages.
/**
 * It calls
 * xline_path_planner__action__PlanPath_FeedbackMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_FeedbackMessage__Sequence__fini(xline_path_planner__action__PlanPath_FeedbackMessage__Sequence * array);

/// Create array of action/PlanPath messages.
/**
 * It allocates the memory for the array and calls
 * xline_path_planner__action__PlanPath_FeedbackMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
xline_path_planner__action__PlanPath_FeedbackMessage__Sequence *
xline_path_planner__action__PlanPath_FeedbackMessage__Sequence__create(size_t size);

/// Destroy array of action/PlanPath messages.
/**
 * It calls
 * xline_path_planner__action__PlanPath_FeedbackMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
void
xline_path_planner__action__PlanPath_FeedbackMessage__Sequence__destroy(xline_path_planner__action__PlanPath_FeedbackMessage__Sequence * array);

/// Check for action/PlanPath message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_FeedbackMessage__Sequence__are_equal(const xline_path_planner__action__PlanPath_FeedbackMessage__Sequence * lhs, const xline_path_planner__action__PlanPath_FeedbackMessage__Sequence * rhs);

/// Copy an array of action/PlanPath messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_xline_path_planner
bool
xline_path_planner__action__PlanPath_FeedbackMessage__Sequence__copy(
  const xline_path_planner__action__PlanPath_FeedbackMessage__Sequence * input,
  xline_path_planner__action__PlanPath_FeedbackMessage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // XLINE_PATH_PLANNER__ACTION__DETAIL__PLAN_PATH__FUNCTIONS_H_
