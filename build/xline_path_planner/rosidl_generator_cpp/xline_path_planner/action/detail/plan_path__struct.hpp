// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from xline_path_planner:action/PlanPath.idl
// generated code does not contain a copyright notice

#ifndef XLINE_PATH_PLANNER__ACTION__DETAIL__PLAN_PATH__STRUCT_HPP_
#define XLINE_PATH_PLANNER__ACTION__DETAIL__PLAN_PATH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__xline_path_planner__action__PlanPath_Goal __attribute__((deprecated))
#else
# define DEPRECATED__xline_path_planner__action__PlanPath_Goal __declspec(deprecated)
#endif

namespace xline_path_planner
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanPath_Goal_
{
  using Type = PlanPath_Goal_<ContainerAllocator>;

  explicit PlanPath_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request_id = "";
      this->cad_json = "";
    }
  }

  explicit PlanPath_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : request_id(_alloc),
    cad_json(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request_id = "";
      this->cad_json = "";
    }
  }

  // field types and members
  using _request_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _request_id_type request_id;
  using _cad_json_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _cad_json_type cad_json;

  // setters for named parameter idiom
  Type & set__request_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->request_id = _arg;
    return *this;
  }
  Type & set__cad_json(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->cad_json = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xline_path_planner::action::PlanPath_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const xline_path_planner::action::PlanPath_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xline_path_planner::action::PlanPath_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xline_path_planner::action::PlanPath_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xline_path_planner::action::PlanPath_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xline_path_planner::action::PlanPath_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xline_path_planner::action::PlanPath_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xline_path_planner::action::PlanPath_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xline_path_planner::action::PlanPath_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xline_path_planner::action::PlanPath_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xline_path_planner__action__PlanPath_Goal
    std::shared_ptr<xline_path_planner::action::PlanPath_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xline_path_planner__action__PlanPath_Goal
    std::shared_ptr<xline_path_planner::action::PlanPath_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanPath_Goal_ & other) const
  {
    if (this->request_id != other.request_id) {
      return false;
    }
    if (this->cad_json != other.cad_json) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanPath_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanPath_Goal_

// alias to use template instance with default allocator
using PlanPath_Goal =
  xline_path_planner::action::PlanPath_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace xline_path_planner


#ifndef _WIN32
# define DEPRECATED__xline_path_planner__action__PlanPath_Result __attribute__((deprecated))
#else
# define DEPRECATED__xline_path_planner__action__PlanPath_Result __declspec(deprecated)
#endif

namespace xline_path_planner
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanPath_Result_
{
  using Type = PlanPath_Result_<ContainerAllocator>;

  explicit PlanPath_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->planned_json = "";
      this->error = "";
      this->warnings = "";
      this->metrics_json = "";
      this->planner_version = "";
      this->config_hash = "";
    }
  }

  explicit PlanPath_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : planned_json(_alloc),
    error(_alloc),
    warnings(_alloc),
    metrics_json(_alloc),
    planner_version(_alloc),
    config_hash(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->planned_json = "";
      this->error = "";
      this->warnings = "";
      this->metrics_json = "";
      this->planner_version = "";
      this->config_hash = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _planned_json_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _planned_json_type planned_json;
  using _error_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _error_type error;
  using _warnings_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _warnings_type warnings;
  using _metrics_json_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _metrics_json_type metrics_json;
  using _planner_version_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _planner_version_type planner_version;
  using _config_hash_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _config_hash_type config_hash;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__planned_json(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->planned_json = _arg;
    return *this;
  }
  Type & set__error(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->error = _arg;
    return *this;
  }
  Type & set__warnings(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->warnings = _arg;
    return *this;
  }
  Type & set__metrics_json(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->metrics_json = _arg;
    return *this;
  }
  Type & set__planner_version(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->planner_version = _arg;
    return *this;
  }
  Type & set__config_hash(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->config_hash = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xline_path_planner::action::PlanPath_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const xline_path_planner::action::PlanPath_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xline_path_planner::action::PlanPath_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xline_path_planner::action::PlanPath_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xline_path_planner::action::PlanPath_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xline_path_planner::action::PlanPath_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xline_path_planner::action::PlanPath_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xline_path_planner::action::PlanPath_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xline_path_planner::action::PlanPath_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xline_path_planner::action::PlanPath_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xline_path_planner__action__PlanPath_Result
    std::shared_ptr<xline_path_planner::action::PlanPath_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xline_path_planner__action__PlanPath_Result
    std::shared_ptr<xline_path_planner::action::PlanPath_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanPath_Result_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->planned_json != other.planned_json) {
      return false;
    }
    if (this->error != other.error) {
      return false;
    }
    if (this->warnings != other.warnings) {
      return false;
    }
    if (this->metrics_json != other.metrics_json) {
      return false;
    }
    if (this->planner_version != other.planner_version) {
      return false;
    }
    if (this->config_hash != other.config_hash) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanPath_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanPath_Result_

// alias to use template instance with default allocator
using PlanPath_Result =
  xline_path_planner::action::PlanPath_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace xline_path_planner


#ifndef _WIN32
# define DEPRECATED__xline_path_planner__action__PlanPath_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__xline_path_planner__action__PlanPath_Feedback __declspec(deprecated)
#endif

namespace xline_path_planner
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanPath_Feedback_
{
  using Type = PlanPath_Feedback_<ContainerAllocator>;

  explicit PlanPath_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->progress = 0;
      this->stage = "";
      this->info = "";
    }
  }

  explicit PlanPath_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stage(_alloc),
    info(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->progress = 0;
      this->stage = "";
      this->info = "";
    }
  }

  // field types and members
  using _progress_type =
    uint8_t;
  _progress_type progress;
  using _stage_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _stage_type stage;
  using _info_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _info_type info;

  // setters for named parameter idiom
  Type & set__progress(
    const uint8_t & _arg)
  {
    this->progress = _arg;
    return *this;
  }
  Type & set__stage(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->stage = _arg;
    return *this;
  }
  Type & set__info(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->info = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xline_path_planner::action::PlanPath_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const xline_path_planner::action::PlanPath_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xline_path_planner::action::PlanPath_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xline_path_planner::action::PlanPath_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xline_path_planner::action::PlanPath_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xline_path_planner::action::PlanPath_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xline_path_planner::action::PlanPath_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xline_path_planner::action::PlanPath_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xline_path_planner::action::PlanPath_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xline_path_planner::action::PlanPath_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xline_path_planner__action__PlanPath_Feedback
    std::shared_ptr<xline_path_planner::action::PlanPath_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xline_path_planner__action__PlanPath_Feedback
    std::shared_ptr<xline_path_planner::action::PlanPath_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanPath_Feedback_ & other) const
  {
    if (this->progress != other.progress) {
      return false;
    }
    if (this->stage != other.stage) {
      return false;
    }
    if (this->info != other.info) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanPath_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanPath_Feedback_

// alias to use template instance with default allocator
using PlanPath_Feedback =
  xline_path_planner::action::PlanPath_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace xline_path_planner


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "xline_path_planner/action/detail/plan_path__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__xline_path_planner__action__PlanPath_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__xline_path_planner__action__PlanPath_SendGoal_Request __declspec(deprecated)
#endif

namespace xline_path_planner
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanPath_SendGoal_Request_
{
  using Type = PlanPath_SendGoal_Request_<ContainerAllocator>;

  explicit PlanPath_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit PlanPath_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    xline_path_planner::action::PlanPath_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const xline_path_planner::action::PlanPath_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xline_path_planner::action::PlanPath_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const xline_path_planner::action::PlanPath_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xline_path_planner::action::PlanPath_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xline_path_planner::action::PlanPath_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xline_path_planner::action::PlanPath_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xline_path_planner::action::PlanPath_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xline_path_planner::action::PlanPath_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xline_path_planner::action::PlanPath_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xline_path_planner::action::PlanPath_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xline_path_planner::action::PlanPath_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xline_path_planner__action__PlanPath_SendGoal_Request
    std::shared_ptr<xline_path_planner::action::PlanPath_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xline_path_planner__action__PlanPath_SendGoal_Request
    std::shared_ptr<xline_path_planner::action::PlanPath_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanPath_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanPath_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanPath_SendGoal_Request_

// alias to use template instance with default allocator
using PlanPath_SendGoal_Request =
  xline_path_planner::action::PlanPath_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace xline_path_planner


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__xline_path_planner__action__PlanPath_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__xline_path_planner__action__PlanPath_SendGoal_Response __declspec(deprecated)
#endif

namespace xline_path_planner
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanPath_SendGoal_Response_
{
  using Type = PlanPath_SendGoal_Response_<ContainerAllocator>;

  explicit PlanPath_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit PlanPath_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xline_path_planner::action::PlanPath_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const xline_path_planner::action::PlanPath_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xline_path_planner::action::PlanPath_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xline_path_planner::action::PlanPath_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xline_path_planner::action::PlanPath_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xline_path_planner::action::PlanPath_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xline_path_planner::action::PlanPath_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xline_path_planner::action::PlanPath_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xline_path_planner::action::PlanPath_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xline_path_planner::action::PlanPath_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xline_path_planner__action__PlanPath_SendGoal_Response
    std::shared_ptr<xline_path_planner::action::PlanPath_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xline_path_planner__action__PlanPath_SendGoal_Response
    std::shared_ptr<xline_path_planner::action::PlanPath_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanPath_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanPath_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanPath_SendGoal_Response_

// alias to use template instance with default allocator
using PlanPath_SendGoal_Response =
  xline_path_planner::action::PlanPath_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace xline_path_planner

namespace xline_path_planner
{

namespace action
{

struct PlanPath_SendGoal
{
  using Request = xline_path_planner::action::PlanPath_SendGoal_Request;
  using Response = xline_path_planner::action::PlanPath_SendGoal_Response;
};

}  // namespace action

}  // namespace xline_path_planner


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__xline_path_planner__action__PlanPath_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__xline_path_planner__action__PlanPath_GetResult_Request __declspec(deprecated)
#endif

namespace xline_path_planner
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanPath_GetResult_Request_
{
  using Type = PlanPath_GetResult_Request_<ContainerAllocator>;

  explicit PlanPath_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit PlanPath_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xline_path_planner::action::PlanPath_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const xline_path_planner::action::PlanPath_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xline_path_planner::action::PlanPath_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xline_path_planner::action::PlanPath_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xline_path_planner::action::PlanPath_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xline_path_planner::action::PlanPath_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xline_path_planner::action::PlanPath_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xline_path_planner::action::PlanPath_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xline_path_planner::action::PlanPath_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xline_path_planner::action::PlanPath_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xline_path_planner__action__PlanPath_GetResult_Request
    std::shared_ptr<xline_path_planner::action::PlanPath_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xline_path_planner__action__PlanPath_GetResult_Request
    std::shared_ptr<xline_path_planner::action::PlanPath_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanPath_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanPath_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanPath_GetResult_Request_

// alias to use template instance with default allocator
using PlanPath_GetResult_Request =
  xline_path_planner::action::PlanPath_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace xline_path_planner


// Include directives for member types
// Member 'result'
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__xline_path_planner__action__PlanPath_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__xline_path_planner__action__PlanPath_GetResult_Response __declspec(deprecated)
#endif

namespace xline_path_planner
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanPath_GetResult_Response_
{
  using Type = PlanPath_GetResult_Response_<ContainerAllocator>;

  explicit PlanPath_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit PlanPath_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    xline_path_planner::action::PlanPath_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const xline_path_planner::action::PlanPath_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xline_path_planner::action::PlanPath_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const xline_path_planner::action::PlanPath_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xline_path_planner::action::PlanPath_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xline_path_planner::action::PlanPath_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xline_path_planner::action::PlanPath_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xline_path_planner::action::PlanPath_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xline_path_planner::action::PlanPath_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xline_path_planner::action::PlanPath_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xline_path_planner::action::PlanPath_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xline_path_planner::action::PlanPath_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xline_path_planner__action__PlanPath_GetResult_Response
    std::shared_ptr<xline_path_planner::action::PlanPath_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xline_path_planner__action__PlanPath_GetResult_Response
    std::shared_ptr<xline_path_planner::action::PlanPath_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanPath_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanPath_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanPath_GetResult_Response_

// alias to use template instance with default allocator
using PlanPath_GetResult_Response =
  xline_path_planner::action::PlanPath_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace xline_path_planner

namespace xline_path_planner
{

namespace action
{

struct PlanPath_GetResult
{
  using Request = xline_path_planner::action::PlanPath_GetResult_Request;
  using Response = xline_path_planner::action::PlanPath_GetResult_Response;
};

}  // namespace action

}  // namespace xline_path_planner


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "xline_path_planner/action/detail/plan_path__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__xline_path_planner__action__PlanPath_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__xline_path_planner__action__PlanPath_FeedbackMessage __declspec(deprecated)
#endif

namespace xline_path_planner
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanPath_FeedbackMessage_
{
  using Type = PlanPath_FeedbackMessage_<ContainerAllocator>;

  explicit PlanPath_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit PlanPath_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    xline_path_planner::action::PlanPath_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const xline_path_planner::action::PlanPath_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xline_path_planner::action::PlanPath_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const xline_path_planner::action::PlanPath_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xline_path_planner::action::PlanPath_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xline_path_planner::action::PlanPath_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xline_path_planner::action::PlanPath_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xline_path_planner::action::PlanPath_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xline_path_planner::action::PlanPath_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xline_path_planner::action::PlanPath_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xline_path_planner::action::PlanPath_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xline_path_planner::action::PlanPath_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xline_path_planner__action__PlanPath_FeedbackMessage
    std::shared_ptr<xline_path_planner::action::PlanPath_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xline_path_planner__action__PlanPath_FeedbackMessage
    std::shared_ptr<xline_path_planner::action::PlanPath_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanPath_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanPath_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanPath_FeedbackMessage_

// alias to use template instance with default allocator
using PlanPath_FeedbackMessage =
  xline_path_planner::action::PlanPath_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace xline_path_planner

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace xline_path_planner
{

namespace action
{

struct PlanPath
{
  /// The goal message defined in the action definition.
  using Goal = xline_path_planner::action::PlanPath_Goal;
  /// The result message defined in the action definition.
  using Result = xline_path_planner::action::PlanPath_Result;
  /// The feedback message defined in the action definition.
  using Feedback = xline_path_planner::action::PlanPath_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = xline_path_planner::action::PlanPath_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = xline_path_planner::action::PlanPath_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = xline_path_planner::action::PlanPath_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct PlanPath PlanPath;

}  // namespace action

}  // namespace xline_path_planner

#endif  // XLINE_PATH_PLANNER__ACTION__DETAIL__PLAN_PATH__STRUCT_HPP_
