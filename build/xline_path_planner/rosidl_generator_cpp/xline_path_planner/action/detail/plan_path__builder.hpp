// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from xline_path_planner:action/PlanPath.idl
// generated code does not contain a copyright notice

#ifndef XLINE_PATH_PLANNER__ACTION__DETAIL__PLAN_PATH__BUILDER_HPP_
#define XLINE_PATH_PLANNER__ACTION__DETAIL__PLAN_PATH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "xline_path_planner/action/detail/plan_path__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace xline_path_planner
{

namespace action
{

namespace builder
{

class Init_PlanPath_Goal_cad_json
{
public:
  explicit Init_PlanPath_Goal_cad_json(::xline_path_planner::action::PlanPath_Goal & msg)
  : msg_(msg)
  {}
  ::xline_path_planner::action::PlanPath_Goal cad_json(::xline_path_planner::action::PlanPath_Goal::_cad_json_type arg)
  {
    msg_.cad_json = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xline_path_planner::action::PlanPath_Goal msg_;
};

class Init_PlanPath_Goal_request_id
{
public:
  Init_PlanPath_Goal_request_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanPath_Goal_cad_json request_id(::xline_path_planner::action::PlanPath_Goal::_request_id_type arg)
  {
    msg_.request_id = std::move(arg);
    return Init_PlanPath_Goal_cad_json(msg_);
  }

private:
  ::xline_path_planner::action::PlanPath_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xline_path_planner::action::PlanPath_Goal>()
{
  return xline_path_planner::action::builder::Init_PlanPath_Goal_request_id();
}

}  // namespace xline_path_planner


namespace xline_path_planner
{

namespace action
{

namespace builder
{

class Init_PlanPath_Result_config_hash
{
public:
  explicit Init_PlanPath_Result_config_hash(::xline_path_planner::action::PlanPath_Result & msg)
  : msg_(msg)
  {}
  ::xline_path_planner::action::PlanPath_Result config_hash(::xline_path_planner::action::PlanPath_Result::_config_hash_type arg)
  {
    msg_.config_hash = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xline_path_planner::action::PlanPath_Result msg_;
};

class Init_PlanPath_Result_planner_version
{
public:
  explicit Init_PlanPath_Result_planner_version(::xline_path_planner::action::PlanPath_Result & msg)
  : msg_(msg)
  {}
  Init_PlanPath_Result_config_hash planner_version(::xline_path_planner::action::PlanPath_Result::_planner_version_type arg)
  {
    msg_.planner_version = std::move(arg);
    return Init_PlanPath_Result_config_hash(msg_);
  }

private:
  ::xline_path_planner::action::PlanPath_Result msg_;
};

class Init_PlanPath_Result_metrics_json
{
public:
  explicit Init_PlanPath_Result_metrics_json(::xline_path_planner::action::PlanPath_Result & msg)
  : msg_(msg)
  {}
  Init_PlanPath_Result_planner_version metrics_json(::xline_path_planner::action::PlanPath_Result::_metrics_json_type arg)
  {
    msg_.metrics_json = std::move(arg);
    return Init_PlanPath_Result_planner_version(msg_);
  }

private:
  ::xline_path_planner::action::PlanPath_Result msg_;
};

class Init_PlanPath_Result_warnings
{
public:
  explicit Init_PlanPath_Result_warnings(::xline_path_planner::action::PlanPath_Result & msg)
  : msg_(msg)
  {}
  Init_PlanPath_Result_metrics_json warnings(::xline_path_planner::action::PlanPath_Result::_warnings_type arg)
  {
    msg_.warnings = std::move(arg);
    return Init_PlanPath_Result_metrics_json(msg_);
  }

private:
  ::xline_path_planner::action::PlanPath_Result msg_;
};

class Init_PlanPath_Result_error
{
public:
  explicit Init_PlanPath_Result_error(::xline_path_planner::action::PlanPath_Result & msg)
  : msg_(msg)
  {}
  Init_PlanPath_Result_warnings error(::xline_path_planner::action::PlanPath_Result::_error_type arg)
  {
    msg_.error = std::move(arg);
    return Init_PlanPath_Result_warnings(msg_);
  }

private:
  ::xline_path_planner::action::PlanPath_Result msg_;
};

class Init_PlanPath_Result_planned_json
{
public:
  explicit Init_PlanPath_Result_planned_json(::xline_path_planner::action::PlanPath_Result & msg)
  : msg_(msg)
  {}
  Init_PlanPath_Result_error planned_json(::xline_path_planner::action::PlanPath_Result::_planned_json_type arg)
  {
    msg_.planned_json = std::move(arg);
    return Init_PlanPath_Result_error(msg_);
  }

private:
  ::xline_path_planner::action::PlanPath_Result msg_;
};

class Init_PlanPath_Result_success
{
public:
  Init_PlanPath_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanPath_Result_planned_json success(::xline_path_planner::action::PlanPath_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PlanPath_Result_planned_json(msg_);
  }

private:
  ::xline_path_planner::action::PlanPath_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xline_path_planner::action::PlanPath_Result>()
{
  return xline_path_planner::action::builder::Init_PlanPath_Result_success();
}

}  // namespace xline_path_planner


namespace xline_path_planner
{

namespace action
{

namespace builder
{

class Init_PlanPath_Feedback_info
{
public:
  explicit Init_PlanPath_Feedback_info(::xline_path_planner::action::PlanPath_Feedback & msg)
  : msg_(msg)
  {}
  ::xline_path_planner::action::PlanPath_Feedback info(::xline_path_planner::action::PlanPath_Feedback::_info_type arg)
  {
    msg_.info = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xline_path_planner::action::PlanPath_Feedback msg_;
};

class Init_PlanPath_Feedback_stage
{
public:
  explicit Init_PlanPath_Feedback_stage(::xline_path_planner::action::PlanPath_Feedback & msg)
  : msg_(msg)
  {}
  Init_PlanPath_Feedback_info stage(::xline_path_planner::action::PlanPath_Feedback::_stage_type arg)
  {
    msg_.stage = std::move(arg);
    return Init_PlanPath_Feedback_info(msg_);
  }

private:
  ::xline_path_planner::action::PlanPath_Feedback msg_;
};

class Init_PlanPath_Feedback_progress
{
public:
  Init_PlanPath_Feedback_progress()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanPath_Feedback_stage progress(::xline_path_planner::action::PlanPath_Feedback::_progress_type arg)
  {
    msg_.progress = std::move(arg);
    return Init_PlanPath_Feedback_stage(msg_);
  }

private:
  ::xline_path_planner::action::PlanPath_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xline_path_planner::action::PlanPath_Feedback>()
{
  return xline_path_planner::action::builder::Init_PlanPath_Feedback_progress();
}

}  // namespace xline_path_planner


namespace xline_path_planner
{

namespace action
{

namespace builder
{

class Init_PlanPath_SendGoal_Request_goal
{
public:
  explicit Init_PlanPath_SendGoal_Request_goal(::xline_path_planner::action::PlanPath_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::xline_path_planner::action::PlanPath_SendGoal_Request goal(::xline_path_planner::action::PlanPath_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xline_path_planner::action::PlanPath_SendGoal_Request msg_;
};

class Init_PlanPath_SendGoal_Request_goal_id
{
public:
  Init_PlanPath_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanPath_SendGoal_Request_goal goal_id(::xline_path_planner::action::PlanPath_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PlanPath_SendGoal_Request_goal(msg_);
  }

private:
  ::xline_path_planner::action::PlanPath_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xline_path_planner::action::PlanPath_SendGoal_Request>()
{
  return xline_path_planner::action::builder::Init_PlanPath_SendGoal_Request_goal_id();
}

}  // namespace xline_path_planner


namespace xline_path_planner
{

namespace action
{

namespace builder
{

class Init_PlanPath_SendGoal_Response_stamp
{
public:
  explicit Init_PlanPath_SendGoal_Response_stamp(::xline_path_planner::action::PlanPath_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::xline_path_planner::action::PlanPath_SendGoal_Response stamp(::xline_path_planner::action::PlanPath_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xline_path_planner::action::PlanPath_SendGoal_Response msg_;
};

class Init_PlanPath_SendGoal_Response_accepted
{
public:
  Init_PlanPath_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanPath_SendGoal_Response_stamp accepted(::xline_path_planner::action::PlanPath_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_PlanPath_SendGoal_Response_stamp(msg_);
  }

private:
  ::xline_path_planner::action::PlanPath_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xline_path_planner::action::PlanPath_SendGoal_Response>()
{
  return xline_path_planner::action::builder::Init_PlanPath_SendGoal_Response_accepted();
}

}  // namespace xline_path_planner


namespace xline_path_planner
{

namespace action
{

namespace builder
{

class Init_PlanPath_GetResult_Request_goal_id
{
public:
  Init_PlanPath_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::xline_path_planner::action::PlanPath_GetResult_Request goal_id(::xline_path_planner::action::PlanPath_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xline_path_planner::action::PlanPath_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xline_path_planner::action::PlanPath_GetResult_Request>()
{
  return xline_path_planner::action::builder::Init_PlanPath_GetResult_Request_goal_id();
}

}  // namespace xline_path_planner


namespace xline_path_planner
{

namespace action
{

namespace builder
{

class Init_PlanPath_GetResult_Response_result
{
public:
  explicit Init_PlanPath_GetResult_Response_result(::xline_path_planner::action::PlanPath_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::xline_path_planner::action::PlanPath_GetResult_Response result(::xline_path_planner::action::PlanPath_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xline_path_planner::action::PlanPath_GetResult_Response msg_;
};

class Init_PlanPath_GetResult_Response_status
{
public:
  Init_PlanPath_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanPath_GetResult_Response_result status(::xline_path_planner::action::PlanPath_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_PlanPath_GetResult_Response_result(msg_);
  }

private:
  ::xline_path_planner::action::PlanPath_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xline_path_planner::action::PlanPath_GetResult_Response>()
{
  return xline_path_planner::action::builder::Init_PlanPath_GetResult_Response_status();
}

}  // namespace xline_path_planner


namespace xline_path_planner
{

namespace action
{

namespace builder
{

class Init_PlanPath_FeedbackMessage_feedback
{
public:
  explicit Init_PlanPath_FeedbackMessage_feedback(::xline_path_planner::action::PlanPath_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::xline_path_planner::action::PlanPath_FeedbackMessage feedback(::xline_path_planner::action::PlanPath_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xline_path_planner::action::PlanPath_FeedbackMessage msg_;
};

class Init_PlanPath_FeedbackMessage_goal_id
{
public:
  Init_PlanPath_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanPath_FeedbackMessage_feedback goal_id(::xline_path_planner::action::PlanPath_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PlanPath_FeedbackMessage_feedback(msg_);
  }

private:
  ::xline_path_planner::action::PlanPath_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xline_path_planner::action::PlanPath_FeedbackMessage>()
{
  return xline_path_planner::action::builder::Init_PlanPath_FeedbackMessage_goal_id();
}

}  // namespace xline_path_planner

#endif  // XLINE_PATH_PLANNER__ACTION__DETAIL__PLAN_PATH__BUILDER_HPP_
