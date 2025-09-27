// include/xline_path_planner/planner_engine.hpp
#pragma once

#include <string>
#include <vector>

#include "xline_path_planner/common_types.hpp"
#include "xline_path_planner/cad_parser.hpp"

namespace daosnrs_planning
{

struct PlannerEngineConfig
{
  CADParserConfig cad_parser;
  GridMapConfig grid_map;
  PathPlannerConfig path_planner;
  PathOffsetConfig offsets;
  TrajectoryConfig trajectory;
};

class PlannerEngine
{
public:
  explicit PlannerEngine(const PlannerEngineConfig& cfg);

  // 从 CAD JSON 字符串规划并返回输出 JSON 字符串（不做磁盘 I/O）
  // 若失败，抛出 std::runtime_error 说明原因
  std::string plan_from_cad_json(const std::string& cad_json, const std::string& source_name = "in_memory.json");

private:
  PlannerEngineConfig cfg_;
};

}  // namespace daosnrs_planning
