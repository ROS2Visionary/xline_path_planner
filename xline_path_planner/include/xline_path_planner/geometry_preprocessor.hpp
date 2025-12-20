#pragma once

#include "xline_path_planner/common_types.hpp"
#include <unordered_set>

namespace path_planner
{

/**
 * @brief CAD 几何预处理：Polyline 拆分等
 */
class GeometryPreprocessor
{
public:
  /**
   * @brief 预处理 CAD 数据（按配置执行 Polyline 拆分）
   */
  static CADData preprocess(const CADData& cad_data, const PathPlannerConfig& config);

private:
  static void preprocess_collection(const std::vector<std::shared_ptr<Line>>& input,
                                    std::vector<std::shared_ptr<Line>>& output,
                                    const PathPlannerConfig& config,
                                    std::unordered_set<int32_t>& used_ids);

  static std::vector<std::shared_ptr<Line>> split_polyline(const Polyline& polyline,
                                                           const PathPlannerConfig& config,
                                                           std::unordered_set<int32_t>& used_ids);

  static int32_t allocate_unique_id(int32_t preferred_id, std::unordered_set<int32_t>& used_ids);
  static int32_t generate_segment_id(int32_t parent_id, int32_t segment_idx);
};

}  // namespace path_planner
