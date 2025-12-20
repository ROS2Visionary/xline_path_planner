#pragma once

#include "xline_path_planner/common_types.hpp"
#include <cstdint>
#include <vector>

namespace path_planner
{

enum class ConnectionType
{
  END_TO_START,
  END_TO_END,
  START_TO_START,
  START_TO_END,
  NONE
};

struct MergeGroup
{
  std::vector<int32_t> line_ids;
  std::vector<bool> need_reverse;
  Point3D merged_start;
  Point3D merged_end;
};

class CollinearMerger
{
public:
  explicit CollinearMerger(double distance_tolerance = 0.001, double angle_tolerance = 0.001,
                           double min_segment_length = 0.0005);

  /**
   * @brief 合并共线且相连的直线线段（仅处理 GeometryType::LINE）
   * @return 每个 MergeGroup 代表一条最终路径（可能仅包含1条线段）
   */
  std::vector<MergeGroup> merge(const std::vector<const Line*>& lines) const;

private:
  double distance_tolerance_;
  double angle_tolerance_;
  double min_segment_length_;

  bool are_collinear(const Line& a, const Line& b) const;
  bool merge_attributes_compatible(const Line& a, const Line& b) const;
  ConnectionType get_connection_type(const Line& a, const Line& b) const;

  double point_distance(const Point3D& a, const Point3D& b) const;
  double point_to_line_distance(const Point3D& point, const Point3D& line_start, const Point3D& line_end) const;
  Point3D direction_unit(const Line& line) const;
};

}  // namespace path_planner

