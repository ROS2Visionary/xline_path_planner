#include "xline_path_planner/geometry_preprocessor.hpp"
#include <limits>
#include <stdexcept>
#include <sstream>

namespace path_planner
{

namespace {

void collect_ids(const std::vector<std::shared_ptr<Line>>& lines, std::unordered_set<int32_t>& used_ids)
{
  for (const auto& l : lines)
  {
    if (!l) continue;
    used_ids.insert(l->id);
  }
}

void copy_line_meta(Line& dst, const Line& src)
{
  dst.is_printed = src.is_printed;
  dst.line_type = src.line_type;
  dst.thickness = src.thickness;
  dst.hidden = src.hidden;
  dst.layer = src.layer;
  dst.layer_id = src.layer_id;
  dst.color = src.color;
  dst.selected = src.selected;
}

}  // namespace

CADData GeometryPreprocessor::preprocess(const CADData& cad_data, const PathPlannerConfig& config)
{
  std::unordered_set<int32_t> used_ids;
  used_ids.reserve(cad_data.path_lines.size() + cad_data.obstacle_lines.size() + cad_data.hole_lines.size());
  collect_ids(cad_data.path_lines, used_ids);
  collect_ids(cad_data.obstacle_lines, used_ids);
  collect_ids(cad_data.hole_lines, used_ids);

  CADData result;
  result.origin_points = cad_data.origin_points;

  preprocess_collection(cad_data.path_lines, result.path_lines, config, used_ids, true);
  preprocess_collection(cad_data.obstacle_lines, result.obstacle_lines, config, used_ids, false);
  preprocess_collection(cad_data.hole_lines, result.hole_lines, config, used_ids, false);

  return result;
}

void GeometryPreprocessor::preprocess_collection(const std::vector<std::shared_ptr<Line>>& input,
                                                 std::vector<std::shared_ptr<Line>>& output,
                                                 const PathPlannerConfig& config,
                                                 std::unordered_set<int32_t>& used_ids,
                                                 bool apply_radius_compensation)
{
  output.clear();
  output.reserve(input.size());

  const bool enable_radius_compensation =
    apply_radius_compensation && (config.circle_radius_compensation != 0.0);

  for (const auto& geom : input)
  {
    if (!geom) continue;

    std::shared_ptr<Line> current = geom;
    if (enable_radius_compensation)
    {
      if (geom->type == GeometryType::CIRCLE)
      {
        auto circle = std::dynamic_pointer_cast<Circle>(geom);
        if (!circle) continue;
        const double new_radius = circle->radius + config.circle_radius_compensation;
        if (new_radius <= 0.0)
        {
          std::ostringstream oss;
          oss << "circle_radius_compensation 导致圆半径<=0: id=" << circle->id
              << ", radius=" << circle->radius << ", compensation=" << config.circle_radius_compensation;
          throw std::runtime_error(oss.str());
        }
        auto compensated = std::make_shared<Circle>(circle->id, circle->center, new_radius);
        copy_line_meta(*compensated, *circle);
        current = std::move(compensated);
      }
      else if (geom->type == GeometryType::ARC)
      {
        auto arc = std::dynamic_pointer_cast<Arc>(geom);
        if (!arc) continue;
        const double new_radius = arc->radius + config.circle_radius_compensation;
        if (new_radius <= 0.0)
        {
          std::ostringstream oss;
          oss << "circle_radius_compensation 导致圆弧半径<=0: id=" << arc->id
              << ", radius=" << arc->radius << ", compensation=" << config.circle_radius_compensation;
          throw std::runtime_error(oss.str());
        }
        auto compensated = std::make_shared<Arc>(arc->id, arc->center, new_radius, arc->start_angle, arc->end_angle);
        copy_line_meta(*compensated, *arc);
        current = std::move(compensated);
      }
    }

    if (current->type == GeometryType::POLYLINE && config.split_polyline)
    {
      auto poly = std::dynamic_pointer_cast<Polyline>(current);
      if (!poly) continue;
      auto segments = split_polyline(*poly, config, used_ids);
      for (auto& seg : segments)
      {
        output.push_back(std::move(seg));
      }
      continue;
    }

    output.push_back(std::move(current));
  }
}

std::vector<std::shared_ptr<Line>> GeometryPreprocessor::split_polyline(
  const Polyline& polyline, const PathPlannerConfig& config, std::unordered_set<int32_t>& used_ids)
{
  std::vector<std::shared_ptr<Line>> segments;
  if (polyline.vertices.size() < 2)
  {
    return segments;
  }

  auto copy_common_meta = [&](Line& dst) {
    dst.layer = polyline.layer;
    dst.layer_id = polyline.layer_id;
    dst.color = polyline.color;
    dst.line_type = polyline.line_type;
    dst.thickness = polyline.thickness;
    dst.hidden = polyline.hidden;
    dst.selected = polyline.selected;
    dst.is_printed = false;
  };

  const auto& vertices = polyline.vertices;
  const int32_t parent_id = polyline.id;

  auto emit_segment = [&](size_t i, const Point3D& a, const Point3D& b) {
    std::shared_ptr<Line> seg;
    if (config.preserve_polyline_info)
    {
      auto ls = std::make_shared<LineSegment>();
      ls->parent_polyline_id = parent_id;
      ls->segment_index = static_cast<int32_t>(i);
      ls->is_from_polyline = true;
      seg = std::move(ls);
    }
    else
    {
      seg = std::make_shared<Line>();
    }

    seg->type = GeometryType::LINE;
    copy_common_meta(*seg);
    seg->start = a;
    seg->end = b;
    seg->length = a.distance(b);

    const int32_t preferred_id = generate_segment_id(parent_id, static_cast<int32_t>(i));
    seg->id = allocate_unique_id(preferred_id, used_ids);

    segments.push_back(std::move(seg));
  };

  for (size_t i = 0; i + 1 < vertices.size(); ++i)
  {
    emit_segment(i, vertices[i], vertices[i + 1]);
  }

  if (polyline.closed && vertices.size() > 2)
  {
    emit_segment(vertices.size() - 1, vertices.back(), vertices.front());
  }

  return segments;
}

int32_t GeometryPreprocessor::generate_segment_id(int32_t parent_id, int32_t segment_idx)
{
  // ID 格式: parent_id * 10000 + segment_idx
  // 使用 int64 避免溢出，再回落到 preferred_id 分配逻辑。
  const int64_t raw = static_cast<int64_t>(parent_id) * 10000LL + static_cast<int64_t>(segment_idx);
  if (raw > std::numeric_limits<int32_t>::max() || raw < std::numeric_limits<int32_t>::min())
  {
    return 0;
  }
  return static_cast<int32_t>(raw);
}

int32_t GeometryPreprocessor::allocate_unique_id(int32_t preferred_id, std::unordered_set<int32_t>& used_ids)
{
  auto take = [&](int32_t id) {
    used_ids.insert(id);
    return id;
  };

  if (preferred_id != 0 && used_ids.find(preferred_id) == used_ids.end())
  {
    return take(preferred_id);
  }

  // 回退：从一个较大的起点开始递增，避免与 CAD 原始 id 冲突
  int32_t candidate = 10000000;
  while (used_ids.find(candidate) != used_ids.end())
  {
    if (candidate == std::numeric_limits<int32_t>::max())
    {
      candidate = 1;
    }
    else
    {
      candidate++;
    }
  }
  return take(candidate);
}

}  // namespace path_planner
