#include "xline_path_planner/path_planner.hpp"
#include "xline_path_planner/collinear_merger.hpp"
#include "xline_path_planner/geometry_preprocessor.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace path_planner
{

namespace {
// 从 line_type 字符串推导 InkMode
InkMode deduceInkMode(const std::string& line_type)
{
  if (line_type == "dashed" || line_type == "dotted") {
    return InkMode::DASHED;
  } else if (line_type == "text") {
    return InkMode::TEXT;
  } else {
    // continuous 或其他默认为 solid
    return InkMode::SOLID;
  }
}

inline double normalize_angle(double angle_rad)
{
  while (angle_rad > M_PI) angle_rad -= 2.0 * M_PI;
  while (angle_rad < -M_PI) angle_rad += 2.0 * M_PI;
  return angle_rad;
}

inline double abs_angle_diff(double from_rad, double to_rad)
{
  return std::abs(normalize_angle(to_rad - from_rad));
}

std::optional<double> heading_from_first_motion(const std::vector<Point3D>& points, double eps = 1e-6)
{
  if (points.size() < 2) return std::nullopt;
  for (std::size_t i = 0; i + 1 < points.size(); ++i)
  {
    const auto& a = points[i];
    const auto& b = points[i + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    if (std::hypot(dx, dy) <= eps) continue;
    return std::atan2(dy, dx);
  }
  return std::nullopt;
}

std::optional<double> heading_from_last_motion(const std::vector<Point3D>& points, double eps = 1e-6)
{
  if (points.size() < 2) return std::nullopt;
  for (std::size_t i = points.size() - 1; i > 0; --i)
  {
    const auto& a = points[i - 1];
    const auto& b = points[i];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    if (std::hypot(dx, dy) <= eps) continue;
    return std::atan2(dy, dx);
  }
  return std::nullopt;
}

std::optional<double> previous_path_end_heading(const std::vector<RouteSegment>& planned_segments)
{
  for (auto it = planned_segments.rbegin(); it != planned_segments.rend(); ++it)
  {
    if (it->points.size() < 2) continue;
    auto heading = heading_from_last_motion(it->points);
    if (heading.has_value()) return heading;
  }
  return std::nullopt;
}

bool should_execute_transition_backward(const std::vector<RouteSegment>& planned_segments_before_transition,
                                       const RouteSegment& transition_segment)
{
  if (transition_segment.type != RouteType::TRANSITION_PATH) return false;

  const auto start_heading = previous_path_end_heading(planned_segments_before_transition);
  const auto transition_heading = heading_from_first_motion(transition_segment.points);
  if (!start_heading.has_value() || !transition_heading.has_value()) return false;

  const double head_time = abs_angle_diff(*start_heading, *transition_heading);
  const double tail_time = abs_angle_diff(*start_heading, *transition_heading + M_PI);
  return tail_time < head_time;
}

/**
 * @brief 根据运动方向和文字位置决定使用哪个打印机
 * 
 * 策略说明：
 * 文字只能使用左边或右边的喷码机，不能使用中间喷码机。
 * 决定使用哪边的喷码机基于以下规则：
 * 
 * 1. 计算机器人从当前位置到文字位置的运动方向向量
 * 2. 计算文字的旋转方向（文字书写方向）
 * 3. 根据运动方向和文字方向的关系，决定使用左侧还是右侧喷码机
 * 
 * 基本原则：
 * - 如果机器人从左向右运动（相对于文字方向），使用右侧喷码机
 * - 如果机器人从右向左运动（相对于文字方向），使用左侧喷码机
 * - 这样可以确保喷码机在运动时能正确地打印文字
 * 
 * @param current_position 机器人当前位置
 * @param text_position 文字位置
 * @param text_rotation 文字旋转角度（度）
 * @return 应该使用的打印机类型（LEFT_PRINTER 或 RIGHT_PRINTER）
 */
PrinterType determineTextPrinterQuiet(const Point3D& current_position, const Point3D& text_position, double text_rotation)
{
  // 计算从当前位置到文字位置的向量
  const double dx = text_position.x - current_position.x;
  const double dy = text_position.y - current_position.y;

  // 计算运动方向角度（弧度）
  const double motion_angle = std::atan2(dy, dx);

  // 将文字旋转角度转换为弧度
  const double text_angle_rad = text_rotation * M_PI / 180.0;

  // 计算运动方向与文字方向的相对角度
  double relative_angle = motion_angle - text_angle_rad;

  // 将角度规范化到 [-π, π] 范围
  while (relative_angle > M_PI) relative_angle -= 2 * M_PI;
  while (relative_angle < -M_PI) relative_angle += 2 * M_PI;

  // 需求更新：在保持路径运动方向不变的前提下，左右喷码机的“旋转/使用侧”需要对调，
  // 因此在逻辑上将原先的左右选择交换。
  if (relative_angle >= -M_PI / 2 && relative_angle <= M_PI / 2)
  {
    return PrinterType::LEFT_PRINTER;
  }
  return PrinterType::RIGHT_PRINTER;
}

PrinterType determineTextPrinterByPositionQuiet(const Point3D& text_position, double canvas_center_x)
{
  return (text_position.x < canvas_center_x) ? PrinterType::LEFT_PRINTER : PrinterType::RIGHT_PRINTER;
}

PrinterType determineTextPrinter(const Point3D& current_position, const Point3D& text_position, double text_rotation)
{
  const auto result = determineTextPrinterQuiet(current_position, text_position, text_rotation);
  if (result == PrinterType::LEFT_PRINTER)
  {
    std::cout << "  文字打印机选择: LEFT_PRINTER (运动方向与文字方向一致，对调后)" << std::endl;
  }
  else
  {
    std::cout << "  文字打印机选择: RIGHT_PRINTER (运动方向与文字方向相反，对调后)" << std::endl;
  }
  return result;
}

/**
 * @brief 根据文字在画布上的位置决定打印机（备用方案）
 */
PrinterType determineTextPrinterByPosition(const Point3D& text_position, double canvas_center_x)
{
  const auto result = determineTextPrinterByPositionQuiet(text_position, canvas_center_x);
  if (result == PrinterType::LEFT_PRINTER)
  {
    std::cout << "  文字打印机选择: LEFT_PRINTER (文字在画布左侧)" << std::endl;
  }
  else
  {
    std::cout << "  文字打印机选择: RIGHT_PRINTER (文字在画布右侧)" << std::endl;
  }
  return result;
}

std::vector<std::shared_ptr<Line>> merge_collinear_lines_if_enabled(const std::vector<std::shared_ptr<Line>>& lines,
                                                                    const PathPlannerConfig& cfg)
{
  if (!cfg.merge_collinear)
  {
    return lines;
  }

  std::unordered_map<int32_t, std::shared_ptr<Line>> id2line;
  id2line.reserve(lines.size());

  std::vector<const Line*> straight;
  straight.reserve(lines.size());

  std::vector<std::shared_ptr<Line>> non_straight;
  non_straight.reserve(lines.size());

  for (const auto& l : lines)
  {
    if (!l) continue;
    if (l->type == GeometryType::LINE)
    {
      id2line[l->id] = l;
      straight.push_back(l.get());
    }
    else
    {
      non_straight.push_back(l);
    }
  }

  CollinearMerger merger(cfg.distance_tolerance, cfg.angle_tolerance, cfg.min_segment_length);
  auto groups = merger.merge(straight);

  std::vector<std::shared_ptr<Line>> merged;
  merged.reserve(groups.size() + non_straight.size());

  for (const auto& g : groups)
  {
    if (g.line_ids.empty())
      continue;

    auto it = id2line.find(g.line_ids.front());
    if (it == id2line.end() || !it->second)
      continue;

    if (g.line_ids.size() <= 1)
    {
      merged.push_back(it->second);
      continue;
    }

    auto base = it->second;
    auto merged_line = std::make_shared<MergedLine>();
    static_cast<Line&>(*merged_line) = *base;
    merged_line->type = GeometryType::LINE;
    merged_line->start = g.merged_start;
    merged_line->end = g.merged_end;
    merged_line->length = merged_line->start.distance(merged_line->end);
    merged_line->is_printed = false;
    merged_line->source_line_ids = g.line_ids;
    merged.push_back(std::move(merged_line));
  }

  merged.insert(merged.end(), non_straight.begin(), non_straight.end());
  return merged;
}

} // anonymous namespace

PathPlanner::PathPlanner(const PathPlannerConfig& config) : planner_config_(config), grid_map_generator_(nullptr)
{
}

PathPlanner::~PathPlanner()
{
}

void PathPlanner::set_grid_map(const std::vector<std::vector<int>>& grid_map,
                               const GridMapGenerator* grid_map_generator)
{
  grid_map_ = grid_map;
  grid_map_generator_ = grid_map_generator;
}

const PathPlannerConfig& PathPlanner::get_config() const
{
  return planner_config_;
}

void PathPlanner::set_config(const PathPlannerConfig& config)
{
  planner_config_ = config;
}

std::vector<RouteSegment> PathPlanner::plan_paths(const CADData& cad_data, const PathOffsetConfig& offset_config,
                                                   const std::optional<Point3D>& robot_start_position)
{
  if (!grid_map_generator_)
  {
    std::cerr << "Grid map generator not set!" << std::endl;
    return {};
  }

  std::vector<RouteSegment> path_segments;

  // 注意：CAD 几何预处理（圆/圆弧半径补偿、Polyline 拆分等）应在外部调用完成
  const CADData& preprocessed = cad_data;

  // 如果提供了机器人起始位置，打印信息
  if (robot_start_position.has_value())
  {
    std::cout << "使用机器人起始位置: [" << robot_start_position->x << ", " << robot_start_position->y << ", "
              << robot_start_position->z << "]" << std::endl;
  }

  // 打印CAD数据中路径线的数量
  std::cout << "Total path lines in CAD data: " << preprocessed.path_lines.size() << std::endl;

  // 统计各类型的轴线数量和已绘制状态
  int line_count = 0, polyline_count = 0, circle_count = 0, arc_count = 0, ellipse_count = 0, curve_count = 0,
      text_count = 0, other_count = 0;
  int printed_count = 0, unprinted_count = 0;

  for (const auto& line : preprocessed.path_lines)
  {
    if (line->type == GeometryType::LINE)
      line_count++;
    else if (line->type == GeometryType::POLYLINE)
      polyline_count++;
    else if (line->type == GeometryType::CIRCLE)
      circle_count++;
    else if (line->type == GeometryType::ARC)
      arc_count++;
    else if (line->type == GeometryType::ELLIPSE)
      ellipse_count++;
    else if (line->type == GeometryType::CURVE)
      curve_count++;
    else if (line->type == GeometryType::TEXT)
      text_count++;
    else
      other_count++;

    if (line->is_printed)
    {
      printed_count++;
    }
    else
    {
      unprinted_count++;
    }
  }

  std::cout << "Path line types: " << line_count << " lines, " << polyline_count << " polylines, " << circle_count
            << " circles, " << arc_count << " arcs, " << ellipse_count << " ellipses, " << curve_count << " curves, "
            << text_count << " texts, " << other_count << " other types" << std::endl;

  std::cout << "Printed/Unprinted status: " << printed_count << " printed, " << unprinted_count << " unprinted"
            << std::endl;

  // 收集所有轴线，无论其is_printed状态
  std::vector<std::shared_ptr<Line>> lines_to_draw =
    merge_collinear_lines_if_enabled(preprocessed.path_lines, planner_config_);

  // 如果所有线段都被标记为已打印，但我们仍然需要处理它们
  const bool all_printed = !lines_to_draw.empty() &&
                           std::all_of(lines_to_draw.begin(), lines_to_draw.end(),
                                       [](const std::shared_ptr<Line>& l) { return l && l->is_printed; });
  if (all_printed)
  {
    std::cout << "All lines are marked as printed. Forcing processing of all lines..." << std::endl;
    // 将所有线段标记为未打印，以便处理它们
    for (auto& line : lines_to_draw)
    {
      line->is_printed = false;
    }
  }

  std::cout << "Planning paths for " << lines_to_draw.size() << " lines" << std::endl;

  // 处理所有线段（传递机器人起始位置）
  processGeometryGroup(lines_to_draw, path_segments, offset_config, robot_start_position);

  return path_segments;
}

std::shared_ptr<Line> PathPlanner::findNearestUnprocessedLine(const Point3D& current_pos,
                                                             const std::vector<std::shared_ptr<Line>>& lines,
                                                             const PathOffsetConfig& offset_config,
                                                             bool has_current_position,
                                                             double canvas_center_x)
{
  double min_distance_any = std::numeric_limits<double>::max();
  std::shared_ptr<Line> nearest_any = nullptr;

  double min_distance_in_range = std::numeric_limits<double>::max();
  std::shared_ptr<Line> nearest_in_range = nullptr;

  double min_len = std::max(0.0, planner_config_.transition_length_min);
  double max_len = planner_config_.transition_length_max;
  if (!(max_len >= 0.0)) max_len = std::numeric_limits<double>::infinity();
  if (max_len < min_len) std::swap(max_len, min_len);

  std::cout << "\n🔍 从位置 [" << current_pos.x << ", " << current_pos.y << ", " << current_pos.z
            << "] 查找最近的未处理线段..." << std::endl;
  if (has_current_position)
  {
    std::cout << "  转场长度优先范围: [" << min_len << ", " << max_len << "] 米" << std::endl;
  }

  const double start_extension_length = std::max(0.0, planner_config_.path_extension_start_length);
  const double end_extension_length = std::max(0.0, planner_config_.path_extension_end_length);

  auto extend_line_points = [&](const Point3D& start, const Point3D& end,
                                double start_ext, double end_ext) -> std::pair<Point3D, Point3D> {
    const double original_length = start.distance(end);
    if (original_length < 1e-6)
    {
      return { start, end };
    }
    const double start_factor = start_ext / original_length;
    const double end_factor = end_ext / original_length;

    Point3D extended_start, extended_end;
    extended_start.x = start.x - (end.x - start.x) * start_factor;
    extended_start.y = start.y - (end.y - start.y) * start_factor;
    extended_start.z = start.z - (end.z - start.z) * start_factor;

    extended_end.x = end.x + (end.x - start.x) * end_factor;
    extended_end.y = end.y + (end.y - start.y) * end_factor;
    extended_end.z = end.z + (end.z - start.z) * end_factor;

    return { extended_start, extended_end };
  };

  auto offset_first_point_of_segment = [](const Point3D& p0, const Point3D& p1, double offset) -> Point3D
  {
    if (std::abs(offset) <= 1e-9) return p0;
    const double dx = p1.x - p0.x;
    const double dy = p1.y - p0.y;
    const double len = std::hypot(dx, dy);
    if (len <= 1e-9) return p0;
    const double nx = -dy / len;
    const double ny = dx / len;
    Point3D out = p0;
    out.x += nx * offset;
    out.y += ny * offset;
    return out;
  };

  auto estimate_next_start = [&](const std::shared_ptr<Line>& line) -> Point3D
  {
    if (!line) return Point3D{};

    if (line->type == GeometryType::LINE)
    {
      auto [s, e] = extend_line_points(line->start, line->end, start_extension_length, end_extension_length);
      return offset_first_point_of_segment(s, e, offset_config.center_offset);
    }
    if (line->type == GeometryType::TEXT)
    {
      auto [s, e] = extend_line_points(line->start, line->end, start_extension_length, end_extension_length);
      auto text = std::dynamic_pointer_cast<Text>(line);
      const auto printer = (has_current_position && text)
                             ? determineTextPrinterQuiet(current_pos, text->position, text->rotation)
                             : (text ? determineTextPrinterByPositionQuiet(text->position, canvas_center_x)
                                     : PrinterType::CENTER_PRINTER);
      double offset = 0.0;
      switch (printer) {
        case PrinterType::LEFT_PRINTER: offset = offset_config.left_offset; break;
        case PrinterType::RIGHT_PRINTER: offset = offset_config.right_offset; break;
        case PrinterType::CENTER_PRINTER: offset = offset_config.center_offset; break;
      }
      return offset_first_point_of_segment(s, e, offset);
    }
    if (line->type == GeometryType::POLYLINE)
    {
      auto poly = std::dynamic_pointer_cast<Polyline>(line);
      if (poly && poly->vertices.size() >= 2)
      {
        return offset_first_point_of_segment(poly->vertices[0], poly->vertices[1], offset_config.center_offset);
      }
      return line->start;
    }

    return line->start;
  };

  for (const auto& line : lines)
  {
    if (!line->is_printed)
    {
      const auto goal = estimate_next_start(line);
      double dist = current_pos.distance(goal);
      if (dist < min_distance_any)
      {
        min_distance_any = dist;
        nearest_any = line;
      }

      if (has_current_position && dist >= min_len && dist <= max_len && dist < min_distance_in_range)
      {
        min_distance_in_range = dist;
        nearest_in_range = line;
      }
    }
  }

  if (nearest_in_range)
  {
    std::cout << "✅ 找到最近线段 ID: " << nearest_in_range->id << "，距离: " << min_distance_in_range
              << " 米（命中优先范围）" << std::endl;
  }
  else if (nearest_any)
  {
    std::cout << "✅ 找到最近线段 ID: " << nearest_any->id << "，距离: " << min_distance_any
              << (has_current_position ? " 米（未命中优先范围，使用最近距离）" : " 米") << std::endl;
  }
  else
  {
    std::cout << "❌ 未找到未处理的线段" << std::endl;
  }

  return nearest_in_range ? nearest_in_range : nearest_any;
}

RouteSegment PathPlanner::planGeometryPath(const std::shared_ptr<Line>& line, const PathOffsetConfig& offset_config)
{
  RouteSegment segment(RouteType::DRAWING_PATH, line->id);
  if (auto merged_line = std::dynamic_pointer_cast<MergedLine>(line))
  {
    segment.merged_line_ids = merged_line->source_line_ids;
  }

  // 默认使用中间喷码机；后续如需按图层/几何特征动态分配，
  // 可以在此处根据 line 元数据修改 segment.printer_type
  segment.printer_type = PrinterType::CENTER_PRINTER;

  // 从 line_type 推导 ink_mode
  segment.ink_mode = deduceInkMode(line->line_type);

  // 绘图路径延长配置（起点端/终点端可分别配置）
  const double start_extension_length = std::max(0.0, planner_config_.path_extension_start_length);
  const double end_extension_length = std::max(0.0, planner_config_.path_extension_end_length);

  // 延长路径的方法
  auto extend_line = [&](const Point3D& start, const Point3D& end,
                         double start_ext, double end_ext) -> std::pair<Point3D, Point3D> {
    // 计算原始线段长度和方向
    double original_length = start.distance(end);
    if (original_length < 1e-6)
    {
      return { start, end };
    }
    const double start_factor = start_ext / original_length;
    const double end_factor = end_ext / original_length;

    Point3D extended_start, extended_end;

    // 延长起点
    extended_start.x = start.x - (end.x - start.x) * start_factor;
    extended_start.y = start.y - (end.y - start.y) * start_factor;
    extended_start.z = start.z - (end.z - start.z) * start_factor;

    // 延长终点
    extended_end.x = end.x + (end.x - start.x) * end_factor;
    extended_end.y = end.y + (end.y - start.y) * end_factor;
    extended_end.z = end.z + (end.z - start.z) * end_factor;

    return { extended_start, extended_end };
  };

  // 根据线段类型选择不同的路径生成方法
  if (line->type == GeometryType::LINE)
  {
    // 仅对直线路径（非转场路径）进行延长
    auto [extended_start, extended_end] =
        extend_line(line->start, line->end, start_extension_length, end_extension_length);

    segment.points = { extended_start, extended_end };
  }
  else if (line->type == GeometryType::POLYLINE)
  {
    auto poly = std::dynamic_pointer_cast<Polyline>(line);
    if (poly && poly->vertices.size() >= 2)
    {
      segment.points = poly->vertices;
      if (poly->closed && poly->vertices.size() > 2)
      {
        segment.points.push_back(poly->vertices.front());
      }
    }
    else
    {
      segment.points = { line->start, line->end };
    }
  }
  else if (line->type == GeometryType::TEXT)
  {
    // 文字类型处理：沿文字基线方向进行路径延长
    auto text = std::dynamic_pointer_cast<Text>(line);
    if (text)
    {
      std::cout << "\n📝 处理文字元素 ID: " << text->id << std::endl;
      std::cout << "  内容: \"" << text->content << "\"" << std::endl;

      // 设置 ink_mode 为 TEXT
      segment.ink_mode = InkMode::TEXT;
      
      // 保存文字内容
      segment.text_content = text->content;

      // 使用 Line 基类的 start/end 作为文字基线，并按与直线相同的规则前后延长
      auto [extended_start, extended_end] =
          extend_line(line->start, line->end, start_extension_length, end_extension_length);
      segment.points = { extended_start, extended_end };
    }
    else
    {
      std::cerr << "Failed to cast line to Text!" << std::endl;
      segment.points = { line->start, line->end };
    }
  }
  else if (line->type == GeometryType::CIRCLE)
  {
    // 圆形，离散化为多个点
    auto circle = std::dynamic_pointer_cast<Circle>(line);
    if (circle)
    {
      // 以固定角度间隔离散化圆
      const int num_points = 36;  // 每10度一个点
      const double angle_step = 2.0 * M_PI / num_points;

      std::vector<Point3D> circle_points;
      for (int i = 0; i <= num_points; ++i)
      {
        double angle = i * angle_step;
        Point3D point;
        point.x = circle->center.x + circle->radius * std::cos(angle);
        point.y = circle->center.y + circle->radius * std::sin(angle);
        point.z = circle->center.z;

        circle_points.push_back(point);
      }

      // 为圆形路径保持原始几何形状，不对起点和终点进行切线方向的延长，
      // 这样 segment.points.front()/back() 将严格落在圆上，便于后续导出的
      // start/end 与 CAD 中的圆几何保持一致。
      segment.points = circle_points;
    }
  }
  else if (line->type == GeometryType::ARC)
  {
    // 圆弧，离散化为多个点
    auto arc = std::dynamic_pointer_cast<Arc>(line);
    if (arc)
    {
      // 根据弧长计算延长角度：弧长 = 半径 × 角度（弧度）
      // 延长角度 = 弧长 / 半径
      const double arc_ext_length = planner_config_.arc_extension_length;
      const double max_ext_angle_rad = planner_config_.arc_extension_max_angle * M_PI / 180.0;

      double ext_angle_from_length = (arc->radius > 1e-6) ? (arc_ext_length / arc->radius) : 0.0;
      // 取弧长计算的角度和最大角度限制的较小值
      const double arc_ext_angle_rad = std::min(ext_angle_from_length, max_ext_angle_rad);

      // 计算角度范围并离散化
      double start_angle = arc->start_angle;
      double end_angle = arc->end_angle;

      // 确保终止角度大于起始角度
      if (end_angle < start_angle)
      {
        end_angle += 2.0 * M_PI;
      }

      // 计算原始角度差
      double original_angle_diff = end_angle - start_angle;

      // 起点处沿圆弧反向延长（角度减小方向）
      double extended_start_angle = start_angle - arc_ext_angle_rad;
      double total_angle_diff = original_angle_diff + arc_ext_angle_rad;

      // 离散化点数（根据总角度范围计算）
      int num_points = std::max(10, static_cast<int>(36 * total_angle_diff / (2.0 * M_PI)));
      double angle_step = total_angle_diff / num_points;

      std::vector<Point3D> arc_points;
      for (int i = 0; i <= num_points; ++i)
      {
        double angle = extended_start_angle + i * angle_step;
        Point3D point;
        point.x = arc->center.x + arc->radius * std::cos(angle);
        point.y = arc->center.y + arc->radius * std::sin(angle);
        point.z = arc->center.z;

        arc_points.push_back(point);
      }

      // 圆弧起点沿圆弧路径反向延长，终点保持原始位置
      segment.points = arc_points;
    }
  }
  else if (line->type == GeometryType::ELLIPSE)
  {
    auto ellipse = std::dynamic_pointer_cast<Ellipse>(line);
    if (ellipse)
    {
      const double a = ellipse->major_radius();
      const double b = ellipse->minor_radius();
      if (a > 1e-12 && b > 1e-12)
      {
        double t0 = ellipse->start_angle;
        double t1 = ellipse->end_angle;
        double angle_diff = t1 - t0;
        if (std::fabs(angle_diff) < 1e-9)
        {
          angle_diff = 2.0 * M_PI;
        }
        if (angle_diff < 0)
        {
          angle_diff += 2.0 * M_PI;
        }

        // 近似弧长用于确定采样密度（对椭圆弧为近似，但足以用于轨迹离散化）
        const double h = std::pow(a - b, 2) / std::pow(a + b, 2);
        const double perimeter = M_PI * (a + b) * (1.0 + (3.0 * h) / (10.0 + std::sqrt(4.0 - 3.0 * h)));
        const double arc_len = perimeter * (angle_diff / (2.0 * M_PI));

        const double fallback_resolution = 0.05;
        const double base_res = grid_map_generator_ ? grid_map_generator_->get_resolution() : fallback_resolution;
        const double point_distance = std::max(1e-4, 0.5 * base_res);

        const int min_points = 60;
        const int max_points = 8000;
        int num_points = std::max(min_points, static_cast<int>(arc_len / point_distance) + 1);
        num_points = std::min(num_points, max_points);

        const double theta = ellipse->orientation();
        const double cth = std::cos(theta);
        const double sth = std::sin(theta);

        std::vector<Point3D> pts;
        pts.reserve(static_cast<size_t>(num_points) + 1);
        for (int i = 0; i <= num_points; ++i)
        {
          const double t = t0 + angle_diff * (static_cast<double>(i) / num_points);
          const double ct = std::cos(t);
          const double st = std::sin(t);
          const double xl = a * ct;
          const double yl = b * st;
          Point3D p;
          p.x = ellipse->center.x + xl * cth - yl * sth;
          p.y = ellipse->center.y + xl * sth + yl * cth;
          p.z = ellipse->center.z;
          pts.push_back(p);
        }
        segment.points = pts;
      }
      else
      {
        segment.points = { line->start, line->end };
      }
    }
  }
  else if (line->type == GeometryType::CURVE)
  {
    // 曲线段,使用离散化方法
    auto curve = std::dynamic_pointer_cast<Curve>(line);
    if (curve)
    {
      // 离散化曲线为多个点,使用足够多的点确保拟合精度
      int num_points = std::max(50, static_cast<int>(curve->length / 0.01));  // 确保点足够密集
      std::vector<Point3D> curve_points = discretize_curve(curve, num_points);

      std::cout << "Discretized curve with " << curve_points.size() << " points" << std::endl;

      // 延长起点和终点
      if (curve_points.size() >= 2)
      {
        // 计算起点处的切线方向
        Point3D tangent_start = curve_points[1] - curve_points[0];
        double tangent_start_length = tangent_start.distance(Point3D(0, 0, 0));

        // 计算终点处的切线方向
        Point3D tangent_end = curve_points[curve_points.size() - 1] - curve_points[curve_points.size() - 2];
        double tangent_end_length = tangent_end.distance(Point3D(0, 0, 0));

        // 延长起点（起点端/终点端支持不同的延长长度）
        Point3D extended_start = curve_points.front();
        if (tangent_start_length > 1e-6)
        {  // 避免除以零
          extended_start = extended_start - tangent_start * (start_extension_length / tangent_start_length);
        }

        // 延长终点
        Point3D extended_end = curve_points.back();
        if (tangent_end_length > 1e-6)
        {  // 避免除以零
          extended_end = extended_end + tangent_end * (end_extension_length / tangent_end_length);
        }

        // 构建新的曲线点序列
        std::vector<Point3D> extended_curve_points;
        extended_curve_points.push_back(extended_start);
        extended_curve_points.insert(extended_curve_points.end(), curve_points.begin(), curve_points.end());
        extended_curve_points.push_back(extended_end);

        segment.points = extended_curve_points;
      }
      else
      {
        // 如果曲线点不足，使用原始点
        segment.points = curve_points;
      }
    }
    else
    {
      std::cerr << "Failed to cast line to curve!" << std::endl;
      segment.points = { line->start, line->end };
    }
  }

  // 应用偏移（根据路径段自身的printer_type）
  // 注意：TEXT类型的 printer_type 在 processGeometryGroup 中动态确定，偏移也在那里应用
  if (!segment.points.empty() && line->type != GeometryType::TEXT)
  {
    double offset = 0.0;
    switch (segment.printer_type) {
      case PrinterType::LEFT_PRINTER:
        offset = offset_config.left_offset;
        break;
      case PrinterType::RIGHT_PRINTER:
        offset = offset_config.right_offset;
        break;
      case PrinterType::CENTER_PRINTER:
        offset = offset_config.center_offset;
        break;
    }

    if (std::abs(offset) > 1e-6) {  // 只有偏移不为0时才应用
      segment.points = applyPathOffset(segment.points, offset);
    }
  }

  return segment;
}

RouteSegment PathPlanner::planConnectionPath(const Point3D& start, const Point3D& goal,
                                             double start_heading, double goal_heading)
{
  RouteSegment segment(RouteType::TRANSITION_PATH);

  // 计算起终点距离
  double distance = start.distance(goal);

  // 计算起点到终点的直线方向
  double straight_heading = std::atan2(goal.y - start.y, goal.x - start.x);

  // 计算转向角度（考虑后退的情况）
  double forward_turn = std::abs(normalize_angle(start_heading - straight_heading));
  double backward_turn = std::abs(normalize_angle(start_heading - straight_heading + M_PI));
  double min_turn_angle = bezier_config_.consider_backward ? std::min(forward_turn, backward_turn) : forward_turn;
  double turn_angle_deg = min_turn_angle * 180.0 / M_PI;

  // 计算到达终点后需要的转向角度
  double goal_turn = std::abs(normalize_angle(straight_heading - goal_heading));
  double goal_turn_deg = goal_turn * 180.0 / M_PI;

  // 总转向角度 = 起点转向 + 终点转向
  double total_turn_deg = turn_angle_deg + goal_turn_deg;

  std::cout << "Planning transition path: dist=" << distance << "m, turn=" << total_turn_deg << "°" << std::endl;

  // 判断是否使用曲线转场的条件：
  // 1. 贝塞尔曲线转场已启用
  // 2. 距离足够大（>= min_curve_distance）
  // 3. 转向角度足够大（>= min_angle_for_curve）
  bool distance_ok = distance >= bezier_config_.min_curve_distance;
  bool angle_ok = total_turn_deg >= bezier_config_.min_angle_for_curve;
  bool use_curve = bezier_config_.enabled && distance_ok && angle_ok;

  if (use_curve)
  {
    std::cout << "  使用贝塞尔曲线转场 (start=" << start_heading * 180.0 / M_PI
              << "°, goal=" << goal_heading * 180.0 / M_PI << "°)" << std::endl;

    if (bezier_config_.use_quintic)
    {
      segment.points = generate_quintic_bezier_transition(start, goal, start_heading, goal_heading);
      std::cout << "  生成五次贝塞尔曲线，点数: " << segment.points.size() << std::endl;
    }
    else
    {
      segment.points = generate_cubic_bezier_transition(start, goal, start_heading, goal_heading);
      std::cout << "  生成三次贝塞尔曲线，点数: " << segment.points.size() << std::endl;
    }
  }
  else
  {
    // 使用直线转场，并说明原因
    std::string reason;
    if (!bezier_config_.enabled)
    {
      reason = "曲线转场未启用";
    }
    else if (!distance_ok)
    {
      reason = "距离过近(" + std::to_string(distance).substr(0,4) + "m<" +
               std::to_string(bezier_config_.min_curve_distance).substr(0,4) + "m)";
    }
    else if (!angle_ok)
    {
      reason = "转向角过小(" + std::to_string(total_turn_deg).substr(0,4) + "°<" +
               std::to_string(bezier_config_.min_angle_for_curve).substr(0,4) + "°)";
    }
    std::cout << "  使用直线转场: " << reason << std::endl;
    segment.points = { start, goal };
  }

  return segment;
}

std::vector<Point3D> PathPlanner::generate_cubic_bezier_transition(const Point3D& start, const Point3D& goal,
                                                                   double start_heading, double goal_heading)
{
  std::vector<Point3D> path_points;

  // 计算起终点距离
  double distance = start.distance(goal);

  // 如果距离太近，直接返回直线
  if (distance < bezier_config_.min_control_distance * 2)
  {
    path_points.push_back(start);
    path_points.push_back(goal);
    return path_points;
  }

  // 计算起点到终点的直线方向
  double straight_heading = std::atan2(goal.y - start.y, goal.x - start.x);

  // 判断是否需要后退执行
  bool execute_backward = false;
  double effective_start_heading = start_heading;
  if (bezier_config_.consider_backward)
  {
    double head_diff = std::abs(normalize_angle(start_heading - straight_heading));
    double tail_diff = std::abs(normalize_angle(start_heading - straight_heading + M_PI));
    execute_backward = (tail_diff < head_diff);
    if (execute_backward)
    {
      effective_start_heading = normalize_angle(start_heading + M_PI);
    }
  }

  // 计算起点方向与直线方向的夹角（用于自适应控制点）
  double angle_diff_start = std::abs(normalize_angle(effective_start_heading - straight_heading));
  double angle_diff_deg = angle_diff_start * 180.0 / M_PI;

  // 计算基础控制点距离
  double base_ratio = bezier_config_.control_point_ratio;

  // 自适应调整：大角度转向时增大控制点距离
  if (bezier_config_.adaptive_control_point && angle_diff_deg > bezier_config_.large_angle_threshold)
  {
    // 角度越大，增量越大（线性插值）
    double angle_factor = (angle_diff_deg - bezier_config_.large_angle_threshold) / (180.0 - bezier_config_.large_angle_threshold);
    angle_factor = std::min(1.0, angle_factor);
    base_ratio += bezier_config_.large_angle_ratio_boost * angle_factor;
    std::cout << "    大角度转向(" << angle_diff_deg << "°)，控制点比例增加到: " << base_ratio << std::endl;
  }

  // 基于最小转弯半径约束控制点距离
  // 对于贝塞尔曲线，起点处的曲率近似为: k ≈ 2/3 * |P0P1 × P0P3| / |P0P1|^3
  // 简化处理：确保控制点距离足够大以满足最小转弯半径
  double min_control_for_radius = bezier_config_.min_turning_radius * angle_diff_start;

  double control_dist = distance * base_ratio;
  control_dist = std::max(control_dist, bezier_config_.min_control_distance);
  control_dist = std::max(control_dist, min_control_for_radius);  // 满足最小转弯半径
  control_dist = std::min(control_dist, bezier_config_.max_control_distance);

  // 计算起点控制点方向
  double p1_heading = effective_start_heading;

  // 根据终点切线策略计算终点控制点方向
  double p2_heading;
  switch (bezier_config_.endpoint_tangent_mode)
  {
    case EndpointTangentMode::ALIGN_PATH:
      p2_heading = goal_heading;
      break;
    case EndpointTangentMode::ALIGN_STRAIGHT:
      p2_heading = straight_heading;
      break;
    case EndpointTangentMode::BLEND:
    default:
      {
        double diff = normalize_angle(goal_heading - straight_heading);
        p2_heading = normalize_angle(straight_heading + diff * bezier_config_.blend_ratio);
      }
      break;
  }

  if (execute_backward)
  {
    p2_heading = normalize_angle(p2_heading + M_PI);
  }

  // 三次贝塞尔曲线的4个控制点
  Point3D p0 = start;
  Point3D p1(start.x + control_dist * std::cos(p1_heading),
             start.y + control_dist * std::sin(p1_heading),
             start.z);
  Point3D p2(goal.x - control_dist * std::cos(p2_heading),
             goal.y - control_dist * std::sin(p2_heading),
             goal.z);
  Point3D p3 = goal;

  std::vector<Point3D> control_points = {p0, p1, p2, p3};

  // 根据分辨率计算采样点数
  double estimated_arc_length = distance * (1.2 + 0.3 * angle_diff_start / M_PI);  // 大角度时增加采样
  int num_points = std::max(2, static_cast<int>(std::ceil(estimated_arc_length / bezier_config_.path_resolution)));

  // 采样贝塞尔曲线
  for (int i = 0; i <= num_points; ++i)
  {
    double t = static_cast<double>(i) / num_points;
    path_points.push_back(evaluate_bezier_point(control_points, t));
  }

  std::cout << "    转向角度: " << angle_diff_deg << "°, 控制点距离: " << control_dist
            << "m, 后退: " << (execute_backward ? "是" : "否") << std::endl;

  return path_points;
}

std::vector<Point3D> PathPlanner::generate_quintic_bezier_transition(const Point3D& start, const Point3D& goal,
                                                                     double start_heading, double goal_heading)
{
  std::vector<Point3D> path_points;

  // 计算起终点距离
  double distance = start.distance(goal);

  // 如果距离太近，直接返回直线
  if (distance < bezier_config_.min_control_distance * 2)
  {
    path_points.push_back(start);
    path_points.push_back(goal);
    return path_points;
  }

  // 计算起点到终点的直线方向
  double straight_heading = std::atan2(goal.y - start.y, goal.x - start.x);

  // 判断是否需要后退执行
  bool execute_backward = false;
  double effective_start_heading = start_heading;
  if (bezier_config_.consider_backward)
  {
    double head_diff = std::abs(normalize_angle(start_heading - straight_heading));
    double tail_diff = std::abs(normalize_angle(start_heading - straight_heading + M_PI));
    execute_backward = (tail_diff < head_diff);
    if (execute_backward)
    {
      effective_start_heading = normalize_angle(start_heading + M_PI);
    }
  }

  // 计算起点方向与直线方向的夹角
  double angle_diff_start = std::abs(normalize_angle(effective_start_heading - straight_heading));
  double angle_diff_deg = angle_diff_start * 180.0 / M_PI;

  // 自适应控制点距离
  double base_ratio = bezier_config_.control_point_ratio;
  if (bezier_config_.adaptive_control_point && angle_diff_deg > bezier_config_.large_angle_threshold)
  {
    double angle_factor = (angle_diff_deg - bezier_config_.large_angle_threshold) / (180.0 - bezier_config_.large_angle_threshold);
    angle_factor = std::min(1.0, angle_factor);
    base_ratio += bezier_config_.large_angle_ratio_boost * angle_factor;
  }

  double min_control_for_radius = bezier_config_.min_turning_radius * angle_diff_start;
  double control_dist = distance * base_ratio;
  control_dist = std::max(control_dist, bezier_config_.min_control_distance);
  control_dist = std::max(control_dist, min_control_for_radius);
  control_dist = std::min(control_dist, bezier_config_.max_control_distance);

  double p1_heading = effective_start_heading;

  double p2_heading;
  switch (bezier_config_.endpoint_tangent_mode)
  {
    case EndpointTangentMode::ALIGN_PATH:
      p2_heading = goal_heading;
      break;
    case EndpointTangentMode::ALIGN_STRAIGHT:
      p2_heading = straight_heading;
      break;
    case EndpointTangentMode::BLEND:
    default:
      {
        double diff = normalize_angle(goal_heading - straight_heading);
        p2_heading = normalize_angle(straight_heading + diff * bezier_config_.blend_ratio);
      }
      break;
  }

  if (execute_backward)
  {
    p2_heading = normalize_angle(p2_heading + M_PI);
  }

  // 五次贝塞尔曲线的6个控制点
  Point3D p0 = start;
  Point3D p1(start.x + control_dist * 0.4 * std::cos(p1_heading),
             start.y + control_dist * 0.4 * std::sin(p1_heading),
             start.z);
  Point3D p2(start.x + control_dist * std::cos(p1_heading),
             start.y + control_dist * std::sin(p1_heading),
             start.z);
  Point3D p3(goal.x - control_dist * std::cos(p2_heading),
             goal.y - control_dist * std::sin(p2_heading),
             goal.z);
  Point3D p4(goal.x - control_dist * 0.4 * std::cos(p2_heading),
             goal.y - control_dist * 0.4 * std::sin(p2_heading),
             goal.z);
  Point3D p5 = goal;

  std::vector<Point3D> control_points = {p0, p1, p2, p3, p4, p5};

  double estimated_arc_length = distance * (1.3 + 0.3 * angle_diff_start / M_PI);
  int num_points = std::max(2, static_cast<int>(std::ceil(estimated_arc_length / bezier_config_.path_resolution)));

  for (int i = 0; i <= num_points; ++i)
  {
    double t = static_cast<double>(i) / num_points;
    path_points.push_back(evaluate_bezier_point(control_points, t));
  }

  std::cout << "    转向角度: " << angle_diff_deg << "°, 控制点距离: " << control_dist
            << "m, 后退: " << (execute_backward ? "是" : "否") << std::endl;

  return path_points;
}

void PathPlanner::processGeometryGroup(const std::vector<std::shared_ptr<Line>>& lines,
                                     std::vector<RouteSegment>& path_segments, const PathOffsetConfig& offset_config,
                                     const std::optional<Point3D>& start_position)
{
  Point3D current_position;
  bool has_current_position = false;

  std::cout << "\n" << std::string(80, '=') << std::endl;
  std::cout << "开始处理几何线段组" << std::endl;
  std::cout << std::string(80, '=') << std::endl;

  // 如果提供了起始位置，使用它作为当前位置
  if (start_position.has_value())
  {
    current_position = start_position.value();
    has_current_position = true;
    std::cout << "\n✅ 使用机器人起始位置:" << std::endl;
    std::cout << "   位置: [" << current_position.x << ", " << current_position.y << ", "
              << current_position.z << "]" << std::endl;
    std::cout << "   将从此位置开始规划第一条转场路径！" << std::endl;
  }
  else
  {
    std::cout << "\n⚠️  未提供机器人起始位置，将使用默认规划方式" << std::endl;
  }

  std::cout << "\n处理线段总数: " << lines.size() << std::endl;

  // 分类处理不同类型的线段
  std::vector<std::shared_ptr<Line>> straight_lines;
  std::vector<std::shared_ptr<Circle>> circles;
  std::vector<std::shared_ptr<Arc>> arcs;
  std::vector<std::shared_ptr<Curve>> curves;
  std::vector<std::shared_ptr<Text>> texts;

  for (const auto& line : lines)
  {
    // 只处理未绘制的线段
    if (!line->is_printed)
    {
      if (line->type == GeometryType::LINE)
      {
        straight_lines.push_back(line);
      }
      else if (line->type == GeometryType::CIRCLE)
      {
        circles.push_back(std::dynamic_pointer_cast<Circle>(line));
      }
      else if (line->type == GeometryType::ARC)
      {
        arcs.push_back(std::dynamic_pointer_cast<Arc>(line));
      }
      else if (line->type == GeometryType::CURVE)
      {
        curves.push_back(std::dynamic_pointer_cast<Curve>(line));
      }
      else if (line->type == GeometryType::TEXT)
      {
        texts.push_back(std::dynamic_pointer_cast<Text>(line));
      }
    }
  }

  std::cout << "Classified lines: " << straight_lines.size() << " straight, " << circles.size() << " circles, "
            << arcs.size() << " arcs, " << curves.size() << " curves, " << texts.size() << " texts" << std::endl;

  // 创建一个包含所有未绘制线段的列表
  std::vector<std::shared_ptr<Line>> remaining_lines;
  for (const auto& line : lines)
  {
    if (!line->is_printed)
    {
      remaining_lines.push_back(line);
    }
  }

  std::cout << "Total remaining lines to process: " << remaining_lines.size() << std::endl;

  // 计算画布中心（用于备用方案确定文字打印机）
  double canvas_center_x = 0.0;
  if (grid_map_generator_)
  {
    double min_x, min_y, max_x, max_y;
    grid_map_generator_->get_map_bounds(min_x, min_y, max_x, max_y);
    canvas_center_x = (min_x + max_x) / 2.0;
  }

  while (!remaining_lines.empty())
  {
    // 找到最近的未绘制线段
    auto nearest_line = findNearestUnprocessedLine(current_position, remaining_lines, offset_config, has_current_position,
                                                   canvas_center_x);

    if (nearest_line)
    {
      std::cout << "\n========== 规划线段 ID: " << nearest_line->id << " ==========" << std::endl;
      std::cout << "线段类型: " << static_cast<int>(nearest_line->type);
      
      // 打印类型名称
      switch (nearest_line->type)
      {
        case GeometryType::LINE: std::cout << " (LINE)"; break;
        case GeometryType::POLYLINE: std::cout << " (POLYLINE)"; break;
        case GeometryType::CIRCLE: std::cout << " (CIRCLE)"; break;
        case GeometryType::ARC: std::cout << " (ARC)"; break;
        case GeometryType::ELLIPSE: std::cout << " (ELLIPSE)"; break;
        case GeometryType::CURVE: std::cout << " (CURVE)"; break;
        case GeometryType::TEXT: std::cout << " (TEXT)"; break;
        default: std::cout << " (UNKNOWN)"; break;
      }
      std::cout << std::endl;
      
      std::cout << "线段起点: [" << nearest_line->start.x << ", " << nearest_line->start.y << ", "
                << nearest_line->start.z << "]" << std::endl;
      std::cout << "线段终点: [" << nearest_line->end.x << ", " << nearest_line->end.y << ", "
                << nearest_line->end.z << "]" << std::endl;

      // 规划该线的绘图路径
      RouteSegment drawing_segment = planGeometryPath(nearest_line, offset_config);

      if (drawing_segment.points.empty())
      {
        std::cerr << "Failed to plan drawing path for line ID: " << nearest_line->id << std::endl;
      }
      else
      {
        // 如果是文字类型，需要根据运动方向确定打印机，然后应用偏移
        if (nearest_line->type == GeometryType::TEXT)
        {
          auto text = std::dynamic_pointer_cast<Text>(nearest_line);
          if (text)
          {
            if (has_current_position)
            {
              // 根据运动方向决定使用左侧还是右侧喷码机
              drawing_segment.printer_type = determineTextPrinter(
                current_position, text->position, text->rotation);
            }
            else
            {
              // 没有当前位置时，根据文字在画布上的位置决定
              drawing_segment.printer_type = determineTextPrinterByPosition(
                text->position, canvas_center_x);
            }

            // TEXT 类型在确定 printer_type 后应用偏移
            double offset = 0.0;
            switch (drawing_segment.printer_type) {
              case PrinterType::LEFT_PRINTER:
                offset = offset_config.left_offset;
                break;
              case PrinterType::RIGHT_PRINTER:
                offset = offset_config.right_offset;
                break;
              case PrinterType::CENTER_PRINTER:
                offset = offset_config.center_offset;
                break;
            }
            if (std::abs(offset) > 1e-6 && !drawing_segment.points.empty()) {
              drawing_segment.points = applyPathOffset(drawing_segment.points, offset);
            }
          }
        }

        // 如果已经有当前位置,则规划一条转场路径
        if (has_current_position)
        {
          double distance = current_position.distance(drawing_segment.points.front());
          std::cout << "\n>>> 生成转场路径 <<<" << std::endl;
          std::cout << "  当前位置: [" << current_position.x << ", " << current_position.y << ", "
                    << current_position.z << "]" << std::endl;
          std::cout << "  目标位置: [" << drawing_segment.points.front().x << ", "
                    << drawing_segment.points.front().y << ", " << drawing_segment.points.front().z << "]" << std::endl;
          std::cout << "  转场距离: " << distance << " 米" << std::endl;

          // 计算起点朝向（从前一条路径的末端方向）
          double start_heading = 0.0;
          auto prev_heading = previous_path_end_heading(path_segments);
          if (prev_heading.has_value())
          {
            start_heading = prev_heading.value();
          }
          else
          {
            // 如果没有前一条路径，使用指向目标的方向
            double dx = drawing_segment.points.front().x - current_position.x;
            double dy = drawing_segment.points.front().y - current_position.y;
            start_heading = std::atan2(dy, dx);
          }

          // 计算终点朝向（从下一条绘图路径的起始方向）
          double goal_heading = 0.0;
          auto next_heading = heading_from_first_motion(drawing_segment.points);
          if (next_heading.has_value())
          {
            goal_heading = next_heading.value();
          }
          else
          {
            // 如果无法计算，使用从起点指向终点的方向
            double dx = drawing_segment.points.front().x - current_position.x;
            double dy = drawing_segment.points.front().y - current_position.y;
            goal_heading = std::atan2(dy, dx);
          }

          RouteSegment transition_segment = planConnectionPath(current_position, drawing_segment.points.front(),
                                                               start_heading, goal_heading);

          if (!transition_segment.points.empty())
          {
            // 转场路径的 ink 信息与下一个绘图路径一致
            transition_segment.printer_type = drawing_segment.printer_type;
            transition_segment.ink_mode = drawing_segment.ink_mode;
            
            // 如果下一个路径是 TEXT 类型，复制文字内容
            if (drawing_segment.ink_mode == InkMode::TEXT)
            {
              transition_segment.text_content = drawing_segment.text_content;
            }

            transition_segment.execute_backward = should_execute_transition_backward(path_segments, transition_segment);

            path_segments.push_back(transition_segment);
            std::cout << "✅ 转场路径已添加，包含 " << transition_segment.points.size() << " 个点" << std::endl;
          }
          else
          {
            std::cerr << "❌ 转场路径生成失败 (line ID: " << nearest_line->id << ")" << std::endl;
          }
        }
        else
        {
          std::cout << "\n⚠️  这是第一条路径，没有当前位置，不生成转场路径" << std::endl;
        }

        // 添加绘图路径
        path_segments.push_back(drawing_segment);
        std::cout << "Added drawing path for line ID: " << nearest_line->id << " with " << drawing_segment.points.size()
                  << " points" << std::endl;

        // 更新当前位置为绘图路径的终点
        current_position = drawing_segment.points.back();
        has_current_position = true;
      }

      // 将已规划的线段标记为已绘制
      nearest_line->is_printed = true;
      remaining_lines.erase(std::remove(remaining_lines.begin(), remaining_lines.end(), nearest_line),
                            remaining_lines.end());

      std::cout << "\n剩余待处理线段: " << remaining_lines.size() << std::endl;
    }
    else
    {
      std::cout << "\n❌ 未找到更多未绘制的线段，规划结束" << std::endl;
      break;
    }
  }

  std::cout << "\n" << std::string(80, '=') << std::endl;
  std::cout << "线段组处理完成，共生成 " << path_segments.size() << " 个路径段" << std::endl;
  std::cout << std::string(80, '=') << std::endl;
}

std::vector<Point3D> PathPlanner::applyPathOffset(const std::vector<Point3D>& original_path, double offset)
{
  if (original_path.size() < 2)
  {
    return original_path;
  }

  // 对闭合路径（首尾点相同）做环形处理，保证偏移后的路径仍然闭合且连续
  const bool is_closed = (original_path.size() >= 3) && (original_path.front().distance(original_path.back()) < 1e-6);
  if (is_closed)
  {
    const size_t n = original_path.size() - 1;  // 去掉最后一个重复点
    if (n < 2)
    {
      return original_path;
    }

    std::vector<Point3D> offset_path;
    offset_path.reserve(original_path.size());

    for (size_t i = 0; i < n; ++i)
    {
      const Point3D& prev = original_path[(i + n - 1) % n];
      const Point3D& current = original_path[i];
      const Point3D& next = original_path[(i + 1) % n];

      double dx1 = current.x - prev.x;
      double dy1 = current.y - prev.y;
      double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);

      double dx2 = next.x - current.x;
      double dy2 = next.y - current.y;
      double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

      if (len1 < 1e-6 || len2 < 1e-6)
      {
        offset_path.push_back(current);
        continue;
      }

      double nx1 = -dy1 / len1;
      double ny1 = dx1 / len1;
      double nx2 = -dy2 / len2;
      double ny2 = dx2 / len2;

      double nx = (nx1 + nx2) / 2.0;
      double ny = (ny1 + ny2) / 2.0;
      double nlen = std::sqrt(nx * nx + ny * ny);
      if (nlen > 1e-6)
      {
        nx /= nlen;
        ny /= nlen;
      }
      else
      {
        nx = nx1;
        ny = ny1;
      }

      Point3D offset_point;
      offset_point.x = current.x + nx * offset;
      offset_point.y = current.y + ny * offset;
      offset_point.z = current.z;
      offset_path.push_back(offset_point);
    }

    offset_path.push_back(offset_path.front());
    return offset_path;
  }

  std::vector<Point3D> offset_path;
  offset_path.reserve(original_path.size());

  // 处理第一个点
  const Point3D& first = original_path[0];
  const Point3D& second = original_path[1];

  // 计算第一段线段的方向向量
  double dx_first = second.x - first.x;
  double dy_first = second.y - first.y;
  double length_first = std::sqrt(dx_first * dx_first + dy_first * dy_first);

  if (length_first > 1e-6)
  {
    // 计算法向量（顺时针90度旋转）
    double nx = -dy_first / length_first;
    double ny = dx_first / length_first;

    // 沿法向量方向偏移
    Point3D offset_point;
    offset_point.x = first.x + nx * offset;
    offset_point.y = first.y + ny * offset;
    offset_point.z = first.z;

    offset_path.push_back(offset_point);
  }
  else
  {
    offset_path.push_back(first);
  }

  // 处理中间的点
  for (size_t i = 1; i < original_path.size() - 1; ++i)
  {
    const Point3D& prev = original_path[i - 1];
    const Point3D& current = original_path[i];
    const Point3D& next = original_path[i + 1];

    // 计算前后两段线段的方向向量
    double dx1 = current.x - prev.x;
    double dy1 = current.y - prev.y;
    double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);

    double dx2 = next.x - current.x;
    double dy2 = next.y - current.y;
    double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

    // 如果线段长度接近0，直接添加原始点
    if (len1 < 1e-6 || len2 < 1e-6)
    {
      offset_path.push_back(current);
      continue;
    }

    // 计算两个法向量
    double nx1 = -dy1 / len1;
    double ny1 = dx1 / len1;

    double nx2 = -dy2 / len2;
    double ny2 = dx2 / len2;

    // 计算两个法向量的平均值
    double nx = (nx1 + nx2) / 2.0;
    double ny = (ny1 + ny2) / 2.0;

    // 归一化平均法向量
    double len = std::sqrt(nx * nx + ny * ny);
    if (len > 1e-6)
    {
      nx /= len;
      ny /= len;
    }
    else
    {
      // 如果平均法向量接近0，使用第一个法向量
      nx = nx1;
      ny = ny1;
    }

    // 沿法向量方向偏移
    Point3D offset_point;
    offset_point.x = current.x + nx * offset;
    offset_point.y = current.y + ny * offset;
    offset_point.z = current.z;

    offset_path.push_back(offset_point);
  }

  // 处理最后一个点
  const Point3D& last = original_path.back();
  const Point3D& second_last = original_path[original_path.size() - 2];

  // 计算最后一段线段的方向向量
  double dx_last = last.x - second_last.x;
  double dy_last = last.y - second_last.y;
  double length_last = std::sqrt(dx_last * dx_last + dy_last * dy_last);

  if (length_last > 1e-6)
  {
    // 计算法向量（顺时针90度旋转）
    double nx = -dy_last / length_last;
    double ny = dx_last / length_last;

    // 沿法向量方向偏移
    Point3D offset_point;
    offset_point.x = last.x + nx * offset;
    offset_point.y = last.y + ny * offset;
    offset_point.z = last.z;

    offset_path.push_back(offset_point);
  }
  else
  {
    offset_path.push_back(last);
  }

  return offset_path;
}

std::vector<Point3D> PathPlanner::discretize_curve(const std::shared_ptr<Curve>& curve, int num_points)
{
  std::vector<Point3D> points;

  if (!curve || curve->control_points.size() < 2)
  {
    std::cerr << "Invalid curve or insufficient control points for curve ID: " << (curve ? curve->id : -1) << std::endl;
    return points;
  }

  std::cout << "Discretizing curve ID: " << curve->id << ", degree: " << curve->degree
            << ", control points: " << curve->control_points.size() << std::endl;

  // 根据曲线长度动态调整采样点数量，确保高精度曲线拟合
  // 对于较长的曲线，使用更多的点
  double curve_length = curve->length > 0 ? curve->length : 100.0;  // 默认长度为100，以防长度为0
  num_points = std::max(200, static_cast<int>(curve_length / 0.005));
  std::cout << "Using " << num_points << " points for discretization (curve length: " << curve_length << ")"
            << std::endl;

  points.reserve(num_points);

  // 确保第一个点是曲线的实际起点（第一个控制点）
  if (curve->control_points.size() > 0)
  {
    points.push_back(curve->control_points[0]);
  }

  // 根据曲线类型生成中间点
  if (curve->degree == 1)
  {
    // 线性曲线
    if (curve->control_points.size() >= 2)
    {
      const Point3D& p0 = curve->control_points[0];
      const Point3D& p1 = curve->control_points[1];

      for (int i = 1; i < num_points - 1; ++i)
      {
        double t = static_cast<double>(i) / (num_points - 1);
        Point3D point;
        point.x = p0.x + t * (p1.x - p0.x);
        point.y = p0.y + t * (p1.y - p0.y);
        point.z = p0.z + t * (p1.z - p0.z);
        points.push_back(point);
      }
    }
  }
  else if (curve->degree == 2 && curve->control_points.size() >= 3)
  {
    // 二次贝塞尔曲线
    const Point3D& p0 = curve->control_points[0];
    const Point3D& p1 = curve->control_points[1];
    const Point3D& p2 = curve->control_points[2];

    for (int i = 1; i < num_points - 1; ++i)
    {
      double t = static_cast<double>(i) / (num_points - 1);
      double mt = 1.0 - t;
      double mt2 = mt * mt;
      double t2 = t * t;

      Point3D point;
      point.x = mt2 * p0.x + 2 * mt * t * p1.x + t2 * p2.x;
      point.y = mt2 * p0.y + 2 * mt * t * p1.y + t2 * p2.y;
      point.z = mt2 * p0.z + 2 * mt * t * p1.z + t2 * p2.z;

      points.push_back(point);
    }
  }
  else if (curve->degree == 3 && curve->control_points.size() >= 4)
  {
    // 三次贝塞尔曲线
    const Point3D& p0 = curve->control_points[0];
    const Point3D& p1 = curve->control_points[1];
    const Point3D& p2 = curve->control_points[2];
    const Point3D& p3 = curve->control_points[3];

    for (int i = 1; i < num_points - 1; ++i)
    {
      double t = static_cast<double>(i) / (num_points - 1);
      double mt = 1.0 - t;
      double mt2 = mt * mt;
      double mt3 = mt2 * mt;
      double t2 = t * t;
      double t3 = t2 * t;

      Point3D point;
      point.x = mt3 * p0.x + 3 * mt2 * t * p1.x + 3 * mt * t2 * p2.x + t3 * p3.x;
      point.y = mt3 * p0.y + 3 * mt2 * t * p1.y + 3 * mt * t2 * p2.y + t3 * p3.y;
      point.z = mt3 * p0.z + 3 * mt2 * t * p1.z + 3 * mt * t2 * p2.z + t3 * p3.z;

      points.push_back(point);
    }
  }
  else if (!curve->weights.empty() && curve->weights.size() == curve->control_points.size() && !curve->knots.empty())
  {
    // NURBS曲线
    // 确保节点向量有效
    if (curve->knots.size() < curve->degree + curve->control_points.size() + 1)
    {
      std::cerr << "Invalid knot vector for NURBS curve ID: " << curve->id << std::endl;

      // 退化为使用De Casteljau算法
      for (int i = 1; i < num_points - 1; ++i)
      {
        double t = static_cast<double>(i) / (num_points - 1);
        points.push_back(evaluate_bezier_point(curve->control_points, t));
      }
    }
    else
    {
      // 使用标准NURBS计算
      for (int i = 1; i < num_points - 1; ++i)
      {
        double t = static_cast<double>(i) / (num_points - 1);
        points.push_back(evaluate_nurbs_point(*curve, t));
      }
    }
  }
  else
  {
    // 其他情况，使用通用的De Casteljau算法
    for (int i = 1; i < num_points - 1; ++i)
    {
      double t = static_cast<double>(i) / (num_points - 1);
      points.push_back(evaluate_bezier_point(curve->control_points, t));
    }
  }

  // 确保最后一个点是曲线的实际终点（最后一个控制点）
  if (curve->control_points.size() > 0)
  {
    points.push_back(curve->control_points.back());
  }

  // 验证生成的点数
  std::cout << "Successfully generated " << points.size() << " points for curve ID " << curve->id << std::endl;

  // 进行数值检查 - 计算点之间的平均距离，判断是否有异常
  if (points.size() >= 2)
  {
    double total_distance = 0.0;
    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      total_distance += points[i].distance(points[i + 1]);
    }
    double avg_distance = total_distance / (points.size() - 1);
    std::cout << "Average distance between points: " << avg_distance << std::endl;

    // 如果平均距离异常大，可能是采样不足
    if (avg_distance > 0.1)
    {
      std::cerr << "Warning: Large distance between curve points, consider increasing sampling density" << std::endl;
    }
  }

  return points;
}

Point3D PathPlanner::evaluate_bezier_point(const std::vector<Point3D>& control_points, double t)
{
  if (control_points.empty())
  {
    return Point3D(0, 0, 0);
  }

  if (control_points.size() == 1 || t <= 0.0)
  {
    return control_points.front();
  }

  if (t >= 1.0)
  {
    return control_points.back();
  }

  // 使用De Casteljau算法计算贝塞尔曲线上的点
  std::vector<Point3D> temp = control_points;
  int n = temp.size();

  for (int r = 1; r < n; ++r)
  {
    for (int i = 0; i < n - r; ++i)
    {
      temp[i].x = (1.0 - t) * temp[i].x + t * temp[i + 1].x;
      temp[i].y = (1.0 - t) * temp[i].y + t * temp[i + 1].y;
      temp[i].z = (1.0 - t) * temp[i].z + t * temp[i + 1].z;
    }
  }

  return temp[0];
}

Point3D PathPlanner::evaluate_nurbs_point(const Curve& curve, double t)
{
  // 如果曲线参数不完整，回退到贝塞尔曲线计算
  if (curve.control_points.empty() || curve.weights.size() != curve.control_points.size() || curve.knots.empty())
  {
    return evaluate_bezier_point(curve.control_points, t);
  }

  // 映射参数t到节点区间
  double u = t * (curve.knots.back() - curve.knots.front()) + curve.knots.front();

  // 查找u所在的节点跨度
  int span = 0;
  for (size_t i = 0; i < curve.knots.size() - 1; ++i)
  {
    if (u >= curve.knots[i] && u < curve.knots[i + 1])
    {
      span = i;
      break;
    }
  }

  // 特殊情况，u在最后一个节点
  if (u >= curve.knots.back())
  {
    span = curve.knots.size() - 2;
  }

  // 计算基函数
  int p = curve.degree;
  std::vector<double> N(p + 1, 0.0);

  // 初始化0次基函数
  for (int i = 0; i <= p; ++i)
  {
    if (span + i - p >= 0 && span + i + 1 < static_cast<int>(curve.knots.size()) &&
        u >= curve.knots[span + i - p] && u < curve.knots[span + i + 1])
    {
      N[i] = 1.0;
    }
  }

  // 计算高阶基函数
  for (int k = 1; k <= p; ++k)
  {
    for (int i = p; i >= k; --i)
    {
      int idx1 = span + i + 1 - k;
      int idx2 = span + i - p;
      int idx3 = span + i + 1;
      int idx4 = span + i - k + 1;
      
      if (idx1 < 0 || idx2 < 0 || idx3 >= static_cast<int>(curve.knots.size()) || 
          idx4 < 0 || idx4 >= static_cast<int>(curve.knots.size()))
      {
        continue;
      }

      double d1 = curve.knots[idx1] - curve.knots[idx2];
      double d2 = curve.knots[idx3] - curve.knots[idx4];

      double left = 0.0, right = 0.0;

      if (std::fabs(d1) > 1e-10 && i > 0)
      {
        left = (u - curve.knots[idx2]) / d1 * N[i - 1];
      }

      if (std::fabs(d2) > 1e-10)
      {
        right = (curve.knots[idx3] - u) / d2 * N[i];
      }

      N[i] = left + right;
    }
  }

  // 计算NURBS点
  double sumW = 0.0;
  Point3D point(0.0, 0.0, 0.0);

  // 考虑权重和基函数
  for (int i = 0; i <= p; ++i)
  {
    int idx = span - p + i;
    if (idx >= 0 && idx < static_cast<int>(curve.control_points.size()))
    {
      double w = curve.weights[idx];
      double R = N[i] * w;
      sumW += R;

      point.x += curve.control_points[idx].x * R;
      point.y += curve.control_points[idx].y * R;
      point.z += curve.control_points[idx].z * R;
    }
  }

  // 规范化
  if (std::fabs(sumW) > 1e-10)
  {
    point.x /= sumW;
    point.y /= sumW;
    point.z /= sumW;
  }

  return point;
}

std::vector<Point3D> PathPlanner::generate_straight_path(const Point3D& start, const Point3D& end, double resolution)
{
  std::vector<Point3D> path;

  // 计算两点之间的距离
  double distance = start.distance(end);

  if (distance < 1e-6)
  {
    path.push_back(start);
    return path;
  }

  // 计算单位方向向量，确保方向精确
  double dx = (end.x - start.x) / distance;
  double dy = (end.y - start.y) / distance;
  double dz = (end.z - start.z) / distance;

  // 根据分辨率计算点数
  int num_points = std::max(2, static_cast<int>(distance / resolution) + 1);
  path.reserve(num_points);

  // 使用方向向量和步长精确生成点
  for (int i = 0; i < num_points; ++i)
  {
    double t = static_cast<double>(i) / (num_points - 1) * distance;
    Point3D point;
    point.x = start.x + dx * t;
    point.y = start.y + dy * t;
    point.z = start.z + dz * t;

    path.push_back(point);
  }

  return path;
}

cv::Mat PathPlanner::create_path_image(const PathVisualizationConfig& config)
{
  // 获取基本的栅格地图尺寸
  int width = grid_map_generator_->get_width();
  int height = grid_map_generator_->get_height();

  // 获取地图世界坐标边界
  double min_x, min_y, max_x, max_y;
  grid_map_generator_->get_map_bounds(min_x, min_y, max_x, max_y);

  // 获取栅格分辨率
  double resolution = grid_map_generator_->get_resolution();

  // 检查图像大小是否过大
  const int MAX_DIMENSION = 8000;
  int scale = config.scale;

  if (height * scale > MAX_DIMENSION || width * scale > MAX_DIMENSION)
  {
    std::cout << "Grid map too large (" << width << "x" << height << "). Reducing scale factor." << std::endl;
    scale = std::max(1, std::min(MAX_DIMENSION / width, MAX_DIMENSION / height));
    std::cout << "New scale factor: " << scale << std::endl;
  }

  // 计算内存需求
  size_t estimated_memory = static_cast<size_t>(height) * width * scale * scale * 3;
  const size_t MAX_MEMORY = 1024 * 1024 * 1024;  // 1GB

  if (estimated_memory > MAX_MEMORY)
  {
    std::cout << "Estimated memory usage too high (" << estimated_memory << " bytes). Using reduced scale."
              << std::endl;
    scale = std::max(1, scale / 2);

    if (static_cast<size_t>(height) * width * scale * scale * 3 > MAX_MEMORY)
    {
      scale = 1;
    }
  }

  // 创建图像
  cv::Mat image(height * scale, width * scale, CV_8UC3, config.free_space_color);

  return image;
}

void PathPlanner::draw_grid_map(cv::Mat& image, const PathVisualizationConfig& config)
{
  int width = grid_map_generator_->get_width();
  int height = grid_map_generator_->get_height();
  int scale = config.scale;

  // 绘制栅格地图
  for (int y = 0; y < height; ++y)
  {
    for (int x = 0; x < width; ++x)
    {
      if (grid_map_[y][x] == 1)
      {
        // 障碍物
        cv::rectangle(image, cv::Point(x * scale, y * scale), cv::Point((x + 1) * scale - 1, (y + 1) * scale - 1),
                      config.obstacle_color,
                      -1);  // -1表示填充矩形
      }
    }
  }

  // 添加栅格线
  if (config.show_grid_lines && scale >= 3 && width * height < 500000)
  {
    // 计算栅格线颜色
    cv::Scalar grid_color(200, 200, 200);

    for (int y = 0; y <= height; ++y)
    {
      cv::line(image, cv::Point(0, y * scale), cv::Point(width * scale, y * scale), grid_color, 1);
    }

    for (int x = 0; x <= width; ++x)
    {
      cv::line(image, cv::Point(x * scale, 0), cv::Point(x * scale, height * scale), grid_color, 1);
    }
  }
}

void PathPlanner::draw_axis_lines(cv::Mat& image, const PathVisualizationConfig& config)
{
  // 获取轴线点
  const auto& axis_points = grid_map_generator_->get_axis_points();
  int scale = config.scale;

  // 绘制所有轴线
  for (const auto& axis_point : axis_points)
  {
    int x1 = axis_point.x1;
    int y1 = axis_point.y1;
    int x2 = axis_point.x2;
    int y2 = axis_point.y2;

    // 使用抗锯齿线条
    cv::line(image, cv::Point(x1 * scale + scale / 2, y1 * scale + scale / 2),
             cv::Point(x2 * scale + scale / 2, y2 * scale + scale / 2), config.axis_line_color,
             std::max(2, scale / 4),  // 线宽
             cv::LINE_AA);            // 使用抗锯齿
  }
}

void PathPlanner::draw_circle(cv::Mat& image, const std::shared_ptr<Circle>& circle, const cv::Scalar& color,
                              int thickness, const PathVisualizationConfig& config)
{
  if (!circle || !grid_map_generator_)
  {
    return;
  }

  int scale = config.scale;

  // 将圆心转换为图像坐标
  int center_x, center_y;
  if (grid_map_generator_->convertWorldToGrid(circle->center.x, circle->center.y, center_x, center_y))
  {
    // 计算半径（像素）
    double radius = circle->radius / grid_map_generator_->get_resolution() * scale;

    // 绘制圆
    cv::circle(image, cv::Point(center_x * scale + scale / 2, center_y * scale + scale / 2), static_cast<int>(radius),
               color, thickness, cv::LINE_AA);
  }
}

void PathPlanner::draw_arc(cv::Mat& image, const std::shared_ptr<Arc>& arc, const cv::Scalar& color, int thickness,
                           const PathVisualizationConfig& config)
{
  if (!arc || !grid_map_generator_)
  {
    return;
  }

  int scale = config.scale;

  // 将圆心转换为图像坐标
  int center_x, center_y;
  if (grid_map_generator_->convertWorldToGrid(arc->center.x, arc->center.y, center_x, center_y))
  {
    // 计算半径（像素）
    double radius = arc->radius / grid_map_generator_->get_resolution() * scale;

    // 转换角度到度
    double start_angle = arc->start_angle * 180.0 / M_PI;
    double end_angle = arc->end_angle * 180.0 / M_PI;

    // 确保角度范围正确
    if (end_angle < start_angle)
    {
      end_angle += 360.0;
    }

    // 绘制圆弧
    cv::ellipse(image, cv::Point(center_x * scale + scale / 2, center_y * scale + scale / 2),
                cv::Size(static_cast<int>(radius), static_cast<int>(radius)),
                0,  // 旋转角度
                start_angle, end_angle, color, thickness, cv::LINE_AA);
  }
}

void PathPlanner::draw_path(cv::Mat& image, const std::vector<Point3D>& path, const cv::Scalar& color, int thickness,
                            const PathVisualizationConfig& config)
{
  if (path.empty())
  {
    return;
  }

  int scale = config.scale;
  std::vector<cv::Point> image_points;
  image_points.reserve(path.size());

  // 将世界坐标转换为图像坐标
  for (const auto& point : path)
  {
    int grid_x, grid_y;
    if (grid_map_generator_->convertWorldToGrid(point.x, point.y, grid_x, grid_y))
    {
      image_points.push_back(cv::Point(grid_x * scale + scale / 2, grid_y * scale + scale / 2));
    }
  }

  // 添加安全检查，确保有足够的点
  if (image_points.size() < 2)
  {
    std::cerr << "Warning: Not enough valid points to draw path (only " << image_points.size() << " points)"
              << std::endl;
    return;
  }

  // 绘制路径线段
  for (size_t i = 0; i < image_points.size() - 1; ++i)
  {
    cv::line(image, image_points[i], image_points[i + 1], color, thickness, cv::LINE_AA);
  }

  // 在路径上的每个点绘制小圆点
  for (const auto& point : image_points)
  {
    cv::circle(image, point, thickness / 2, color, -1, cv::LINE_AA);
  }
}

void PathPlanner::draw_legend(cv::Mat& image, const PathVisualizationConfig& config)
{
  int text_scale = std::max(1, config.scale / 2);
  double font_scale = 0.5 * text_scale;
  int text_thickness = std::max(1, text_scale / 2);

  // 起始位置
  int legend_x = 10;
  int legend_y = 20 * text_scale;

  // 标题
  cv::putText(image, "Path Planning Visualization", cv::Point(legend_x, legend_y), cv::FONT_HERSHEY_SIMPLEX, font_scale,
              cv::Scalar(0, 0, 0), text_thickness);
  legend_y += 25 * text_scale;

  // 地图图例
  cv::rectangle(image, cv::Point(legend_x, legend_y - text_scale * 3),
                cv::Point(legend_x + text_scale * 10, legend_y - text_scale), config.obstacle_color, -1);
  cv::putText(image, "Obstacles", cv::Point(legend_x + text_scale * 12, legend_y), cv::FONT_HERSHEY_SIMPLEX, font_scale,
              cv::Scalar(0, 0, 0), text_thickness);
  legend_y += 20 * text_scale;

  // 轴线图例
  cv::line(image, cv::Point(legend_x, legend_y - text_scale * 2),
           cv::Point(legend_x + text_scale * 10, legend_y - text_scale * 2), config.axis_line_color,
           std::max(2, text_scale), cv::LINE_AA);
  cv::putText(image, "Axis Lines", cv::Point(legend_x + text_scale * 12, legend_y), cv::FONT_HERSHEY_SIMPLEX,
              font_scale, cv::Scalar(0, 0, 0), text_thickness);
  legend_y += 20 * text_scale;

  // 绘图路径图例
  cv::line(image, cv::Point(legend_x, legend_y - text_scale * 2),
           cv::Point(legend_x + text_scale * 10, legend_y - text_scale * 2), config.path_color, std::max(2, text_scale),
           cv::LINE_AA);
  cv::putText(image, "Drawing Path", cv::Point(legend_x + text_scale * 12, legend_y), cv::FONT_HERSHEY_SIMPLEX,
              font_scale, cv::Scalar(0, 0, 0), text_thickness);
  legend_y += 20 * text_scale;

  // 转场路径图例
  cv::line(image, cv::Point(legend_x, legend_y - text_scale * 2),
           cv::Point(legend_x + text_scale * 10, legend_y - text_scale * 2), config.transition_path_color,
           std::max(2, text_scale), cv::LINE_AA);
  cv::putText(image, "Transition Path", cv::Point(legend_x + text_scale * 12, legend_y), cv::FONT_HERSHEY_SIMPLEX,
              font_scale, cv::Scalar(0, 0, 0), text_thickness);
  legend_y += 20 * text_scale;

  // 起点图例
  cv::circle(image, cv::Point(legend_x + text_scale * 5, legend_y - text_scale * 2), config.point_radius,
             config.start_point_color, -1, cv::LINE_AA);
  cv::putText(image, "Start Point", cv::Point(legend_x + text_scale * 12, legend_y), cv::FONT_HERSHEY_SIMPLEX,
              font_scale, cv::Scalar(0, 0, 0), text_thickness);
  legend_y += 20 * text_scale;

  // 终点图例
  cv::circle(image, cv::Point(legend_x + text_scale * 5, legend_y - text_scale * 2), config.point_radius,
             config.goal_point_color, -1, cv::LINE_AA);
  cv::putText(image, "Goal Point", cv::Point(legend_x + text_scale * 12, legend_y), cv::FONT_HERSHEY_SIMPLEX,
              font_scale, cv::Scalar(0, 0, 0), text_thickness);
}

void PathPlanner::draw_path_label(cv::Mat& image, const std::vector<Point3D>& path, int label,
                                  const PathVisualizationConfig& config, double min_x, double min_y, double resolution)
{
  if (path.size() < 2)
  {
    return;
  }

  // 对于只有两个点的路径（如直线），计算中间点
  Point3D label_point;
  if (path.size() == 2)
  {
    // 计算起点和终点的中间点
    label_point.x = (path[0].x + path[1].x) / 2.0;
    label_point.y = (path[0].y + path[1].y) / 2.0;
    label_point.z = (path[0].z + path[1].z) / 2.0;
  }
  else
  {
    // 对于有多个点的路径，选择适当的中间点
    size_t label_position_index;

    if (path.size() <= 3)
    {
      label_position_index = path.size() / 2;
    }
    else
    {
      // 对于较长的路径，选择靠前位置以避免拥挤
      label_position_index = path.size() / 4;
    }

    label_point = path[label_position_index];
  }

  // 转换到新的坐标系的图像坐标
  int img_x = static_cast<int>((label_point.x - min_x) / resolution) * config.scale + config.scale / 2;
  int img_y = static_cast<int>((label_point.y - min_y) / resolution) * config.scale + config.scale / 2;

  cv::Point text_pos(img_x, img_y);
  std::string label_text = std::to_string(label);

  double font_scale = config.label_font_size / 100.0;
  int thickness = config.label_thickness;
  int baseline = 0;

  // 计算文本大小以便居中
  cv::Size text_size = cv::getTextSize(label_text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);

  text_pos.x -= text_size.width / 2;
  text_pos.y += text_size.height / 2;

  // 为不同类型的路径选择不同的颜色
  cv::Scalar label_color = config.label_color;

  // 绘制带背景的文本，提高可读性
  cv::Scalar bg_color(255, 255, 255, 100);  // 半透明白色背景

  // 绘制背景矩形
  cv::rectangle(image, cv::Point(text_pos.x - 5, text_pos.y - text_size.height - 5),
                cv::Point(text_pos.x + text_size.width + 5, text_pos.y + 5), bg_color,
                -1);  // 填充

  // 绘制文本
  cv::putText(image, label_text, text_pos, cv::FONT_HERSHEY_SIMPLEX, font_scale, label_color, thickness, cv::LINE_AA);
}

bool PathPlanner::visualize_paths(const std::vector<RouteSegment>& path_segments, const std::string& file_path,
                                  const PathVisualizationConfig& config)
{
  if (!grid_map_generator_)
  {
    std::cerr << "Grid map generator not set!" << std::endl;
    return false;
  }

  try
  {
    // 检查所有路径点是否都在当前地图范围内
    double min_x, min_y, max_x, max_y;
    grid_map_generator_->get_map_bounds(min_x, min_y, max_x, max_y);

    // 扩展地图边界以包含所有路径点
    bool need_resize = false;
    double path_min_x = min_x;
    double path_min_y = min_y;
    double path_max_x = max_x;
    double path_max_y = max_y;

    // 遍历所有路径段点，找出最大边界
    for (const auto& segment : path_segments)
    {
      for (const auto& point : segment.points)
      {
        path_min_x = std::min(path_min_x, point.x);
        path_min_y = std::min(path_min_y, point.y);
        path_max_x = std::max(path_max_x, point.x);
        path_max_y = std::max(path_max_y, point.y);
      }
    }

    // 如果路径超出了地图边界，需要调整地图大小
    double padding = 0.5;  // 添加额外边距，单位：米
    if (path_min_x < min_x || path_min_y < min_y || path_max_x > max_x || path_max_y > max_y)
    {
      std::cout << "Paths exceed map bounds. Adjusting visualization..." << std::endl;

      // 扩展边界，添加额外填充
      min_x = std::min(min_x, path_min_x) - padding;
      min_y = std::min(min_y, path_min_y) - padding;
      max_x = std::max(max_x, path_max_x) + padding;
      max_y = std::max(max_y, path_max_y) + padding;

      need_resize = true;
    }

    // 根据栅格分辨率计算新的地图尺寸
    double resolution = grid_map_generator_->get_resolution();
    int new_width = static_cast<int>((max_x - min_x) / resolution) + 1;
    int new_height = static_cast<int>((max_y - min_y) / resolution) + 1;

    // 检查图像大小是否过大
    const int MAX_DIMENSION = 8000;
    int scale = config.scale;

    if (new_height * scale > MAX_DIMENSION || new_width * scale > MAX_DIMENSION)
    {
      std::cout << "Adjusted map too large (" << new_width << "x" << new_height << "). Reducing scale factor."
                << std::endl;
      scale = std::max(1, std::min(MAX_DIMENSION / new_width, MAX_DIMENSION / new_height));
      std::cout << "New scale factor: " << scale << std::endl;
    }

    // 注意：visualize_paths 可能会在不改变地图尺寸(need_resize=false)的情况下缩小 scale。
    // 为确保轴线/标签/图例与路径绘制保持一致，后续统一使用 effective_config。
    PathVisualizationConfig effective_config = config;
    effective_config.scale = scale;

    // 创建自适应大小的图像
    cv::Mat image(new_height * scale, new_width * scale, CV_8UC3, config.free_space_color);

    // 如果需要重新绘制栅格地图（尺寸变化）
    if (need_resize)
    {
      // 只绘制有效的栅格区域
      int old_width = grid_map_generator_->get_width();
      int old_height = grid_map_generator_->get_height();

      for (int y = 0; y < old_height; ++y)
      {
        for (int x = 0; x < old_width; ++x)
        {
          if (grid_map_[y][x] == 1)
          {
            // 障碍物
            double world_x, world_y;
            grid_map_generator_->convertGridToWorld(x, y, world_x, world_y);

            // 转换到新的坐标系
            int new_x = static_cast<int>((world_x - min_x) / resolution);
            int new_y = static_cast<int>((world_y - min_y) / resolution);

            if (new_x >= 0 && new_x < new_width && new_y >= 0 && new_y < new_height)
            {
              // 绘制障碍物
              cv::rectangle(image, cv::Point(new_x * scale, new_y * scale),
                            cv::Point((new_x + 1) * scale - 1, (new_y + 1) * scale - 1), config.obstacle_color, -1);
            }
          }
        }
      }

      // 添加栅格线
      if (config.show_grid_lines && scale >= 3 && new_width * new_height < 500000)
      {
        // 计算栅格线颜色
        cv::Scalar grid_color(200, 200, 200);

        for (int y = 0; y <= new_height; ++y)
        {
          cv::line(image, cv::Point(0, y * scale), cv::Point(new_width * scale, y * scale), grid_color, 1);
        }

        for (int x = 0; x <= new_width; ++x)
        {
          cv::line(image, cv::Point(x * scale, 0), cv::Point(x * scale, new_height * scale), grid_color, 1);
        }
      }

      // 轴线绘制放到最后一层（覆盖在规划路径之上），见下方统一处理。
    }
    else
    {
      // 使用原有的绘制方法
      draw_grid_map(image, effective_config);
    }

    // 先绘制轴线（作为参考底图），再绘制规划路径（绘图路径会覆盖轴线）
    if (need_resize)
    {
      const auto& axis_points = grid_map_generator_->get_axis_points();
      const int axis_thickness = std::max(1, scale / 6);

      for (const auto& axis_point : axis_points)
      {
        double world_x1, world_y1, world_x2, world_y2;
        grid_map_generator_->convertGridToWorld(axis_point.x1, axis_point.y1, world_x1, world_y1);
        grid_map_generator_->convertGridToWorld(axis_point.x2, axis_point.y2, world_x2, world_y2);

        const int img_x1 = static_cast<int>((world_x1 - min_x) / resolution) * scale + scale / 2;
        const int img_y1 = static_cast<int>((world_y1 - min_y) / resolution) * scale + scale / 2;
        const int img_x2 = static_cast<int>((world_x2 - min_x) / resolution) * scale + scale / 2;
        const int img_y2 = static_cast<int>((world_y2 - min_y) / resolution) * scale + scale / 2;

        if ((img_x1 >= 0 && img_x1 < new_width * scale && img_y1 >= 0 && img_y1 < new_height * scale) ||
            (img_x2 >= 0 && img_x2 < new_width * scale && img_y2 >= 0 && img_y2 < new_height * scale))
        {
          cv::line(image, cv::Point(img_x1, img_y1), cv::Point(img_x2, img_y2), effective_config.axis_line_color,
                   axis_thickness, cv::LINE_AA);
        }
      }
    }
    else
    {
      draw_axis_lines(image, effective_config);
    }

    int path_label = 1;  // 路径编号,从1开始

    // 绘制所有路径段
    for (const auto& segment : path_segments)
    {
      // 根据路径类型选择颜色
      cv::Scalar path_color =
          (segment.type == RouteType::DRAWING_PATH) ? config.path_color : config.transition_path_color;

      // 绘制路径，使用自定义的函数，考虑坐标转换
      if (!segment.points.empty())
      {
        std::vector<cv::Point> image_points;
        image_points.reserve(segment.points.size());

        // 将世界坐标转换为图像坐标
        for (const auto& point : segment.points)
        {
          // 转换到新的坐标系
          int img_x = static_cast<int>((point.x - min_x) / resolution) * scale + scale / 2;
          int img_y = static_cast<int>((point.y - min_y) / resolution) * scale + scale / 2;

          if (img_x >= 0 && img_x < new_width * scale && img_y >= 0 && img_y < new_height * scale)
          {
            image_points.push_back(cv::Point(img_x, img_y));
          }
        }

        // 绘制路径线段
        if (image_points.size() >= 2)
        {
          for (size_t i = 0; i < image_points.size() - 1; ++i)
          {
            cv::line(image, image_points[i], image_points[i + 1], path_color, config.path_thickness, cv::LINE_AA);
          }

          // 在路径上的每个点绘制小圆点
          for (const auto& point : image_points)
          {
            cv::circle(image, point, config.path_thickness / 2, path_color, -1, cv::LINE_AA);
          }

          // 绘制路径编号
          draw_path_label(image, segment.points, path_label, effective_config, min_x, min_y, resolution);
          path_label++;
        }

        // 对于绘图路径段，标记起点和终点
        if (segment.type == RouteType::DRAWING_PATH && segment.points.size() >= 2)
        {
          const Point3D& start = segment.points.front();
          const Point3D& end = segment.points.back();

          // 转换到新的坐标系
          int start_x = static_cast<int>((start.x - min_x) / resolution) * scale + scale / 2;
          int start_y = static_cast<int>((start.y - min_y) / resolution) * scale + scale / 2;
          int end_x = static_cast<int>((end.x - min_x) / resolution) * scale + scale / 2;
          int end_y = static_cast<int>((end.y - min_y) / resolution) * scale + scale / 2;

          // 绘制起点
          if (start_x >= 0 && start_x < new_width * scale && start_y >= 0 && start_y < new_height * scale)
          {
            cv::circle(image, cv::Point(start_x, start_y), config.point_radius, config.start_point_color, -1,
                       cv::LINE_AA);
          }

          // 绘制终点
          if (end_x >= 0 && end_x < new_width * scale && end_y >= 0 && end_y < new_height * scale)
          {
            cv::circle(image, cv::Point(end_x, end_y), config.point_radius, config.goal_point_color, -1, cv::LINE_AA);
          }
        }
      }
    }

    // 绘制图例
    if (effective_config.draw_legend)
    {
      draw_legend(image, effective_config);
    }

    // 设置图像压缩参数
    std::vector<int> compression_params;

    if (config.image_format == "png")
    {
      compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
      compression_params.push_back(9);  // 0-9,越高质量越好但文件更大
    }
    else if (config.image_format == "jpg" || config.image_format == "jpeg")
    {
      compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
      compression_params.push_back(95);  // 0-100,越高质量越好
    }

    // 保存图像
    if (cv::imwrite(file_path, image, compression_params))
    {
      std::cout << "Successfully saved paths visualization as image: " << file_path << std::endl;
      return true;
    }
    else
    {
      std::cerr << "Failed to save paths visualization as image" << std::endl;
      return false;
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error visualizing paths: " << e.what() << std::endl;
    return false;
  }
}
}  // namespace path_planner
