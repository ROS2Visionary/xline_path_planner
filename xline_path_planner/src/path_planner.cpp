#include "xline_path_planner/path_planner.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>

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

std::vector<RouteSegment> PathPlanner::plan_paths(const CADData& cad_data, const PathOffsetConfig& offset_config)
{
  if (!grid_map_generator_)
  {
    std::cerr << "Grid map generator not set!" << std::endl;
    return {};
  }

  std::vector<RouteSegment> path_segments;

  // 打印CAD数据中路径线的数量
  std::cout << "Total path lines in CAD data: " << cad_data.path_lines.size() << std::endl;

  // 统计各类型的轴线数量和已绘制状态
  int line_count = 0, circle_count = 0, arc_count = 0, curve_count = 0, other_count = 0;
  int printed_count = 0, unprinted_count = 0;

  for (const auto& line : cad_data.path_lines)
  {
    if (line->type == GeometryType::LINE)
      line_count++;
    else if (line->type == GeometryType::CIRCLE)
      circle_count++;
    else if (line->type == GeometryType::ARC)
      arc_count++;
    else if (line->type == GeometryType::CURVE)
      curve_count++;
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

  std::cout << "Path line types: " << line_count << " lines, " << circle_count << " circles, " << arc_count << " arcs, "
            << curve_count << " curves, " << other_count << " other types" << std::endl;

  std::cout << "Printed/Unprinted status: " << printed_count << " printed, " << unprinted_count << " unprinted"
            << std::endl;

  // 收集所有轴线，无论其is_printed状态
  std::vector<std::shared_ptr<Line>> lines_to_draw = cad_data.path_lines;

  // 如果所有线段都被标记为已打印，但我们仍然需要处理它们
  if (!lines_to_draw.empty() && printed_count == cad_data.path_lines.size())
  {
    std::cout << "All lines are marked as printed. Forcing processing of all lines..." << std::endl;
    // 将所有线段标记为未打印，以便处理它们
    for (auto& line : lines_to_draw)
    {
      line->is_printed = false;
    }
  }

  std::cout << "Planning paths for " << lines_to_draw.size() << " lines" << std::endl;

  // 处理所有线段
  processGeometryGroup(lines_to_draw, path_segments, offset_config);

  return path_segments;
}

std::shared_ptr<Line> PathPlanner::findNearestUnprocessedLine(const Point3D& current_pos,
                                                             const std::vector<std::shared_ptr<Line>>& lines)
{
  double min_distance = std::numeric_limits<double>::max();
  std::shared_ptr<Line> nearest_line = nullptr;

  for (const auto& line : lines)
  {
    if (!line->is_printed)
    {
      double dist = current_pos.distance(line->start);
      if (dist < min_distance)
      {
        min_distance = dist;
        nearest_line = line;
      }
    }
  }

  return nearest_line;
}

RouteSegment PathPlanner::planGeometryPath(const std::shared_ptr<Line>& line, const PathOffsetConfig& offset_config)
{
  RouteSegment segment(RouteType::DRAWING_PATH, line->id);

  // 从 line_type 推导 ink_mode
  segment.ink_mode = deduceInkMode(line->line_type);

  // 新增路径延长长度的配置参数
  double path_extension_length = planner_config_.path_extension_length;

  // 延长路径的方法
  auto extend_line = [&](const Point3D& start, const Point3D& end,
                         double extension_length) -> std::pair<Point3D, Point3D> {
    // 计算原始线段长度和方向
    double original_length = start.distance(end);
    double unit_factor = extension_length / original_length;

    Point3D extended_start, extended_end;

    // 延长起点
    extended_start.x = start.x - (end.x - start.x) * unit_factor;
    extended_start.y = start.y - (end.y - start.y) * unit_factor;
    extended_start.z = start.z - (end.z - start.z) * unit_factor;

    // 延长终点
    extended_end.x = end.x + (end.x - start.x) * unit_factor;
    extended_end.y = end.y + (end.y - start.y) * unit_factor;
    extended_end.z = end.z + (end.z - start.z) * unit_factor;

    return { extended_start, extended_end };
  };

  // 根据线段类型选择不同的路径生成方法
  if (line->type == GeometryType::LINE)
  {
    // 仅对直线路径（非转场路径）进行延长
    auto [extended_start, extended_end] = extend_line(line->start, line->end, path_extension_length);

    segment.points = { extended_start, extended_end };
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

      // 延长圆的起点和终点（按切线方向）
      if (circle_points.size() >= 3)
      {
        // 计算起点的切线方向（使用起点和下一个点的方向近似）
        Point3D tangent_start = circle_points[1] - circle_points[0];
        double tangent_length = tangent_start.distance(Point3D(0, 0, 0));

        // 起点沿切线向后延长
        Point3D extended_start = circle_points.front();
        if (tangent_length > 1e-6)
        {
          extended_start.x -= tangent_start.x * path_extension_length / tangent_length;
          extended_start.y -= tangent_start.y * path_extension_length / tangent_length;
          extended_start.z -= tangent_start.z * path_extension_length / tangent_length;
        }

        // 计算终点的切线方向（使用终点和前一个点的方向近似）
        Point3D tangent_end = circle_points.back() - circle_points[circle_points.size() - 2];
        tangent_length = tangent_end.distance(Point3D(0, 0, 0));

        // 终点沿切线向前延长
        Point3D extended_end = circle_points.back();
        if (tangent_length > 1e-6)
        {
          extended_end.x += tangent_end.x * path_extension_length / tangent_length;
          extended_end.y += tangent_end.y * path_extension_length / tangent_length;
          extended_end.z += tangent_end.z * path_extension_length / tangent_length;
        }

        // 构建新的点序列
        segment.points.push_back(extended_start);
        for (size_t i = 1; i < circle_points.size() - 1; ++i)
        {
          segment.points.push_back(circle_points[i]);
        }
        segment.points.push_back(extended_end);
      }
      else
      {
        segment.points = circle_points;
      }
    }
  }
  else if (line->type == GeometryType::ARC)
  {
    // 圆弧，离散化为多个点
    auto arc = std::dynamic_pointer_cast<Arc>(line);
    if (arc)
    {
      // 计算角度范围并离散化
      double start_angle = arc->start_angle;
      double end_angle = arc->end_angle;

      // 确保终止角度大于起始角度
      if (end_angle < start_angle)
      {
        end_angle += 2.0 * M_PI;
      }

      // 计算角度差
      double angle_diff = end_angle - start_angle;

      // 离散化点数
      int num_points = std::max(10, static_cast<int>(36 * angle_diff / (2.0 * M_PI)));
      double angle_step = angle_diff / num_points;

      std::vector<Point3D> arc_points;
      for (int i = 0; i <= num_points; ++i)
      {
        double angle = start_angle + i * angle_step;
        Point3D point;
        point.x = arc->center.x + arc->radius * std::cos(angle);
        point.y = arc->center.y + arc->radius * std::sin(angle);
        point.z = arc->center.z;

        arc_points.push_back(point);
      }

      // 延长圆弧的起点和终点（按切线方向）
      if (arc_points.size() >= 3)
      {
        // 计算起点的切线方向（使用起点和下一个点的方向）
        Point3D tangent_start = arc_points[1] - arc_points[0];
        double tangent_length = tangent_start.distance(Point3D(0, 0, 0));

        // 起点沿切线向后延长
        Point3D extended_start = arc_points.front();
        if (tangent_length > 1e-6)
        {
          extended_start.x -= tangent_start.x * path_extension_length / tangent_length;
          extended_start.y -= tangent_start.y * path_extension_length / tangent_length;
          extended_start.z -= tangent_start.z * path_extension_length / tangent_length;
        }

        // 计算终点的切线方向（使用终点和前一个点的方向）
        Point3D tangent_end = arc_points.back() - arc_points[arc_points.size() - 2];
        tangent_length = tangent_end.distance(Point3D(0, 0, 0));

        // 终点沿切线向前延长
        Point3D extended_end = arc_points.back();
        if (tangent_length > 1e-6)
        {
          extended_end.x += tangent_end.x * path_extension_length / tangent_length;
          extended_end.y += tangent_end.y * path_extension_length / tangent_length;
          extended_end.z += tangent_end.z * path_extension_length / tangent_length;
        }

        // 构建新的点序列
        segment.points.push_back(extended_start);
        for (size_t i = 1; i < arc_points.size() - 1; ++i)
        {
          segment.points.push_back(arc_points[i]);
        }
        segment.points.push_back(extended_end);
      }
      else
      {
        segment.points = arc_points;
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

        // 延长起点
        Point3D extended_start = curve_points.front();
        if (tangent_start_length > 1e-6)
        {  // 避免除以零
          extended_start = extended_start - tangent_start * (path_extension_length / tangent_start_length);
        }

        // 延长终点
        Point3D extended_end = curve_points.back();
        if (tangent_end_length > 1e-6)
        {  // 避免除以零
          extended_end = extended_end + tangent_end * (path_extension_length / tangent_end_length);
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
  if (!segment.points.empty())
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

RouteSegment PathPlanner::planConnectionPath(const Point3D& start, const Point3D& goal)
{
  RouteSegment segment(RouteType::TRANSITION_PATH);

  std::cout << "Planning transition path from (" << start.x << ", " << start.y << ") to (" << goal.x << ", " << goal.y
            << ")" << std::endl;

  // 直接使用起点和终点生成直线路径
  segment.points = { start, goal };

  return segment;
}

void PathPlanner::processGeometryGroup(const std::vector<std::shared_ptr<Line>>& lines,
                                     std::vector<RouteSegment>& path_segments, const PathOffsetConfig& offset_config)
{
  Point3D current_position;
  bool has_current_position = false;

  std::cout << "Processing " << lines.size() << " lines in group" << std::endl;

  // 分类处理不同类型的线段
  std::vector<std::shared_ptr<Line>> straight_lines;
  std::vector<std::shared_ptr<Circle>> circles;
  std::vector<std::shared_ptr<Arc>> arcs;
  std::vector<std::shared_ptr<Curve>> curves;

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
    }
  }

  std::cout << "Classified lines: " << straight_lines.size() << " straight, " << circles.size() << " circles, "
            << arcs.size() << " arcs, " << curves.size() << " curves" << std::endl;

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

  while (!remaining_lines.empty())
  {
    // 找到最近的未绘制线段
    auto nearest_line = findNearestUnprocessedLine(current_position, remaining_lines);

    if (nearest_line)
    {
      std::cout << "Planning path for line ID: " << nearest_line->id
                << " (type: " << static_cast<int>(nearest_line->type) << ")" << std::endl;

      // 规划该线的绘图路径
      RouteSegment drawing_segment = planGeometryPath(nearest_line, offset_config);

      if (drawing_segment.points.empty())
      {
        std::cerr << "Failed to plan drawing path for line ID: " << nearest_line->id << std::endl;
      }
      else
      {
        // 如果已经有当前位置,则规划一条直线转场路径
        if (has_current_position)
        {
          RouteSegment transition_segment = planConnectionPath(current_position, drawing_segment.points.front());

          if (!transition_segment.points.empty())
          {
            path_segments.push_back(transition_segment);
            std::cout << "Added transition path with " << transition_segment.points.size() << " points" << std::endl;
          }
          else
          {
            std::cerr << "Failed to plan transition path to line ID: " << nearest_line->id << std::endl;
          }
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

      std::cout << "Remaining lines to process: " << remaining_lines.size() << std::endl;
    }
    else
    {
      std::cout << "No more undrawn lines found" << std::endl;
      break;
    }
  }
}

std::vector<Point3D> PathPlanner::applyPathOffset(const std::vector<Point3D>& original_path, double offset)
{
  if (original_path.size() < 2)
  {
    return original_path;
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
    if (u >= curve.knots[span + i - p] && u < curve.knots[span + i + 1])
    {
      N[i] = 1.0;
    }
  }

  // 计算高阶基函数
  for (int k = 1; k <= p; ++k)
  {
    for (int i = p; i >= k; --i)
    {
      double d1 = curve.knots[span + i + 1 - k] - curve.knots[span + i - p];
      double d2 = curve.knots[span + i + 1] - curve.knots[span + i - k + 1];

      double left = 0.0, right = 0.0;

      if (std::fabs(d1) > 1e-10)
      {
        left = (u - curve.knots[span + i - p]) / d1;
      }

      if (std::fabs(d2) > 1e-10)
      {
        right = (curve.knots[span + i + 1] - u) / d2;
      }

      N[i] = left * N[i - 1] + right * N[i];
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

      // 访问CAD数据以获取所有线段并绘制
      // 这里需要访问原始CAD数据，可以从外部传入或保存一个引用
      // 简化起见，这里仅使用轴线点
      const auto& axis_points = grid_map_generator_->get_axis_points();
      for (const auto& axis_point : axis_points)
      {
        double world_x1, world_y1, world_x2, world_y2;
        grid_map_generator_->convertGridToWorld(axis_point.x1, axis_point.y1, world_x1, world_y1);
        grid_map_generator_->convertGridToWorld(axis_point.x2, axis_point.y2, world_x2, world_y2);

        // 转换到新的坐标系
        int new_x1 = static_cast<int>((world_x1 - min_x) / resolution);
        int new_y1 = static_cast<int>((world_y1 - min_y) / resolution);
        int new_x2 = static_cast<int>((world_x2 - min_x) / resolution);
        int new_y2 = static_cast<int>((world_y2 - min_y) / resolution);

        if ((new_x1 >= 0 && new_x1 < new_width && new_y1 >= 0 && new_y1 < new_height) ||
            (new_x2 >= 0 && new_x2 < new_width && new_y2 >= 0 && new_y2 < new_height))
        {
          // 绘制轴线
          cv::line(image, cv::Point(new_x1 * scale + scale / 2, new_y1 * scale + scale / 2),
                   cv::Point(new_x2 * scale + scale / 2, new_y2 * scale + scale / 2), config.axis_line_color,
                   std::max(2, scale / 4), cv::LINE_AA);
        }
      }
    }
    else
    {
      // 使用原有的绘制方法
      draw_grid_map(image, config);
      draw_axis_lines(image, config);
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
          draw_path_label(image, segment.points, path_label, config, min_x, min_y, resolution);
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
    if (config.draw_legend)
    {
      draw_legend(image, config);
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
