#include "xline_path_planner/trajectory_generator.hpp"
#include <iostream>

namespace path_planner
{

std::vector<ExecutionNode> TrajectoryGenerator::generate_from_path(
    const std::vector<Point3D>& path, const Line& line, bool is_drawing_path)
{
  if (path.size() < 2) {
    std::cerr << "Path must contain at least two points!" << std::endl;
    return {};
  }
  
  std::vector<ExecutionNode> trajectory;
  trajectory.reserve(path.size() * 2);  // 每个路径点可能生成接近和绘图两个轨迹点
  
  // 生成接近起点的轨迹点（不绘图）
  const Point3D& start = path.front();
  const Point3D& second = path[1];
  double approach_angle = computeAngleBetweenPoints(start, second);
  
  // 添加接近点
  ExecutionNode approach_point;
  approach_point.position = start;
  approach_point.angle = approach_angle * 180.0 / M_PI;  // 转换为角度
  approach_point.move_state = 1;  // 移动状态
  approach_point.work_state = 0;  // 不工作
  approach_point.order = 0;
  
  trajectory.push_back(approach_point);
  
  // 如果是绘图路径，添加开始绘图的点
  if (is_drawing_path) {
    ExecutionNode start_drawing = approach_point;
    start_drawing.work_state = 1;  // 开始工作
    start_drawing.left_work_type = LineStyle::SOLID;  // 默认使用实线
    start_drawing.order = 1;
    
    // 添加工作数据
    LineDrawingData work_data;
    work_data.start = line.start;
    work_data.end = line.end;
    work_data.line_id = line.id;
    work_data.is_left_printer = true;  // 默认使用左侧打印机
    
    start_drawing.work_data.push_back(work_data);
    
    trajectory.push_back(start_drawing);
  }
  
  // 生成绘图过程中的轨迹点
  for (size_t i = 1; i < path.size(); ++i) {
    const Point3D& current = path[i];
    
    // 计算当前点的角度
    double current_angle;
    if (i < path.size() - 1) {
      current_angle = computeAngleBetweenPoints(current, path[i + 1]);
    } else if (i > 0) {
      current_angle = computeAngleBetweenPoints(path[i - 1], current);
    } else {
      current_angle = 0.0;
    }
    
    ExecutionNode point;
    point.position = current;
    point.angle = current_angle * 180.0 / M_PI;  // 转换为角度
    point.move_state = 1;  // 移动状态
    
    // 根据路径类型设置工作状态
    if (is_drawing_path) {
      point.work_state = 1;  // 绘图路径工作
      point.left_work_type = LineStyle::SOLID;  // 默认使用实线
      point.order = i + 1;  // 绘图路径从1开始
    } else {
      point.work_state = 0;  // 转场路径不工作
      point.order = i;       // 转场路径从0开始
    }
    
    // 只在最后一个点添加工作数据，并且只针对绘图路径
    if (is_drawing_path && i == path.size() - 1) {
      LineDrawingData work_data;
      work_data.start = line.start;
      work_data.end = line.end;
      work_data.line_id = line.id;
      work_data.is_left_printer = true;  // 默认使用左侧打印机
      
      point.work_data.push_back(work_data);
    }
    
    trajectory.push_back(point);
  }
  
  // 生成离开终点的轨迹点（不绘图）
  const Point3D& end = path.back();
  const Point3D& second_last = path[path.size() - 2];
  double leave_angle = computeAngleBetweenPoints(second_last, end);
  
  ExecutionNode leave_point;
  leave_point.position = end;
  leave_point.angle = leave_angle * 180.0 / M_PI;  // 转换为角度
  leave_point.move_state = 1;  // 移动状态
  leave_point.work_state = 0;  // 不工作
  leave_point.order = is_drawing_path ? path.size() + 1 : path.size();
  
  trajectory.push_back(leave_point);
  
  return trajectory;
}

std::vector<ExecutionNode> TrajectoryGenerator::generate_from_curve(
  const Curve& curve, const PathOffsetConfig& offset_config)
{
  // 将曲线离散化为一系列点
  std::vector<Point3D> curve_points = convertCurveToPoints(curve, 100);  // 使用100个点离散曲线

  // 应用偏移
  bool is_left_printer;

  if (offset_config.printer_type == PrinterType::LEFT_PRINTER) {
    is_left_printer = true;
  } else if (offset_config.printer_type == PrinterType::RIGHT_PRINTER) {
    is_left_printer = false;
  } else {
    // 默认使用左侧打印机
    is_left_printer = true;
  }

  // 创建一个虚拟线段用于轨迹生成
  Line virtual_line(curve.id, curve.start, curve.end, GeometryType::CURVE);

  // 生成轨迹点
  std::vector<ExecutionNode> trajectory;
  trajectory.reserve(curve_points.size() + 2);  // 曲线点 + 接近点 + 离开点

  // 添加接近点
  if (curve_points.size() >= 2) {
    ExecutionNode approach_point;
    approach_point.position = curve_points.front();
    approach_point.angle = computeAngleBetweenPoints(curve_points[0], curve_points[1]) * 180.0 / M_PI;
    approach_point.move_state = 1;  // 移动状态
    approach_point.work_state = 0;  // 不工作
    approach_point.order = 0;
    
    trajectory.push_back(approach_point);
  }

  // 添加曲线点
  for (size_t i = 0; i < curve_points.size(); ++i) {
    ExecutionNode curve_point;
    curve_point.position = curve_points[i];
    
    // 计算角度
    double angle;
    if (i < curve_points.size() - 1) {
      angle = computeAngleBetweenPoints(curve_points[i], curve_points[i + 1]);
    } else if (i > 0) {
      angle = computeAngleBetweenPoints(curve_points[i - 1], curve_points[i]);
    } else {
      angle = 0.0;
    }
    
    curve_point.angle = angle * 180.0 / M_PI;
    curve_point.move_state = 1;  // 移动状态
    curve_point.work_state = 1;  // 工作
    
    // 设置绘图类型
    if (is_left_printer) {
      curve_point.left_work_type = LineStyle::SOLID;
    } else {
      curve_point.right_work_type = LineStyle::SOLID;
    }
    
    curve_point.order = i + 1;
    
    // 添加工作数据（只在第一个点添加）
    if (i == 0) {
      LineDrawingData work_data;
      work_data.start = curve.start;
      work_data.end = curve.end;
      work_data.line_id = curve.id;
      work_data.is_left_printer = is_left_printer;
      
      curve_point.work_data.push_back(work_data);
    }
    
    trajectory.push_back(curve_point);
  }

  // 添加离开点
  if (curve_points.size() >= 2) {
    const Point3D& end = curve_points.back();
    const Point3D& second_last = curve_points[curve_points.size() - 2];
    
    ExecutionNode leave_point;
    leave_point.position = end;
    leave_point.angle = computeAngleBetweenPoints(second_last, end) * 180.0 / M_PI;
    leave_point.move_state = 1;  // 移动状态
    leave_point.work_state = 0;  // 不工作
    leave_point.order = curve_points.size() + 1;
    
    trajectory.push_back(leave_point);
  }

  return trajectory;
}

std::vector<ExecutionNode> TrajectoryGenerator::generate_from_circle(
    const Circle& circle, const PathOffsetConfig& offset_config)
{
  // 离散化圆为一系列点
  std::vector<Point3D> circle_points;
  
  // 以固定角度间隔离散化圆
  const int num_points = 36;  // 每10度一个点
  const double angle_step = 2.0 * M_PI / num_points;
  
  for (int i = 0; i <= num_points; ++i) {
    double angle = i * angle_step;
    Point3D point;
    point.x = circle.center.x + circle.radius * std::cos(angle);
    point.y = circle.center.y + circle.radius * std::sin(angle);
    point.z = circle.center.z;
    
    circle_points.push_back(point);
  }
  
  // 应用偏移
  bool is_left_printer;
  if (offset_config.printer_type == PrinterType::LEFT_PRINTER) {
    is_left_printer = true;
  } else if (offset_config.printer_type == PrinterType::RIGHT_PRINTER) {
    is_left_printer = false;
  } else {
    // 默认使用左侧打印机
    is_left_printer = true;
  }
  
  // 创建一个虚拟线段用于轨迹生成
  Line virtual_line(circle.id, circle_points.front(), circle_points.back(), GeometryType::CIRCLE);
  
  // 生成轨迹点
  std::vector<ExecutionNode> trajectory;
  trajectory.reserve(circle_points.size() + 2);  // 圆点 + 接近点 + 离开点
  
  // 添加接近点
  if (circle_points.size() >= 2) {
    ExecutionNode approach_point;
    approach_point.position = circle_points.front();
    approach_point.angle = computeAngleBetweenPoints(circle_points[0], circle_points[1]) * 180.0 / M_PI;
    approach_point.move_state = 1;  // 移动状态
    approach_point.work_state = 0;  // 不工作
    approach_point.order = 0;
    
    trajectory.push_back(approach_point);
  }
  
  // 添加圆点
  for (size_t i = 0; i < circle_points.size(); ++i) {
    ExecutionNode circle_point;
    circle_point.position = circle_points[i];
    
    // 计算角度
    double angle;
    if (i < circle_points.size() - 1) {
      angle = computeAngleBetweenPoints(circle_points[i], circle_points[i + 1]);
    } else if (i > 0) {
      angle = computeAngleBetweenPoints(circle_points[i - 1], circle_points[i]);
    } else {
      angle = 0.0;
    }
    
    circle_point.angle = angle * 180.0 / M_PI;
    circle_point.move_state = 1;  // 移动状态
    circle_point.work_state = 1;  // 工作
    
    // 设置绘图类型
    if (is_left_printer) {
      circle_point.left_work_type = LineStyle::SOLID;
    } else {
      circle_point.right_work_type = LineStyle::SOLID;
    }
    
    circle_point.order = i + 1;
    
    // 添加工作数据（只在第一个点添加）
    if (i == 0) {
      LineDrawingData work_data;
      work_data.start = circle.start;
      work_data.end = circle.end;
      work_data.line_id = circle.id;
      work_data.is_left_printer = is_left_printer;
      
      circle_point.work_data.push_back(work_data);
    }
    
    trajectory.push_back(circle_point);
  }
  
  // 添加离开点
  if (circle_points.size() >= 2) {
    const Point3D& end = circle_points.back();
    const Point3D& second_last = circle_points[circle_points.size() - 2];
    
    ExecutionNode leave_point;
    leave_point.position = end;
    leave_point.angle = computeAngleBetweenPoints(second_last, end) * 180.0 / M_PI;
    leave_point.move_state = 1;  // 移动状态
    leave_point.work_state = 0;  // 不工作
    leave_point.order = circle_points.size() + 1;
    
    trajectory.push_back(leave_point);
  }
  
  return trajectory;
}

std::vector<ExecutionNode> TrajectoryGenerator::generate_from_arc(
    const Arc& arc, const PathOffsetConfig& offset_config)
{
  // 离散化圆弧为一系列点
  std::vector<Point3D> arc_points;
  
  // 计算角度范围并离散化
  double start_angle = arc.start_angle;
  double end_angle = arc.end_angle;
  
  // 确保终止角度大于起始角度
  if (end_angle < start_angle) {
    end_angle += 2.0 * M_PI;
  }
  
  // 计算角度差
  double angle_diff = end_angle - start_angle;
  
  // 离散化点数
  int num_points = std::max(10, static_cast<int>(36 * angle_diff / (2.0 * M_PI)));
  double angle_step = angle_diff / num_points;
  
  for (int i = 0; i <= num_points; ++i) {
    double angle = start_angle + i * angle_step;
    Point3D point;
    point.x = arc.center.x + arc.radius * std::cos(angle);
    point.y = arc.center.y + arc.radius * std::sin(angle);
    point.z = arc.center.z;
    
    arc_points.push_back(point);
  }
  
  // 应用偏移
  bool is_left_printer;
  if (offset_config.printer_type == PrinterType::LEFT_PRINTER) {
    is_left_printer = true;
  } else if (offset_config.printer_type == PrinterType::RIGHT_PRINTER) {
    is_left_printer = false;
  } else {
    // 默认使用左侧打印机
    is_left_printer = true;
  }
  
  // 创建一个虚拟线段用于轨迹生成
  Line virtual_line(arc.id, arc_points.front(), arc_points.back(), GeometryType::ARC);
  
  // 生成轨迹点
  std::vector<ExecutionNode> trajectory;
  trajectory.reserve(arc_points.size() + 2);  // 圆弧点 + 接近点 + 离开点
  
  // 添加接近点
  if (arc_points.size() >= 2) {
    ExecutionNode approach_point;
    approach_point.position = arc_points.front();
    approach_point.angle = computeAngleBetweenPoints(arc_points[0], arc_points[1]) * 180.0 / M_PI;
    approach_point.move_state = 1;  // 移动状态
    approach_point.work_state = 0;  // 不工作
    approach_point.order = 0;
    
    trajectory.push_back(approach_point);
  }
  
  // 添加圆弧点
  for (size_t i = 0; i < arc_points.size(); ++i) {
    ExecutionNode arc_point;
    arc_point.position = arc_points[i];
    
    // 计算角度
    double angle;
    if (i < arc_points.size() - 1) {
      angle = computeAngleBetweenPoints(arc_points[i], arc_points[i + 1]);
    } else if (i > 0) {
      angle = computeAngleBetweenPoints(arc_points[i - 1], arc_points[i]);
    } else {
      angle = 0.0;
    }
    
    arc_point.angle = angle * 180.0 / M_PI;
    arc_point.move_state = 1;  // 移动状态
    arc_point.work_state = 1;  // 工作
    
    // 设置绘图类型
    if (is_left_printer) {
      arc_point.left_work_type = LineStyle::SOLID;
    } else {
      arc_point.right_work_type = LineStyle::SOLID;
    }
    
    arc_point.order = i + 1;
    
    // 添加工作数据（只在第一个点添加）
    if (i == 0) {
      LineDrawingData work_data;
      work_data.start = arc.start;
      work_data.end = arc.end;
      work_data.line_id = arc.id;
      work_data.is_left_printer = is_left_printer;
      
      arc_point.work_data.push_back(work_data);
    }
    
    trajectory.push_back(arc_point);
  }
  
  // 添加离开点
  if (arc_points.size() >= 2) {
    const Point3D& end = arc_points.back();
    const Point3D& second_last = arc_points[arc_points.size() - 2];
    
    ExecutionNode leave_point;
    leave_point.position = end;
    leave_point.angle = computeAngleBetweenPoints(second_last, end) * 180.0 / M_PI;
    leave_point.move_state = 1;  // 移动状态
    leave_point.work_state = 0;  // 不工作
    leave_point.order = arc_points.size() + 1;
    
    trajectory.push_back(leave_point);
  }
  
  return trajectory;
}

double TrajectoryGenerator::computeAngleBetweenPoints(const Point3D& p1, const Point3D& p2)
{
  // 计算从p1指向p2的角度（弧度）
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;
  
  return std::atan2(dy, dx);
}

std::vector<double> TrajectoryGenerator::angle_to_quaternion(double angle)
{
  // 将2D平面上的角度转换为四元数
  double half_angle = angle / 2.0;
  
  std::vector<double> quaternion(4);
  quaternion[0] = 0.0;  // x
  quaternion[1] = 0.0;  // y
  quaternion[2] = std::sin(half_angle);  // z
  quaternion[3] = std::cos(half_angle);  // w
  
  return quaternion;
}

std::vector<ExecutionNode> TrajectoryGenerator::generateSegmentTrajectory(
    const Point3D& start, const Point3D& end, bool drawing, int32_t line_id, bool is_left_printer)
{
  // 计算线段方向
  double angle = computeAngleBetweenPoints(start, end);
  
  // 创建轨迹点
  std::vector<ExecutionNode> trajectory;
  trajectory.reserve(2);  // 起点和终点
  
  // 起点
  ExecutionNode start_point;
  start_point.position = start;
  start_point.angle = angle * 180.0 / M_PI;
  start_point.move_state = 1;  // 移动状态
  start_point.work_state = drawing ? 1 : 0;  // 工作状态
  start_point.order = 0;
  
  // 设置绘图类型
  if (drawing) {
    if (is_left_printer) {
      start_point.left_work_type = LineStyle::SOLID;
    } else {
      start_point.right_work_type = LineStyle::SOLID;
    }
    
    // 添加工作数据
    LineDrawingData work_data;
    work_data.start = start;
    work_data.end = end;
    work_data.line_id = line_id;
    work_data.is_left_printer = is_left_printer;
    
    start_point.work_data.push_back(work_data);
  }
  
  trajectory.push_back(start_point);
  
  // 终点
  ExecutionNode end_point;
  end_point.position = end;
  end_point.angle = angle * 180.0 / M_PI;
  end_point.move_state = 1;  // 移动状态
  end_point.work_state = drawing ? 1 : 0;  // 工作状态
  end_point.order = 1;
  
  // 设置绘图类型
  if (drawing) {
    if (is_left_printer) {
      end_point.left_work_type = LineStyle::SOLID;
    } else {
      end_point.right_work_type = LineStyle::SOLID;
    }
  }
  
  trajectory.push_back(end_point);
  
  return trajectory;
}

std::vector<Point3D> TrajectoryGenerator::convertCurveToPoints(const Curve& curve, int num_points)
{
  std::vector<Point3D> points;
  points.reserve(num_points);
  
  if (curve.control_points.size() < 2) {
    return points;
  }
  
  if (curve.degree == 1) {
    // 线性曲线
    const Point3D& start = curve.control_points.front();
    const Point3D& end = curve.control_points.back();
    
    for (int i = 0; i < num_points; ++i) {
      double t = static_cast<double>(i) / (num_points - 1);
      Point3D point;
      point.x = start.x + t * (end.x - start.x);
      point.y = start.y + t * (end.y - start.y);
      point.z = start.z + t * (end.z - start.z);
      
      points.push_back(point);
    }
  } else {
    // 对于高次曲线，使用NURBS求值
    for (int i = 0; i < num_points; ++i) {
      double t = static_cast<double>(i) / (num_points - 1);
      points.push_back(computeNURBSPoint(curve, t));
    }
  }
  
  return points;
}

Point3D TrajectoryGenerator::computeNURBSPoint(const Curve& curve, double t)
{
  // 这里实现一个简化版的NURBS求值
  // 对于复杂的NURBS曲线，可能需要使用专门的库
  
  // 对于简单的曲线，我们可以使用De Casteljau算法
  if (curve.control_points.size() < 2) {
    return Point3D();
  }
  
  if (curve.degree == 1) {
    // 线性曲线
    const Point3D& start = curve.control_points.front();
    const Point3D& end = curve.control_points.back();
    
    Point3D point;
    point.x = start.x + t * (end.x - start.x);
    point.y = start.y + t * (end.y - start.y);
    point.z = start.z + t * (end.z - start.z);
    
    return point;
  } else if (curve.degree == 2) {
    // 二次曲线
    if (curve.control_points.size() < 3) {
      return Point3D();
    }
    
    // 二次贝塞尔曲线
    const Point3D& p0 = curve.control_points[0];
    const Point3D& p1 = curve.control_points[1];
    const Point3D& p2 = curve.control_points[2];
    
    double mt = 1.0 - t;
    
    Point3D point;
    point.x = mt * mt * p0.x + 2 * mt * t * p1.x + t * t * p2.x;
    point.y = mt * mt * p0.y + 2 * mt * t * p1.y + t * t * p2.y;
    point.z = mt * mt * p0.z + 2 * mt * t * p1.z + t * t * p2.z;
    
    return point;
  } else if (curve.degree == 3) {
    // 三次曲线
    if (curve.control_points.size() < 4) {
      return Point3D();
    }
    
    // 三次贝塞尔曲线
    const Point3D& p0 = curve.control_points[0];
    const Point3D& p1 = curve.control_points[1];
    const Point3D& p2 = curve.control_points[2];
    const Point3D& p3 = curve.control_points[3];
    
    double mt = 1.0 - t;
    double mt2 = mt * mt;
    double mt3 = mt2 * mt;
    double t2 = t * t;
    double t3 = t2 * t;
    
    Point3D point;
    point.x = mt3 * p0.x + 3 * mt2 * t * p1.x + 3 * mt * t2 * p2.x + t3 * p3.x;
    point.y = mt3 * p0.y + 3 * mt2 * t * p1.y + 3 * mt * t2 * p2.y + t3 * p3.y;
    point.z = mt3 * p0.z + 3 * mt2 * t * p1.z + 3 * mt * t2 * p2.z + t3 * p3.z;
    
    return point;
  } else {
    // 对于更高次的曲线，这里使用简化处理
    // 实际应用中应该使用完整的NURBS计算
    const Point3D& start = curve.control_points.front();
    const Point3D& end = curve.control_points.back();
    
    Point3D point;
    point.x = start.x + t * (end.x - start.x);
    point.y = start.y + t * (end.y - start.y);
    point.z = start.z + t * (end.z - start.z);
    
    return point;
  }
}

} // namespace path_planner
