// include/daosnrs_planning/trajectory_generator.hpp
#pragma once

#include <vector>
#include <memory>
#include <string>
#include <cmath>
#include <algorithm>

#include "xline_path_planner/common_types.hpp"

namespace path_planner
{

/**
 * @brief 轨迹生成器类
 * 用于根据路径点生成实际执行轨迹
 */
class TrajectoryGenerator
{
public:
  /**
   * @brief 构造函数
   * @param config 轨迹生成配置
   */
  explicit TrajectoryGenerator(const TrajectoryConfig& config = TrajectoryConfig());

  /**
   * @brief 析构函数
   */
  ~TrajectoryGenerator() = default;

  /**
   * @brief 从路径生成轨迹点
   * @param path 路径点序列
   * @param line 要绘制的线段
   * @param is_drawing_path 是否为绘图路径（否则为转场路径）
   * @return 绘图工作点序列
   */
  std::vector<ExecutionNode> generate_from_path(const std::vector<Point3D>& path, const Line& line,
                                                bool is_drawing_path = true);

  /**
   * @brief 生成曲线的绘图轨迹
   * @param curve 要绘制的曲线
   * @param offset_config 路径偏移配置
   * @return 绘图工作点序列
   */
  std::vector<ExecutionNode> generate_from_curve(const Curve& curve, const PathOffsetConfig& offset_config);

  /**
   * @brief 生成圆的绘图轨迹
   * @param circle 要绘制的圆
   * @param offset_config 路径偏移配置
   * @return 绘图工作点序列
   */
  std::vector<ExecutionNode> generate_from_circle(const Circle& circle, const PathOffsetConfig& offset_config);

  /**
   * @brief 生成圆弧的绘图轨迹
   * @param arc 要绘制的圆弧
   * @param offset_config 路径偏移配置
   * @return 绘图工作点序列
   */
  std::vector<ExecutionNode> generate_from_arc(const Arc& arc, const PathOffsetConfig& offset_config);

  /**
   * @brief 设置轨迹生成配置
   * @param config 轨迹生成配置
   */
  void set_config(const TrajectoryConfig& config);

private:
  /**
   * @brief 计算两点之间的角度
   * @param p1 第一个点
   * @param p2 第二个点
   * @return 角度（弧度）
   */
  double computeAngleBetweenPoints(const Point3D& p1, const Point3D& p2);

  /**
   * @brief 从角度计算四元数
   * @param angle 角度（弧度）
   * @return 四元数 [x, y, z, w]
   */
  std::vector<double> angle_to_quaternion(double angle);

  /**
   * @brief 规划直线段轨迹
   * @param start 起点
   * @param end 终点
   * @param drawing 是否在绘图
   * @param line_id 线段ID
   * @param is_left_printer 是否使用左侧打印机
   * @return 轨迹点序列
   */
  std::vector<ExecutionNode> generateSegmentTrajectory(const Point3D& start, const Point3D& end, bool drawing,
                                                       int32_t line_id, bool is_left_printer);

  /**
   * @brief 离散化曲线为多个点
   * @param curve 曲线
   * @param num_points 离散点数量
   * @return 离散后的点序列
   */
  std::vector<Point3D> convertCurveToPoints(const Curve& curve, int num_points);

  /**
   * @brief 计算NURBS曲线上的点
   * @param curve NURBS曲线
   * @param t 参数值 [0, 1]
   * @return 曲线上的点
   */
  Point3D computeNURBSPoint(const Curve& curve, double t);

  TrajectoryConfig trajectory_config_;  ///< 轨迹生成配置
};

}  // namespace path_planner
