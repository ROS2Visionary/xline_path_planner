// include/daosnrs_planning/path_planner.hpp
#pragma once

#include <vector>
#include <memory>
#include <string>
#include <algorithm>
#include <cmath>

#include "xline_path_planner/common_types.hpp"
#include "xline_path_planner/grid_map_generator.hpp"

// 添加OpenCV头文件
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace daosnrs_planning
{

/**
 * @brief 路径可视化配置结构体
 * 用于配置路径的可视化参数
 */
struct PathVisualizationConfig
{
  int scale = 4;                                               ///< 图像缩放因子
  bool show_grid_lines = true;                                 ///< 是否显示栅格线
  bool use_antialiasing = true;                                ///< 是否使用抗锯齿
  std::string image_format = "png";                            ///< 图像格式
  cv::Scalar obstacle_color = cv::Scalar(0, 0, 0);             ///< 障碍物颜色 (黑色)
  cv::Scalar free_space_color = cv::Scalar(255, 255, 255);     ///< 自由空间颜色 (白色)
  cv::Scalar axis_line_color = cv::Scalar(0, 0, 255);          ///< 轴线颜色 (红色)
  cv::Scalar path_color = cv::Scalar(255, 0, 0);               ///< 路径颜色 (蓝色)
  cv::Scalar start_point_color = cv::Scalar(0, 255, 0);        ///< 起点颜色 (绿色)
  cv::Scalar goal_point_color = cv::Scalar(255, 0, 255);       ///< 终点颜色 (紫色)
  cv::Scalar offset_path_color = cv::Scalar(255, 165, 0);      ///< 偏移路径颜色 (橙色)
  cv::Scalar transition_path_color = cv::Scalar(0, 255, 255);  ///< 转场路径颜色 (黄色)
  int path_thickness = 2;                                      ///< 路径线宽
  int point_radius = 4;                                        ///< 起点和终点的半径
  bool draw_legend = true;                                     ///< 是否绘制图例
  int label_font_size = 36;                                    ///< 路径编号字体大小
  cv::Scalar label_color = cv::Scalar(0, 0, 255);              ///< 路径编号颜色
  int label_thickness = 1;                                     ///< 路径编号线宽
};

/**
 * @brief 路径规划器类
 * 用于规划从CAD数据到实际执行路径的转换
 */
class PathPlanner
{
public:
  /**
   * @brief 构造函数
   * @param config 路径规划配置
   */
  explicit PathPlanner(const PathPlannerConfig& config = PathPlannerConfig());

  /**
   * @brief 析构函数
   */
  ~PathPlanner();

  /**
   * @brief 设置栅格地图
   * @param grid_map 栅格地图
   * @param grid_map_generator 栅格地图生成器指针
   */
  void set_grid_map(const std::vector<std::vector<int>>& grid_map, const GridMapGenerator* grid_map_generator);

  /**
   * @brief 根据CAD数据规划所有路径
   * @param cad_data CAD数据
   * @param offset_config 路径偏移配置
   * @return 返回路径段序列，包括绘图路径和转场路径
   */
  std::vector<RouteSegment> plan_paths(const CADData& cad_data, const PathOffsetConfig& offset_config);

  /**
   * @brief 处理一组线段，规划路径
   * @param lines 线段组
   * @param path_segments 输出的路径段（输出参数）
   * @param offset_config 偏移配置
   */
  void processGeometryGroup(const std::vector<std::shared_ptr<Line>>& lines, std::vector<RouteSegment>& path_segments,
                            const PathOffsetConfig& offset_config);

  /**
   * @brief 应用偏移量到路径
   * @param original_path 原始路径
   * @param offset 偏移量（正值向右偏移，负值向左偏移）
   * @return 偏移后的路径
   */
  std::vector<Point3D> applyPathOffset(const std::vector<Point3D>& original_path, double offset);

  /**
   * @brief 可视化路径并保存为图片
   * @param path_segments 路径段序列
   * @param file_path 输出文件路径
   * @param config 可视化配置
   * @return 是否成功
   */
  bool visualize_paths(const std::vector<RouteSegment>& path_segments, const std::string& file_path,
                       const PathVisualizationConfig& config = PathVisualizationConfig());

  /**
   * @brief 获取规划配置
   * @return 当前配置的常量引用
   */
  const PathPlannerConfig& get_config() const;

  /**
   * @brief 设置规划配置
   * @param config 新配置
   */
  void set_config(const PathPlannerConfig& config);

private:
  /**
   * @brief 规划单条线段的路径
   * @param line 线段
   * @param offset_config 偏移配置
   * @return 绘图路径段
   */
  RouteSegment planGeometryPath(const std::shared_ptr<Line>& line, const PathOffsetConfig& offset_config);

  /**
   * @brief 规划转场路径
   * @param start 起点
   * @param goal 终点
   * @return 转场路径段
   */
  RouteSegment planConnectionPath(const Point3D& start, const Point3D& goal);

  /**
   * @brief 离散化曲线为多个点
   * @param curve 曲线
   * @param num_points 离散点数量
   * @return 曲线上的点序列
   */
  std::vector<Point3D> discretize_curve(const std::shared_ptr<Curve>& curve, int num_points);

  /**
   * @brief 生成直线路径
   * @param start 起点
   * @param end 终点
   * @param resolution 路径点间距
   * @return 路径点序列
   */
  std::vector<Point3D> generate_straight_path(const Point3D& start, const Point3D& end, double resolution = 0.05);

  /**
   * @brief 创建路径图像
   * @param config 可视化配置
   * @return OpenCV图像矩阵
   */
  cv::Mat create_path_image(const PathVisualizationConfig& config);

  /**
   * @brief 绘制栅格地图到图像
   * @param image 图像（输入/输出参数）
   * @param config 可视化配置
   */
  void draw_grid_map(cv::Mat& image, const PathVisualizationConfig& config);

  /**
   * @brief 绘制轴线到图像
   * @param image 图像（输入/输出参数）
   * @param config 可视化配置
   */
  void draw_axis_lines(cv::Mat& image, const PathVisualizationConfig& config);

  /**
   * @brief 绘制圆到图像
   * @param image 图像（输入/输出参数）
   * @param circle 圆形对象
   * @param color 颜色
   * @param thickness 线宽
   * @param config 可视化配置
   */
  void draw_circle(cv::Mat& image, const std::shared_ptr<Circle>& circle, const cv::Scalar& color, int thickness,
                   const PathVisualizationConfig& config);

  /**
   * @brief 绘制圆弧到图像
   * @param image 图像（输入/输出参数）
   * @param arc 圆弧对象
   * @param color 颜色
   * @param thickness 线宽
   * @param config 可视化配置
   */
  void draw_arc(cv::Mat& image, const std::shared_ptr<Arc>& arc, const cv::Scalar& color, int thickness,
                const PathVisualizationConfig& config);

  /**
   * @brief 绘制路径到图像
   * @param image 图像（输入/输出参数）
   * @param path 路径
   * @param color 颜色
   * @param thickness 线宽
   * @param config 可视化配置
   */
  void draw_path(cv::Mat& image, const std::vector<Point3D>& path, const cv::Scalar& color, int thickness,
                 const PathVisualizationConfig& config);

  /**
   * @brief 绘制图例
   * @param image 图像（输入/输出参数）
   * @param config 可视化配置
   */
  void draw_legend(cv::Mat& image, const PathVisualizationConfig& config);

  /**
   * @brief 计算贝塞尔曲线上的点
   * @param control_points 控制点数组
   * @param t 参数 [0,1]
   * @return 曲线上的点
   */
  Point3D evaluate_bezier_point(const std::vector<Point3D>& control_points, double t);

  /**
   * @brief 计算NURBS曲线上的点
   * @param curve NURBS曲线
   * @param t 参数 [0,1]
   * @return 曲线上的点
   */
  Point3D evaluate_nurbs_point(const Curve& curve, double t);

  /**
   * @brief 找到与当前位置最近的未绘制线段
   * @param current_pos 当前位置
   * @param lines 线段数组
   * @return 最近的未绘制线段，如果没有则返回nullptr
   */
  std::shared_ptr<Line> findNearestUnprocessedLine(const Point3D& current_pos,
                                                   const std::vector<std::shared_ptr<Line>>& lines);

  /**
   * @brief 在路径中间绘制编号
   * @param image 图像（输入/输出参数）
   * @param path 路径
   * @param label 编号
   * @param config 可视化配置
   * @param min_x 地图X坐标最小值
   * @param min_y 地图Y坐标最小值
   * @param resolution 地图分辨率
   */
  void draw_path_label(cv::Mat& image, const std::vector<Point3D>& path, int label,
                       const PathVisualizationConfig& config, double min_x, double min_y, double resolution);

  PathPlannerConfig planner_config_;            ///< 路径规划配置
  std::vector<std::vector<int>> grid_map_;      ///< 栅格地图
  const GridMapGenerator* grid_map_generator_;  ///< 栅格地图生成器指针
};

}  // namespace daosnrs_planning