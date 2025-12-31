#pragma once

#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "xline_path_planner/common_types.hpp"

// 添加OpenCV头文件
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace path_planner
{

/**
 * @brief 栅格地图可视化配置结构体
 * 用于配置栅格地图可视化的参数
 */
struct GridMapVisualizationConfig
{
  int scale = 4;                                            ///< 图像缩放因子
  bool show_grid_lines = true;                              ///< 是否显示栅格线
  bool use_antialiasing = true;                             ///< 是否使用抗锯齿
  std::string image_format = "png";                         ///< 图像格式
  double grid_line_opacity = 0.3;                           ///< 栅格线不透明度 (0-1)
  int max_axis_lines = 1000;                                ///< 最大显示的轴线数量
  cv::Scalar obstacle_color = cv::Scalar(0, 0, 0);          ///< 障碍物颜色 (黑色)
  cv::Scalar free_space_color = cv::Scalar(255, 255, 255);  ///< 自由空间颜色 (白色)
  cv::Scalar axis_line_color = cv::Scalar(0, 0, 255);       ///< 轴线颜色 (红色)
  cv::Scalar grid_line_color = cv::Scalar(200, 200, 200);   ///< 栅格线颜色 (浅灰色)
  bool draw_legend = true;                                  ///< 是否绘制图例
};

/**
 * @brief 栅格地图生成器类
 * 用于将CAD数据转换为栅格地图
 */
class GridMapGenerator
{
public:
  /**
   * @brief 用于存储轴线点的结构体
   * 记录轴线的栅格坐标和ID
   */
  struct AxisPoint
  {
    int x1, y1;       ///< 起点栅格坐标
    int x2, y2;       ///< 终点栅格坐标
    int32_t line_id;  ///< 对应的线段ID
  };

  /**
   * @brief 构造函数
   * @param config 栅格地图配置
   */
  explicit GridMapGenerator(const GridMapConfig& config = GridMapConfig());

  /**
   * @brief 析构函数
   */
  ~GridMapGenerator() = default;

  /**
   * @brief 从CAD数据生成栅格地图
   * @param cad_data CAD数据
   * @return 成功返回true，否则返回false
   */
  bool generate_from_cad(const CADData& cad_data);

  /**
   * @brief 获取栅格地图
   * @return 栅格地图（0表示空闲，1表示障碍物）的常量引用
   */
  const std::vector<std::vector<int>>& get_grid_map() const;

  /**
   * @brief 获取轴线点数据
   * @return 轴线点数据集合的常量引用
   */
  const std::vector<AxisPoint>& get_axis_points() const;

  /**
   * @brief 获取地图的边界信息
   * @param min_x 最小X坐标（输出参数）
   * @param min_y 最小Y坐标（输出参数）
   * @param max_x 最大X坐标（输出参数）
   * @param max_y 最大Y坐标（输出参数）
   */
  void get_map_bounds(double& min_x, double& min_y, double& max_x, double& max_y) const;

  /**
   * @brief 将世界坐标转换为栅格坐标
   * @param world_x 世界坐标系X
   * @param world_y 世界坐标系Y
   * @param grid_x 返回栅格坐标X（输出参数）
   * @param grid_y 返回栅格坐标Y（输出参数）
   * @return 如果坐标在地图范围内返回true，否则返回false
   */
  bool convertWorldToGrid(double world_x, double world_y, int& grid_x, int& grid_y) const;

  /**
   * @brief 将栅格坐标转换为世界坐标
   * @param grid_x 栅格坐标X
   * @param grid_y 栅格坐标Y
   * @param world_x 返回世界坐标系X（输出参数）
   * @param world_y 返回世界坐标系Y（输出参数）
   */
  void convertGridToWorld(int grid_x, int grid_y, double& world_x, double& world_y) const;

  /**
   * @brief 清空地图数据
   * 重置所有地图相关数据
   */
  void clear();

  /**
   * @brief 获取栅格分辨率
   * @return 栅格分辨率（米/栅格）
   */
  double get_resolution() const;

  /**
   * @brief 更新地图大小
   * @param width 宽度（栅格数）
   * @param height 高度（栅格数）
   */
  void resize(int width, int height);

  /**
   * @brief 检查坐标是否在地图范围内
   * @param x 栅格X坐标
   * @param y 栅格Y坐标
   * @return 如果在范围内返回true，否则返回false
   */
  bool isCoordinateValid(int x, int y) const;

  /**
   * @brief 获取栅格地图宽度
   * @return 地图宽度（栅格数）
   */
  int get_width() const;

  /**
   * @brief 获取栅格地图高度
   * @return 地图高度（栅格数）
   */
  int get_height() const;

  /**
   * @brief 将栅格地图保存为图片
   * @param file_path 输出图片路径
   * @param cad_data CAD数据，用于绘制轴线等信息
   * @param config 可视化配置
   * @return 成功返回true，否则返回false
   */
  bool save_as_image(const std::string& file_path, const CADData& cad_data,
                     const GridMapVisualizationConfig& config = GridMapVisualizationConfig()) const;

private:
  /**
   * @brief 将线段光栅化到栅格地图上
   * @param line 线段指针
   * @param value 需要标记的值（1表示障碍物，2表示轴线）
   */
  void convertToGridLine(const std::shared_ptr<Line>& line, int value = 1);

  /**
   * @brief 将曲线光栅化到栅格地图上
   * @param curve 曲线指针
   * @param value 需要标记的值
   */
  void convertToGridCurve(const std::shared_ptr<Curve>& curve, int value = 1);

  /**
   * @brief 将圆光栅化到栅格地图上
   * @param circle 圆指针
   * @param value 需要标记的值
   */
  void convertToGridCircle(const std::shared_ptr<Circle>& circle, int value = 1);

  /**
   * @brief 将圆弧光栅化到栅格地图上
   * @param arc 圆弧指针
   * @param value 需要标记的值
   */
  void convertToGridArc(const std::shared_ptr<Arc>& arc, int value = 1);

  /**
   * @brief 将椭圆/椭圆弧光栅化到栅格地图上
   * @param ellipse 椭圆指针
   * @param value 需要标记的值
   */
  void convertToGridEllipse(const std::shared_ptr<Ellipse>& ellipse, int value = 1);

  /**
   * @brief 使用Bresenham算法将线段光栅化
   * @param x1 起点X栅格坐标
   * @param y1 起点Y栅格坐标
   * @param x2 终点X栅格坐标
   * @param y2 终点Y栅格坐标
   * @param value 设置的栅格值
   */
  void drawGridLine(int x1, int y1, int x2, int y2, int value = 1);

  /**
   * @brief 安全地设置栅格点值
   * @param x 栅格X坐标
   * @param y 栅格Y坐标
   * @param value 设置的栅格值
   */
  void set_grid_point(int x, int y, int value);

  /**
   * @brief 存储轴线点信息，用于可视化
   * @param line 轴线
   */
  void store_axis_line_points(const std::shared_ptr<Line>& line);

  /**
   * @brief 存储圆轴线点，用于可视化
   * @param circle 圆形轴线
   */
  void store_circle_axis_points(const std::shared_ptr<Circle>& circle);

  /**
   * @brief 存储圆弧轴线点，用于可视化
   * @param arc 圆弧轴线
   */
  void store_arc_axis_points(const std::shared_ptr<Arc>& arc);

  /**
   * @brief 存储椭圆轴线点，用于可视化
   * @param ellipse 椭圆/椭圆弧轴线
   */
  void store_ellipse_axis_points(const std::shared_ptr<Ellipse>& ellipse);

  /**
   * @brief 计算地图边界
   * @param cad_data CAD数据
   */
  void calculate_map_bounds(const CADData& cad_data);

  /**
   * @brief 计算任意次贝塞尔曲线上的点 (De Casteljau 算法)
   * @param control_points 控制点
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

  GridMapConfig map_config_;                                      ///< 地图配置
  std::vector<std::vector<int>> grid_map_;                        ///< 栅格地图数据
  std::vector<AxisPoint> axis_points_;                            ///< 存储轴线的坐标
  double boundaryMinX, boundaryMinY, boundaryMaxX, boundaryMaxY;  ///< 地图世界坐标边界
};

}  // namespace path_planner
