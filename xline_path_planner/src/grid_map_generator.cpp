#include "daosnrs_planning/grid_map_generator.hpp"
#include <iostream>

namespace daosnrs_planning
{

GridMapGenerator::GridMapGenerator(const GridMapConfig& config)
  : map_config_(config), boundaryMinX(0), boundaryMinY(0), boundaryMaxX(0), boundaryMaxY(0)
{
}

bool GridMapGenerator::generate_from_cad(const CADData& cad_data)
{
  // 计算地图边界
  calculate_map_bounds(cad_data);

  // 计算地图尺寸（栅格数）
  int width = static_cast<int>((boundaryMaxX - boundaryMinX) / map_config_.resolution) + 1;
  int height = static_cast<int>((boundaryMaxY - boundaryMinY) / map_config_.resolution) + 1;

  // 调整地图大小
  resize(width, height);

  // 首先光栅化轴线，使用2作为标记值
  for (const auto& line : cad_data.axis_lines)
  {
    // 将轴线标记为值 2
    convertToGridLine(line, 2);
  }

  // 然后光栅化墙线和其他非障碍物线条，使用1作为标记值
  for (const auto& line : cad_data.wall_lines)
  {
    convertToGridLine(line, 1);
  }

  for (const auto& line : cad_data.control_lines)
  {
    convertToGridLine(line, 1);
  }

  for (const auto& line : cad_data.cross_lines)
  {
    convertToGridLine(line, 1);
  }

  // 将障碍物线添加到地图中，使用1作为标记值
  for (const auto& line : cad_data.barrier_lines)
  {
    convertToGridLine(line, 1);
  }

  for (const auto& line : cad_data.low_barrier_lines)
  {
    convertToGridLine(line, 1);
  }

  // 处理完所有线条后，将轴线（值为2）重置为可通行区域（值为0）
  // 这样轴线位置在路径规划时是可通行的，但在可视化时仍然可以特殊标记
  for (int y = 0; y < map_config_.height; ++y)
  {
    for (int x = 0; x < map_config_.width; ++x)
    {
      if (grid_map_[y][x] == 2)
      {
        grid_map_[y][x] = 0;  // 轴线是可通行的
      }
    }
  }

  // 保存轴线的位置供可视化使用
  axis_points_.clear();
  for (const auto& line : cad_data.axis_lines)
  {
    store_axis_line_points(line);
  }

  return true;
}

const std::vector<std::vector<int>>& GridMapGenerator::get_grid_map() const
{
  return grid_map_;
}

void GridMapGenerator::get_map_bounds(double& min_x, double& min_y, double& max_x, double& max_y) const
{
  min_x = boundaryMinX;
  min_y = boundaryMinY;
  max_x = boundaryMaxX;
  max_y = boundaryMaxY;
}

bool GridMapGenerator::convertWorldToGrid(double world_x, double world_y, int& grid_x, int& grid_y) const
{
  grid_x = static_cast<int>((world_x - boundaryMinX) / map_config_.resolution);
  grid_y = static_cast<int>((world_y - boundaryMinY) / map_config_.resolution);

  return isCoordinateValid(grid_x, grid_y);
}

void GridMapGenerator::convertGridToWorld(int grid_x, int grid_y, double& world_x, double& world_y) const
{
  world_x = boundaryMinX + grid_x * map_config_.resolution;
  world_y = boundaryMinY + grid_y * map_config_.resolution;
}

void GridMapGenerator::clear()
{
  grid_map_.clear();
  axis_points_.clear();
  boundaryMinX = 0;
  boundaryMinY = 0;
  boundaryMaxX = 0;
  boundaryMaxY = 0;
}

double GridMapGenerator::get_resolution() const
{
  return map_config_.resolution;
}

void GridMapGenerator::resize(int width, int height)
{
  map_config_.width = width;
  map_config_.height = height;

  // 调整栅格地图大小并初始化为0（可通行）
  grid_map_.resize(height);
  for (auto& row : grid_map_)
  {
    row.resize(width, 0);
  }
}

bool GridMapGenerator::isCoordinateValid(int x, int y) const
{
  return (x >= 0 && x < map_config_.width && y >= 0 && y < map_config_.height);
}

int GridMapGenerator::get_width() const
{
  return map_config_.width;
}

int GridMapGenerator::get_height() const
{
  return map_config_.height;
}

const std::vector<GridMapGenerator::AxisPoint>& GridMapGenerator::get_axis_points() const
{
  return axis_points_;
}

bool GridMapGenerator::save_as_image(const std::string& file_path, const CADData& cad_data,
                                     const GridMapVisualizationConfig& config) const
{
  try
  {
    // 获取栅格地图的宽度和高度
    int height = grid_map_.size();
    if (height == 0)
    {
      std::cerr << "Grid map is empty" << std::endl;
      return false;
    }

    int width = grid_map_[0].size();
    if (width == 0)
    {
      std::cerr << "Grid map row is empty" << std::endl;
      return false;
    }

    // 使用配置中的缩放因子
    int scale = config.scale;

    // 检查图像大小是否过大
    const int MAX_DIMENSION = 8000;
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
      estimated_memory = static_cast<size_t>(height) * width * scale * scale * 3;

      if (estimated_memory > MAX_MEMORY)
      {
        scale = 1;
      }
    }

    std::cout << "Creating grid map visualization with scale factor " << scale << std::endl;

    // 创建高分辨率图像
    cv::Mat image(height * scale, width * scale, CV_8UC3, config.free_space_color);

    // 创建临时的更大分辨率图像用于抗锯齿效果
    int super_scale = scale * 2;
    cv::Mat high_res_image;

    // 根据配置决定是否使用超采样
    bool use_supersampling = config.use_antialiasing && (scale < 3) && (width * height < 1000000);

    if (use_supersampling)
    {
      high_res_image = cv::Mat(height * super_scale, width * super_scale, CV_8UC3, config.free_space_color);
    }

    // 填充障碍物数据
    for (int y = 0; y < height; ++y)
    {
      for (int x = 0; x < width; ++x)
      {
        cv::Scalar color;
        if (grid_map_[y][x] == 1)
        {
          color = config.obstacle_color;  // 黑色表示障碍物
        }
        else
        {
          color = config.free_space_color;  // 白色表示可通行区域
        }

        // 绘制矩形而不是单个像素，提高可视性
        cv::rectangle(image, cv::Point(x * scale, y * scale), cv::Point((x + 1) * scale - 1, (y + 1) * scale - 1),
                      color,
                      -1);  // -1表示填充矩形

        // 如果使用超采样，也在高分辨率图像上绘制
        if (use_supersampling)
        {
          cv::rectangle(high_res_image, cv::Point(x * super_scale, y * super_scale),
                        cv::Point((x + 1) * super_scale - 1, (y + 1) * super_scale - 1), color, -1);
        }
      }
    }

    // 添加栅格线，提高可视性
    if (config.show_grid_lines && scale >= 3 && width * height < 500000)
    {
      // 计算栅格线颜色
      cv::Scalar grid_color = config.grid_line_color;

      for (int y = 0; y <= height; ++y)
      {
        cv::line(image, cv::Point(0, y * scale), cv::Point(width * scale, y * scale), grid_color, 1);
      }

      for (int x = 0; x <= width; ++x)
      {
        cv::line(image, cv::Point(x * scale, 0), cv::Point(x * scale, height * scale), grid_color, 1);
      }
    }

    // 使用的图像和缩放因子
    cv::Mat& drawing_image = use_supersampling ? high_res_image : image;
    int drawing_scale = use_supersampling ? super_scale : scale;

    // 轴线使用红色
    cv::Scalar axis_color = config.axis_line_color;

    // 绘制所有轴线
    for (const auto& axis_point : axis_points_)
    {
      int x1 = axis_point.x1;
      int y1 = axis_point.y1;
      int x2 = axis_point.x2;
      int y2 = axis_point.y2;

      // 确保坐标在图像范围内
      x1 = std::max(0, std::min(x1, width - 1));
      y1 = std::max(0, std::min(y1, height - 1));
      x2 = std::max(0, std::min(x2, width - 1));
      y2 = std::max(0, std::min(y2, height - 1));

      // 使用抗锯齿线条
      cv::line(drawing_image, cv::Point(x1 * drawing_scale, y1 * drawing_scale),
               cv::Point(x2 * drawing_scale, y2 * drawing_scale), axis_color,
               std::max(2, drawing_scale / 4),  // 增加线宽
               cv::LINE_AA);                    // 使用抗锯齿
    }

    // 如果使用了超采样，将高分辨率图像缩小到原始大小
    if (use_supersampling)
    {
      cv::resize(high_res_image, image, image.size(), 0, 0, cv::INTER_AREA);
    }

    // 添加图例和标题
    if (config.draw_legend)
    {
      int text_scale = std::max(1, scale / 2);
      double font_scale = 0.5 * text_scale;
      int text_thickness = std::max(1, text_scale / 2);

      int legend_y = 20 * text_scale;
      cv::putText(image, "Grid Map Visualization", cv::Point(10, legend_y), cv::FONT_HERSHEY_SIMPLEX, font_scale,
                  cv::Scalar(0, 0, 0), text_thickness);
      legend_y += 25 * text_scale;

      cv::putText(image, "Black: Obstacles", cv::Point(10, legend_y), cv::FONT_HERSHEY_SIMPLEX, font_scale,
                  cv::Scalar(0, 0, 0), text_thickness);
      legend_y += 25 * text_scale;

      cv::putText(image, "Red: Axis Lines", cv::Point(10, legend_y), cv::FONT_HERSHEY_SIMPLEX, font_scale,
                  cv::Scalar(0, 0, 255), text_thickness);
      legend_y += 25 * text_scale;

      // 添加比例尺信息
      char resolution_str[50];
      sprintf(resolution_str, "Resolution: %.3f m/cell", map_config_.resolution);

      cv::putText(image, resolution_str, cv::Point(10, legend_y), cv::FONT_HERSHEY_SIMPLEX, font_scale,
                  cv::Scalar(0, 0, 0), text_thickness);
      legend_y += 25 * text_scale;

      // 添加地图信息
      char map_info[100];
      sprintf(map_info, "Map Size: %d x %d cells (%.2f m x %.2f m)", width, height, (boundaryMaxX - boundaryMinX),
              (boundaryMaxY - boundaryMinY));

      cv::putText(image, map_info, cv::Point(10, legend_y), cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 0, 0),
                  text_thickness);
    }

    // 设置图像压缩参数
    std::vector<int> compression_params;

    if (config.image_format == "png")
    {
      compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
      compression_params.push_back(9);  // 0-9，越高质量越好但文件更大
    }
    else if (config.image_format == "jpg" || config.image_format == "jpeg")
    {
      compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
      compression_params.push_back(95);  // 0-100，越高质量越好
    }

    // 保存图像
    if (cv::imwrite(file_path, image, compression_params))
    {
      std::cout << "Successfully saved grid map as image: " << file_path << std::endl;
      return true;
    }
    else
    {
      std::cerr << "Failed to save grid map as image" << std::endl;
      return false;
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error saving grid map as image: " << e.what() << std::endl;
    return false;
  }
}

void GridMapGenerator::convertToGridLine(const std::shared_ptr<Line>& line, int value)
{
  if (!line)
  {
    return;
  }

  if (line->type == GeometryType::LINE)
  {
    // 将线段起点和终点转换为栅格坐标
    int x1, y1, x2, y2;
    if (convertWorldToGrid(line->start.x, line->start.y, x1, y1) && convertWorldToGrid(line->end.x, line->end.y, x2, y2))
    {
      // 使用Bresenham算法将线段光栅化，并使用指定的值
      drawGridLine(x1, y1, x2, y2, value);
    }
  }
  else if (line->type == GeometryType::CIRCLE)
  {
    // 对于圆，我们转换为Circle类型
    auto circle = std::dynamic_pointer_cast<Circle>(line);
    if (circle)
    {
      convertToGridCircle(circle, value);
    }
  }
  else if (line->type == GeometryType::CURVE)
  {
    // 对于曲线，我们转换为Curve类型
    auto curve = std::dynamic_pointer_cast<Curve>(line);
    if (curve)
    {
      convertToGridCurve(curve, value);
    }
  }
  else if (line->type == GeometryType::ARC)
  {
    // 对于圆弧，我们转换为Arc类型
    auto arc = std::dynamic_pointer_cast<Arc>(line);
    if (arc)
    {
      convertToGridArc(arc, value);
    }
  }
}

void GridMapGenerator::convertToGridCurve(const std::shared_ptr<Curve>& curve, int value)
{
  if (!curve || curve->control_points.size() < 2)
  {
    return;
  }

  // 贝塞尔曲线的密度参数 - 曲线上点的数量取决于曲线长度
  const double point_distance = 0.5 * map_config_.resolution;  // 取栅格分辨率的一半，确保足够密集
  const int min_points = 10;                               // 最小点数，确保短曲线也有足够的采样点

  // 估计曲线长度并计算采样点数量
  double curve_length = 0.0;
  for (size_t i = 0; i < curve->control_points.size() - 1; ++i)
  {
    curve_length += curve->control_points[i].distance(curve->control_points[i + 1]);
  }

  int num_points = std::max(min_points, static_cast<int>(curve_length / point_distance) + 1);

  // 根据曲线类型和次数生成适当的曲线点
  std::vector<Point3D> curve_points;
  curve_points.reserve(num_points);

  // 生成t参数值序列 [0, 1]
  for (int i = 0; i < num_points; ++i)
  {
    double t = static_cast<double>(i) / (num_points - 1);

    // 根据曲线次数采用不同的曲线计算方法
    if (curve->degree == 1)
    {
      // 线性曲线（不需要拟合）
      if (curve->control_points.size() < 2)
        continue;

      double mt = 1.0 - t;
      Point3D point;
      point.x = mt * curve->control_points[0].x + t * curve->control_points[1].x;
      point.y = mt * curve->control_points[0].y + t * curve->control_points[1].y;
      point.z = mt * curve->control_points[0].z + t * curve->control_points[1].z;
      curve_points.push_back(point);
    }
    else if (curve->degree == 2)
    {
      // 二次贝塞尔曲线
      if (curve->control_points.size() < 3)
        continue;

      const Point3D& p0 = curve->control_points[0];
      const Point3D& p1 = curve->control_points[1];
      const Point3D& p2 = curve->control_points[2];

      double mt = 1.0 - t;
      double mt2 = mt * mt;
      double t2 = t * t;
      double mt_t_2 = 2 * mt * t;

      Point3D point;
      point.x = mt2 * p0.x + mt_t_2 * p1.x + t2 * p2.x;
      point.y = mt2 * p0.y + mt_t_2 * p1.y + t2 * p2.y;
      point.z = mt2 * p0.z + mt_t_2 * p1.z + t2 * p2.z;
      curve_points.push_back(point);
    }
    else if (curve->degree == 3)
    {
      // 三次贝塞尔曲线
      if (curve->control_points.size() < 4)
        continue;

      const Point3D& p0 = curve->control_points[0];
      const Point3D& p1 = curve->control_points[1];
      const Point3D& p2 = curve->control_points[2];
      const Point3D& p3 = curve->control_points[3];

      double mt = 1.0 - t;
      double mt2 = mt * mt;
      double mt3 = mt2 * mt;
      double t2 = t * t;
      double t3 = t2 * t;

      Point3D point;
      point.x = mt3 * p0.x + 3 * mt2 * t * p1.x + 3 * mt * t2 * p2.x + t3 * p3.x;
      point.y = mt3 * p0.y + 3 * mt2 * t * p1.y + 3 * mt * t2 * p2.y + t3 * p3.y;
      point.z = mt3 * p0.z + 3 * mt2 * t * p1.z + 3 * mt * t2 * p2.z + t3 * p3.z;
      curve_points.push_back(point);
    }
    else
    {
      // 处理特定NURBS曲线
      if (curve->weights.size() == curve->control_points.size() && !curve->knots.empty())
      {
        // 如果有权重和节点，使用NURBS算法
        Point3D point = evaluate_nurbs_point(*curve, t);
        curve_points.push_back(point);
      }
      else
      {
        // 回退到使用De Casteljau算法
        Point3D point = evaluate_bezier_point(curve->control_points, t);
        curve_points.push_back(point);
      }
    }
  }

  // 将生成的曲线点光栅化到栅格地图上
  if (curve_points.size() < 2)
  {
    return;
  }

  // 将曲线点转换为栅格坐标并光栅化
  for (size_t i = 0; i < curve_points.size() - 1; ++i)
  {
    const Point3D& p1 = curve_points[i];
    const Point3D& p2 = curve_points[i + 1];

    int x1, y1, x2, y2;
    if (convertWorldToGrid(p1.x, p1.y, x1, y1) && convertWorldToGrid(p2.x, p2.y, x2, y2))
    {
      // 使用Bresenham算法连接点
      drawGridLine(x1, y1, x2, y2, value);
    }
  }
}

void GridMapGenerator::convertToGridCircle(const std::shared_ptr<Circle>& circle, int value)
{
  if (!circle)
  {
    return;
  }

  // 将圆心坐标转换为栅格坐标
  int center_x, center_y;
  if (!convertWorldToGrid(circle->center.x, circle->center.y, center_x, center_y))
  {
    return;  // 如果圆心不在地图范围内，返回
  }

  // 计算圆半径的栅格数量
  int radius_grid = static_cast<int>(circle->radius / map_config_.resolution + 0.5);

  // 使用中点画圆算法
  int x = 0;
  int y = radius_grid;
  int d = 1 - radius_grid;

  // 画八个对称点
  auto draw_circle_points = [this, center_x, center_y, value](int x, int y) {
    this->set_grid_point(center_x + x, center_y + y, value);
    this->set_grid_point(center_x - x, center_y + y, value);
    this->set_grid_point(center_x + x, center_y - y, value);
    this->set_grid_point(center_x - x, center_y - y, value);
    this->set_grid_point(center_x + y, center_y + x, value);
    this->set_grid_point(center_x - y, center_y + x, value);
    this->set_grid_point(center_x + y, center_y - x, value);
    this->set_grid_point(center_x - y, center_y - x, value);
  };

  draw_circle_points(x, y);

  while (y > x)
  {
    if (d < 0)
    {
      d += 2 * x + 3;
    }
    else
    {
      d += 2 * (x - y) + 5;
      y--;
    }
    x++;
    draw_circle_points(x, y);
  }
}

void GridMapGenerator::convertToGridArc(const std::shared_ptr<Arc>& arc, int value)
{
  if (!arc)
  {
    return;
  }

  // 将圆心坐标转换为栅格坐标
  int center_x, center_y;
  if (!convertWorldToGrid(arc->center.x, arc->center.y, center_x, center_y))
  {
    return;  // 如果圆心不在地图范围内，返回
  }

  // 计算圆半径的栅格数量
  int radius_grid = static_cast<int>(arc->radius / map_config_.resolution + 0.5);

  // 计算角度范围
  double start_angle = arc->start_angle;
  double end_angle = arc->end_angle;

  // 确保终止角度大于起始角度
  if (end_angle < start_angle)
  {
    end_angle += 2.0 * M_PI;
  }

  // 基于角度和半径计算弧上的点
  const double angle_step = 0.01;  // 每步角度增量（弧度）
  for (double angle = start_angle; angle <= end_angle; angle += angle_step)
  {
    int x = center_x + static_cast<int>(radius_grid * std::cos(angle));
    int y = center_y + static_cast<int>(radius_grid * std::sin(angle));

    if (isCoordinateValid(x, y))
    {
      grid_map_[y][x] = value;
    }
  }
}

void GridMapGenerator::set_grid_point(int x, int y, int value)
{
  if (isCoordinateValid(x, y))
  {
    grid_map_[y][x] = value;
  }
}

void GridMapGenerator::drawGridLine(int x1, int y1, int x2, int y2, int value)
{
  // Bresenham算法实现
  int dx = std::abs(x2 - x1);
  int dy = std::abs(y2 - y1);
  int sx = (x1 < x2) ? 1 : -1;
  int sy = (y1 < y2) ? 1 : -1;
  int err = dx - dy;

  while (true)
  {
    // 如果坐标在范围内，设置为指定值
    if (isCoordinateValid(x1, y1))
    {
      grid_map_[y1][x1] = value;
    }

    // 如果到达终点，退出
    if (x1 == x2 && y1 == y2)
    {
      break;
    }

    int e2 = 2 * err;
    if (e2 > -dy)
    {
      err -= dy;
      x1 += sx;
    }
    if (e2 < dx)
    {
      err += dx;
      y1 += sy;
    }
  }
}

void GridMapGenerator::store_axis_line_points(const std::shared_ptr<Line>& line)
{
  if (!line)
  {
    return;
  }

  if (line->type == GeometryType::LINE)
  {
    // 将线段起点和终点转换为栅格坐标
    int x1, y1, x2, y2;
    if (convertWorldToGrid(line->start.x, line->start.y, x1, y1) && convertWorldToGrid(line->end.x, line->end.y, x2, y2))
    {
      // 保存线段的起点和终点
      axis_points_.push_back({ x1, y1, x2, y2, line->id });
    }
  }
  else if (line->type == GeometryType::CIRCLE)
  {
    // 处理圆
    auto circle = std::dynamic_pointer_cast<Circle>(line);
    if (circle)
    {
      store_circle_axis_points(circle);
    }
  }
  else if (line->type == GeometryType::ARC)
  {
    // 处理圆弧
    auto arc = std::dynamic_pointer_cast<Arc>(line);
    if (arc)
    {
      store_arc_axis_points(arc);
    }
  }
  else if (line->type == GeometryType::CURVE)
  {
    // 对于曲线，使用离散化方法生成一系列点，然后保存为多条线段
    auto curve = std::dynamic_pointer_cast<Curve>(line);
    if (curve && curve->control_points.size() >= 2)
    {
      // 曲线的密度参数，确保有足够的点
      const double point_distance = 0.5 * map_config_.resolution;
      const int min_points = 20;  // 最小点数，确保短曲线也有足够采样点

      // 估计曲线长度
      double curve_length = 0.0;
      if (curve->length > 0)
      {
        curve_length = curve->length;
      }
      else
      {
        // 如果没有提供长度，通过控制点估计
        for (size_t i = 0; i < curve->control_points.size() - 1; ++i)
        {
          curve_length += curve->control_points[i].distance(curve->control_points[i + 1]);
        }
      }

      // 计算采样点数量
      int num_points = std::max(min_points, static_cast<int>(curve_length / point_distance) + 1);

      // 生成曲线点
      std::vector<Point3D> curve_points;
      curve_points.reserve(num_points);

      for (int i = 0; i < num_points; ++i)
      {
        double t = static_cast<double>(i) / (num_points - 1);

        // 根据曲线次数和控制点采用不同的计算方法
        Point3D point;
        if (curve->degree <= 3 && curve->control_points.size() >= curve->degree + 1)
        {
          // 使用贝塞尔曲线计算
          if (curve->degree == 1 && curve->control_points.size() >= 2)
          {
            // 线性插值
            const Point3D& p0 = curve->control_points[0];
            const Point3D& p1 = curve->control_points[1];

            point.x = (1 - t) * p0.x + t * p1.x;
            point.y = (1 - t) * p0.y + t * p1.y;
            point.z = (1 - t) * p0.z + t * p1.z;
          }
          else if (curve->degree == 2 && curve->control_points.size() >= 3)
          {
            // 二次贝塞尔曲线
            const Point3D& p0 = curve->control_points[0];
            const Point3D& p1 = curve->control_points[1];
            const Point3D& p2 = curve->control_points[2];

            double mt = 1.0 - t;
            double mt2 = mt * mt;
            double t2 = t * t;

            point.x = mt2 * p0.x + 2 * mt * t * p1.x + t2 * p2.x;
            point.y = mt2 * p0.y + 2 * mt * t * p1.y + t2 * p2.y;
            point.z = mt2 * p0.z + 2 * mt * t * p1.z + t2 * p2.z;
          }
          else if (curve->degree == 3 && curve->control_points.size() >= 4)
          {
            // 三次贝塞尔曲线
            const Point3D& p0 = curve->control_points[0];
            const Point3D& p1 = curve->control_points[1];
            const Point3D& p2 = curve->control_points[2];
            const Point3D& p3 = curve->control_points[3];

            double mt = 1.0 - t;
            double mt2 = mt * mt;
            double mt3 = mt2 * mt;
            double t2 = t * t;
            double t3 = t2 * t;

            point.x = mt3 * p0.x + 3 * mt2 * t * p1.x + 3 * mt * t2 * p2.x + t3 * p3.x;
            point.y = mt3 * p0.y + 3 * mt2 * t * p1.y + 3 * mt * t2 * p2.y + t3 * p3.y;
            point.z = mt3 * p0.z + 3 * mt2 * t * p1.z + 3 * mt * t2 * p2.z + t3 * p3.z;
          }
          else
          {
            // 回退到使用De Casteljau算法
            point = evaluate_bezier_point(curve->control_points, t);
          }
        }
        else if (curve->weights.size() == curve->control_points.size() && !curve->knots.empty())
        {
          // 使用NURBS算法
          point = evaluate_nurbs_point(*curve, t);
        }
        else
        {
          // 回退到使用De Casteljau算法
          point = evaluate_bezier_point(curve->control_points, t);
        }

        curve_points.push_back(point);
      }

      // 将曲线点转换为栅格坐标并保存为线段
      for (size_t i = 0; i < curve_points.size() - 1; ++i)
      {
        const Point3D& p1 = curve_points[i];
        const Point3D& p2 = curve_points[i + 1];

        int x1, y1, x2, y2;
        if (convertWorldToGrid(p1.x, p1.y, x1, y1) && convertWorldToGrid(p2.x, p2.y, x2, y2))
        {
          axis_points_.push_back({ x1, y1, x2, y2, curve->id });
        }
      }
    }
  }
}

void GridMapGenerator::store_circle_axis_points(const std::shared_ptr<Circle>& circle)
{
  if (!circle)
  {
    return;
  }

  // 将圆心坐标转换为栅格坐标
  int center_x, center_y;
  if (!convertWorldToGrid(circle->center.x, circle->center.y, center_x, center_y))
  {
    return;  // 如果圆心不在地图范围内，返回
  }

  // 计算圆半径的栅格数量
  int radius_grid = static_cast<int>(circle->radius / map_config_.resolution + 0.5);

  // 以固定角度间隔存储圆周上的点
  const int num_points = 36;  // 每10度一个点
  const double angle_step = 2.0 * M_PI / num_points;

  for (int i = 0; i < num_points; ++i)
  {
    double angle1 = i * angle_step;
    double angle2 = (i + 1) * angle_step;

    int x1 = center_x + static_cast<int>(radius_grid * std::cos(angle1));
    int y1 = center_y + static_cast<int>(radius_grid * std::sin(angle1));
    int x2 = center_x + static_cast<int>(radius_grid * std::cos(angle2));
    int y2 = center_y + static_cast<int>(radius_grid * std::sin(angle2));

    // 保存为多个线段
    axis_points_.push_back({ x1, y1, x2, y2, circle->id });
  }
}

void GridMapGenerator::store_arc_axis_points(const std::shared_ptr<Arc>& arc)
{
  if (!arc)
  {
    return;
  }

  // 将圆心坐标转换为栅格坐标
  int center_x, center_y;
  if (!convertWorldToGrid(arc->center.x, arc->center.y, center_x, center_y))
  {
    return;  // 如果圆心不在地图范围内，返回
  }

  // 计算圆半径的栅格数量
  int radius_grid = static_cast<int>(arc->radius / map_config_.resolution + 0.5);

  // 计算角度范围
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

  for (int i = 0; i < num_points; ++i)
  {
    double angle1 = start_angle + i * angle_step;
    double angle2 = start_angle + (i + 1) * angle_step;

    int x1 = center_x + static_cast<int>(radius_grid * std::cos(angle1));
    int y1 = center_y + static_cast<int>(radius_grid * std::sin(angle1));
    int x2 = center_x + static_cast<int>(radius_grid * std::cos(angle2));
    int y2 = center_y + static_cast<int>(radius_grid * std::sin(angle2));

    // 保存为多个线段
    axis_points_.push_back({ x1, y1, x2, y2, arc->id });
  }
}

void GridMapGenerator::calculate_map_bounds(const CADData& cad_data)
{
  // 初始化边界为无限大/小
  boundaryMinX = std::numeric_limits<double>::max();
  boundaryMinY = std::numeric_limits<double>::max();
  boundaryMaxX = std::numeric_limits<double>::lowest();
  boundaryMaxY = std::numeric_limits<double>::lowest();

  // 检查原点坐标
  for (const auto& point : cad_data.origin_points)
  {
    boundaryMinX = std::min(boundaryMinX, point.x);
    boundaryMinY = std::min(boundaryMinY, point.y);
    boundaryMaxX = std::max(boundaryMaxX, point.x);
    boundaryMaxY = std::max(boundaryMaxY, point.y);
  }

  // 函数用于更新边界
  auto update_bounds_from_line = [&](const std::shared_ptr<Line>& line) {
    if (!line)
    {
      return;
    }

    boundaryMinX = std::min(boundaryMinX, line->start.x);
    boundaryMinY = std::min(boundaryMinY, line->start.y);
    boundaryMaxX = std::max(boundaryMaxX, line->start.x);
    boundaryMaxY = std::max(boundaryMaxY, line->start.y);

    boundaryMinX = std::min(boundaryMinX, line->end.x);
    boundaryMinY = std::min(boundaryMinY, line->end.y);
    boundaryMaxX = std::max(boundaryMaxX, line->end.x);
    boundaryMaxY = std::max(boundaryMaxY, line->end.y);

    // 如果是曲线，还需要检查所有控制点
    if (line->type == GeometryType::CURVE)
    {
      auto curve = std::dynamic_pointer_cast<Curve>(line);
      if (curve)
      {
        for (const auto& point : curve->control_points)
        {
          boundaryMinX = std::min(boundaryMinX, point.x);
          boundaryMinY = std::min(boundaryMinY, point.y);
          boundaryMaxX = std::max(boundaryMaxX, point.x);
          boundaryMaxY = std::max(boundaryMaxY, point.y);
        }
      }
    }
  };

  // 遍历所有线段更新边界
  for (const auto& line : cad_data.axis_lines)
  {
    update_bounds_from_line(line);
  }

  for (const auto& line : cad_data.wall_lines)
  {
    update_bounds_from_line(line);
  }

  for (const auto& line : cad_data.control_lines)
  {
    update_bounds_from_line(line);
  }

  for (const auto& line : cad_data.cross_lines)
  {
    update_bounds_from_line(line);
  }

  for (const auto& line : cad_data.barrier_lines)
  {
    update_bounds_from_line(line);
  }

  for (const auto& line : cad_data.low_barrier_lines)
  {
    update_bounds_from_line(line);
  }

  // 添加一些边界填充
  boundaryMinX -= map_config_.padding;
  boundaryMinY -= map_config_.padding;
  boundaryMaxX += map_config_.padding;
  boundaryMaxY += map_config_.padding;
}

Point3D GridMapGenerator::evaluate_bezier_point(const std::vector<Point3D>& control_points, double t)
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

  // 使用De Casteljau算法递归计算
  std::vector<Point3D> reduced_points(control_points.size() - 1);
  for (size_t i = 0; i < reduced_points.size(); ++i)
  {
    const Point3D& p1 = control_points[i];
    const Point3D& p2 = control_points[i + 1];

    reduced_points[i] = Point3D(p1.x * (1 - t) + p2.x * t, p1.y * (1 - t) + p2.y * t, p1.z * (1 - t) + p2.z * t);
  }

  return evaluate_bezier_point(reduced_points, t);
}

Point3D GridMapGenerator::evaluate_nurbs_point(const Curve& curve, double t)
{
  // 这里简化实现NURBS计算，实际使用中可能需要更完整的NURBS库
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

  // 计算基函数
  int p = curve.degree;
  std::vector<double> N(p + 1, 0.0);

  // 初始化0次基函数
  for (int i = 0; i <= p; ++i)
  {
    if (u >= curve.knots[span + i] && u < curve.knots[span + i + 1])
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

}  // namespace daosnrs_planning