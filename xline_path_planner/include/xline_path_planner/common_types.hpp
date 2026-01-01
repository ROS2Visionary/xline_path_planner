#pragma once

#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <cstdint>
#include <stdexcept>
#include <limits>

namespace path_planner
{

/**
 * @brief 二维点结构体
 * 表示二维平面上的点
 */
struct Point2D
{
  double x = 0.0;  ///< X坐标
  double y = 0.0;  ///< Y坐标

  /**
   * @brief 默认构造函数
   */
  Point2D() = default;

  /**
   * @brief 带参数的构造函数
   * @param x X坐标
   * @param y Y坐标
   */
  Point2D(double x, double y) : x(x), y(y)
  {
  }

  /**
   * @brief 计算与另一个点的距离
   * @param other 另一个点
   * @return 两点之间的欧几里得距离
   */
  double distance(const Point2D& other) const
  {
    return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
  }

  /**
   * @brief 相等运算符
   * @param other 另一个点
   * @return 如果两点坐标相同则返回true
   */
  bool operator==(const Point2D& other) const
  {
    return std::fabs(x - other.x) < 1e-6 && std::fabs(y - other.y) < 1e-6;
  }

  /**
   * @brief 不等运算符
   * @param other 另一个点
   * @return 如果两点坐标不同则返回true
   */
  bool operator!=(const Point2D& other) const
  {
    return !(*this == other);
  }
};

/**
 * @brief 三维点结构体，继承自二维点
 * 表示三维空间中的点
 */
struct Point3D : public Point2D
{
  double z = 0.0;  ///< Z坐标

  /**
   * @brief 默认构造函数
   */
  Point3D() = default;

  /**
   * @brief 带参数的构造函数
   * @param x X坐标
   * @param y Y坐标
   * @param z Z坐标
   */
  Point3D(double x, double y, double z) : Point2D(x, y), z(z)
  {
  }

  /**
   * @brief 标量乘法运算符
   * @param scalar 标量值
   * @return 返回缩放后的点
   */
  Point3D operator*(double scalar) const
  {
    return Point3D(x * scalar, y * scalar, z * scalar);
  }

  /**
   * @brief 标量除法运算符
   * @param scalar 标量值
   * @return 返回缩放后的点
   * @throw std::runtime_error 当标量接近零时抛出异常
   */
  Point3D operator/(double scalar) const
  {
    if (std::abs(scalar) < 1e-10)
    {
      throw std::runtime_error("除数接近零");
    }
    return Point3D(x / scalar, y / scalar, z / scalar);
  }

  /**
   * @brief 向量减法运算符
   * @param other 另一个点
   * @return 返回两点之差
   */
  Point3D operator-(const Point3D& other) const
  {
    return Point3D(x - other.x, y - other.y, z - other.z);
  }

  /**
   * @brief 向量加法运算符
   * @param other 另一个点
   * @return 返回两点之和
   */
  Point3D operator+(const Point3D& other) const
  {
    return Point3D(x + other.x, y + other.y, z + other.z);
  }

  /**
   * @brief 计算与另一个点的距离
   * @param other 另一个点
   * @return 两点之间的三维欧几里得距离
   */
  double distance(const Point3D& other) const
  {
    return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2) + std::pow(z - other.z, 2));
  }

  /**
   * @brief 相等运算符
   * @param other 另一个点
   * @return 如果两点坐标相同则返回true
   */
  bool operator==(const Point3D& other) const
  {
    return Point2D::operator==(other) && std::fabs(z - other.z) < 1e-6;
  }

  /**
   * @brief 不等运算符
   * @param other 另一个点
   * @return 如果两点坐标不同则返回true
   */
  bool operator!=(const Point3D& other) const
  {
    return !(*this == other);
  }
};

/**
 * @brief 二维向量结构体
 * 表示二维平面上的向量
 */
struct Vector2D
{
  double x = 0.0;  ///< X分量
  double y = 0.0;  ///< Y分量

  /**
   * @brief 默认构造函数
   */
  Vector2D() = default;

  /**
   * @brief 带参数的构造函数
   * @param x X分量
   * @param y Y分量
   */
  Vector2D(double x, double y) : x(x), y(y)
  {
  }

  /**
   * @brief 计算向量的模长
   * @return 向量的长度
   */
  double magnitude() const
  {
    return std::sqrt(x * x + y * y);
  }

  /**
   * @brief 获取向量的单位向量
   * @return 单位向量
   */
  Vector2D normalize() const
  {
    double mag = magnitude();
    if (mag < 1e-6)
    {
      return Vector2D(0.0, 0.0);
    }
    return Vector2D(x / mag, y / mag);
  }

  /**
   * @brief 计算与另一个向量的点积
   * @param other 另一个向量
   * @return 两向量的点积
   */
  double dot(const Vector2D& other) const
  {
    return x * other.x + y * other.y;
  }
};

/**
 * @brief 几何类型枚举
 * 定义不同的几何图元类型
 */
enum class GeometryType
{
  LINE = 1,      ///< 直线
  POLYLINE = 2,  ///< 多段线（顶点序列）
  CIRCLE = 4,  ///< 圆
  CURVE = 6,   ///< 曲线
  ARC = 7,     ///< 圆弧
  TEXT = 8,    ///< 文字
  ELLIPSE = 9, ///< 椭圆/椭圆弧
  SPLINE = 10  ///< 样条曲线（CAD spline）
};

/**
 * @brief 路径类型枚举
 * 定义不同的路径类型
 */
enum class RouteType
{
  DRAWING_PATH,    ///< 绘图路径
  TRANSITION_PATH  ///< 轨迹间转场路径
};

/**
 * @brief 打印机类型枚举
 * 定义不同的打印机类型
 */
enum class PrinterType
{
  LEFT_PRINTER,    ///< 左侧打印机
  RIGHT_PRINTER,   ///< 右侧打印机
  CENTER_PRINTER   ///< 中心打印机（默认）
};

/**
 * @brief 墨水打印模式枚举
 * 定义不同的打印模式
 */
enum class InkMode
{
  SOLID,   ///< 实线打印
  DASHED,  ///< 虚线打印
  TEXT     ///< 文字打印
};

/**
 * @brief 文字对齐方式结构体
 */
struct TextAlign
{
  std::string horizontal = "left";    ///< 水平对齐: left, center, right
  std::string vertical = "baseline";  ///< 垂直对齐: top, middle, bottom, baseline
};

/**
 * @brief 几何实体基类（直线为核心，圆/弧继承）
 * 说明：
 *  - 与新JSON格式对齐，新增若干可选元数据字段（line_type/thickness/hidden/layer/layer_id/color/selected）。
 *  - start/end/length 在直线中直接来源于JSON，在圆/弧中按中心/半径/角度推导。
 */
struct Line
{
  // 基本标识
  int32_t id = 0;                          ///< 实体ID（来自JSON的id）
  GeometryType type = GeometryType::LINE;  ///< 几何类型（Line/Circle/Curve/Arc/Text）

  // 几何参数
  Point3D start;                           ///< 起点（line直接提供；circle/arc为推导点）
  Point3D end;                             ///< 终点（line直接提供；circle起点=终点；arc按角度推导）
  double length = 0.0;                     ///< 几何长度（线段长度/圆周长/弧长）

  // 绘制状态（内部使用）
  bool is_printed = false;                 ///< 是否已被处理/绘制（规划阶段使用）

  // 新JSON相关的可选元数据（若存在则填充）
  std::string line_type;                   ///< 线型（如"continuous"）
  double thickness = 0.0;                  ///< 线宽/厚度
  bool hidden = false;                     ///< 是否隐藏
  std::string layer;                       ///< 图层名称（由layer_id映射得到或直接来自元素内layer）
  int32_t layer_id = -1;                   ///< 图层ID（与layers[*].layer_id对应）
  std::string color;                       ///< 颜色（十六进制，如"#FFFFFF"）
  bool selected = false;                   ///< 是否被选中（仅元数据，规划不依赖）

  /**
   * @brief 默认构造函数
   */
  Line() = default;

  /**
   * @brief 带参数的构造函数
   * @param id 线段ID
   * @param start 起点
   * @param end 终点
   * @param type 几何类型
   */
  Line(int32_t id, const Point3D& start, const Point3D& end, GeometryType type = GeometryType::LINE)
    : id(id), type(type), start(start), end(end)
  {
    length = start.distance(end);
  }

  /**
   * @brief 虚析构函数
   * 使Line成为多态类
   */
  virtual ~Line() = default;

  /**
   * @brief 计算线段的方向向量
   * @return 表示线段方向的单位向量
   */
  Vector2D direction() const
  {
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < 1e-6)
    {
      return Vector2D(0.0, 0.0);
    }
    return Vector2D(dx / dist, dy / dist);
  }
};

/**
 * @brief 多段线（Polyline），由多个顶点构成
 * 说明：用于兼容 CAD JSON 中的 "polyline" 类型；通常会在预处理阶段拆分为若干 LineSegment。
 */
struct Polyline : public Line
{
  std::vector<Point3D> vertices;  ///< 顶点序列（至少2个点）
  bool closed = false;            ///< 是否闭合（闭合时最后点连回第一个点）

  Polyline() { type = GeometryType::POLYLINE; }

  void update_geometry()
  {
    type = GeometryType::POLYLINE;
    if (!vertices.empty())
    {
      start = vertices.front();
      end = vertices.back();
    }
    length = 0.0;
    if (vertices.size() >= 2)
    {
      for (size_t i = 0; i + 1 < vertices.size(); ++i)
      {
        length += vertices[i].distance(vertices[i + 1]);
      }
      if (closed && vertices.size() > 2)
      {
        length += vertices.back().distance(vertices.front());
      }
    }
  }
};

/**
 * @brief 样条曲线（Spline），继承自Line
 *
 * 说明：
 * - CAD 导出的 spline 可能同时包含 control_points/knots/weights（NURBS），也可能直接给出离散后的 vertices。
 * - 当前规划阶段优先使用 vertices（若非空）作为路径点序列；否则可由 control_points/knots/weights 做进一步离散化（预留）。
 */
struct Spline : public Line
{
  int degree = 3;                 ///< 次数（常见为3）
  bool periodic = false;          ///< 是否周期样条
  bool closed = false;            ///< 是否闭合
  std::vector<Point3D> control_points;  ///< 控制点（可选）
  std::vector<double> weights;          ///< 权重（可选）
  std::vector<double> knots;            ///< 节点向量（可选）
  std::vector<Point3D> vertices;        ///< 离散点（若提供则优先使用）

  Spline() : Line()
  {
    type = GeometryType::SPLINE;
  }

  void update_geometry()
  {
    type = GeometryType::SPLINE;
    if (!vertices.empty())
    {
      start = vertices.front();
      end = vertices.back();
      length = 0.0;
      if (vertices.size() >= 2)
      {
        for (size_t i = 0; i + 1 < vertices.size(); ++i)
        {
          length += vertices[i].distance(vertices[i + 1]);
        }
        if (closed && vertices.size() > 2)
        {
          length += vertices.back().distance(vertices.front());
        }
      }
    }
  }
};

/**
 * @brief Polyline 拆分后的线段（带来源信息）
 */
struct LineSegment : public Line
{
  int32_t parent_polyline_id = -1;  ///< 原始 Polyline 的 ID（-1 表示非 Polyline 来源）
  int32_t segment_index = -1;       ///< 在原始 Polyline 中的线段索引
  bool is_from_polyline = false;    ///< 是否来自 Polyline 拆分
};

/**
 * @brief 共线合并后的“虚拟线段”，记录来源线段 ID
 */
struct MergedLine : public Line
{
  std::vector<int32_t> source_line_ids;  ///< 合并来源线段 ID（包含主 ID）
};

/**
 * @brief 曲线结构体，继承自Line（预留未来扩展）
 * 说明：当前新JSON未提供曲线规范；保留对Bézier/NURBS的基本支持，
 *       以便后续若引入曲线数据可直接对接。
 */
struct Curve : public Line
{
  int degree = 0;                       ///< 曲线次数
  std::vector<Point3D> control_points;  ///< 控制点数组
  std::vector<double> weights;          ///< NURBS权重数组
  std::vector<double> knots;            ///< NURBS节点数组

  /**
   * @brief 默认构造函数
   */
  Curve() : Line()
  {
    type = GeometryType::CURVE;
  }

  /**
   * @brief 带参数的构造函数
   * @param id 曲线ID
   * @param start 起点
   * @param end 终点
   */
  Curve(int32_t id, const Point3D& start, const Point3D& end) : Line(id, start, end, GeometryType::CURVE)
  {
  }

  /**
   * @brief 虚析构函数
   */
  virtual ~Curve() override = default;
};

/**
 * @brief 圆形结构体，继承自Line
 * 表示一个圆（center+radius），start/end为圆上的同一点（x轴正向），length为周长。
 */
struct Circle : public Line
{
  Point3D center;  ///< 圆心
  double radius;   ///< 半径

  /**
   * @brief 默认构造函数
   */
  Circle() : Line()
  {
    type = GeometryType::CIRCLE;
  }

  /**
   * @brief 带参数的构造函数
   * @param id 圆的ID
   * @param center 圆心
   * @param radius 半径
   */
  Circle(int32_t id, const Point3D& center, double radius)
    : Line(id, Point3D(), Point3D(), GeometryType::CIRCLE), center(center), radius(radius)
  {
    // 计算一个点在圆上作为起点
    start.x = center.x + radius;
    start.y = center.y;
    start.z = center.z;

    // 计算另一个点在圆上作为终点（与起点相同，因为圆是闭合的）
    end = start;

    // 计算周长作为长度
    length = 2.0 * M_PI * radius;
  }

  /**
   * @brief 虚析构函数
   */
  virtual ~Circle() override = default;
};

/**
 * @brief 圆弧结构体，继承自Circle
 * 表示一个圆弧（center+radius+start_angle+end_angle），角度单位为弧度。
 */
struct Arc : public Circle
{
  double start_angle;  ///< 起始角度（弧度）
  double end_angle;    ///< 结束角度（弧度）

  /**
   * @brief 默认构造函数
   */
  Arc() : Circle()
  {
    type = GeometryType::ARC;
    start_angle = 0.0;
    end_angle = 2.0 * M_PI;
  }

  /**
   * @brief 带参数的构造函数
   * @param id 圆弧ID
   * @param center 圆心
   * @param radius 半径
   * @param start_angle 起始角度（弧度）
   * @param end_angle 结束角度（弧度）
   */
  Arc(int32_t id, const Point3D& center, double radius, double start_angle, double end_angle)
    : Circle(id, center, radius), start_angle(start_angle), end_angle(end_angle)
  {
    type = GeometryType::ARC;

    // 计算起点（基于起始角度）
    start.x = center.x + radius * std::cos(start_angle);
    start.y = center.y + radius * std::sin(start_angle);
    start.z = center.z;

    // 计算终点（基于结束角度）
    end.x = center.x + radius * std::cos(end_angle);
    end.y = center.y + radius * std::sin(end_angle);
    end.z = center.z;

    // 计算圆弧长度
    double angle_diff = end_angle - start_angle;
    if (angle_diff < 0)
    {
      angle_diff += 2.0 * M_PI;
    }
    length = radius * angle_diff;
  }

  /**
   * @brief 虚析构函数
   */
  virtual ~Arc() override = default;
};

/**
 * @brief 椭圆结构体，继承自Line
 * 表示一个椭圆/椭圆弧（center + major_axis + ratio + start_angle + end_angle + rotation）。
 *
 * 说明：
 * - major_axis 表示从 center 指向“长轴端点”的向量，其模长为长半轴 a。
 * - ratio 表示短半轴与长半轴之比（b = a * ratio）。
 * - start_angle/end_angle 为参数角（弧度），用于限定椭圆弧范围；若为完整椭圆，通常覆盖 2π。
 * - rotation 为附加旋转（弧度），用于兼容某些数据源将方向单独以 rotation 表示的情况；
 *   实际长轴方向角 = atan2(major_axis.y, major_axis.x) + rotation。
 */
struct Ellipse : public Line
{
  Point3D center;          ///< 椭圆中心
  Point3D major_axis;      ///< 长轴向量（从中心指向长轴端点）
  double ratio = 1.0;      ///< 短长轴比（b/a）
  double start_angle = 0.0;///< 起始参数角（弧度）
  double end_angle = 2.0 * M_PI;  ///< 结束参数角（弧度）
  double rotation = 0.0;   ///< 附加旋转（弧度）

  Ellipse() : Line()
  {
    type = GeometryType::ELLIPSE;
  }

  double major_radius() const
  {
    return std::hypot(major_axis.x, major_axis.y);
  }

  double minor_radius() const
  {
    return major_radius() * ratio;
  }

  double orientation() const
  {
    return std::atan2(major_axis.y, major_axis.x) + rotation;
  }
};

/**
 * @brief 文字结构体，继承自Line
 * 表示一个文字元素（position+content+height+rotation），用于喷码机打印文字。
 * 文字只能使用左边或右边的喷码机，具体使用哪边需要根据实际情况（运动方向）来决定。
 */
struct Text : public Line
{
  Point3D position;              ///< 文字位置（锚点）
  std::string content;           ///< 文字内容
  double height = 50.0;          ///< 文字高度（毫米，转换前）
  double rotation = 0.0;         ///< 旋转角度（度）
  std::string style = "Standard"; ///< 字体样式
  double width_factor = 1.0;     ///< 宽度因子
  double oblique = 0.0;          ///< 倾斜角度
  TextAlign align;               ///< 对齐方式
  std::string subtype = "text";  ///< 子类型（text/mtext）

  /**
   * @brief 默认构造函数
   */
  Text() : Line()
  {
    type = GeometryType::TEXT;
    line_type = "text";  // 标记为文字类型
  }

  /**
   * @brief 带参数的构造函数
   * @param id 文字ID
   * @param position 文字位置
   * @param content 文字内容
   * @param height 文字高度
   * @param rotation 旋转角度（度）
   */
  Text(int32_t id, const Point3D& position, const std::string& content, double height = 50.0, double rotation = 0.0)
    : Line(id, position, position, GeometryType::TEXT), position(position), content(content), height(height), rotation(rotation)
  {
    line_type = "text";
    // 文字的起点和终点都设置为位置点
    // 实际规划时会根据文字宽度和对齐方式计算实际的绘制范围
    start = position;
    end = position;
    length = 0.0;  // 文字长度在规划时动态计算
  }

  /**
   * @brief 虚析构函数
   */
  virtual ~Text() override = default;

  /**
   * @brief 计算文字的估计宽度
   * @return 文字宽度（米）
   * 简化估算：假设每个字符宽度约为高度的0.6倍
   */
  double estimateWidth() const
  {
    // 估算宽度：字符数 * 高度 * 宽度因子 * 0.6
    return static_cast<double>(content.length()) * height * width_factor * 0.6;
  }

  /**
   * @brief 根据对齐方式计算实际绘制的起点和终点
   * @param out_start 输出起点
   * @param out_end 输出终点
   */
  void calculateDrawingBounds(Point3D& out_start, Point3D& out_end) const
  {
    double text_width = estimateWidth();
    double rotation_rad = rotation * M_PI / 180.0;

    // 计算文字方向向量
    double dir_x = std::cos(rotation_rad);
    double dir_y = std::sin(rotation_rad);

    // 根据水平对齐方式计算偏移
    double offset_x = 0.0;
    if (align.horizontal == "center")
    {
      offset_x = -text_width / 2.0;
    }
    else if (align.horizontal == "right")
    {
      offset_x = -text_width;
    }
    // "left" 默认不偏移

    // 计算起点和终点
    out_start.x = position.x + offset_x * dir_x;
    out_start.y = position.y + offset_x * dir_y;
    out_start.z = position.z;

    out_end.x = out_start.x + text_width * dir_x;
    out_end.y = out_start.y + text_width * dir_y;
    out_end.z = position.z;
  }
};

/**
 * @brief 栅格坐标结构体
 * 表示栅格地图中的整数坐标
 */
struct GridCoordinate
{
  int x = 0;  ///< X坐标
  int y = 0;  ///< Y坐标

  /**
   * @brief 默认构造函数
   */
  GridCoordinate() = default;

  /**
   * @brief 带参数的构造函数
   * @param x X坐标
   * @param y Y坐标
   */
  GridCoordinate(int x, int y) : x(x), y(y)
  {
  }

  /**
   * @brief 相等运算符
   * @param other 另一个栅格坐标
   * @return 如果坐标相同则返回true
   */
  bool operator==(const GridCoordinate& other) const
  {
    return x == other.x && y == other.y;
  }

  /**
   * @brief 不等运算符
   * @param other 另一个栅格坐标
   * @return 如果坐标不同则返回true
   */
  bool operator!=(const GridCoordinate& other) const
  {
    return !(*this == other);
  }
};

/**
 * @brief 航点数据结构体
 * 表示路径上的一个点
 */
struct WaypointData
{
  Point3D position;         ///< 位置
  double angle = 0.0;       ///< 角度
  bool is_drawing = false;  ///< 是否在绘图
  int32_t line_id = 0;      ///< 相关联的线ID

  /**
   * @brief 默认构造函数
   */
  WaypointData() = default;

  /**
   * @brief 带参数的构造函数
   * @param pos 位置
   * @param angle 角度
   * @param drawing 是否在绘图
   * @param line_id 线ID
   */
  WaypointData(const Point3D& pos, double angle, bool drawing = false, int32_t line_id = 0)
    : position(pos), angle(angle), is_drawing(drawing), line_id(line_id)
  {
  }
};

/**
 * @brief 路径段信息结构体
 * 表示一段连续的路径
 */
struct RouteSegment
{
  std::vector<Point3D> points;  ///< 路径点
  RouteType type;               ///< 路径类型
  int32_t line_id;              ///< 相关联的线ID
  std::vector<int32_t> merged_line_ids;  ///< 若发生共线合并，记录所有来源线段ID（包含主ID）
  PrinterType printer_type;     ///< 该路径段使用的打印机类型
  InkMode ink_mode;             ///< 墨水打印模式（默认实线）
  std::string text_content;     ///< 文字内容（仅当 ink_mode == TEXT 时有效）
  bool execute_backward = false;  ///< 是否以后退方式执行（仅对转场路径有意义）

  /**
   * @brief 构造函数
   * @param type 路径类型
   * @param id 线ID
   * @param printer 打印机类型（默认使用中心打印机）
   * @param ink 墨水打印模式（默认实线）
   */
  RouteSegment(RouteType type = RouteType::DRAWING_PATH,
               int32_t id = -1,
               PrinterType printer = PrinterType::CENTER_PRINTER,
               InkMode ink = InkMode::SOLID)
    : type(type), line_id(id), printer_type(printer), ink_mode(ink)
  {
  }
};

/**
 * @brief 线段绘制数据结构体
 * 包含绘制一条线段所需的信息
 */
struct LineDrawingData
{
  Point3D start;                                      ///< 起点
  Point3D end;                                        ///< 终点
  int32_t line_id = 0;                                ///< 线ID
  PrinterType printer_type = PrinterType::CENTER_PRINTER;  ///< 打印机类型

  /**
   * @brief 默认构造函数
   */
  LineDrawingData() = default;

  /**
   * @brief 带参数的构造函数
   * @param start 起点
   * @param end 终点
   * @param line_id 线ID
   * @param printer 打印机类型（默认使用中心打印机）
   */
  LineDrawingData(const Point3D& start, const Point3D& end, int32_t line_id,
                  PrinterType printer = PrinterType::CENTER_PRINTER)
    : start(start), end(end), line_id(line_id), printer_type(printer)
  {
  }
};

/**
 * @brief 线型枚举
 * 定义不同的线型样式
 */
enum class LineStyle
{
  SOLID,   ///< 实线
  DOTTED,  ///< 虚线
  NONE     ///< 无线型
};

/**
 * @brief 执行节点结构体
 * 表示轨迹上的一个执行点
 */
struct ExecutionNode
{
  Point3D position;                              ///< 位置
  double angle = 0.0;                            ///< 角度
  uint32_t move_state = 1;                       ///< 移动状态
  uint32_t work_state = 0;                       ///< 工作状态
  LineStyle left_work_type = LineStyle::NONE;    ///< 左侧打印机线型
  LineStyle right_work_type = LineStyle::NONE;   ///< 右侧打印机线型
  LineStyle center_work_type = LineStyle::NONE;  ///< 中心打印机线型
  std::vector<LineDrawingData> work_data;        ///< 工作数据
  int32_t order = 0;                             ///< 执行顺序

  /**
   * @brief 默认构造函数
   */
  ExecutionNode() = default;
};

/**
 * @brief CAD数据结构体
 * 包含从CAD文件解析出的所有几何数据
 */
struct CADData
{
  // 说明：根据新需求，简化为三类集合，图层名称由JSON中的"layers"决定
  std::vector<Point3D> origin_points;                           ///< 原点坐标集合（若提供）
  std::vector<std::shared_ptr<Line>> path_lines;                ///< 路径（用于实际绘制/规划）
  std::vector<std::shared_ptr<Line>> obstacle_lines;            ///< 障碍物（不可通行边界）
  std::vector<std::shared_ptr<Line>> hole_lines;                ///< 空洞（区域内孔洞边界）
};

/**
 * @brief 栅格地图配置结构体
 * 包含栅格地图的配置参数
 */
struct GridMapConfig
{
  double resolution = 0.05;  ///< 每个栅格的物理尺寸(m)
  double padding = 0.1;      ///< 障碍物填充(m)
  int width = 0;             ///< 地图宽度(栅格数)
  int height = 0;            ///< 地图高度(栅格数)
};

/**
 * @brief 路径规划配置结构体
 * 包含路径规划的参数
 */
struct PathPlannerConfig
{
  double path_extension_start_length = 0.1;  ///< 起点端延长长度(m)，默认0.1米
  double path_extension_end_length = 0.1;    ///< 终点端延长长度(m)，默认0.1米
  double arc_extension_length = 0.4;         ///< 圆弧起点延长弧长(m)，沿圆弧反向延长
  double arc_extension_max_angle = 90.0;     ///< 圆弧起点延长最大角度(度)，防止小半径圆弧延长过多
  double ellipse_extension_length = 0.4;     ///< 椭圆弧起点延长弧长(m)，沿椭圆弧反向延长（完整椭圆不延长）
  double ellipse_extension_max_angle = 90.0; ///< 椭圆弧起点延长最大参数角(度)，防止延长过多
  double transition_length_min = 0.0;        ///< 选择下一条路径时，转场路径长度优先下限(m)
  double transition_length_max = std::numeric_limits<double>::infinity();  ///< 选择下一条路径时，转场路径长度优先上限(m)

  // 几何预处理（REQ-01/REQ-02）
  bool split_polyline = true;           ///< 是否启用 Polyline 拆分
  bool preserve_polyline_info = true;   ///< 是否保留 Polyline 来源信息（LineSegment）
  bool merge_collinear = true;          ///< 是否启用共线合并
  double distance_tolerance = 0.001;    ///< 端点距离容差(米)
  double angle_tolerance = 0.001;       ///< 方向角容差(弧度)
  double min_segment_length = 0.0005;   ///< 最小线段长度(米)，过短线段不参与合并
  double circle_radius_compensation = 0.0;  ///< 圆/圆弧半径补偿(米)，仅对 path_lines 生效（radius += compensation）
};

/**
 * @brief 路径偏移配置结构体
 * 定义各打印机的偏移参数（全局配置）
 */
struct PathOffsetConfig
{
  double left_offset = -0.05;    ///< 左侧打印机偏移距离(m)
  double right_offset = 0.05;    ///< 右侧打印机偏移距离(m)
  double center_offset = 0.0;    ///< 中心打印机偏移距离(m)，默认无偏移
};

/**
 * @brief 终点切线策略枚举
 */
enum class EndpointTangentMode
{
  ALIGN_PATH,     ///< 与下一条路径方向对齐（曲率可能较大）
  ALIGN_STRAIGHT, ///< 与起终点直线方向对齐（曲率更小）
  BLEND           ///< 混合策略
};

/**
 * @brief 贝塞尔曲线转场路径配置结构体
 * 用于配置转场路径的贝塞尔曲线参数
 */
struct BezierTransitionConfig
{
  bool enabled = true;                  ///< 是否启用贝塞尔曲线转场
  double min_curve_distance = 0.5;      ///< 最小曲线转场距离(m)，小于此距离使用直线
  double min_angle_for_curve = 15.0;    ///< 最小曲线转场角度(度)，小于此角度使用直线
  double control_point_ratio = 0.4;     ///< 基础控制点距离比例(0.0-1.0)，相对于起终点距离
  double min_control_distance = 0.1;    ///< 控制点最小距离(m)
  double max_control_distance = 2.0;    ///< 控制点最大距离(m)
  double path_resolution = 0.02;        ///< 路径离散分辨率(m)
  bool use_quintic = false;             ///< 是否使用五次贝塞尔曲线（更平滑）
  EndpointTangentMode endpoint_tangent_mode = EndpointTangentMode::ALIGN_STRAIGHT;  ///< 终点切线策略
  double blend_ratio = 0.5;             ///< 混合比例(0.0=直线,1.0=路径方向)，仅blend模式有效
  bool consider_backward = true;        ///< 是否考虑后退执行

  // 曲率控制参数
  double min_turning_radius = 0.3;      ///< 最小转弯半径(m)，用于限制最大曲率
  bool adaptive_control_point = true;   ///< 是否根据转向角度自适应调整控制点距离
  double large_angle_threshold = 60.0;  ///< 大角度转向阈值(度)
  double large_angle_ratio_boost = 0.3; ///< 大角度时控制点距离增量
};

}  // namespace path_planner
