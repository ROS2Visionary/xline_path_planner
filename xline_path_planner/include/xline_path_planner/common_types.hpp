#pragma once

#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <cstdint>
#include <stdexcept>

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
  LINE = 1,    ///< 直线
  CIRCLE = 4,  ///< 圆
  CURVE = 6,   ///< 曲线
  ARC = 7      ///< 圆弧
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
 * @brief 几何实体基类（直线为核心，圆/弧继承）
 * 说明：
 *  - 与新JSON格式对齐，新增若干可选元数据字段（line_type/thickness/hidden/layer/layer_id/color/selected）。
 *  - start/end/length 在直线中直接来源于JSON，在圆/弧中按中心/半径/角度推导。
 */
struct Line
{
  // 基本标识
  int32_t id = 0;                          ///< 实体ID（来自JSON的id）
  GeometryType type = GeometryType::LINE;  ///< 几何类型（Line/Circle/Curve/Arc）

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
  double speed;                 ///< 建议速度

  /**
   * @brief 构造函数
   * @param type 路径类型
   * @param id 线ID
   */
  RouteSegment(RouteType type = RouteType::DRAWING_PATH, int32_t id = -1) : type(type), line_id(id), speed(0.3)
  {
  }
};

/**
 * @brief 线段绘制数据结构体
 * 包含绘制一条线段所需的信息
 */
struct LineDrawingData
{
  Point3D start;                ///< 起点
  Point3D end;                  ///< 终点
  int32_t line_id = 0;          ///< 线ID
  bool is_left_printer = true;  ///< 是否使用左侧打印机

  /**
   * @brief 默认构造函数
   */
  LineDrawingData() = default;

  /**
   * @brief 带参数的构造函数
   * @param start 起点
   * @param end 终点
   * @param line_id 线ID
   * @param is_left 是否使用左侧打印机
   */
  LineDrawingData(const Point3D& start, const Point3D& end, int32_t line_id, bool is_left = true)
    : start(start), end(end), line_id(line_id), is_left_printer(is_left)
  {
  }
};

/**
 * @brief 打印机类型枚举
 * 定义不同的打印机类型
 */
enum class PrinterType
{
  LEFT_PRINTER,   ///< 左侧打印机
  RIGHT_PRINTER,  ///< 右侧打印机
  BOTH_PRINTERS   ///< 双打印机
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
  Point3D position;                             ///< 位置
  double angle = 0.0;                           ///< 角度
  uint32_t move_state = 1;                      ///< 移动状态
  uint32_t work_state = 0;                      ///< 工作状态
  LineStyle left_work_type = LineStyle::NONE;   ///< 左侧打印机线型
  LineStyle right_work_type = LineStyle::NONE;  ///< 右侧打印机线型
  std::vector<LineDrawingData> work_data;       ///< 工作数据
  int32_t order = 0;                            ///< 执行顺序

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
  bool allow_diagonal = true;              ///< 是否允许对角线移动
  double heuristic_weight = 1.0;           ///< 启发式函数权重
  int max_iterations = 10000;              ///< 最大迭代次数
  double path_simplify_tolerance = 0.1;    ///< 路径简化容差
  double angle_tolerance = 15.0;           ///< 角度容差（度）
  double line_of_sight_resolution = 0.05;  ///< 视线算法分辨率
  bool use_theta_star = true;              ///< 是否使用Theta*算法
  double transition_speed = 0.5;           ///< 转场速度(m/s)
  double drawing_speed = 0.3;              ///< 绘图速度(m/s)
  bool smooth_paths = true;                ///< 是否平滑路径
  int smoothing_iterations = 5;            ///< 平滑迭代次数
  double smoothing_weight = 0.5;           ///< 平滑权重
  double path_extension_length = 0.1;      ///< 路径延长长度(m)，默认延长0.1米
};

/**
 * @brief 路径偏移配置结构体
 * 定义路径生成时的偏移参数
 */
struct PathOffsetConfig
{
  double left_offset = -0.2;                             ///< 左偏移距离(m)
  double right_offset = 0.2;                             ///< 右偏移距离(m)
  PrinterType printer_type = PrinterType::LEFT_PRINTER;  ///< 使用的打印机类型
};

/**
 * @brief 轨迹生成配置结构体
 * 包含轨迹生成的参数
 */
struct TrajectoryConfig
{
  double max_velocity = 0.5;      ///< 最大速度(m/s)
  double max_acceleration = 0.5;  ///< 最大加速度(m/s^2)
  double angle_velocity = 0.5;    ///< 角速度(rad/s)
};

}  // namespace path_planner
