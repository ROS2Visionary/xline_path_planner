// CAD JSON 解析相关接口
#pragma once

/*
================================================================================
CADParser 负责读取新格式的 CAD JSON（cad_transformed.json），
将几何图元（直线 / 圆 / 圆弧 / 椭圆 / 文字）解析为 CADData，供栅格化和路径规划使用。
支持按图层将图元划分为三类集合（路径 / 障碍物 / 空洞），并在需要时做单位换算
（毫米→米）和角度单位处理。
================================================================================
*/

#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include <unordered_map>

#include "xline_path_planner/common_types.hpp"
#include <nlohmann/json.hpp>

namespace path_planner
{

/**
 * @brief 角度单位类型
 */
enum class AngleUnit
{
  DEGREES,  ///< 角度制
  RADIANS   ///< 弧度制
};

/**
 * @brief CAD解析器配置结构体
 * 定义单位缩放与角度单位等解析参数。
 */
struct CADParserConfig
{
  /**
   * @brief 单位转换因子（原始值→米）
   * 若 auto_scale_coordinates=true，则执行：scaled = raw / unit_conversion_factor。
   * 默认 1000.0（毫米→米）。若原始单位已为米，可设为 1。
   */
  double unit_conversion_factor = 1000.0;

  /**
   * @brief 是否自动缩放坐标/尺寸
   * true：启用单位换算；false：不换算（以原始单位工作）。
   */
  bool auto_scale_coordinates = true;

  /**
   * @brief 角度单位（应用于圆弧等角度字段）
   * 默认按"度"处理；若输入JSON角度已为弧度，请设置为 AngleUnit::RADIANS。
   */
  AngleUnit angle_unit = AngleUnit::DEGREES;
};

/**
 * @brief CAD文件解析器类
 * 支持解析新JSON格式（cad_transformed.json），完整解析根节点"lines"数组中的：
 * - 直线（type: "line"，键：start/end{x,y[,z]}）
 * - 圆（type: "circle"，键：center{x,y[,z]}、radius）
 * - 圆弧（type: "arc"，键：center{x,y[,z]}、radius、start_angle、end_angle）
 * - 椭圆（type: "ellipse"，键：center{x,y[,z]}、major_axis{x,y[,z]}、ratio、start_angle、end_angle、rotation）
 * - 样条（type: "spline"，键：degree、control_points/knots/weights（可选）、vertices（离散点，可选）、is_closed/periodic）
 * - 文字（type: "text"，键：position{x,y[,z]}、content、height、rotation、align）
 * 类别归属由根节点中的"layers"动态决定（优先 layer_id 对应 name，失败回退读取元素内 layer 字段）。
 * 我们根据图层名称关键词将图元分配至CADData的不同容器（路径/障碍物/空洞）。
 * 若无法识别图层类别，则默认归入路径集合（path_lines）。
 * 角度单位自动识别（>2π 视为"度"并转换为弧度）。
 */
class CADParser
{
public:
  /**
   * @brief 构造函数
   * @param config 解析配置。若不提供则采用默认配置（毫米→米）。
   * 注意：CADParser 非线程安全，不建议多线程共享同一实例。
   */
  explicit CADParser(const CADParserConfig& config = CADParserConfig());

  /**
   * @brief 析构函数
   */
  ~CADParser() = default;

  /**
   * @brief 解析CAD文件（新JSON格式）
   * 前置条件：文件为 UTF-8，根键包含数组 "lines"；当存在 "layers" 时用于 layer_id→name 映射。
   * 解析范围：type 为 "line"/"circle"/"arc"/"ellipse"/"spline"/"text" 的元素；其他类型会被忽略。
   * 坐标与尺寸：start/end/center、radius 等数值若启用缩放则统一换算到米。
   * 分类归属：优先使用 layer_id 查表得到图层名称；若失败，回退读取元素内 "layer" 字段。
   * 成功条件：至少解析出一个有效几何图元返回 true；否则返回 false。
   * @param file_path CAD文件路径
   * @return 解析成功返回true，否则返回false
   */
  bool parse(const std::string& file_path);

  /**
   * @brief 从JSON字符串解析CAD数据（无文件I/O）
   * @param json_text 输入的CAD JSON文本
   * @return 解析成功返回true，否则返回false
   */
  bool parse_from_string(const std::string& json_text);

  /**
   * @brief 获取解析后的CAD数据
   * 内容：path_lines / obstacle_lines / hole_lines 三类集合与可扩展的 origin_points。
   * 注意：返回常量引用；若需外部修改，请先复制。
   * @return CAD数据的常量引用
   */
  const CADData& get_cad_data() const;

  /**
   * @brief 清除所有已解析的数据（复位到初始状态）
   */
  void clear();

  /**
   * @brief 设置解析配置
   * 说明：仅影响后续 parse()；不会对已解析数据进行重缩放。
   * @param config 新的解析配置
   */
  void set_config(const CADParserConfig& config);

  /**
   * @brief 获取当前解析配置
   * @return 当前的解析配置的常量引用
   */
  const CADParserConfig& get_config() const;

private:
  /**
   * @brief 从已解析的JSON对象填充CADData
   * @param cad_json 已解析的JSON对象
   * @return 解析成功返回true，否则返回false
   */
  bool parse_from_json_obj(const nlohmann::json& cad_json);
  /**
   * @brief 根据 JSON 根节点构建 layer_id → layer_name 映射
   * 要求：layers 为数组，元素具备 layer_id(int) 与 name(string)。
   * 用途：分类归属时将 lines[*].layer_id 翻译为图层名称；失败时回退读取元素内 layer 字段。
   */
  void build_layer_map(const nlohmann::json& cad_json);

  /**
   * @brief 按图层名称将几何图元写入对应容器
   * 规则：
   *   - 含"空洞/ hole / hollow / void / opening" → hole_lines
   *   - 含"障碍/ barrier / obstacle" → obstacle_lines
   *   - 含"路径/ path / axis / draw / drawing" → path_lines
   *   - 其他未知 → 默认 path_lines
   * @param geom 几何图元（Line/Circle/Arc/Text 均可）
   * @param layer_name 图层名称（可为空串）
   */
  void store_by_layer(const std::shared_ptr<Line>& geom, const std::string& layer_name);

  /**
   * @brief 解析新JSON格式中的直线实体
   * 期望字段：
   * - id: 整数（可选）
   * - type: 字符串，需为"line"（否则忽略）
   * - start: { x: double, y: double[, z: double] }
   * - end:   { x: double, y: double[, z: double] }
   * 处理：缺失 start/end 跳过；z 缺省视为 0。length 由缩放后的起止点距离计算。
   * 其他字段（如 line_type、thickness、color 等）当前不入库。
   * @param json_line JSON对象
   * @return 解析后的直线对象
   */
  Line parse_line(const nlohmann::json& json_line);

  /**
   * @brief 解析多段线（polyline）
   * 期望字段：
   * - id: 整数（可选）
   * - type: 字符串，需为"polyline"
   * - vertices: [ {x,y[,z]}, ... ]（至少2个点）
   * - closed: bool（可选，默认false）
   * 其他字段（line_type、thickness、layer_id、color 等）按 Line 元数据填充。
   */
  Polyline parse_polyline(const nlohmann::json& json_polyline);

  /**
   * @brief 解析曲线数据（暂未使用，预留扩展）
   * 说明：保留该接口以便未来支持 Bézier/NURBS 等曲线类型。
   */
  Curve parse_curve(const nlohmann::json& json_curve);

  /**
   * @brief 解析圆形数据
   * 期望字段：id（可选）、center{x,y[,z]}、radius。
   * 处理：按配置缩放 center 与 radius；length=2πr；start/end 置为 x 轴正向点。
   */
  Circle parse_circle(const nlohmann::json& json_circle);

  /**
   * @brief 解析圆弧数据
   * 期望字段：id（可选）、center{x,y[,z]}、radius、start_angle、end_angle。
   * 处理：按配置缩放 center 与 radius；角度>2π 视为"度"并转弧度；计算起止点与弧长。
   */
  Arc parse_arc(const nlohmann::json& json_arc);

  /**
   * @brief 解析椭圆/椭圆弧数据
   * 期望字段：
   * - id（可选）
   * - center{x,y[,z]}
   * - major_axis{x,y[,z]}（向量，模长为长半轴）
   * - ratio（短长轴比 b/a）
   * - start_angle/end_angle（参数角）
   * - rotation（可选，附加旋转角）
   * 处理：按配置缩放 center 与 major_axis；角度按配置转换为弧度；计算起止点。
   */
  Ellipse parse_ellipse(const nlohmann::json& json_ellipse);

  /**
   * @brief 解析样条（Spline）数据
   * 支持字段：
   * - degree（次数）
   * - control_points/knots/weights（NURBS，可选）
   * - vertices（离散点序列，可选；若提供则规划优先使用）
   * - is_closed/closed、periodic
   */
  Spline parse_spline(const nlohmann::json& json_spline);

  /**
   * @brief 解析文字数据
   * 期望字段：
   * - id: 整数（可选）
   * - type: 字符串，需为"text"
   * - position: { x: double, y: double[, z: double] }
   * - content: 字符串（文字内容）
   * - height: double（文字高度）
   * - rotation: double（旋转角度，度）
   * - align: { horizontal: string, vertical: string }（对齐方式）
   * 处理：按配置缩放 position 与 height；计算文字绘制范围。
   * @param json_text JSON对象
   * @return 解析后的文字对象
   */
  Text parse_text(const nlohmann::json& json_text);

  /**
   * @brief 从JSON对象解析点坐标
   * 支持大小写键名：x/X, y/Y, z/Z（缺省 z=0），并按配置进行单位缩放。
   * @param json_point JSON形式的点坐标
   * @return 解析后的3D点
   */
  Point3D parse_point(const nlohmann::json& json_point);

  /**
   * @brief 对数值进行单位转换
   * 根据配置将原始单位转换为目标单位：
   *   return auto_scale ? (value / unit_conversion_factor) : value;
   * @param value 原始数值
   * @return 转换后的数值
   */
  double convert_units(double value) const;

  CADData cad_data_;        ///< 存储解析后的 CAD 数据（三大集合 + 可扩展 origin）
  CADParserConfig config_;  ///< 解析配置（单位缩放等）
  std::unordered_map<int, std::string> layer_id_to_name_;  ///< 图层ID到名称的映射（分类用）
};

}  // namespace path_planner
