// include/daosnrs_planning/cad_parser.hpp
#pragma once

/*
================================================================================
超详细中文说明（CADParser模块）
--------------------------------------------------------------------------------
本模块用于解析新格式的 CAD JSON（cad_transformed.json），将几何图元（直线/圆/圆弧）
按图层（layers→layer_id）动态归入 CADData 的三类集合（路径/障碍物/空洞），供后续
栅格化与路径规划使用。模块具备单位缩放（毫米→米）与角度单位自动识别（度/弧度）能力。

一、输入 JSON 结构（关键字段）
  - 根键 layers：图层列表，每项需包含 layer_id（int）与 name（string）。
  - 根键 lines：几何图元数组，每项至少包含：
      • id（int，可选）
      • type（string）："line" | "circle" | "arc"
      • layer_id（int，推荐）或 layer（string，备用）
      • 根据 type 的几何字段：
          - line   ：start{x,y[,z]}, end{x,y[,z]}
          - circle ：center{x,y[,z]}, radius
          - arc    ：center{x,y[,z]}, radius, start_angle, end_angle

二、单位与角度
  - 数值单位：通常 CAD 输出为毫米。若 CADParserConfig::auto_scale_coordinates=true，
    所有数值会除以 CADParserConfig::unit_conversion_factor（默认1000），从而换算到米。
  - 圆弧角度：若 |角度值| > 2π 则视为“度”并自动转换为“弧度”，否则认为原本就是弧度。

三、分类规则（图层→集合）
  - 先通过 layer_id → layers.name 建立映射；找不到时回退读取元素内 layer 字段。
  - 名称包含以下关键词时归类：
      • path_lines    ：“路径”、“path”、“axis”、“draw”、“drawing”等
      • obstacle_lines：“障碍”、“barrier”、“obstacle”等
      • hole_lines    ：“空洞”、“hole”、“hollow”、“void”、“opening”等
  - 无法识别则默认归入 path_lines，保证系统可运行。

四、健壮性与失败策略
  - 缺失关键字段（如 line 缺少 start/end）会跳过该元素并继续处理其他元素。
  - parse() 内部捕获异常并返回 false，不向外抛出。
  - 坐标键名大小写兼容（x/X, y/Y, z/Z）。

五、扩展建议
  - 可将分类关键字抽象为配置（例如加入 CADParserConfig 自定义词表）。
  - 新增几何类型时在 parse() 中识别并扩展对应解析与下游处理。

================================================================================
*/

#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include <unordered_map>

#include "xline_path_planner/common_types.hpp"
#include <nlohmann/json.hpp>

namespace daosnrs_planning
{

/**
 * @brief CAD解析器配置结构体
 * 定义单位缩放等解析参数。
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
};

/**
 * @brief CAD文件解析器类
 * 支持解析新JSON格式（cad_transformed.json），完整解析根节点"lines"数组中的：
 * - 直线（type: "line"，键：start/end{x,y[,z]}）
 * - 圆（type: "circle"，键：center{x,y[,z]}、radius）
 * - 圆弧（type: "arc"，键：center{x,y[,z]}、radius、start_angle、end_angle）
 * 类别归属由根节点中的"layers"动态决定（优先 layer_id 对应 name，失败回退读取元素内 layer 字段）。
 * 我们根据图层名称关键词将图元分配至CADData的不同容器（路径/障碍物/空洞）。
 * 若无法识别图层类别，则默认归入路径集合（path_lines）。
 * 角度单位自动识别（>2π 视为“度”并转换为弧度）。
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
   * 解析范围：type 为 "line"/"circle"/"arc" 的元素；其他类型（如 text）会被忽略。
   * 坐标与尺寸：start/end/center、radius 等数值若启用缩放则统一换算到米。
   * 分类归属：优先使用 layer_id 查表得到图层名称；若失败，回退读取元素内 "layer" 字段。
   * 成功条件：至少解析出一个有效几何图元返回 true；否则返回 false。
   * @param file_path CAD文件路径
   * @return 解析成功返回true，否则返回false
   */
  bool parse(const std::string& file_path);

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
   * @brief 根据 JSON 根节点构建 layer_id → layer_name 映射
   * 要求：layers 为数组，元素具备 layer_id(int) 与 name(string)。
   * 用途：分类归属时将 lines[*].layer_id 翻译为图层名称；失败时回退读取元素内 layer 字段。
   */
  void build_layer_map(const nlohmann::json& cad_json);

  /**
   * @brief 按图层名称将几何图元写入对应容器
   * 规则：
   *   - 含“空洞/ hole / hollow / void / opening” → hole_lines
   *   - 含“障碍/ barrier / obstacle” → obstacle_lines
   *   - 含“路径/ path / axis / draw / drawing” → path_lines
   *   - 其他未知 → 默认 path_lines
   * @param geom 几何图元（Line/Circle/Arc 均可）
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
   * 处理：按配置缩放 center 与 radius；角度>2π 视为“度”并转弧度；计算起止点与弧长。
   */
  Arc parse_arc(const nlohmann::json& json_arc);

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

}  // namespace daosnrs_planning
