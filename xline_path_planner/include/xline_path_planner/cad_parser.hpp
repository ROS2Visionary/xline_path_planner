// include/daosnrs_planning/cad_parser.hpp
#pragma once

#include <string>
#include <vector>
#include <memory>
#include <fstream>

#include "daosnrs_planning/common_types.hpp"
#include <nlohmann/json.hpp>

namespace daosnrs_planning
{

/**
 * @brief CAD解析器配置结构体
 * 定义CAD文件解析的配置参数
 */
struct CADParserConfig
{
  /**
   * @brief 单位转换因子，默认将毫米转为米（除以1000）
   * 值为1000表示将毫米单位转换为米
   */
  double unit_conversion_factor = 1000.0;

  /**
   * @brief 是否自动缩放坐标
   * 当为true时，会根据单位转换因子自动缩放坐标
   */
  bool auto_scale_coordinates = true;
};

/**
 * @brief CAD文件解析器类
 * 用于解析CAD文件并提取几何数据
 */
class CADParser
{
public:
  /**
   * @brief 构造函数
   * @param config 解析配置，默认使用默认配置
   */
  explicit CADParser(const CADParserConfig& config = CADParserConfig());

  /**
   * @brief 析构函数
   */
  ~CADParser() = default;

  /**
   * @brief 解析CAD文件
   * @param file_path CAD文件路径
   * @return 解析成功返回true，否则返回false
   */
  bool parse(const std::string& file_path);

  /**
   * @brief 获取解析后的CAD数据
   * @return CAD数据的常量引用
   */
  const CADData& get_cad_data() const;

  /**
   * @brief 清除所有已解析的数据
   * 重置CAD数据到初始状态
   */
  void clear();

  /**
   * @brief 设置解析配置
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
   * @brief 解析直线数据
   * @param json_line JSON形式的直线数据
   * @return 解析后的直线对象
   */
  Line parse_line(const nlohmann::json& json_line);

  /**
   * @brief 解析曲线数据
   * @param json_curve JSON形式的曲线数据
   * @return 解析后的曲线对象
   */
  Curve parse_curve(const nlohmann::json& json_curve);

  /**
   * @brief 解析圆形数据
   * @param json_circle JSON形式的圆形数据
   * @return 解析后的圆形对象
   */
  Circle parse_circle(const nlohmann::json& json_circle);

  /**
   * @brief 解析圆弧数据
   * @param json_arc JSON形式的圆弧数据
   * @return 解析后的圆弧对象
   */
  Arc parse_arc(const nlohmann::json& json_arc);

  /**
   * @brief 从JSON对象解析点坐标
   * @param json_point JSON形式的点坐标
   * @return 解析后的3D点
   */
  Point3D parse_point(const nlohmann::json& json_point);

  /**
   * @brief 对数值进行单位转换
   * 根据配置的转换因子将原始单位转换为目标单位
   * @param value 原始数值
   * @return 转换后的数值
   */
  double convert_units(double value) const;

  CADData cad_data_;        ///< 存储解析后的CAD数据
  CADParserConfig config_;  ///< 解析配置
};

}  // namespace daosnrs_planning