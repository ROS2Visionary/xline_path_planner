// 规划结果导出工具
#pragma once

#include <vector>
#include <string>
#include <memory>
#include <fstream>

#include "xline_path_planner/common_types.hpp"
#include <nlohmann/json.hpp>

namespace path_planner
{

/**
 * @brief 输出格式化器类
 * 用于将轨迹数据转换为JSON格式
 */
class OutputFormatter
{
public:
  OutputFormatter() = default;
  ~OutputFormatter() = default;

  /**
   * @brief 将JSON数据保存到文件
   * @param json_data JSON数据
   * @param file_path 输出文件路径
   * @return 是否成功
   */
  bool save_to_file(const nlohmann::json& json_data, const std::string& file_path);

  /**
   * @brief 将规划好的路径段导出为新版 JSON 的 lines 数组
   * @details 仅返回 { "lines": [...] }，不包含其他顶层字段。
   *          其中对于转场路径（RouteType::TRANSITION_PATH）生成的实体：
   *            - color 统一为浅灰色（#D3D3D3），并附加 opacity=0.5 字段；
   *            - id 统一为 1000000；
   *            - type 统一为 "spline"（vertices 为转场路径离散点序列）；
   *            - selected 统一为 false；
   *            - line_type 统一为空字符串；
   *            - hidden 统一为 false；
   *            - thickness 统一为 1.0；
   *            - layer_id 统一为 1000000；
   *            - layer 统一为 "TRANSITION"。
   *            - backward 为是否后退执行（转场路径由规划阶段计算；其他路径固定为 false）。
   * @param segments 路径段序列（包含绘图与转场）
   * @param source_file 保留参数（不再使用）
   * @return 仅包含 lines 数组的 JSON 对象
   */
  nlohmann::json format_planned_paths_to_cad_json(const std::vector<RouteSegment>& segments,
                                                  const std::string& source_file = "planned_paths");

  /**
   * @brief 同上，但可传入 CADData 以便为绘图路径补充原始元数据
   * @details 仅返回 { "lines": [...] }，lines 顺序与 segments 一致，且包含字段 order=段索引。
   */
  nlohmann::json format_planned_paths_to_cad_json(const std::vector<RouteSegment>& segments, const CADData& cad_data,
                                                  const std::string& source_file = "planned_paths");

private:
  /**
   * @brief 构建一个转场路径的样条 JSON（按照新版导出结构的 lines[*] 条目）
   * @param points 转场路径点序列
   * @param order 顺序
   * @param seg 路径段信息（用于获取 ink 信息）
   * @return JSON 对象
   */
  nlohmann::json constructTransitionSplineJSON(const std::vector<Point3D>& points, int order, const RouteSegment& seg);
};

}  // namespace path_planner
