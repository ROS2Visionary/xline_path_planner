// include/daosnrs_planning/output_formatter.hpp
#pragma once

#include <vector>
#include <string>
#include <memory>
#include <fstream>

#include "daosnrs_planning/common_types.hpp"
#include <nlohmann/json.hpp>

namespace daosnrs_planning
{

/**
 * @brief 输出格式化器类
 * 用于将轨迹数据转换为JSON格式
 */
class OutputFormatter
{
public:
  /**
   * @brief 默认构造函数
   */
  OutputFormatter() = default;

  /**
   * @brief 析构函数
   */
  ~OutputFormatter() = default;

  /**
   * @brief 将绘图工作点序列转换为JSON格式
   * @param trajectory 绘图工作点序列
   * @param device_code 设备代码
   * @param device_ip 设备IP
   * @param map_id 地图ID
   * @param msg_id 消息ID
   * @param task_id 任务ID
   * @return JSON格式的轨迹数据
   */
  nlohmann::json format_trajectory(const std::vector<ExecutionNode>& trajectory,
                                   const std::string& device_code = "Dev001",
                                   const std::string& device_ip = "192.168.0.1", const std::string& map_id = "1",
                                   const std::string& msg_id = "Msg001", const std::string& task_id = "100");

  /**
   * @brief 将JSON数据保存到文件
   * @param json_data JSON数据
   * @param file_path 输出文件路径
   * @return 是否成功
   */
  bool save_to_file(const nlohmann::json& json_data, const std::string& file_path);

  /**
   * @brief 将工作数据转换为JSON字符串
   * @param work_data 工作数据
   * @param printer_type 打印机类型
   * @return JSON字符串
   */
  std::string convertDrawingDataToJSONString(const LineDrawingData& work_data, PrinterType printer_type);

private:
  /**
   * @brief 构建轨迹点JSON对象
   * @param point 绘图工作点
   * @param order 序号
   * @return JSON对象
   */
  nlohmann::json constructExecutionPointJSON(const ExecutionNode& point, int order);

  /**
   * @brief 构建工作数据JSON对象
   * @param work_data 工作数据
   * @param printer_type 打印机类型
   * @return JSON对象
   */
  nlohmann::json constructDrawingDataJSON(const LineDrawingData& work_data, PrinterType printer_type);

  /**
   * @brief 获取打印机类型字符串
   * @param printer_type 打印机类型
   * @return 打印机类型字符串
   */
  std::string getPrinterTypeString(PrinterType printer_type);

  /**
   * @brief 获取线型字符串
   * @param drawing_type 线型
   * @return 线型字符串
   */
  std::string getLineStyleString(LineStyle drawing_type);
};

}  // namespace daosnrs_planning