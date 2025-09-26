#include "xline_path_planner/output_formatter.hpp"
#include <iostream>

namespace daosnrs_planning
{

nlohmann::json OutputFormatter::format_trajectory(const std::vector<ExecutionNode>& trajectory,
                                                  const std::string& device_code, const std::string& device_ip,
                                                  const std::string& map_id, const std::string& msg_id,
                                                  const std::string& task_id)
{
  nlohmann::json output;

  // 创建JSON数组
  output = nlohmann::json::array();

  // 创建轨迹对象
  nlohmann::json trajectory_obj;

  // 添加设备信息
  trajectory_obj["DeviceCode"] = device_code;
  trajectory_obj["DeviceIP"] = device_ip;
  trajectory_obj["MapID"] = map_id;
  trajectory_obj["MsgID"] = msg_id;

  // 创建任务数据对象
  nlohmann::json task_data;
  task_data["TaskID"] = task_id;
  task_data["navigationLoopNumber"] = 1;
  task_data["realLoop"] = 0;

  // 创建路径数据对象
  nlohmann::json route_data;
  route_data["BIMMapCode"] = "default_task";
  route_data["Name"] = "路径1";
  route_data["NumOfPoints"] = trajectory.size();

  // 创建放样洞口信息
  nlohmann::json extend_field;
  nlohmann::json payoff_hole = nlohmann::json::array();

  // 添加3个放样洞口
  for (int i = 1; i <= 3; ++i)
  {
    nlohmann::json hole;
    hole["Name"] = i;
    hole["X"] = 0.0;
    hole["Y"] = 0.0;
    hole["Z"] = 0.0;

    payoff_hole.push_back(hole);
  }

  extend_field["PayoffHole"] = payoff_hole;
  route_data["ExtendField"] = extend_field;

  // 创建原点信息（第一个点）
  if (!trajectory.empty())
  {
    route_data["OrgPt"] = constructExecutionPointJSON(trajectory[0], 0);
  }

  // 创建路径点数组
  nlohmann::json route_points = nlohmann::json::array();

  for (size_t i = 0; i < trajectory.size(); ++i)
  {
    route_points.push_back(constructExecutionPointJSON(trajectory[i], i + 1));
  }

  route_data["RoutePts"] = route_points;
  task_data["RouteData"] = route_data;
  trajectory_obj["TaskData"] = task_data;

  output.push_back(trajectory_obj);

  return output;
}

bool OutputFormatter::save_to_file(const nlohmann::json& json_data, const std::string& file_path)
{
  try
  {
    std::ofstream file(file_path);
    if (!file.is_open())
    {
      std::cerr << "Failed to open output file: " << file_path << std::endl;
      return false;
    }

    file << json_data.dump(2);  // 缩进2个空格
    file.close();

    return true;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error saving JSON to file: " << e.what() << std::endl;
    return false;
  }
}

std::string OutputFormatter::convertDrawingDataToJSONString(const LineDrawingData& work_data, PrinterType printer_type)
{
  nlohmann::json work_data_json = constructDrawingDataJSON(work_data, printer_type);
  return work_data_json.dump();
}

nlohmann::json OutputFormatter::constructExecutionPointJSON(const ExecutionNode& point, int order)
{
  nlohmann::json route_point;

  route_point["Angle"] = point.angle;
  route_point["MoveState"] = point.move_state;
  route_point["Order"] = order;

  // 工作数据
  nlohmann::json work_data_array = nlohmann::json::array();

  if (point.work_state == 1 && !point.work_data.empty())
  {
    // 确定使用的打印机类型
    PrinterType printer_type;
    if (point.left_work_type != LineStyle::NONE && point.right_work_type != LineStyle::NONE)
    {
      printer_type = PrinterType::BOTH_PRINTERS;
    }
    else if (point.left_work_type != LineStyle::NONE)
    {
      printer_type = PrinterType::LEFT_PRINTER;
    }
    else
    {
      printer_type = PrinterType::RIGHT_PRINTER;
    }

    // 将工作数据转换为JSON
    for (const auto& data : point.work_data)
    {
      work_data_array.push_back(convertDrawingDataToJSONString(data, printer_type));
    }
  }
  else
  {
    // 如果不是工作点，添加空的工作数据
    work_data_array.push_back("{}");
  }

  route_point["WorkData"] = work_data_array;
  route_point["WorkState"] = point.work_state;
  route_point["X"] = point.position.x;
  route_point["Y"] = point.position.y;
  route_point["Z"] = point.position.z;

  // 添加默认的移动参数
  route_point["angular_rate_max"] = -1;
  route_point["controlMode"] = 0;
  route_point["control_mode"] = 0;
  route_point["curve_mode"] = -1;
  route_point["end_angle"] = 360;
  route_point["end_speed"] = 0;
  route_point["is_backwards"] = 1;
  route_point["is_indoor"] = 1;
  route_point["is_use_local_plan"] = 1;
  route_point["linear_vel_max"] = point.work_state == 1 ? 400.0 : 500.0;  // 工作时速度慢点
  route_point["moveSpeed"] = 0;
  route_point["move_dir"] = -1;
  route_point["road_finding_mode"] = -1;
  route_point["run_mode"] = 0;

  return route_point;
}

nlohmann::json OutputFormatter::constructDrawingDataJSON(const LineDrawingData& work_data, PrinterType printer_type)
{
  nlohmann::json json_obj;

  // 创建工作信息对象
  nlohmann::json work_info;

  // 设置打印机工作类型
  if (printer_type == PrinterType::LEFT_PRINTER || printer_type == PrinterType::BOTH_PRINTERS)
  {
    work_info["leftWorkType"] = "SOLID";  // 默认使用实线
  }

  if (printer_type == PrinterType::RIGHT_PRINTER || printer_type == PrinterType::BOTH_PRINTERS)
  {
    work_info["rightWorkType"] = "SOLID";  // 默认使用实线
  }

  json_obj["workInfo"] = work_info;

  // 创建线段数组
  nlohmann::json lines = nlohmann::json::array();

  // 添加线段信息
  nlohmann::json line_obj;
  line_obj["lineIds"] = nlohmann::json::array({ work_data.line_id });

  // 设置参考打印机
  if (work_data.is_left_printer)
  {
    line_obj["reference"] = "LEFTPRINTER";
  }
  else
  {
    line_obj["reference"] = "RIGHTPRINTER";
  }

  // 添加线段坐标
  nlohmann::json line_info;

  nlohmann::json origin;
  origin["x"] = work_data.start.x;
  origin["y"] = work_data.start.y;

  nlohmann::json end;
  end["x"] = work_data.end.x;
  end["y"] = work_data.end.y;

  line_info["origin"] = origin;
  line_info["end"] = end;

  line_obj["lineInfo"] = line_info;
  lines.push_back(line_obj);

  json_obj["lines"] = lines;

  return json_obj;
}

std::string OutputFormatter::getPrinterTypeString(PrinterType printer_type)
{
  switch (printer_type)
  {
    case PrinterType::LEFT_PRINTER:
      return "LEFTPRINTER";
    case PrinterType::RIGHT_PRINTER:
      return "RIGHTPRINTER";
    case PrinterType::BOTH_PRINTERS:
      return "BOTHPRINTERS";
    default:
      return "LEFTPRINTER";
  }
}

std::string OutputFormatter::getLineStyleString(LineStyle drawing_type)
{
  switch (drawing_type)
  {
    case LineStyle::SOLID:
      return "SOLID";
    case LineStyle::DOTTED:
      return "DOTTED";
    case LineStyle::NONE:
      return "NONE";
    default:
      return "SOLID";
  }
}

}  // namespace daosnrs_planning