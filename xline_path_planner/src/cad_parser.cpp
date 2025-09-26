#include "daosnrs_planning/cad_parser.hpp"
#include <iostream>

namespace daosnrs_planning
{

CADParser::CADParser(const CADParserConfig& config) : config_(config)
{
}

bool CADParser::parse(const std::string& file_path)
{
  try
  {
    // 打开并读取JSON文件
    std::ifstream file(file_path);
    if (!file.is_open())
    {
      std::cerr << "Failed to open CAD file: " << file_path << std::endl;
      return false;
    }

    // 解析JSON
    nlohmann::json cad_json;
    file >> cad_json;
    file.close();

    // 清除之前的数据
    clear();

    // 解析地图ID
    if (cad_json.contains("mapId"))
    {
      cad_data_.map_id = cad_json["mapId"];
    }

    // 解析原点坐标
    if (cad_json.contains("origin") && cad_json["origin"].is_array())
    {
      for (const auto& point : cad_json["origin"])
      {
        cad_data_.origin_points.push_back(parse_point(point));
      }
    }

    // 解析轴线
    if (cad_json.contains("axisLines") && cad_json["axisLines"].is_array())
    {
      std::cout << "Processing " << cad_json["axisLines"].size() << " axis lines" << std::endl;

      for (const auto& line : cad_json["axisLines"])
      {
        if (line.contains("type"))
        {
          int type = line["type"];
          std::cout << "Processing line with type: " << type
                    << ", ID: " << (line.contains("lineId") ? line["lineId"].get<int>() : -1) << std::endl;

          if (type == 1)
          {
            cad_data_.axis_lines.push_back(std::make_shared<Line>(parse_line(line)));
          }
          else if (type == 4)
          {
            cad_data_.axis_lines.push_back(std::make_shared<Circle>(parse_circle(line)));
          }
          else if (type == 6)
          {
            cad_data_.axis_lines.push_back(std::make_shared<Curve>(parse_curve(line)));
          }
          else if (type == 7)
          {
            cad_data_.axis_lines.push_back(std::make_shared<Arc>(parse_arc(line)));
          }
          else
          {
            std::cerr << "Unrecognized line type: " << type << std::endl;
          }
        }
      }
    }

    // 解析墙线
    if (cad_json.contains("wallLines") && cad_json["wallLines"].is_array())
    {
      for (const auto& line : cad_json["wallLines"])
      {
        if (line.contains("type"))
        {
          int type = line["type"];
          if (type == 1)
          {
            cad_data_.wall_lines.push_back(std::make_shared<Line>(parse_line(line)));
          }
          else if (type == 4)
          {
            cad_data_.wall_lines.push_back(std::make_shared<Circle>(parse_circle(line)));
          }
          else if (type == 6)
          {
            cad_data_.wall_lines.push_back(std::make_shared<Curve>(parse_curve(line)));
          }
          else if (type == 7)
          {
            cad_data_.wall_lines.push_back(std::make_shared<Arc>(parse_arc(line)));
          }
        }
      }
    }

    // 解析控制线
    if (cad_json.contains("controlLines") && cad_json["controlLines"].is_array())
    {
      for (const auto& line : cad_json["controlLines"])
      {
        if (line.contains("type"))
        {
          int type = line["type"];
          if (type == 1)
          {
            cad_data_.control_lines.push_back(std::make_shared<Line>(parse_line(line)));
          }
          else if (type == 4)
          {
            cad_data_.control_lines.push_back(std::make_shared<Circle>(parse_circle(line)));
          }
          else if (type == 6)
          {
            cad_data_.control_lines.push_back(std::make_shared<Curve>(parse_curve(line)));
          }
          else if (type == 7)
          {
            cad_data_.control_lines.push_back(std::make_shared<Arc>(parse_arc(line)));
          }
        }
      }
    }

    // 解析十字线
    if (cad_json.contains("crossLines") && cad_json["crossLines"].is_array())
    {
      for (const auto& line : cad_json["crossLines"])
      {
        if (line.contains("type"))
        {
          int type = line["type"];
          if (type == 1)
          {
            cad_data_.cross_lines.push_back(std::make_shared<Line>(parse_line(line)));
          }
          else if (type == 4)
          {
            cad_data_.cross_lines.push_back(std::make_shared<Circle>(parse_circle(line)));
          }
          else if (type == 6)
          {
            cad_data_.cross_lines.push_back(std::make_shared<Curve>(parse_curve(line)));
          }
          else if (type == 7)
          {
            cad_data_.cross_lines.push_back(std::make_shared<Arc>(parse_arc(line)));
          }
        }
      }
    }

    // 解析障碍物线
    if (cad_json.contains("barrierLines") && cad_json["barrierLines"].is_array())
    {
      for (const auto& line : cad_json["barrierLines"])
      {
        if (line.contains("type"))
        {
          int type = line["type"];
          if (type == 1)
          {
            cad_data_.barrier_lines.push_back(std::make_shared<Line>(parse_line(line)));
          }
          else if (type == 4)
          {
            cad_data_.barrier_lines.push_back(std::make_shared<Circle>(parse_circle(line)));
          }
          else if (type == 6)
          {
            cad_data_.barrier_lines.push_back(std::make_shared<Curve>(parse_curve(line)));
          }
          else if (type == 7)
          {
            cad_data_.barrier_lines.push_back(std::make_shared<Arc>(parse_arc(line)));
          }
        }
      }
    }

    // 解析低障碍物线
    if (cad_json.contains("lowbarrierLines") && cad_json["lowbarrierLines"].is_array())
    {
      for (const auto& line : cad_json["lowbarrierLines"])
      {
        if (line.contains("type"))
        {
          int type = line["type"];
          if (type == 1)
          {
            cad_data_.low_barrier_lines.push_back(std::make_shared<Line>(parse_line(line)));
          }
          else if (type == 4)
          {
            cad_data_.low_barrier_lines.push_back(std::make_shared<Circle>(parse_circle(line)));
          }
          else if (type == 6)
          {
            cad_data_.low_barrier_lines.push_back(std::make_shared<Curve>(parse_curve(line)));
          }
          else if (type == 7)
          {
            cad_data_.low_barrier_lines.push_back(std::make_shared<Arc>(parse_arc(line)));
          }
        }
      }
    }

    return true;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error parsing CAD file: " << e.what() << std::endl;
    return false;
  }
}

const CADData& CADParser::get_cad_data() const
{
  return cad_data_;
}

void CADParser::clear()
{
  cad_data_ = CADData();
}

void CADParser::set_config(const CADParserConfig& config)
{
  config_ = config;
}

const CADParserConfig& CADParser::get_config() const
{
  return config_;
}

double CADParser::convert_units(double value) const
{
  if (config_.auto_scale_coordinates)
  {
    return value / config_.unit_conversion_factor;
  }
  return value;
}

Line CADParser::parse_line(const nlohmann::json& json_line)
{
  Line line;

  if (json_line.contains("lineId"))
  {
    line.id = json_line["lineId"];
  }

  if (json_line.contains("type"))
  {
    line.type = static_cast<GeometryType>(json_line["type"].get<int>());
  }

  if (json_line.contains("hasPrinted"))
  {
    line.is_printed = json_line["hasPrinted"];
  }

  if (json_line.contains("Length"))
  {
    double length_value = json_line["Length"].get<double>();
    line.length = convert_units(length_value);
  }

  // 解析直线的起点和终点
  if (json_line.contains("Origin") && json_line.contains("Direction") && json_line.contains("Length"))
  {
    Point3D origin = parse_point(json_line["Origin"]);
    line.start = origin;

    // 提取方向向量
    double dx = 0.0, dy = 0.0, dz = 0.0;
    if (json_line["Direction"].contains("X"))
    {
      dx = json_line["Direction"]["X"];
    }
    if (json_line["Direction"].contains("Y"))
    {
      dy = json_line["Direction"]["Y"];
    }
    if (json_line["Direction"].contains("Z"))
    {
      dz = json_line["Direction"]["Z"];
    }

    // 计算终点 - 注意：这里使用原始长度计算，然后再进行单位转换
    double length = json_line["Length"].get<double>();
    double scaled_length = convert_units(length);

    // 使用方向向量和缩放后的长度计算终点
    line.end.x = origin.x + dx * scaled_length;
    line.end.y = origin.y + dy * scaled_length;
    line.end.z = origin.z + dz * scaled_length;
  }

  return line;
}

Curve CADParser::parse_curve(const nlohmann::json& json_curve)
{
  Curve curve;

  if (json_curve.contains("lineId"))
  {
    curve.id = json_curve["lineId"];
  }

  if (json_curve.contains("type"))
  {
    curve.type = static_cast<GeometryType>(json_curve["type"].get<int>());
  }

  // 默认设置为未绘制
  curve.is_printed = false;

  // 明确检查hasPrinted字段
  if (json_curve.contains("hasPrinted") && json_curve["hasPrinted"].is_boolean())
  {
    curve.is_printed = json_curve["hasPrinted"].get<bool>();
  }

  if (json_curve.contains("Length"))
  {
    double length_value = json_curve["Length"].get<double>();
    curve.length = convert_units(length_value);
  }

  if (json_curve.contains("Degree"))
  {
    curve.degree = json_curve["Degree"];
  }

  // 解析控制点
  if (json_curve.contains("ControlPoints") && json_curve["ControlPoints"].is_array())
  {
    for (const auto& point : json_curve["ControlPoints"])
    {
      curve.control_points.push_back(parse_point(point));
    }

    // 设置起点和终点
    if (!curve.control_points.empty())
    {
      curve.start = curve.control_points.front();
      curve.end = curve.control_points.back();
    }
  }

  // 解析权重
  if (json_curve.contains("Weights") && json_curve["Weights"].is_array())
  {
    for (const auto& weight : json_curve["Weights"])
    {
      curve.weights.push_back(weight);
    }
  }

  // 解析节点
  if (json_curve.contains("Knots") && json_curve["Knots"].is_array())
  {
    for (const auto& knot : json_curve["Knots"])
    {
      curve.knots.push_back(knot);
    }
  }

  std::cout << "Parsed curve with ID: " << curve.id << ", degree: " << curve.degree
            << ", control points: " << curve.control_points.size() << ", weights: " << curve.weights.size()
            << ", knots: " << curve.knots.size() << ", is_printed: " << (curve.is_printed ? "true" : "false")
            << std::endl;

  return curve;
}

Circle CADParser::parse_circle(const nlohmann::json& json_circle)
{
  Circle circle;

  if (json_circle.contains("lineId"))
  {
    circle.id = json_circle["lineId"];
  }

  if (json_circle.contains("type"))
  {
    circle.type = static_cast<GeometryType>(json_circle["type"].get<int>());
  }

  if (json_circle.contains("hasPrinted"))
  {
    circle.is_printed = json_circle["hasPrinted"];
  }

  // 解析圆心
  if (json_circle.contains("Center"))
  {
    circle.center = parse_point(json_circle["Center"]);
  }

  // 解析半径
  if (json_circle.contains("Radius"))
  {
    double radius_value = json_circle["Radius"].get<double>();
    circle.radius = convert_units(radius_value);

    // 更新圆周长
    circle.length = 2.0 * M_PI * circle.radius;

    // 设置起点和终点
    circle.start.x = circle.center.x + circle.radius;
    circle.start.y = circle.center.y;
    circle.start.z = circle.center.z;
    circle.end = circle.start;  // 圆是闭合的，起点=终点
  }

  return circle;
}

Arc CADParser::parse_arc(const nlohmann::json& json_arc)
{
  Arc arc;

  if (json_arc.contains("lineId"))
  {
    arc.id = json_arc["lineId"];
  }

  if (json_arc.contains("type"))
  {
    arc.type = static_cast<GeometryType>(json_arc["type"].get<int>());
  }

  if (json_arc.contains("hasPrinted"))
  {
    arc.is_printed = json_arc["hasPrinted"];
  }

  // 解析圆心
  if (json_arc.contains("Center"))
  {
    arc.center = parse_point(json_arc["Center"]);
  }

  // 解析半径
  if (json_arc.contains("Radius"))
  {
    double radius_value = json_arc["Radius"].get<double>();
    arc.radius = convert_units(radius_value);
  }

  // 解析起始角度
  if (json_arc.contains("StartAngle") && !json_arc["StartAngle"].is_null())
  {
    arc.start_angle = json_arc["StartAngle"].get<double>();
  }
  else
  {
    arc.start_angle = 0.0;
  }

  // 解析结束角度
  if (json_arc.contains("EndAngle") && !json_arc["EndAngle"].is_null())
  {
    arc.end_angle = json_arc["EndAngle"].get<double>();
  }
  else
  {
    arc.end_angle = 2.0 * M_PI;
  }

  // 计算起点和终点
  arc.start.x = arc.center.x + arc.radius * std::cos(arc.start_angle);
  arc.start.y = arc.center.y + arc.radius * std::sin(arc.start_angle);
  arc.start.z = arc.center.z;

  arc.end.x = arc.center.x + arc.radius * std::cos(arc.end_angle);
  arc.end.y = arc.center.y + arc.radius * std::sin(arc.end_angle);
  arc.end.z = arc.center.z;

  // 计算圆弧长度
  double angle_diff = arc.end_angle - arc.start_angle;
  if (angle_diff < 0)
  {
    angle_diff += 2.0 * M_PI;  // 确保角度差为正
  }
  arc.length = arc.radius * angle_diff;

  return arc;
}

Point3D CADParser::parse_point(const nlohmann::json& json_point)
{
  Point3D point;

  if (json_point.contains("X"))
  {
    double x_value = json_point["X"].get<double>();
    point.x = convert_units(x_value);
  }

  if (json_point.contains("Y"))
  {
    double y_value = json_point["Y"].get<double>();
    point.y = convert_units(y_value);
  }

  if (json_point.contains("Z"))
  {
    double z_value = json_point["Z"].get<double>();
    point.z = convert_units(z_value);
  }

  return point;
}

}  // namespace daosnrs_planning