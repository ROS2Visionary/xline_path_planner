/*
================================================================================
CAD 解析实现（超详细中文注释）
--------------------------------------------------------------------------------
目标与能力
  - 解析新一代 CAD JSON（cad_transformed.json）中的几何数据，处理：
    • 直线（type: "line"）
    • 圆   （type: "circle"）
    • 圆弧（type: "arc"）
    • 文字（type: "text"）
  - 动态分类：依据 lines[*].layer_id → layers[*].name 将图元归入 CADData 三大集合：
    • path_lines（路径）
    • obstacle_lines（障碍物）
    • hole_lines（空洞）
  - 单位换算：若启用，则将毫米等原始单位转换为米（默认 /1000）。
  - 角度单位自动识别：|value| > 2π 视为"度"并转换为弧度。

整体流程
  1) 读取与解析 JSON（含健壮性检查）
  2) 构建 layer_id → layer_name 映射（若有）
  3) 遍历 lines：解析四类几何 → 分类入 CADData
  4) 输出解析统计并返回成功与否

健壮性与兼容
  - 缺失关键字段（如 line 缺失 start/end）→ 跳过该元素但不中断流程
  - 所有异常在 parse() 内部捕获，返回 false，不向外抛出
  - 坐标键名 x/X, y/Y, z/Z 兼容

扩展位点
  - store_by_layer 中的关键词可改为配置化（例如注入词表）
  - 可按需新增其它几何类型的解析与下游处理
================================================================================
*/
#include "xline_path_planner/cad_parser.hpp"
#include <iostream>
#include <algorithm>
#include <cctype>

namespace path_planner
{

CADParser::CADParser(const CADParserConfig& config) : config_(config)
{
}

/*
解析流程总览（parse）
--------------------------------
输入：
  - file_path：CAD JSON 文件路径（UTF-8）。
行为：
  1) 打开并解析 JSON；
  2) 清空内部数据（clear）；
  3) 构建图层映射（build_layer_map）；
  4) 遍历 lines：按 type 调用 parse_line/parse_circle/parse_arc/parse_text 解析；
  5) 根据图层名称（由 layer_id → name 或元素内 layer）分类（store_by_layer）；
  6) 打印解析统计，成功返回 true（至少解析到一个几何），否则返回 false。
健壮性：
  - 缺失关键字段的元素跳过处理；
  - 任何异常均被捕获并返回 false。
*/
bool CADParser::parse(const std::string& file_path)
{
  try
  {
    std::ifstream file(file_path);
    if (!file.is_open())
    {
      std::cerr << "Failed to open CAD file: " << file_path << std::endl;
      return false;
    }
    nlohmann::json cad_json;
    file >> cad_json;
    file.close();
    return parse_from_json_obj(cad_json);
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error parsing CAD file: " << e.what() << std::endl;
    return false;
  }
}

bool CADParser::parse_from_string(const std::string& json_text)
{
  try
  {
    nlohmann::json cad_json = nlohmann::json::parse(json_text);
    return parse_from_json_obj(cad_json);
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error parsing CAD JSON text: " << e.what() << std::endl;
    return false;
  }
}

bool CADParser::parse_from_json_obj(const nlohmann::json& cad_json)
{
  try
  {
    // 清除之前的数据
    clear();
    // 预先构建 layer_id -> layer_name 映射，供分类用
    build_layer_map(cad_json);

    // 新格式：仅解析根节点"lines"，支持 line/circle/arc/text 四类
    if (!cad_json.contains("lines") || !cad_json["lines"].is_array())
    {
      std::cerr << "Invalid CAD JSON: missing 'lines' array" << std::endl;
      return false;
    }

    const auto& lines = cad_json["lines"];
    size_t parsed_lines = 0, parsed_arcs = 0, parsed_circles = 0, parsed_texts = 0;
    for (const auto& item : lines)
    {
      // 若存在 selected 且为 false，则舍弃该元素
      if (item.contains("selected") && item["selected"].is_boolean() && !item["selected"].get<bool>())
      {
        continue;
      }
      std::string t = "line";
      if (item.contains("type") && item["type"].is_string())
      {
        t = item["type"].get<std::string>();
        for (auto& c : t)
          c = static_cast<char>(::tolower(c));
      }

      // 提取图层名称（使用layer_id映射）
      std::string layer_name;
      if (item.contains("layer_id") && (item["layer_id"].is_number_integer()))
      {
        int lid = item["layer_id"].get<int>();
        auto it = layer_id_to_name_.find(lid);
        if (it != layer_id_to_name_.end())
        {
          layer_name = it->second;
        }
      }
      if (layer_name.empty() && item.contains("layer") && item["layer"].is_string())
      {
        layer_name = item["layer"].get<std::string>();
      }

      if (t == "line")
      {
        if (!item.contains("start") || !item.contains("end"))
        {
          continue;
        }
        if (!item["start"].is_object() || !item["end"].is_object())
        {
          continue;
        }
        auto line_ptr = std::make_shared<Line>(parse_line(item));
        store_by_layer(line_ptr, layer_name);
        ++parsed_lines;
      }
      else if (t == "circle")
      {
        if (!item.contains("center") || !item.contains("radius"))
        {
          continue;
        }
        if (!item["center"].is_object())
        {
          continue;
        }
        auto circle_ptr = std::make_shared<Circle>(parse_circle(item));
        store_by_layer(circle_ptr, layer_name);
        ++parsed_circles;
      }
      else if (t == "arc")
      {
        if (!item.contains("center") || !item.contains("radius"))
        {
          continue;
        }
        if (!item["center"].is_object())
        {
          continue;
        }
        auto arc_ptr = std::make_shared<Arc>(parse_arc(item));
        store_by_layer(arc_ptr, layer_name);
        ++parsed_arcs;
      }
      else if (t == "text")
      {
        // 解析文字类型
        if (!item.contains("position"))
        {
          continue;
        }
        if (!item["position"].is_object())
        {
          continue;
        }
        auto text_ptr = std::make_shared<Text>(parse_text(item));
        store_by_layer(text_ptr, layer_name);
        ++parsed_texts;
      }
      else
      {
        // 其他类型忽略
        continue;
      }
    }

    std::cout << "Parsed from 'lines' (by layers): " << parsed_lines << " line(s), " << parsed_circles << " circle(s), "
              << parsed_arcs << " arc(s), " << parsed_texts << " text(s)" << std::endl;

    return (parsed_lines + parsed_arcs + parsed_circles + parsed_texts) > 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error parsing CAD JSON object: " << e.what() << std::endl;
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

/*
单位换算（convert_units）
--------------------------------
用途：
  - 当 auto_scale_coordinates=true 时，将原始数值按 unit_conversion_factor 进行缩放。
    典型配置为 1000.0（毫米→米）。
注意：
  - 仅对"数值型"字段调用（坐标分量/半径/长度等）。
*/
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

  // 读取可选的 id（若无则保持默认0）
  // id
  if (json_line.contains("id") && json_line["id"].is_number_integer())
  {
    line.id = json_line["id"].get<int32_t>();
  }

  // 设置几何类型为直线；新 JSON 无 hasPrinted 语义，默认未绘制
  line.type = GeometryType::LINE;
  line.is_printed = false;  // 新格式无此语义，默认为未绘制

  // 解析起点/终点：新格式使用 start/end，对象内为 x/y(/z)
  if (json_line.contains("start") && json_line["start"].is_object())
  {
    line.start = parse_point(json_line["start"]);
  }
  if (json_line.contains("end") && json_line["end"].is_object())
  {
    line.end = parse_point(json_line["end"]);
  }

  // 可选元数据
  if (json_line.contains("line_type") && json_line["line_type"].is_string())
  {
    line.line_type = json_line["line_type"].get<std::string>();
  }
  if (json_line.contains("thickness") && json_line["thickness"].is_number())
  {
    line.thickness = json_line["thickness"].get<double>();
  }
  if (json_line.contains("hidden") && json_line["hidden"].is_boolean())
  {
    line.hidden = json_line["hidden"].get<bool>();
  }
  if (json_line.contains("layer_id") && json_line["layer_id"].is_number_integer())
  {
    line.layer_id = json_line["layer_id"].get<int32_t>();
  }
  if (json_line.contains("layer") && json_line["layer"].is_string())
  {
    line.layer = json_line["layer"].get<std::string>();
  }
  if (json_line.contains("color") && json_line["color"].is_string())
  {
    line.color = json_line["color"].get<std::string>();
  }
  if (json_line.contains("selected") && json_line["selected"].is_boolean())
  {
    line.selected = json_line["selected"].get<bool>();
  }

  // 长度由（可能缩放后的）起止点计算
  line.length = line.start.distance(line.end);
  return line;
}

Curve CADParser::parse_curve(const nlohmann::json& json_curve)
{
  Curve curve;

  // id
  if (json_curve.contains("Id") && json_curve["Id"].is_number_integer())
  {
    curve.id = json_curve["Id"].get<int32_t>();
  }

  // 设置几何类型为曲线
  curve.type = GeometryType::CURVE;
  curve.is_printed = false;

  // 旧格式兼容
  if (json_curve.contains("HasPrinted") && json_curve["HasPrinted"].is_boolean())
  {
    curve.is_printed = json_curve["HasPrinted"].get<bool>();
  }

  // 解析曲线次数
  if (json_curve.contains("Degree") && json_curve["Degree"].is_number())
  {
    curve.degree = json_curve["Degree"].get<int>();
  }

  // 解析控制点
  if (json_curve.contains("ControlPoints") && json_curve["ControlPoints"].is_array())
  {
    for (const auto& point : json_curve["ControlPoints"])
    {
      Point3D p;
      if (point.contains("X"))
        p.x = convert_units(point["X"].get<double>());
      if (point.contains("Y"))
        p.y = convert_units(point["Y"].get<double>());
      if (point.contains("Z"))
        p.z = convert_units(point["Z"].get<double>());
      curve.control_points.push_back(p);
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

  // id
  if (json_circle.contains("id") && json_circle["id"].is_number_integer())
  {
    circle.id = json_circle["id"].get<int32_t>();
  }

  // 明确几何类型与默认绘制标志
  circle.type = GeometryType::CIRCLE;
  circle.is_printed = false;  // 新格式无此语义

  // 解析圆心
  if (json_circle.contains("center"))
  {
    circle.center = parse_point(json_circle["center"]);
  }

  // 解析半径（单位换算）
  if (json_circle.contains("radius"))
  {
    circle.radius = convert_units(json_circle["radius"].get<double>());
  }

  // 可选元数据（继承自Line）
  if (json_circle.contains("line_type") && json_circle["line_type"].is_string())
  {
    circle.line_type = json_circle["line_type"].get<std::string>();
  }
  if (json_circle.contains("thickness") && json_circle["thickness"].is_number())
  {
    circle.thickness = json_circle["thickness"].get<double>();
  }
  if (json_circle.contains("hidden") && json_circle["hidden"].is_boolean())
  {
    circle.hidden = json_circle["hidden"].get<bool>();
  }
  if (json_circle.contains("layer_id") && json_circle["layer_id"].is_number_integer())
  {
    circle.layer_id = json_circle["layer_id"].get<int32_t>();
  }
  if (json_circle.contains("layer") && json_circle["layer"].is_string())
  {
    circle.layer = json_circle["layer"].get<std::string>();
  }
  if (json_circle.contains("color") && json_circle["color"].is_string())
  {
    circle.color = json_circle["color"].get<std::string>();
  }
  if (json_circle.contains("selected") && json_circle["selected"].is_boolean())
  {
    circle.selected = json_circle["selected"].get<bool>();
  }

  // 更新圆周长
  circle.length = 2.0 * M_PI * circle.radius;

  // 设置起点和终点
  circle.start.x = circle.center.x + circle.radius;
  circle.start.y = circle.center.y;
  circle.start.z = circle.center.z;
  circle.end = circle.start;  // 圆是闭合的，起点=终点

  return circle;
}

Arc CADParser::parse_arc(const nlohmann::json& json_arc)
{
  Arc arc;

  // id
  if (json_arc.contains("id") && json_arc["id"].is_number_integer())
  {
    arc.id = json_arc["id"].get<int32_t>();
  }

  // 明确几何类型与默认绘制标志
  arc.type = GeometryType::ARC;
  arc.is_printed = false;  // 新格式无此语义

  // 解析圆心
  if (json_arc.contains("center"))
  {
    arc.center = parse_point(json_arc["center"]);
  }

  // 解析半径（单位换算）
  if (json_arc.contains("radius"))
  {
    arc.radius = convert_units(json_arc["radius"].get<double>());
  }

  // 可选元数据（继承自Line）
  if (json_arc.contains("line_type") && json_arc["line_type"].is_string())
  {
    arc.line_type = json_arc["line_type"].get<std::string>();
  }
  if (json_arc.contains("thickness") && json_arc["thickness"].is_number())
  {
    arc.thickness = json_arc["thickness"].get<double>();
  }
  if (json_arc.contains("hidden") && json_arc["hidden"].is_boolean())
  {
    arc.hidden = json_arc["hidden"].get<bool>();
  }
  if (json_arc.contains("layer_id") && json_arc["layer_id"].is_number_integer())
  {
    arc.layer_id = json_arc["layer_id"].get<int32_t>();
  }
  if (json_arc.contains("layer") && json_arc["layer"].is_string())
  {
    arc.layer = json_arc["layer"].get<std::string>();
  }
  if (json_arc.contains("color") && json_arc["color"].is_string())
  {
    arc.color = json_arc["color"].get<std::string>();
  }
  if (json_arc.contains("selected") && json_arc["selected"].is_boolean())
  {
    arc.selected = json_arc["selected"].get<bool>();
  }

  // 将角度按配置转换为弧度
  auto to_radians = [&](double angle_val) {
    if (config_.angle_unit == AngleUnit::DEGREES)
    {
      return angle_val * M_PI / 180.0;
    }
    return angle_val;  // 已为弧度
  };

  // 解析起始/结束角度（兼容 snake_case 与 camelCase），并按配置统一为弧度
  double start_val = 0.0;
  double end_val = 2.0 * M_PI;
  if (json_arc.contains("start_angle") && !json_arc["start_angle"].is_null())
  {
    start_val = json_arc["start_angle"].get<double>();
  }
  else if (json_arc.contains("startAngle") && !json_arc["startAngle"].is_null())
  {
    start_val = json_arc["startAngle"].get<double>();
  }

  if (json_arc.contains("end_angle") && !json_arc["end_angle"].is_null())
  {
    end_val = json_arc["end_angle"].get<double>();
  }
  else if (json_arc.contains("endAngle") && !json_arc["endAngle"].is_null())
  {
    end_val = json_arc["endAngle"].get<double>();
  }

  arc.start_angle = to_radians(start_val);
  arc.end_angle = to_radians(end_val);

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

/*
解析文字数据（parse_text）
--------------------------------
输入：JSON对象，包含文字的各种属性
输出：解析后的 Text 对象

文字在 CAD JSON 中的典型结构：
{
  "id": 9,
  "type": "text",
  "position": { "x": 500.0, "y": 500.0 },
  "height": 50.0,
  "rotation": 0.0,
  "style": "Standard",
  "content": "中心点",
  "width_factor": 1.0,
  "oblique": 0.0,
  "align": { "horizontal": "center", "vertical": "middle" }
}

处理逻辑：
1. 解析位置（position）并进行单位转换
2. 解析文字内容（content）
3. 解析高度（height）并进行单位转换
4. 解析旋转角度（rotation），保持度数
5. 解析对齐方式（align）
6. 计算文字的绘制边界（起点和终点）
*/
Text CADParser::parse_text(const nlohmann::json& json_text)
{
  Text text;

  // 解析ID
  if (json_text.contains("id") && json_text["id"].is_number_integer())
  {
    text.id = json_text["id"].get<int32_t>();
  }

  // 设置几何类型
  text.type = GeometryType::TEXT;
  text.is_printed = false;
  text.line_type = "text";  // 标记为文字类型

  // 解析位置
  if (json_text.contains("position") && json_text["position"].is_object())
  {
    text.position = parse_point(json_text["position"]);
  }

  // 解析文字内容
  if (json_text.contains("content") && json_text["content"].is_string())
  {
    text.content = json_text["content"].get<std::string>();
  }

  // 解析高度（单位换算）
  if (json_text.contains("height") && json_text["height"].is_number())
  {
    text.height = convert_units(json_text["height"].get<double>());
  }

  // 解析旋转角度（保持度数，不转换）
  if (json_text.contains("rotation") && json_text["rotation"].is_number())
  {
    text.rotation = json_text["rotation"].get<double>();
  }

  // 解析字体样式
  if (json_text.contains("style") && json_text["style"].is_string())
  {
    text.style = json_text["style"].get<std::string>();
  }

  // 解析宽度因子
  if (json_text.contains("width_factor") && json_text["width_factor"].is_number())
  {
    text.width_factor = json_text["width_factor"].get<double>();
  }

  // 解析倾斜角度
  if (json_text.contains("oblique") && json_text["oblique"].is_number())
  {
    text.oblique = json_text["oblique"].get<double>();
  }

  // 解析子类型
  if (json_text.contains("subtype") && json_text["subtype"].is_string())
  {
    text.subtype = json_text["subtype"].get<std::string>();
  }

  // 解析对齐方式
  if (json_text.contains("align") && json_text["align"].is_object())
  {
    const auto& align_obj = json_text["align"];
    if (align_obj.contains("horizontal") && align_obj["horizontal"].is_string())
    {
      text.align.horizontal = align_obj["horizontal"].get<std::string>();
    }
    if (align_obj.contains("vertical") && align_obj["vertical"].is_string())
    {
      text.align.vertical = align_obj["vertical"].get<std::string>();
    }
  }

  // 可选元数据（继承自Line）
  if (json_text.contains("layer_id") && json_text["layer_id"].is_number_integer())
  {
    text.layer_id = json_text["layer_id"].get<int32_t>();
  }
  if (json_text.contains("layer") && json_text["layer"].is_string())
  {
    text.layer = json_text["layer"].get<std::string>();
  }
  if (json_text.contains("color") && json_text["color"].is_string())
  {
    text.color = json_text["color"].get<std::string>();
  }
  if (json_text.contains("thickness") && json_text["thickness"].is_number())
  {
    text.thickness = json_text["thickness"].get<double>();
  }
  if (json_text.contains("hidden") && json_text["hidden"].is_boolean())
  {
    text.hidden = json_text["hidden"].get<bool>();
  }
  if (json_text.contains("selected") && json_text["selected"].is_boolean())
  {
    text.selected = json_text["selected"].get<bool>();
  }

  // 计算文字的绘制边界（起点和终点）
  text.calculateDrawingBounds(text.start, text.end);

  // 计算长度（文字宽度）
  text.length = text.estimateWidth();

  std::cout << "Parsed text with ID: " << text.id << ", content: \"" << text.content << "\""
            << ", position: [" << text.position.x << ", " << text.position.y << "]"
            << ", height: " << text.height << " m"
            << ", rotation: " << text.rotation << " deg"
            << std::endl;

  return text;
}

Point3D CADParser::parse_point(const nlohmann::json& json_point)
{
  Point3D point;
  // 支持小写/大写键（x/X, y/Y, z/Z），并对数值进行单位换算
  if (json_point.contains("x"))
  {
    point.x = convert_units(json_point["x"].get<double>());
  }
  else if (json_point.contains("X"))
  {
    point.x = convert_units(json_point["X"].get<double>());
  }

  if (json_point.contains("y"))
  {
    point.y = convert_units(json_point["y"].get<double>());
  }
  else if (json_point.contains("Y"))
  {
    point.y = convert_units(json_point["Y"].get<double>());
  }

  if (json_point.contains("z"))
  {
    point.z = convert_units(json_point["z"].get<double>());
  }
  else if (json_point.contains("Z"))
  {
    point.z = convert_units(json_point["Z"].get<double>());
  }

  return point;
}

void CADParser::build_layer_map(const nlohmann::json& cad_json)
{
  layer_id_to_name_.clear();
  if (!cad_json.contains("layers") || !cad_json["layers"].is_array())
  {
    return;
  }

  for (const auto& layer : cad_json["layers"])
  {
    if (!layer.is_object())
      continue;
    if (!layer.contains("name") || !layer["name"].is_string())
      continue;
    std::string name = layer["name"].get<std::string>();
    if (layer.contains("layer_id") && layer["layer_id"].is_number_integer())
    {
      int id = layer["layer_id"].get<int>();
      layer_id_to_name_[id] = name;
    }
  }
}

void CADParser::store_by_layer(const std::shared_ptr<Line>& geom, const std::string& layer_name)
{
  // 基于图层名称关键字进行分类（不区分大小写），默认归入"路径"集合
  std::string lname = layer_name;
  std::transform(lname.begin(), lname.end(), lname.begin(), [](unsigned char c) { return std::tolower(c); });

  auto contains = [&](const char* kw) { return !lname.empty() && lname.find(kw) != std::string::npos; };

  // 记录解析得到的图层名称（若有）到几何元数据中，便于后续可视化/调试
  if (!layer_name.empty())
  {
    geom->layer = layer_name;
  }

  if (contains("空洞") || contains("hole") || contains("hollow") || contains("void") || contains("opening"))
  {
    cad_data_.hole_lines.push_back(geom);
  }
  else if (contains("障碍") || contains("barrier") || contains("obstacle"))
  {
    cad_data_.obstacle_lines.push_back(geom);
  }
  else if (contains("路径") || contains("path") || contains("axis") || contains("draw") || contains("drawing"))
  {
    cad_data_.path_lines.push_back(geom);
  }
  else
  {
    // 未知图层归入路径集合，保证后续模块可工作
    cad_data_.path_lines.push_back(geom);
  }
}

}  // namespace path_planner
