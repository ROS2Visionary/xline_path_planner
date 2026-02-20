#include "xline_path_planner/output_formatter.hpp"
#include <iostream>
#include <unordered_map>
#include <cmath>

namespace {
inline double to_mm(double meters) { return meters * 1000.0; }
inline nlohmann::json point_mm(const path_planner::Point3D& p)
{
  return nlohmann::json{ {"x", to_mm(p.x)}, {"y", to_mm(p.y)} };
}

// 将 PrinterType 转换为字符串（大写形式，用于 printer_type 字段）
inline std::string printerTypeToString(path_planner::PrinterType type)
{
  switch (type) {
    case path_planner::PrinterType::LEFT_PRINTER:
      return "LEFT_PRINTER";
    case path_planner::PrinterType::RIGHT_PRINTER:
      return "RIGHT_PRINTER";
    case path_planner::PrinterType::CENTER_PRINTER:
      return "CENTER_PRINTER";
    default:
      return "CENTER_PRINTER";
  }
}

// 将 PrinterType 转换为小写字符串（用于 ink.printer 字段）
inline std::string printerTypeToLowerString(path_planner::PrinterType type)
{
  switch (type) {
    case path_planner::PrinterType::LEFT_PRINTER:
      return "left";
    case path_planner::PrinterType::RIGHT_PRINTER:
      return "right";
    case path_planner::PrinterType::CENTER_PRINTER:
      return "center";
    default:
      return "center";
  }
}

// 将 InkMode 转换为字符串
inline std::string inkModeToString(path_planner::InkMode mode)
{
  switch (mode) {
    case path_planner::InkMode::SOLID:
      return "solid";
    case path_planner::InkMode::DASHED:
      return "dashed";
    case path_planner::InkMode::TEXT:
      return "text";
    default:
      return "solid";
  }
}

// 构造 ink JSON 对象
inline nlohmann::json constructInkJSON(bool enabled,
                                       path_planner::InkMode mode,
                                       path_planner::PrinterType printer)
{
  nlohmann::json ink;
  ink["enabled"] = enabled;
  ink["mode"] = inkModeToString(mode);
  ink["printer"] = printerTypeToLowerString(printer);
  return ink;
}

// 计算下一条路径的航向（如果下一条路径是 line 类型）
// 返回航向值（弧度）或 -999.0（表示无效）
inline double calculateNextPathHeading(const path_planner::RouteSegment* next_segment)
{
  if (next_segment != nullptr && next_segment->points.size() == 2)
  {
    // 下一条路径是直线（2个点）
    const auto& start = next_segment->points.front();
    const auto& end = next_segment->points.back();
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    return std::atan2(dy, dx);
  }
  else
  {
    // 不是 line 类型或没有下一条路径，返回 -999.0 表示无效
    return -999.0;
  }
}

} // anonymous namespace

namespace path_planner
{

// ---------------- 导出 planned paths 为 CAD JSON（仅返回 lines） ----------------

nlohmann::json OutputFormatter::format_planned_paths_to_cad_json(const std::vector<RouteSegment>& segments,
                                                                 const std::string& source_file)
{
  (void)source_file; // 参数保留但当前未使用
  // 顶层对象
  nlohmann::json root;
  root["lines"] = nlohmann::json::array();

  for (std::size_t seg_idx = 0; seg_idx < segments.size(); ++seg_idx)
  {
    const auto& seg = segments[seg_idx];
    if (seg.type != RouteType::TRANSITION_PATH)
      continue;

    if (seg.points.size() < 2)
      continue;

    // 查找下一条路径段（可能是转场路径或绘图路径）
    const RouteSegment* next_segment = nullptr;
    if (seg_idx + 1 < segments.size())
    {
      next_segment = &segments[seg_idx + 1];
    }

    root["lines"].push_back(constructTransitionSplineJSON(seg.points, static_cast<int>(seg_idx), seg, next_segment));
  }

  // 仅返回 { "lines": [...] }
  nlohmann::json only_lines;
  only_lines["lines"] = root["lines"];
  return only_lines;
}

nlohmann::json OutputFormatter::format_planned_paths_to_cad_json(const std::vector<RouteSegment>& segments,
                                                                 const CADData& cad_data,
                                                                 const std::string& source_file)
{
  (void)source_file; // 参数保留但当前未使用
  // 顶层对象
  nlohmann::json root;
  root["lines"] = nlohmann::json::array();

  // id -> Line* 映射
  std::unordered_map<int32_t, const Line*> id2line;
  id2line.reserve(cad_data.path_lines.size());
  for (const auto& lp : cad_data.path_lines)
  {
    if (lp)
      id2line[lp->id] = lp.get();
  }

  // 按 segments 顺序填充 lines，并设置 order 字段
  for (std::size_t seg_idx = 0; seg_idx < segments.size(); ++seg_idx)
  {
    const auto& seg = segments[seg_idx];

    if (seg.type == RouteType::TRANSITION_PATH)
    {
      if (seg.points.size() < 2) continue;

      // 查找下一条路径段（可能是转场路径或绘图路径）
      const RouteSegment* next_segment = nullptr;
      if (seg_idx + 1 < segments.size())
      {
        next_segment = &segments[seg_idx + 1];
      }

      root["lines"].push_back(constructTransitionSplineJSON(seg.points, static_cast<int>(seg_idx), seg, next_segment));
    }
    else if (seg.type == RouteType::DRAWING_PATH)
    {
      if (seg.points.size() < 2) continue;

      // 查找下一条路径段（用于计算 next_path_heading）
      const RouteSegment* next_segment = nullptr;
      if (seg_idx + 1 < segments.size())
      {
        next_segment = &segments[seg_idx + 1];
      }
      double next_path_heading = calculateNextPathHeading(next_segment);

      const Line* src = nullptr;
      auto it = id2line.find(seg.line_id);
      if (it != id2line.end()) src = it->second;

      // 根据源几何类型进行导出
	      if (src && src->type == GeometryType::TEXT)
	      {
	        // 文字类型：输出格式与 line 类似
	        const Text* text = dynamic_cast<const Text*>(src);
	        nlohmann::json j;
	        j["id"] = (src ? src->id : seg.line_id);
	        j["type"] = "text";
	        j["order"] = static_cast<int>(seg_idx);
	        j["work"] = true;
	        j["printed"] = false;
	        j["backward"] = false;
	        j["printer_type"] = printerTypeToString(seg.printer_type);
	        j["ink"] = constructInkJSON(true, seg.ink_mode, seg.printer_type);

        j["line_type"] = "text";
        j["thickness"] = (src ? src->thickness : 1.0);
        j["hidden"] = (src ? src->hidden : false);
        j["layer"] = (src && !src->layer.empty()) ? src->layer : std::string("Default");
        j["color"] = (src && !src->color.empty()) ? src->color : std::string("#FFFFFF");
        if (src)
        {
          j["selected"] = src->selected;
          j["layer_id"] = src->layer_id;
        }

        // 文字内容
        j["content"] = text ? text->content : seg.text_content;

        // 起点和终点
        const auto& p0 = seg.points.front();
        const auto& p1 = seg.points.back();
        j["start"] = point_mm(p0);
        j["end"] = point_mm(p1);

        // 添加下一条路径的航向
        j["next_path_heading"] = next_path_heading;

        root["lines"].push_back(j);
      }
	      else if (src && src->type == GeometryType::CIRCLE)
	      {
	        // 圆：输出 center/radius，并补充 start/end
	        const Circle* circle = dynamic_cast<const Circle*>(src);
	        nlohmann::json j;
	        j["id"] = (src ? src->id : seg.line_id);
	        j["type"] = "circle";
	        j["order"] = static_cast<int>(seg_idx);
	        j["work"] = true;
	        j["printed"] = false;
	        j["backward"] = false;
	        j["printer_type"] = printerTypeToString(seg.printer_type);  // 新增：打印机类型
	        j["ink"] = constructInkJSON(true, seg.ink_mode, seg.printer_type);  // 新增：ink 对象

        j["line_type"] = (src ? src->line_type : std::string("continuous"));
        j["thickness"] = (src ? src->thickness : 1.0);
        j["hidden"] = (src ? src->hidden : false);
        j["layer"] = (src && !src->layer.empty()) ? src->layer : std::string("Default");
        j["color"] = (src && !src->color.empty()) ? src->color : std::string("#FFFFFF");
        if (src)
        {
          j["selected"] = src->selected;
          j["layer_id"] = src->layer_id;
        }

        if (circle)
        {
          j["center"] = { {"x", to_mm(circle->center.x)}, {"y", to_mm(circle->center.y)} };
          // 使用调整后的半径（如果有），否则使用原始半径
          double export_radius = seg.adjusted_radius.has_value() ? seg.adjusted_radius.value() : circle->radius;
          j["radius"] = to_mm(export_radius);
        }

        const auto& p0 = seg.points.front();
        const auto& p1 = seg.points.back();
        j["start"] = point_mm(p0);
        j["end"] = point_mm(p1);

        // 添加下一条路径的航向
        j["next_path_heading"] = next_path_heading;

        root["lines"].push_back(j);
      }
	      else if (src && src->type == GeometryType::ARC)
	      {
	        // 圆弧：输出 center/radius/start_angle/end_angle（度），并补充 start/end
	        const Arc* arc = dynamic_cast<const Arc*>(src);
	        nlohmann::json j;
	        j["id"] = (src ? src->id : seg.line_id);
	        j["type"] = "arc";
	        j["order"] = static_cast<int>(seg_idx);
	        j["work"] = true;
	        j["printed"] = false;
	        j["backward"] = false;
	        j["printer_type"] = printerTypeToString(seg.printer_type);  // 新增：打印机类型
	        j["ink"] = constructInkJSON(true, seg.ink_mode, seg.printer_type);  // 新增：ink 对象

        j["line_type"] = (src ? src->line_type : std::string("continuous"));
        j["thickness"] = (src ? src->thickness : 1.0);
        j["hidden"] = (src ? src->hidden : false);
        j["layer"] = (src && !src->layer.empty()) ? src->layer : std::string("Default");
        j["color"] = (src && !src->color.empty()) ? src->color : std::string("#FFFFFF");
        if (src)
        {
          j["selected"] = src->selected;
          j["layer_id"] = src->layer_id;
        }

        if (arc)
        {
          j["center"] = { {"x", to_mm(arc->center.x)}, {"y", to_mm(arc->center.y)} };
          // 使用调整后的半径（如果有），否则使用原始半径
          double export_radius = seg.adjusted_radius.has_value() ? seg.adjusted_radius.value() : arc->radius;
          j["radius"] = to_mm(export_radius);
          // 角度以度导出（避免依赖 M_PI）
          auto rad2deg = [](double r) { return r * 180.0 / 3.14159265358979323846; };
          j["start_angle"] = rad2deg(arc->start_angle);
          j["end_angle"] = rad2deg(arc->end_angle);
        }

        const auto& p0 = seg.points.front();
        const auto& p1 = seg.points.back();
        j["start"] = point_mm(p0);
        j["end"] = point_mm(p1);

        // 添加下一条路径的航向
        j["next_path_heading"] = next_path_heading;

        root["lines"].push_back(j);
      }
	      else if (src && src->type == GeometryType::ELLIPSE)
	      {
        const Ellipse* ellipse = dynamic_cast<const Ellipse*>(src);
        nlohmann::json j;
        j["id"] = (src ? src->id : seg.line_id);
        j["type"] = "ellipse";
        j["order"] = static_cast<int>(seg_idx);
        j["work"] = true;
        j["printed"] = false;
        j["backward"] = false;
        j["printer_type"] = printerTypeToString(seg.printer_type);
        j["ink"] = constructInkJSON(true, seg.ink_mode, seg.printer_type);

        j["line_type"] = (src ? src->line_type : std::string("continuous"));
        j["thickness"] = (src ? src->thickness : 1.0);
        j["hidden"] = (src ? src->hidden : false);
        j["layer"] = (src && !src->layer.empty()) ? src->layer : std::string("Default");
        j["color"] = (src && !src->color.empty()) ? src->color : std::string("#FFFFFF");
        if (src)
        {
          j["selected"] = src->selected;
          j["layer_id"] = src->layer_id;
        }

        if (ellipse)
        {
          j["center"] = { {"x", to_mm(ellipse->center.x)}, {"y", to_mm(ellipse->center.y)} };

          // 使用调整后的长短轴半径（如果有），重新计算 major_axis 和 ratio
          if (seg.adjusted_major_radius.has_value() && seg.adjusted_minor_radius.has_value())
          {
            double adjusted_a = seg.adjusted_major_radius.value();
            double adjusted_b = seg.adjusted_minor_radius.value();

            // 计算调整后的 major_axis 向量（保持原始方向，调整长度）
            double original_length = std::sqrt(ellipse->major_axis.x * ellipse->major_axis.x +
                                              ellipse->major_axis.y * ellipse->major_axis.y);
            double scale = adjusted_a / std::max(1e-9, original_length);

            j["major_axis"] = { {"x", to_mm(ellipse->major_axis.x * scale)},
                               {"y", to_mm(ellipse->major_axis.y * scale)} };
            j["ratio"] = adjusted_b / adjusted_a;
          }
          else
          {
            j["major_axis"] = { {"x", to_mm(ellipse->major_axis.x)}, {"y", to_mm(ellipse->major_axis.y)} };
            j["ratio"] = ellipse->ratio;
          }

          auto rad2deg = [](double r) { return r * 180.0 / 3.14159265358979323846; };
          j["start_angle"] = rad2deg(ellipse->start_angle);
          j["end_angle"] = rad2deg(ellipse->end_angle);
          j["rotation"] = rad2deg(ellipse->rotation);
        }

        const auto& p0 = seg.points.front();
        const auto& p1 = seg.points.back();
        j["start"] = point_mm(p0);
        j["end"] = point_mm(p1);

        // 添加下一条路径的航向
        j["next_path_heading"] = next_path_heading;

        root["lines"].push_back(j);
      }
      else if (src && src->type == GeometryType::SPLINE)
      {
        const Spline* spline = dynamic_cast<const Spline*>(src);
        nlohmann::json j;
        j["id"] = (src ? src->id : seg.line_id);
        j["type"] = "spline";
        j["order"] = static_cast<int>(seg_idx);
        j["work"] = true;
        j["printed"] = false;
        j["backward"] = false;
        j["printer_type"] = printerTypeToString(seg.printer_type);
        j["ink"] = constructInkJSON(true, seg.ink_mode, seg.printer_type);

        j["line_type"] = (src ? src->line_type : std::string("continuous"));
        j["thickness"] = (src ? src->thickness : 1.0);
        j["hidden"] = (src ? src->hidden : false);
        j["layer"] = (src && !src->layer.empty()) ? src->layer : std::string("Default");
        j["color"] = (src && !src->color.empty()) ? src->color : std::string("#FFFFFF");
        if (src)
        {
          j["selected"] = src->selected;
          j["layer_id"] = src->layer_id;
        }

        if (spline)
        {
          j["degree"] = spline->degree;
          j["periodic"] = spline->periodic;
          j["is_closed"] = spline->closed;
          if (!spline->knots.empty()) j["knots"] = spline->knots;
          if (!spline->weights.empty()) j["weights"] = spline->weights;
          if (!spline->control_points.empty())
          {
            nlohmann::json cps = nlohmann::json::array();
            for (const auto& p : spline->control_points)
            {
              cps.push_back(point_mm(p));
            }
            j["control_points"] = cps;
          }
        }

        // 规划后的离散点（用于执行/可视化）
        nlohmann::json vertices = nlohmann::json::array();
        for (const auto& p : seg.points)
        {
          vertices.push_back(point_mm(p));
        }
        j["vertices"] = vertices;

        const auto& p0 = seg.points.front();
        const auto& p1 = seg.points.back();
        j["start"] = point_mm(p0);
        j["end"] = point_mm(p1);

        // 添加下一条路径的航向
        j["next_path_heading"] = next_path_heading;

        root["lines"].push_back(j);
      }
      else
      {
        // 非圆/圆弧/文字：保留之前的 line/polyline 导出策略
        if (seg.points.size() == 2)
        {
          const auto& p0 = seg.points.front();
          const auto& p1 = seg.points.back();

          nlohmann::json line;
          line["id"] = (src ? src->id : seg.line_id);
	          line["type"] = "line";
	          line["order"] = static_cast<int>(seg_idx);
	          line["work"] = true; // 绘图段：工作
	          line["printed"] = false;
	          line["backward"] = false;
	          line["printer_type"] = printerTypeToString(seg.printer_type);  // 新增：打印机类型
	          line["ink"] = constructInkJSON(true, seg.ink_mode, seg.printer_type);  // 新增：ink 对象

          line["line_type"] = (src ? src->line_type : std::string("continuous"));
          line["thickness"] = (src ? src->thickness : 1.0);
          line["hidden"] = (src ? src->hidden : false);
          line["layer"] = (src && !src->layer.empty()) ? src->layer : std::string("Default");
          line["color"] = (src && !src->color.empty()) ? src->color : std::string("#FFFFFF");
          if (src)
          {
            line["selected"] = src->selected;
            line["layer_id"] = src->layer_id;
          }

          line["start"] = point_mm(p0);
          line["end"] = point_mm(p1);

          // 添加下一条路径的航向
          line["next_path_heading"] = next_path_heading;

          root["lines"].push_back(line);
        }
        else
        {
          nlohmann::json poly;
          poly["id"] = (src ? src->id : seg.line_id);
	          poly["type"] = "polyline";
	          poly["order"] = static_cast<int>(seg_idx);
	          poly["work"] = true; // 绘图段：工作
	          poly["printed"] = false;
	          poly["backward"] = false;
	          poly["printer_type"] = printerTypeToString(seg.printer_type);  // 新增：打印机类型
	          poly["ink"] = constructInkJSON(true, seg.ink_mode, seg.printer_type);  // 新增：ink 对象

          poly["line_type"] = (src ? src->line_type : std::string("continuous"));
          poly["thickness"] = (src ? src->thickness : 1.0);
          poly["hidden"] = (src ? src->hidden : false);
          poly["layer"] = (src && !src->layer.empty()) ? src->layer : std::string("Default");
          poly["color"] = (src && !src->color.empty()) ? src->color : std::string("#FFFFFF");
          if (src)
          {
            poly["selected"] = src->selected;
            poly["layer_id"] = src->layer_id;
          }

          nlohmann::json vertices = nlohmann::json::array();
          for (const auto& p : seg.points)
          {
            vertices.push_back(point_mm(p));
          }
          poly["vertices"] = vertices;
          // 同步给出起点/终点，避免消费端仅解析 start/end 的场景
          const auto& p0 = seg.points.front();
          const auto& p1 = seg.points.back();
          poly["start"] = point_mm(p0);
          poly["end"] = point_mm(p1);

          // 添加下一条路径的航向
          poly["next_path_heading"] = next_path_heading;

          root["lines"].push_back(poly);
        }
      }
    }
  }

  // 仅返回 { "lines": [...] }
  nlohmann::json only_lines;
  only_lines["lines"] = root["lines"];
  return only_lines;
}

// 辅助：构造转场路径的 JSON（根据点数自动判断类型）
nlohmann::json OutputFormatter::constructTransitionSplineJSON(const std::vector<Point3D>& points, int order, const RouteSegment& seg,
                                                               const RouteSegment* next_segment)
{
  nlohmann::json j;

  // 根据点数判断类型：2个点为直线，多个点为样条
  bool is_straight_line = (points.size() == 2);

  j["id"] = 1000000;
  j["type"] = is_straight_line ? "line" : "spline";
  j["line_type"] = "";
  j["thickness"] = 1.0;
  j["hidden"] = false;
  j["selected"] = false;
  j["layer_id"] = 1000000;
  j["layer"] = "TRANSITION";
  j["color"] = "#D3D3D3";
  j["opacity"] = 0.5;
  j["order"] = order;
  j["work"] = false;
  j["printed"] = false;
  j["backward"] = seg.execute_backward;
  j["printer_type"] = printerTypeToString(seg.printer_type);

  // 仅当为样条类型时添加样条相关字段
  if (!is_straight_line)
  {
    // 与导入侧 spline 描述保持一致的字段（control_points/knots/weights 允许为空）
    j["degree"] = 3;
    j["periodic"] = false;
    j["is_closed"] = false;
    j["control_points"] = nlohmann::json::array();
    j["knots"] = nlohmann::json::array();
    j["weights"] = nlohmann::json::array();
  }

  // 使用下一个路径的 ink 信息
  nlohmann::json ink;
  ink["enabled"] = false;  // 转场路径不喷墨
  ink["mode"] = inkModeToString(seg.ink_mode);
  ink["printer"] = printerTypeToLowerString(seg.printer_type);

  // 如果下一个路径是 TEXT 类型，包含文字内容
  if (seg.ink_mode == InkMode::TEXT && !seg.text_content.empty())
  {
    ink["content"] = seg.text_content;
  }

  j["ink"] = ink;

  // 计算下一条路径的航向（如果下一条路径是 line 类型）
  // 为了保持数据一致性，所有路径都包含此字段
  // 使用 -999.0 表示无效值（不是 line 类型或没有下一条路径）
  j["next_path_heading"] = calculateNextPathHeading(next_segment);

  // 根据类型导出路径点
  if (is_straight_line)
  {
    // 直线类型：只导出起点和终点
    j["start"] = point_mm(points.front());
    j["end"] = point_mm(points.back());
  }
  else
  {
    // 样条类型：导出所有离散点序列（用于执行/可视化）
    nlohmann::json vertices = nlohmann::json::array();
    for (const auto& p : points)
    {
      vertices.push_back(point_mm(p));
    }
    j["vertices"] = vertices;

    // 同时提供起点和终点，方便消费端使用
    j["start"] = point_mm(points.front());
    j["end"] = point_mm(points.back());
  }

  return j;
}

// 输出文件工具
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

    file << json_data.dump(2);
    file.close();

    return true;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error saving JSON to file: " << e.what() << std::endl;
    return false;
  }
}

}  // namespace path_planner
