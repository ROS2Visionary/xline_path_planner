#include "xline_path_planner/output_formatter.hpp"
#include <iostream>
#include <unordered_map>

namespace {
inline double to_mm(double meters) { return meters * 1000.0; }
inline nlohmann::json point_mm(const path_planner::Point3D& p)
{
  return nlohmann::json{ {"x", to_mm(p.x)}, {"y", to_mm(p.y)} };
}

// 将 PrinterType 转换为字符串
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

    for (std::size_t i = 0; i + 1 < seg.points.size(); ++i)
    {
      root["lines"].push_back(constructTransitionLineJSON(seg.points[i], seg.points[i + 1], static_cast<int>(seg_idx)));
    }
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
      for (std::size_t i = 0; i + 1 < seg.points.size(); ++i)
      {
        auto j = constructTransitionLineJSON(seg.points[i], seg.points[i + 1], static_cast<int>(seg_idx));
        root["lines"].push_back(j);

      }
    }
    else if (seg.type == RouteType::DRAWING_PATH)
    {
      if (seg.points.size() < 2) continue;

      const Line* src = nullptr;
      auto it = id2line.find(seg.line_id);
      if (it != id2line.end()) src = it->second;

      // 根据源几何类型进行导出，确保圆/圆弧被正确导出，且包含起点/终点
      if (src && src->type == GeometryType::CIRCLE)
      {
        // 圆：输出 center/radius，并补充 start/end
        const Circle* circle = dynamic_cast<const Circle*>(src);
        nlohmann::json j;
        j["id"] = (src ? src->id : seg.line_id);
        j["type"] = "circle";
        j["order"] = static_cast<int>(seg_idx);
        j["work"] = true;
        j["printed"] = false;
        j["printer_type"] = printerTypeToString(seg.printer_type);  // 新增：打印机类型

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
          j["radius"] = to_mm(circle->radius);
        }

        const auto& p0 = seg.points.front();
        const auto& p1 = seg.points.back();
        j["start"] = point_mm(p0);
        j["end"] = point_mm(p1);

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
        j["printer_type"] = printerTypeToString(seg.printer_type);  // 新增：打印机类型

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
          j["radius"] = to_mm(arc->radius);
          // 角度以度导出（避免依赖 M_PI）
          auto rad2deg = [](double r) { return r * 180.0 / 3.14159265358979323846; };
          j["start_angle"] = rad2deg(arc->start_angle);
          j["end_angle"] = rad2deg(arc->end_angle);
        }

        const auto& p0 = seg.points.front();
        const auto& p1 = seg.points.back();
        j["start"] = point_mm(p0);
        j["end"] = point_mm(p1);

        root["lines"].push_back(j);
      }
      else
      {
        // 非圆/圆弧：保留之前的 line/polyline 导出策略
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
          line["printer_type"] = printerTypeToString(seg.printer_type);  // 新增：打印机类型

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
          poly["printer_type"] = printerTypeToString(seg.printer_type);  // 新增：打印机类型

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

// 辅助：构造转场线条 JSON
nlohmann::json OutputFormatter::constructTransitionLineJSON(const Point3D& start, const Point3D& end, int order)
{
  nlohmann::json j;

  j["id"] = 1000000;
  j["type"] = "line";
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
  j["start"] = point_mm(start);
  j["end"] = point_mm(end);

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
