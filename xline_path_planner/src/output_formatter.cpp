#include "xline_path_planner/output_formatter.hpp"
#include <iostream>
#include <unordered_map>

namespace daosnrs_planning
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

      // 若仅两个点，则导出为 line，使用 start/end 结构；否则仍输出 polyline
      if (seg.points.size() == 2)
      {
        const auto& p0 = seg.points.front();
        const auto& p1 = seg.points.back();

        nlohmann::json line;
        line["id"] = (src ? src->id : seg.line_id);
        line["type"] = "line";
        line["order"] = static_cast<int>(seg_idx);
        line["work"] = true; // 绘图段：工作

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

        line["start"] = { {"x", p0.x}, {"y", p0.y} };
        line["end"] = { {"x", p1.x}, {"y", p1.y} };

        root["lines"].push_back(line);
      }
      else
      {
        nlohmann::json poly;
        poly["id"] = (src ? src->id : seg.line_id);
        poly["type"] = "polyline";
        poly["order"] = static_cast<int>(seg_idx);
        poly["work"] = true; // 绘图段：工作

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
          vertices.push_back({ {"x", p.x}, {"y", p.y} });
        }
        poly["vertices"] = vertices;

        root["lines"].push_back(poly);
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
  j["order"] = order;
  j["work"] = false;

  j["start"] = { {"x", start.x}, {"y", start.y} };
  j["end"] = { {"x", end.x}, {"y", end.y} };

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

}  // namespace daosnrs_planning
