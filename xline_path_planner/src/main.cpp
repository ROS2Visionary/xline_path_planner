#include <memory>
#include <string>
#include <fstream>
#include <filesystem>

#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "xline_path_planner/cad_parser.hpp"
#include "xline_path_planner/grid_map_generator.hpp"
#include "xline_path_planner/path_planner.hpp"
#include "xline_path_planner/trajectory_generator.hpp"
#include "xline_path_planner/output_formatter.hpp"

using namespace daosnrs_planning;

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode() : Node("drawing_planner_node")
  {
    // 更新参数
    updateParameters();
    // 执行路径规划
    plan_trajectory();
  }

  void updateParameters()
  {
    // 1) 定位并加载 YAML 配置文件
    std::string yaml_path;
    try
    {
      auto share_dir = ament_index_cpp::get_package_share_directory("xline_path_planner");
      yaml_path = share_dir + "/config/planner.yaml";
    }
    catch (const std::exception& e)
    {
      RCLCPP_FATAL(this->get_logger(), "无法获取包共享目录: %s", e.what());
      throw;
    }

    if (!std::filesystem::exists(yaml_path))
    {
      RCLCPP_FATAL(this->get_logger(), "配置文件不存在: %s", yaml_path.c_str());
      throw std::runtime_error("配置文件缺失");
    }

    YAML::Node root;
    try
    {
      root = YAML::LoadFile(yaml_path);
    }
    catch (const std::exception& e)
    {
      RCLCPP_FATAL(this->get_logger(), "读取配置文件失败: %s", e.what());
      throw;
    }

    auto require = [&](const YAML::Node& n, const char* path_desc)
    {
      if (!n || n.IsNull())
      {
        std::string msg = std::string("配置缺失: ") + path_desc;
        RCLCPP_FATAL(this->get_logger(), "%s", msg.c_str());
        throw std::runtime_error(msg);
      }
    };

    // 2) 读取 文件路径 参数
    YAML::Node files = root["files"];
    require(files, "files");
    require(files["cad_file_path"], "files.cad_file_path");
    require(files["output_file_path"], "files.output_file_path");
    require(files["path_visualization_dir"], "files.path_visualization_dir");
    cad_file_path = files["cad_file_path"].as<std::string>();
    output_file_path = files["output_file_path"].as<std::string>();
    path_visualization_dir = files["path_visualization_dir"].as<std::string>();

    // 3) 读取 可视化 参数（用于栅格与路径可视化）
    YAML::Node vis = root["visualization"];
    require(vis, "visualization");
    require(vis["scale"], "visualization.scale");
    require(vis["show_grid_lines"], "visualization.show_grid_lines");
    require(vis["use_antialiasing"], "visualization.use_antialiasing");
    require(vis["image_format"], "visualization.image_format");
    require(vis["save_path_visualization"], "visualization.save_path_visualization");
    grid_map_scale = vis["scale"].as<int>();
    show_grid_lines = vis["show_grid_lines"].as<bool>();
    use_antialiasing = vis["use_antialiasing"].as<bool>();
    image_format = vis["image_format"].as<std::string>();
    save_path_visualization = vis["save_path_visualization"].as<bool>();

    // 4) 读取 CAD 解析 参数
    YAML::Node cad = root["cad_parser"];
    require(cad, "cad_parser");
    require(cad["unit_conversion_factor"], "cad_parser.unit_conversion_factor");
    require(cad["auto_scale_coordinates"], "cad_parser.auto_scale_coordinates");
    require(cad["angle_unit"], "cad_parser.angle_unit");
    cad_unit_conversion = cad["unit_conversion_factor"].as<double>();
    bool auto_scale = cad["auto_scale_coordinates"].as<bool>();
    std::string angle_unit_str = cad["angle_unit"].as<std::string>();

    // 5) 读取 栅格地图 参数
    YAML::Node grid = root["grid_map"];
    require(grid, "grid_map");
    require(grid["resolution"], "grid_map.resolution");
    require(grid["padding"], "grid_map.padding");
    grid_resolution = grid["resolution"].as<double>();
    grid_padding = grid["padding"].as<double>();

    // 6) 读取 偏移与规划 参数
    YAML::Node offsets = root["offsets"];
    require(offsets, "offsets");
    require(offsets["left_offset"], "offsets.left_offset");
    require(offsets["right_offset"], "offsets.right_offset");
    require(offsets["printer_type"], "offsets.printer_type");
    left_offset = offsets["left_offset"].as<double>();
    right_offset = offsets["right_offset"].as<double>();
    printer_type_str = offsets["printer_type"].as<std::string>();

    YAML::Node planner = root["path_planner"];
    require(planner, "path_planner");
    require(planner["transition_speed"], "path_planner.transition_speed");
    require(planner["drawing_speed"], "path_planner.drawing_speed");
    require(planner["path_extension_length"], "path_planner.path_extension_length");
    transition_speed = planner["transition_speed"].as<double>();
    drawing_speed = planner["drawing_speed"].as<double>();
    path_extension_length = planner["path_extension_length"].as<double>();

    // 7) 读取 轨迹 参数
    YAML::Node traj = root["trajectory"];
    require(traj, "trajectory");
    require(traj["max_velocity"], "trajectory.max_velocity");
    require(traj["max_acceleration"], "trajectory.max_acceleration");
    max_velocity = traj["max_velocity"].as<double>();
    max_acceleration = traj["max_acceleration"].as<double>();

    // 8) 基于读取参数进行配置对象赋值与检查
    if (cad_file_path.empty())
    {
      RCLCPP_FATAL(this->get_logger(), "CAD文件路径未提供或为空");
      throw std::runtime_error("缺少CAD文件路径");
    }

    if (save_path_visualization)
    {
      std::error_code ec;
      std::filesystem::create_directories(path_visualization_dir, ec);
      if (ec)
      {
        RCLCPP_ERROR(this->get_logger(), "创建可视化目录失败: %s", ec.message().c_str());
      }
    }

    // 打印机类型
    PrinterType printer_type = PrinterType::LEFT_PRINTER;
    if (printer_type_str == "RIGHT_PRINTER")
    {
      printer_type = PrinterType::RIGHT_PRINTER;
    }
    else if (printer_type_str == "BOTH_PRINTERS")
    {
      printer_type = PrinterType::BOTH_PRINTERS;
    }
    else if (printer_type_str == "LEFT_PRINTER")
    {
      printer_type = PrinterType::LEFT_PRINTER;
    }
    else
    {
      RCLCPP_FATAL(this->get_logger(), "未知的打印机类型: %s", printer_type_str.c_str());
      throw std::runtime_error("非法的printer_type");
    }

    // CAD 解析配置
    cad_parser_config.unit_conversion_factor = cad_unit_conversion;
    cad_parser_config.auto_scale_coordinates = auto_scale;
    if (angle_unit_str == "DEGREES" || angle_unit_str == "degrees" || angle_unit_str == "deg")
    {
      cad_parser_config.angle_unit = AngleUnit::DEGREES;
    }
    else if (angle_unit_str == "RADIANS" || angle_unit_str == "radians" || angle_unit_str == "rad")
    {
      cad_parser_config.angle_unit = AngleUnit::RADIANS;
    }
    else
    {
      RCLCPP_FATAL(this->get_logger(), "未知的角度单位: %s", angle_unit_str.c_str());
      throw std::runtime_error("非法的angle_unit");
    }

    // 栅格地图参数
    grid_map_config.resolution = grid_resolution;
    grid_map_config.padding = grid_padding;

    // 路径规划参数
    path_planner_config.transition_speed = transition_speed;
    path_planner_config.drawing_speed = drawing_speed;
    path_planner_config.path_extension_length = path_extension_length;

    // 偏移参数
    offset_config.left_offset = left_offset;
    offset_config.right_offset = right_offset;
    offset_config.printer_type = printer_type;

    // 轨迹参数
    trajectory_config.max_velocity = max_velocity;
    trajectory_config.max_acceleration = max_acceleration;

    // 可视化参数（同时用于栅格与路径）
    grid_viz_config.scale = grid_map_scale;
    grid_viz_config.show_grid_lines = show_grid_lines;
    grid_viz_config.use_antialiasing = use_antialiasing;
    grid_viz_config.image_format = image_format;

    path_viz_config.scale = grid_map_scale;
    path_viz_config.show_grid_lines = show_grid_lines;
    path_viz_config.use_antialiasing = use_antialiasing;
    path_viz_config.image_format = image_format;

    RCLCPP_INFO(this->get_logger(), "配置加载完成: %s", yaml_path.c_str());
  }

private:
  // 主函数 - 调用5个子函数完成整个流程
  void plan_trajectory()
  {
    // 创建处理组件
    CADParser cad_parser(cad_parser_config);
    GridMapGenerator grid_map_generator(grid_map_config);
    PathPlanner path_planner(path_planner_config);
    TrajectoryGenerator trajectory_generator(trajectory_config);
    OutputFormatter output_formatter;

    // 1. 解析CAD文件
    CADData cad_data;
    if (!parse_cad_file(cad_parser, cad_data))
    {
      return;
    }

    // 2. 生成栅格地图
    std::vector<std::vector<int>> grid_map;
    if (!generate_grid_map(grid_map_generator, cad_data, grid_map))
    {
      return;
    }

    // 3. 规划路径
    std::vector<RouteSegment> path_segments;
    if (!plan_paths(path_planner, cad_data, grid_map, grid_map_generator, path_segments))
    {
      return;
    }

    // 4. 生成轨迹
    std::vector<ExecutionNode> all_trajectory_points;
    if (!generate_trajectory(trajectory_generator, cad_data, path_segments, all_trajectory_points))
    {
      return;
    }

    // 5. 保存规划结果为新版 CAD JSON 结构
    // 舍弃旧的设备/RoutePts 格式，改为 docs/导出JSON数据结构说明.md 结构
    {
      RCLCPP_INFO(this->get_logger(), "Formatting planned paths to CAD-style JSON");

      // 使用 CAD 源文件名作为 metadata.source_file
      std::string source_name = std::filesystem::path(cad_file_path).filename().string();

      nlohmann::json cad_style_json = output_formatter.format_planned_paths_to_cad_json(path_segments, cad_data, source_name);

      RCLCPP_INFO(this->get_logger(), "Saving CAD-style JSON to: %s", output_file_path.c_str());
      if (output_formatter.save_to_file(cad_style_json, output_file_path))
      {
        RCLCPP_INFO(this->get_logger(), "Successfully saved CAD-style JSON");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to save CAD-style JSON");
      }
    }
  }

  // 1: 解析CAD文件
  bool parse_cad_file(CADParser& cad_parser, CADData& cad_data)
  {
    RCLCPP_INFO(this->get_logger(), "Parsing CAD file: %s", cad_file_path.c_str());

    if (!cad_parser.parse(cad_file_path))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse CAD file");
      return false;
    }

    // 获取CAD数据的副本，以便我们可以修改它
    cad_data = cad_parser.get_cad_data();

    // 打印路径线信息
    RCLCPP_INFO(this->get_logger(), "Parsed CAD data with %zu path lines", cad_data.path_lines.size());

    return true;
  }

  // 2: 生成栅格地图
  bool generate_grid_map(GridMapGenerator& grid_map_generator, const CADData& cad_data,
                         std::vector<std::vector<int>>& grid_map)
  {
    RCLCPP_INFO(this->get_logger(), "Generating grid map");

    if (!grid_map_generator.generate_from_cad(cad_data))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to generate grid map");
      return false;
    }

    // 获取栅格地图数据
    grid_map = grid_map_generator.get_grid_map();

    if (save_path_visualization)
    {
      grid_map_image_path = path_visualization_dir + "/grid_map." + image_format;
      // 保存栅格地图为图片
      RCLCPP_INFO(this->get_logger(), "Saving grid map as image: %s", grid_map_image_path.c_str());
      if (!grid_map_generator.save_as_image(grid_map_image_path, cad_data, grid_viz_config))
      {
        RCLCPP_WARN(this->get_logger(), "Failed to save grid map as image, but continuing with path planning");
      }
    }

    return true;
  }

  // 3: 规划路径
  bool plan_paths(PathPlanner& path_planner, const CADData& cad_data, const std::vector<std::vector<int>>& grid_map,
                  GridMapGenerator& grid_map_generator, std::vector<RouteSegment>& path_segments)
  {
    // 设置路径规划器的栅格地图
    path_planner.set_grid_map(grid_map, &grid_map_generator);

    // 使用简化的方法规划路径 - 从CAD数据规划
    RCLCPP_INFO(this->get_logger(), "Planning paths from CAD data");

    // 使用简化的路径规划器处理CAD数据
    path_segments = path_planner.plan_paths(cad_data, offset_config);

    if (path_segments.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "No path segments generated");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Generated %zu path segments", path_segments.size());

    // 保存路径可视化
    if (save_path_visualization)
    {
      std::string all_paths_viz_file = path_visualization_dir + "/all_paths." + image_format;

      RCLCPP_INFO(this->get_logger(), "Saving all paths visualization as image: %s", all_paths_viz_file.c_str());

      if (!path_planner.visualize_paths(path_segments, all_paths_viz_file, path_viz_config))
      {
        RCLCPP_WARN(this->get_logger(), "Failed to save all paths visualization as image");
      }
    }

    return true;
  }

  // 4: 生成轨迹
  bool generate_trajectory(TrajectoryGenerator& trajectory_generator, const CADData& cad_data,
                           const std::vector<RouteSegment>& path_segments, std::vector<ExecutionNode>& all_trajectory_points)
  {
    all_trajectory_points.clear();

    for (const auto& segment : path_segments)
    {
      RCLCPP_INFO(this->get_logger(), "Processing path segment (type: %s, line ID: %d)",
                  segment.type == RouteType::DRAWING_PATH ? "DRAWING" : "TRANSITION", segment.line_id);

      if (segment.points.empty())
      {
        RCLCPP_WARN(this->get_logger(), "Empty path segment, skipping");
        continue;
      }

      // 处理绘图路径或转场路径
      std::vector<ExecutionNode> trajectory_points;

      if (segment.type == RouteType::DRAWING_PATH && segment.line_id >= 0)
      {
        // 绘图路径 - 查找匹配的线
        std::shared_ptr<Line> line_ptr;
        for (const auto& line : cad_data.path_lines)
        {
          if (line->id == segment.line_id)
          {
            line_ptr = line;
            break;
          }
        }

        if (line_ptr)
        {
          if (line_ptr->type == GeometryType::LINE)
          {
            // 直线类型使用三个参数，指明这是绘图路径
            trajectory_points = trajectory_generator.generate_from_path(segment.points, *line_ptr, true);
          }
          else if (line_ptr->type == GeometryType::CIRCLE)
          {
            auto circle_ptr = std::dynamic_pointer_cast<Circle>(line_ptr);
            if (circle_ptr)
            {
              trajectory_points = trajectory_generator.generate_from_circle(*circle_ptr, offset_config);
            }
          }
          else if (line_ptr->type == GeometryType::ARC)
          {
            auto arc_ptr = std::dynamic_pointer_cast<Arc>(line_ptr);
            if (arc_ptr)
            {
              trajectory_points = trajectory_generator.generate_from_arc(*arc_ptr, offset_config);
            }
          }
          else if (line_ptr->type == GeometryType::CURVE)
          {
            auto curve_ptr = std::dynamic_pointer_cast<Curve>(line_ptr);
            if (curve_ptr)
            {
              trajectory_points = trajectory_generator.generate_from_curve(*curve_ptr, offset_config);
            }
            else
            {
              // 如果无法转换为曲线，尝试使用离散化的路径点
              RCLCPP_WARN(this->get_logger(), "Failed to cast to Curve, using path points directly for line ID: %d",
                          line_ptr->id);
              trajectory_points = trajectory_generator.generate_from_path(segment.points, *line_ptr, true);
            }
          }
          else
          {
            // 对于未知类型，使用通用路径生成
            RCLCPP_WARN(this->get_logger(), "Unknown line type: %d, using generic path for line ID: %d",
                        static_cast<int>(line_ptr->type), line_ptr->id);
            trajectory_points = trajectory_generator.generate_from_path(segment.points, *line_ptr, true);
          }
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "Line with ID %d not found in CAD data", segment.line_id);
          // 创建虚拟线用于生成轨迹
          Line virtual_line(segment.line_id, segment.points.front(), segment.points.back());
          trajectory_points = trajectory_generator.generate_from_path(segment.points, virtual_line, true);
        }
      }
      else
      {
        // 转场路径 - 创建虚拟线
        Line virtual_line(-1, segment.points.front(), segment.points.back());
        // 明确指定这是转场路径，而不是绘图路径
        trajectory_points = trajectory_generator.generate_from_path(segment.points, virtual_line, false);
      }

      // 添加到总轨迹
      if (!trajectory_points.empty())
      {
        all_trajectory_points.insert(all_trajectory_points.end(), trajectory_points.begin(), trajectory_points.end());

        RCLCPP_INFO(this->get_logger(), "Added %zu trajectory points", trajectory_points.size());
      }
    }

    // 如果没有生成任何轨迹点，则报错并退出
    if (all_trajectory_points.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "No trajectory points generated");
      return false;
    }

    return true;
  }

  // 旧格式保存逻辑已废弃

  std::string cad_file_path, output_file_path, grid_map_image_path, image_format, printer_type_str;
  std::string path_visualization_dir;
  int grid_map_scale;
  double grid_resolution, grid_padding, cad_unit_conversion;
  bool show_grid_lines, use_antialiasing, save_path_visualization;
  double left_offset, right_offset, max_velocity, max_acceleration;
  double transition_speed, drawing_speed, path_extension_length;

  // 设置CAD解析配置
  CADParserConfig cad_parser_config;
  // 设置栅格地图配置
  GridMapConfig grid_map_config;
  // 设置路径规划配置
  PathPlannerConfig path_planner_config;
  // 设置路径偏移配置
  PathOffsetConfig offset_config;
  // 设置轨迹生成配置
  TrajectoryConfig trajectory_config;
  // 设置栅格地图可视化配置
  GridMapVisualizationConfig grid_viz_config;
  // 设置路径可视化配置
  PathVisualizationConfig path_viz_config;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
