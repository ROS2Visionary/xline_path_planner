#include <memory>
#include <string>
#include <fstream>
#include <filesystem>
#include <system_error>
#include <thread>
#include <cctype>

#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "xline_path_planner/srv/plan_path.hpp"

#include "xline_path_planner/cad_parser.hpp"
#include "xline_path_planner/grid_map_generator.hpp"
#include "xline_path_planner/path_planner.hpp"
#include "xline_path_planner/trajectory_generator.hpp"
#include "xline_path_planner/output_formatter.hpp"

using namespace daosnrs_planning;

struct PlannerEngineConfig
{
  CADParserConfig cad_parser;
  GridMapConfig grid_map;
  PathPlannerConfig path_planner;
  PathOffsetConfig offsets;
  TrajectoryConfig trajectory;
};

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode() : Node("drawing_planner_node")
  {
    // 加载基础配置（严格 YAML）
    updateParameters();
    // 启动 Service Server
    start_service_server();
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

    // 2) 读取 可视化 参数（用于栅格与路径可视化）
    YAML::Node vis = root["visualization"];
    require(vis, "visualization");
    require(vis["scale"], "visualization.scale");
    require(vis["show_grid_lines"], "visualization.show_grid_lines");
    require(vis["use_antialiasing"], "visualization.use_antialiasing");
    require(vis["image_format"], "visualization.image_format");
    require(vis["save_path_visualization"], "visualization.save_path_visualization");
    require(vis["output_dir"], "visualization.output_dir");
    grid_map_scale = vis["scale"].as<int>();
    show_grid_lines = vis["show_grid_lines"].as<bool>();
    use_antialiasing = vis["use_antialiasing"].as<bool>();
    image_format = vis["image_format"].as<std::string>();
    save_path_visualization = vis["save_path_visualization"].as<bool>();
    visualization_output_dir = vis["output_dir"].as<std::string>();

    // 3) 读取 CAD 解析 参数
    YAML::Node cad = root["cad_parser"];
    require(cad, "cad_parser");
    require(cad["unit_conversion_factor"], "cad_parser.unit_conversion_factor");
    require(cad["auto_scale_coordinates"], "cad_parser.auto_scale_coordinates");
    require(cad["angle_unit"], "cad_parser.angle_unit");
    require(cad["cad_file_dir"], "cad_parser.cad_file_dir");
    cad_unit_conversion = cad["unit_conversion_factor"].as<double>();
    bool auto_scale = cad["auto_scale_coordinates"].as<bool>();
    std::string angle_unit_str = cad["angle_unit"].as<std::string>();
    cad_files_dir_ = cad["cad_file_dir"].as<std::string>();

    // 4) 读取 栅格地图 参数
    YAML::Node grid = root["grid_map"];
    require(grid, "grid_map");
    require(grid["resolution"], "grid_map.resolution");
    require(grid["padding"], "grid_map.padding");
    grid_resolution = grid["resolution"].as<double>();
    grid_padding = grid["padding"].as<double>();

    // 5) 读取 偏移与规划 参数
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

    // 6) 读取 轨迹 参数
    YAML::Node traj = root["trajectory"];
    require(traj, "trajectory");
    require(traj["max_velocity"], "trajectory.max_velocity");
    require(traj["max_acceleration"], "trajectory.max_acceleration");
    max_velocity = traj["max_velocity"].as<double>();
    max_acceleration = traj["max_acceleration"].as<double>();

    // 7) 基于读取参数进行配置对象赋值与检查（不涉及任何文件路径）

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

  // 启动 Service Server（plan_path）
  void start_service_server()
  {
    using namespace std::placeholders;
    service_ = this->create_service<xline_path_planner::srv::PlanPath>(
      "plan_path",
      std::bind(&PlannerNode::handle_service, this, _1, _2, _3)
    );
    RCLCPP_INFO(this->get_logger(), "Service server 'plan_path' is ready");
  }

  // 将当前节点配置打包为引擎配置
  PlannerEngineConfig build_engine_cfg_from_node() const
  {
    PlannerEngineConfig cfg;
    cfg.cad_parser = cad_parser_config;
    cfg.grid_map = grid_map_config;
    cfg.path_planner = path_planner_config;
    cfg.offsets = offset_config;
    cfg.trajectory = trajectory_config;
    return cfg;
  }

  // Service: 处理请求
  void handle_service(const std::shared_ptr<rmw_request_id_t>,
                      const std::shared_ptr<xline_path_planner::srv::PlanPath::Request> req,
                      std::shared_ptr<xline_path_planner::srv::PlanPath::Response> res)
  {
    try
    {
      // 1) 构造配置
      PlannerEngineConfig cfg = build_engine_cfg_from_node();

      // 2) 解析 CAD 文件：文件名 + 配置目录 拼完整路径
      if (req->file_name.empty())
      {
        throw std::runtime_error("请求中的 file_name 为空");
      }
      std::filesystem::path input_path(req->file_name);
      if (!input_path.is_absolute())
      {
        if (cad_files_dir_.empty())
        {
          throw std::runtime_error("配置 cad_parser.cad_file_dir 为空，无法构建CAD文件完整路径");
        }
        input_path = std::filesystem::path(cad_files_dir_) / input_path;
      }
      if (!std::filesystem::exists(input_path))
      {
        throw std::runtime_error(std::string("CAD 文件不存在: ") + input_path.string());
      }
      CADParser cad_parser(cfg.cad_parser);
      if (!cad_parser.parse(input_path.string()))
      {
        throw std::runtime_error("CAD 文件解析失败: " + input_path.string());
      }
      CADData cad_data = cad_parser.get_cad_data();

      // 3) 生成栅格地图
      GridMapGenerator grid_map_generator(cfg.grid_map);
      if (!grid_map_generator.generate_from_cad(cad_data))
      {
        throw std::runtime_error("栅格地图生成失败");
      }
      const auto& grid_map = grid_map_generator.get_grid_map();

      // 4) 路径规划
      PathPlanner path_planner(cfg.path_planner);
      path_planner.set_grid_map(grid_map, &grid_map_generator);
      auto path_segments = path_planner.plan_paths(cad_data, cfg.offsets);
      if (path_segments.empty())
      {
        throw std::runtime_error("路径规划失败，未生成任何路径段");
      }

      // 5) 轨迹生成（可选：为了验证路径可行性）
      TrajectoryGenerator trajectory_generator(cfg.trajectory);
      std::vector<ExecutionNode> all_nodes;
      for (const auto& seg : path_segments)
      {
        if (seg.points.size() < 2) continue;
        Line virtual_line(seg.line_id, seg.points.front(), seg.points.back());
        auto nodes = trajectory_generator.generate_from_path(
          seg.points, virtual_line, seg.type == RouteType::DRAWING_PATH);
        if (!nodes.empty())
        {
          all_nodes.insert(all_nodes.end(), nodes.begin(), nodes.end());
        }
      }

      // 6) 格式化输出（内存）
      OutputFormatter formatter;
      // 使用输入文件名作为来源标签
      std::string source_name = input_path.filename().string();
      std::string out_json = formatter
        .format_planned_paths_to_cad_json(path_segments, cad_data, source_name)
        .dump(2);

      // 7) 可视化/结果持久化到目录
      if (save_path_visualization)
      {
        try
        {
          std::filesystem::path out_dir = visualization_output_dir;
          std::error_code ec;
          std::filesystem::create_directories(out_dir, ec);
          if (ec)
          {
            RCLCPP_WARN(this->get_logger(), "创建输出目录失败: %s", ec.message().c_str());
          }
          auto grid_img = (out_dir / (std::string("grid_map.") + image_format)).string();
          auto path_img = (out_dir / (std::string("planned_paths.") + image_format)).string();
          (void)grid_map_generator.save_as_image(grid_img, cad_data, grid_viz_config);
          (void)path_planner.visualize_paths(path_segments, path_img, path_viz_config);

          // 保存规划结果 JSON：基于输入文件名构造
          std::string file_safe_id = input_path.stem().string();
          if (file_safe_id.empty()) file_safe_id = "request";
          for (char& ch : file_safe_id)
          {
            unsigned char uch = static_cast<unsigned char>(ch);
            if (!(std::isalnum(uch) || ch == '-' || ch == '_')) ch = '_';
          }
          auto json_path = (out_dir / (std::string("planned_") + file_safe_id + ".json")).string();
          std::ofstream jf(json_path);
          if (jf.is_open())
          {
            jf << out_json;
            jf.close();
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "保存规划JSON失败: 无法打开文件 %s", json_path.c_str());
          }
        }
        catch (const std::exception& e)
        {
          RCLCPP_WARN(this->get_logger(), "保存可视化图像失败: %s", e.what());
        }
      }

      res->success = true;
      res->error = "";
      res->message = std::string("规划完成，结果已保存至目录: ") + visualization_output_dir;
    }
    catch (const std::exception& e)
    {
      res->success = false;
      res->error = e.what();
      res->message = "";
      RCLCPP_ERROR(this->get_logger(), "服务处理失败: %s", e.what());
    }
  }

  // 成员变量
  rclcpp::Service<xline_path_planner::srv::PlanPath>::SharedPtr service_;

  std::string image_format, printer_type_str;
  std::string cad_files_dir_;
  std::string visualization_output_dir;
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
