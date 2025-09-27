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
#include "rclcpp_action/rclcpp_action.hpp"
#include "xline_path_planner/action/plan_path.hpp"

#include "xline_path_planner/cad_parser.hpp"
#include "xline_path_planner/grid_map_generator.hpp"
#include "xline_path_planner/path_planner.hpp"
#include "xline_path_planner/trajectory_generator.hpp"
#include "xline_path_planner/output_formatter.hpp"
#include "xline_path_planner/planner_engine.hpp"

using namespace daosnrs_planning;

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode() : Node("drawing_planner_node")
  {
    // 加载基础配置（严格 YAML）
    updateParameters();
    // 启动 Action Server
    start_action_server();
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
    cad_unit_conversion = cad["unit_conversion_factor"].as<double>();
    bool auto_scale = cad["auto_scale_coordinates"].as<bool>();
    std::string angle_unit_str = cad["angle_unit"].as<std::string>();

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

  // 启动 Action Server（plan_path）
  void start_action_server()
  {
    using PlanPath = xline_path_planner::action::PlanPath;
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<PlanPath>(
      this,
      "plan_path",
      std::bind(&PlannerNode::handle_goal, this, _1, _2),
      std::bind(&PlannerNode::handle_cancel, this, _1),
      std::bind(&PlannerNode::handle_accepted, this, _1)
    );
    RCLCPP_INFO(this->get_logger(), "Action server 'plan_path' is ready");
  }

  // Action: 接收目标
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                          std::shared_ptr<const xline_path_planner::action::PlanPath::Goal> goal)
  {
    if (goal->cad_json.empty())
    {
      RCLCPP_WARN(this->get_logger(), "拒绝空 cad_json 的请求");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Action: 处理取消
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<xline_path_planner::action::PlanPath>>)
  {
    RCLCPP_INFO(this->get_logger(), "收到取消请求");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Action: 接受后执行
  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<xline_path_planner::action::PlanPath>> goal_handle)
  {
    std::thread([this, goal_handle]() { this->execute(goal_handle); }).detach();
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

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<xline_path_planner::action::PlanPath>> goal_handle)
  {
    using PlanPath = xline_path_planner::action::PlanPath;
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<PlanPath::Feedback>();
    auto result = std::make_shared<PlanPath::Result>();

    try
    {
      // 1) 构造配置
      PlannerEngineConfig cfg = build_engine_cfg_from_node();

      // 2) 解析 CAD JSON
      feedback->progress = 5; feedback->stage = "parse"; feedback->info = "解析CAD"; goal_handle->publish_feedback(feedback);
      CADParser cad_parser(cfg.cad_parser);
      if (!cad_parser.parse_from_string(goal->cad_json))
      {
        throw std::runtime_error("CAD JSON 解析失败");
      }
      CADData cad_data = cad_parser.get_cad_data();

      // 3) 生成栅格地图
      feedback->progress = 25; feedback->stage = "grid"; feedback->info = "生成栅格"; goal_handle->publish_feedback(feedback);
      GridMapGenerator grid_map_generator(cfg.grid_map);
      if (!grid_map_generator.generate_from_cad(cad_data))
      {
        throw std::runtime_error("栅格地图生成失败");
      }
      const auto& grid_map = grid_map_generator.get_grid_map();

      // 4) 路径规划
      feedback->progress = 55; feedback->stage = "plan"; feedback->info = "规划路径"; goal_handle->publish_feedback(feedback);
      PathPlanner path_planner(cfg.path_planner);
      path_planner.set_grid_map(grid_map, &grid_map_generator);
      auto path_segments = path_planner.plan_paths(cad_data, cfg.offsets);
      if (path_segments.empty())
      {
        throw std::runtime_error("路径规划失败，未生成任何路径段");
      }

      // 5) 轨迹生成（可选，当前仅用于校验，不影响最终JSON输出）
      feedback->progress = 75; feedback->stage = "traj"; feedback->info = "生成轨迹"; goal_handle->publish_feedback(feedback);
      TrajectoryGenerator trajectory_generator(cfg.trajectory);
      std::vector<ExecutionNode> all_nodes;
      for (const auto& seg : path_segments)
      {
        if (seg.points.size() < 2) continue;
        Line virtual_line(seg.line_id, seg.points.front(), seg.points.back());
        auto nodes = trajectory_generator.generate_from_path(seg.points, virtual_line, seg.type == RouteType::DRAWING_PATH);
        if (!nodes.empty())
        {
          all_nodes.insert(all_nodes.end(), nodes.begin(), nodes.end());
        }
      }

      // 6) 格式化输出
      feedback->progress = 90; feedback->stage = "format"; feedback->info = "格式化输出"; goal_handle->publish_feedback(feedback);
      OutputFormatter formatter;
      std::string out_json = formatter.format_planned_paths_to_cad_json(path_segments, cad_data, "request.json").dump(2);

      // 7) 可视化持久化：根据配置将图像与规划JSON保存到目录
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

          // 另存规划结果 JSON
          std::string file_safe_id = goal->request_id.empty() ? std::string("request") : goal->request_id;
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

      feedback->progress = 100; feedback->stage = "done"; feedback->info = "完成"; goal_handle->publish_feedback(feedback);

      result->success = true;
      result->planned_json = std::move(out_json);
      result->error = "";
      result->warnings = "";
      result->metrics_json = "{}";
      result->planner_version = "xline_path_planner-0.0.0";
      result->config_hash = "";
      goal_handle->succeed(result);
    }
    catch (const std::exception& e)
    {
      result->success = false;
      result->planned_json = "";
      result->error = e.what();
      result->warnings = "";
      result->metrics_json = "{}";
      result->planner_version = "xline_path_planner-0.0.0";
      result->config_hash = "";
      goal_handle->abort(result);
    }
  }

  // 已移除基于文件的直通式运行辅助函数

  // 成员变量
  rclcpp_action::Server<xline_path_planner::action::PlanPath>::SharedPtr action_server_;

  std::string image_format, printer_type_str;
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
