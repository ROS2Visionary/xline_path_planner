#include <memory>
#include <string>
#include <fstream>
#include <filesystem>
#include <system_error>
#include <thread>
#include <cctype>
#include <mutex>
#include <atomic>
#include <cstdlib>

// TCP server includes
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "xline_path_planner/srv/plan_path.hpp"

#include "xline_path_planner/cad_parser.hpp"
#include "xline_path_planner/grid_map_generator.hpp"
#include "xline_path_planner/path_planner.hpp"
#include "xline_path_planner/trajectory_generator.hpp"
#include "xline_path_planner/output_formatter.hpp"
#include "xline_path_planner/filename_utils.hpp"

using namespace path_planner;

static std::string expand_env(const std::string& path)
{
  std::string result;
  result.reserve(path.size());

  for (size_t i = 0; i < path.size(); ++i)
  {
    if (path[i] == '$')
    {
      std::string var;
      size_t j = i + 1;

      if (j < path.size() && path[j] == '{')
      {
        ++j;
        while (j < path.size() && path[j] != '}')
        {
          var += path[j++];
        }
        i = (j < path.size()) ? j : path.size() - 1;
      }
      else
      {
        while (j < path.size()
               && (std::isalnum(static_cast<unsigned char>(path[j])) || path[j] == '_'))
        {
          var += path[j++];
        }
        i = j - 1;
      }

      const char* val = std::getenv(var.c_str());
      if (val)
      {
        result += val;
      }
    }
    else
    {
      result += path[i];
    }
  }

  return result;
}

static std::string resolve_path(const std::string& raw)
{
  std::string p = expand_env(raw);

  if (!p.empty() && p[0] == '/')
  {
    return p;
  }

  const char* ws_root = std::getenv("XLINE_WS_ROOT");
  if (ws_root && *ws_root)
  {
    if (p.empty())
    {
      return std::string(ws_root);
    }
    return std::string(ws_root) + "/" + p;
  }

  return p;
}

struct PlannerEngineConfig
{
  CADParserConfig cad_parser;
  GridMapConfig grid_map;
  PathPlannerConfig path_planner;
  PathOffsetConfig offsets;
};

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode() : Node("drawing_planner_node")
  {
    // 加载基础配置（严格 YAML）
    updateParameters();
    // 订阅机器人位置
    start_robot_pose_subscriber();
    // 启动 Service Server
    start_service_server();
    // 启动 TCP Server（如启用）
    start_tcp_server();
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
    require(vis["planned_output_dir"], "visualization.planned_output_dir");
    grid_map_scale = vis["scale"].as<int>();
    show_grid_lines = vis["show_grid_lines"].as<bool>();
    use_antialiasing = vis["use_antialiasing"].as<bool>();
    image_format = vis["image_format"].as<std::string>();
    save_path_visualization = vis["save_path_visualization"].as<bool>();
    visualization_output_dir = resolve_path(vis["output_dir"].as<std::string>());
    planned_output_dir_ = resolve_path(vis["planned_output_dir"].as<std::string>());

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
    cad_files_dir_ = resolve_path(cad["cad_file_dir"].as<std::string>());

    // 3.1) 读取 TCP Server 参数（可选）
    YAML::Node tcp = root["tcp_server"];
    if (tcp && !tcp.IsNull())
    {
      if (tcp["enabled"]) tcp_enabled_ = tcp["enabled"].as<bool>();
      if (tcp["host"]) tcp_host_ = tcp["host"].as<std::string>();
      if (tcp["port"]) tcp_port_ = tcp["port"].as<int>();
    }

    // 3.2) 读取 Robot Start 参数（可选）
    YAML::Node robot_start = root["robot_start"];
    if (robot_start && !robot_start.IsNull())
    {
      if (robot_start["enabled"]) robot_start_enabled_ = robot_start["enabled"].as<bool>();
      if (robot_start["pose_topic"]) robot_pose_topic_ = robot_start["pose_topic"].as<std::string>();
      if (robot_start["pose_timeout_sec"]) robot_pose_timeout_ = robot_start["pose_timeout_sec"].as<double>();
      if (robot_start["required"]) robot_pose_required_ = robot_start["required"].as<bool>();
      if (robot_start["max_initial_distance"]) max_initial_distance_ = robot_start["max_initial_distance"].as<double>();
    }

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
    require(offsets["center_offset"], "offsets.center_offset");
    left_offset = offsets["left_offset"].as<double>();
    right_offset = offsets["right_offset"].as<double>();
    center_offset = offsets["center_offset"].as<double>();

    YAML::Node planner = root["path_planner"];
    require(planner, "path_planner");
    require(planner["path_extension_length"], "path_planner.path_extension_length");
    path_extension_length = planner["path_extension_length"].as<double>();

    // 6) 基于读取参数进行配置对象赋值与检查（不涉及任何文件路径）

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
    path_planner_config.path_extension_length = path_extension_length;

    // 偏移参数
    offset_config.left_offset = left_offset;
    offset_config.right_offset = right_offset;
    offset_config.center_offset = center_offset;

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

  // 启动机器人位置订阅器
  void start_robot_pose_subscriber()
  {
    if (!robot_start_enabled_)
    {
      RCLCPP_INFO(this->get_logger(), "机器人起始位置功能已禁用");
      return;
    }
    robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      robot_pose_topic_,
      rclcpp::QoS{10},
      std::bind(&PlannerNode::robot_pose_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "订阅机器人位置话题: %s", robot_pose_topic_.c_str());
  }

  // 机器人位置回调函数
  void robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(robot_pose_mutex_);
    robot_pose_ = *msg;
    RCLCPP_DEBUG(this->get_logger(), "收到机器人位置: [%.3f, %.3f, %.3f]",
                 msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
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
    return cfg;
  }

  // 执行实际规划（线程安全），输入为完整文件路径
  bool plan_from_input_path(const std::filesystem::path& input_path, std::string& out_message, std::string& out_error,
                             const std::optional<Point3D>& robot_start_position = std::nullopt)
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    try
    {
      PlannerEngineConfig cfg = build_engine_cfg_from_node();

      // 解析 CAD 文件
      CADParser cad_parser(cfg.cad_parser);
      if (!cad_parser.parse(input_path.string()))
      {
        throw std::runtime_error("CAD 文件解析失败: " + input_path.string());
      }
      CADData cad_data = cad_parser.get_cad_data();

      // 生成栅格
      GridMapGenerator grid_map_generator(cfg.grid_map);
      if (!grid_map_generator.generate_from_cad(cad_data))
      {
        throw std::runtime_error("栅格地图生成失败");
      }
      const auto& grid_map = grid_map_generator.get_grid_map();

      // 路径规划（传递机器人起始位置）
      PathPlanner path_planner(cfg.path_planner);
      path_planner.set_grid_map(grid_map, &grid_map_generator);
      auto path_segments = path_planner.plan_paths(cad_data, cfg.offsets, robot_start_position);
      if (path_segments.empty())
      {
        throw std::runtime_error("路径规划失败，未生成任何路径段");
      }

      // 轨迹生成
      TrajectoryGenerator trajectory_generator;
      std::vector<ExecutionNode> all_nodes;
      for (const auto& seg : path_segments)
      {
        if (seg.points.size() < 2) continue;
        Line virtual_line(seg.line_id, seg.points.front(), seg.points.back());
        auto nodes = trajectory_generator.generate_from_path(
          seg.points, virtual_line, seg.type == RouteType::DRAWING_PATH, seg.printer_type);
        if (!nodes.empty())
        {
          all_nodes.insert(all_nodes.end(), nodes.begin(), nodes.end());
        }
      }

      // 格式化输出
      OutputFormatter formatter;
      std::string source_name = input_path.filename().string();
      std::string out_json = formatter
        .format_planned_paths_to_cad_json(path_segments, cad_data, source_name)
        .dump(2);

      // 可视化图像保存（可选）
      if (save_path_visualization)
      {
        try
        {
          std::filesystem::path vis_dir = visualization_output_dir;
          std::error_code ec;
          std::filesystem::create_directories(vis_dir, ec);
          if (ec)
          {
            RCLCPP_WARN(this->get_logger(), "创建可视化输出目录失败: %s", ec.message().c_str());
          }
          auto grid_img = (vis_dir / (std::string("grid_map.") + image_format)).string();
          auto path_img = (vis_dir / (std::string("planned_paths.") + image_format)).string();
          (void)grid_map_generator.save_as_image(grid_img, cad_data, grid_viz_config);
          (void)path_planner.visualize_paths(path_segments, path_img, path_viz_config);
        }
        catch (const std::exception& e)
        {
          RCLCPP_WARN(this->get_logger(), "保存可视化图像失败: %s", e.what());
        }
      }

      // 始终保存规划 JSON 至 planned_output_dir_
      std::string json_path_str;
      try
      {
        std::filesystem::path out_dir = planned_output_dir_;
        std::error_code ec;
        std::filesystem::create_directories(out_dir, ec);
        if (ec)
        {
          RCLCPP_WARN(this->get_logger(), "创建规划结果输出目录失败: %s", ec.message().c_str());
        }
        std::filesystem::path json_path = planned_json_output_path(out_dir, input_path);
        json_path_str = json_path.u8string();
        std::ofstream jf(json_path);
        if (jf.is_open())
        {
          jf << out_json;
          jf.close();
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "保存规划JSON失败: 无法打开文件 %s", json_path_str.c_str());
        }
      }
      catch (const std::exception& e)
      {
        RCLCPP_WARN(this->get_logger(), "保存规划JSON失败: %s", e.what());
      }

      // 汇总消息
      out_message = std::string("规划完成。JSON 目录: ") + planned_output_dir_;
      if (save_path_visualization)
      {
        out_message += std::string("；可视化目录: ") + visualization_output_dir;
      }
      out_error.clear();
      return true;
    }
    catch (const std::exception& e)
    {
      out_message.clear();
      out_error = e.what();
      return false;
    }
  }

  // Service: 处理请求
  void handle_service(const std::shared_ptr<rmw_request_id_t>,
                      const std::shared_ptr<xline_path_planner::srv::PlanPath::Request> req,
                      std::shared_ptr<xline_path_planner::srv::PlanPath::Response> res)
  {
    try
    {
      // 1) 获取机器人位置（如果启用且可用）
      std::optional<Point3D> robot_start;
      if (robot_start_enabled_)
      {
        RCLCPP_INFO(this->get_logger(), "机器人起始位置功能已启用，开始检查位置...");

        std::lock_guard<std::mutex> lock(robot_pose_mutex_);
        bool pose_valid = false;

        if (robot_pose_.has_value())
        {
          // 检查位置是否过期
          auto now = this->get_clock()->now();
          auto pose_time = rclcpp::Time(robot_pose_->header.stamp);
          auto age = (now - pose_time).seconds();

          RCLCPP_INFO(this->get_logger(), "已收到机器人位置，时间戳年龄: %.2f 秒 (超时限制: %.2f 秒)",
                      age, robot_pose_timeout_);

          // bool is_timeout_ = age > robot_pose_timeout_;
          // TODO 暂时不检查是否超时
          bool is_timeout_ = false;
          if (is_timeout_)
          {
            
            RCLCPP_WARN(this->get_logger(), "❌ 机器人位置已过期 (%.1f秒前，超时限制: %.1f秒)", age, robot_pose_timeout_);
          }
          else
          {
            pose_valid = true;
            const auto& pose = robot_pose_->pose.position;
            robot_start = Point3D{pose.x, pose.y, pose.z};
            RCLCPP_INFO(this->get_logger(), "✅ 使用机器人起始位置: [%.3f, %.3f, %.3f]", pose.x, pose.y, pose.z);
          }
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "❌ 未收到任何机器人位置消息");
          RCLCPP_INFO(this->get_logger(), "   提示：请确保有节点发布位置到话题: %s", robot_pose_topic_.c_str());
        }

        if (!pose_valid)
        {
          if (robot_pose_required_)
          {
            std::string error_msg = "机器人位置不可用！\n";
            error_msg += "  - 配置要求必须有位置 (required: true)\n";
            error_msg += "  - 位置话题: " + robot_pose_topic_ + "\n";
            if (!robot_pose_.has_value())
            {
              error_msg += "  - 状态: 未收到任何位置消息\n";
              error_msg += "  - 解决方案: 请先发布机器人位置，或将 planner.yaml 中的 required 设置为 false";
            }
            else
            {
              error_msg += "  - 状态: 位置已过期\n";
              error_msg += "  - 解决方案: 请在调用服务前 " + std::to_string(robot_pose_timeout_) + " 秒内发布位置";
            }
            RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
            throw std::runtime_error(error_msg);
          }
          RCLCPP_WARN(this->get_logger(), "⚠️  未收到机器人位置或位置已过期，使用默认起点规划");
        }
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "机器人起始位置功能已禁用，使用默认规划方式");
      }

      // 2) 构造输入路径并校验（配置在执行函数中使用）

      // 3) 解析 CAD 文件：文件名 + 配置目录 拼完整路径
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
      std::string message, error;
      bool ok = plan_from_input_path(input_path, message, error, robot_start);
      res->success = ok;
      res->error = ok ? std::string("") : error;
      res->message = ok ? message : std::string("");
    }
    catch (const std::exception& e)
    {
      res->success = false;
      res->error = e.what();
      res->message = "";
      RCLCPP_ERROR(this->get_logger(), "服务处理失败: %s", e.what());
    }
  }

  // TCP Server：启动（如启用）
  void start_tcp_server()
  {
    if (!tcp_enabled_) {
      RCLCPP_INFO(this->get_logger(), "TCP server disabled by config");
      return;
    }
    tcp_running_.store(true);
    tcp_thread_ = std::thread([this]() { this->tcp_server_loop(); });
    tcp_thread_.detach();
    RCLCPP_INFO(this->get_logger(), "TCP server listening on %s:%d", tcp_host_.c_str(), tcp_port_);
  }

  // TCP Server 主循环
  void tcp_server_loop()
  {
    int server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "TCP socket 创建失败: %s", std::strerror(errno));
      return;
    }
    tcp_server_fd_ = server_fd;

    int opt = 1;
    ::setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(tcp_port_));
    addr.sin_addr.s_addr = (tcp_host_ == "0.0.0.0") ? INADDR_ANY : ::inet_addr(tcp_host_.c_str());

    if (::bind(server_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "TCP bind 失败: %s", std::strerror(errno));
      ::close(server_fd);
      tcp_server_fd_ = -1;
      return;
    }
    if (::listen(server_fd, 4) < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "TCP listen 失败: %s", std::strerror(errno));
      ::close(server_fd);
      tcp_server_fd_ = -1;
      return;
    }

    while (rclcpp::ok() && tcp_running_.load())
    {
      sockaddr_in client_addr{};
      socklen_t len = sizeof(client_addr);
      int client_fd = ::accept(server_fd, reinterpret_cast<sockaddr*>(&client_addr), &len);
      if (client_fd < 0)
      {
        if (errno == EINTR) continue;
        RCLCPP_WARN(this->get_logger(), "TCP accept 失败: %s", std::strerror(errno));
        continue;
      }

      // 读取一行作为文件名
      std::string file_name;
      char buf[1024];
      ssize_t n;
      while ((n = ::recv(client_fd, buf, sizeof(buf), 0)) > 0)
      {
        file_name.append(buf, buf + n);
        // 以换行作为结尾
        if (file_name.find('\n') != std::string::npos) break;
      }
      // 去掉换行/回车
      while (!file_name.empty() && (file_name.back() == '\n' || file_name.back() == '\r'))
        file_name.pop_back();

      // 基于配置目录拼接路径并执行
      std::string message, error;
      bool ok = false;
      if (file_name.empty())
      {
        error = "空文件名";
      }
      else
      {
        std::filesystem::path input_path(file_name);
        if (!input_path.is_absolute())
        {
          input_path = std::filesystem::path(cad_files_dir_) / input_path;
        }
        if (std::filesystem::exists(input_path))
        {
          // 如果启用了机器人起始位置功能，则尝试使用当前机器人位置
          std::optional<Point3D> robot_start;
          if (robot_start_enabled_)
          {
            RCLCPP_INFO(this->get_logger(),
                        "TCP 请求：机器人起始位置功能已启用，尝试读取当前位置用于路径规划...");

            {
              std::lock_guard<std::mutex> lock(robot_pose_mutex_);
              if (robot_pose_.has_value())
              {
                const auto& pose = robot_pose_->pose.position;
                robot_start = Point3D{pose.x, pose.y, pose.z};
                RCLCPP_INFO(this->get_logger(),
                            "TCP 请求：使用机器人起始位置: [%.3f, %.3f, %.3f]",
                            pose.x, pose.y, pose.z);
              }
              else
              {
                RCLCPP_WARN(this->get_logger(),
                            "TCP 请求：尚未收到任何机器人位置，将使用默认起点规划");
              }
            }
          }
          else
          {
            RCLCPP_INFO(this->get_logger(),
                        "TCP 请求：机器人起始位置功能已禁用，将使用默认起点规划");
          }

          ok = plan_from_input_path(input_path, message, error, robot_start);
        }
        else
        {
          error = std::string("CAD 文件不存在: ") + input_path.string();
        }
      }

      // 构造 JSON 响应
      std::string resp = std::string("{\"success\": ") + (ok ? "true" : "false") +
                         ", \"error\": \"" + escape_json(error) +
                         "\", \"message\": \"" + escape_json(message) + "\"}\n";
      (void)::send(client_fd, resp.c_str(), resp.size(), 0);
      ::shutdown(client_fd, SHUT_RDWR);
      ::close(client_fd);
    }

    if (tcp_server_fd_ >= 0)
    {
      ::close(tcp_server_fd_);
      tcp_server_fd_ = -1;
    }
  }

  // 简单 JSON 转义（仅处理常见字符）
  std::string escape_json(const std::string& s)
  {
    std::string out;
    out.reserve(s.size());
    for (char c : s)
    {
      switch (c)
      {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default: out.push_back(c); break;
      }
    }
    return out;
  }

  // 成员变量
  rclcpp::Service<xline_path_planner::srv::PlanPath>::SharedPtr service_;

  // 机器人位置订阅器和数据
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;
  std::optional<geometry_msgs::msg::PoseStamped> robot_pose_;
  std::mutex robot_pose_mutex_;
  std::string robot_pose_topic_ = "/robot_pose";
  bool robot_start_enabled_ = true;
  double robot_pose_timeout_ = 5.0;
  bool robot_pose_required_ = false;
  double max_initial_distance_ = 2.0;

  std::string image_format;
  std::string cad_files_dir_;
  std::string planned_output_dir_;
  std::string visualization_output_dir;
  int grid_map_scale;
  double grid_resolution, grid_padding, cad_unit_conversion;
  bool show_grid_lines, use_antialiasing, save_path_visualization;
  double left_offset, right_offset, center_offset;
  double path_extension_length;

  // 规划互斥，防止并发执行
  std::mutex plan_mutex_;

  // TCP server 配置与状态
  bool tcp_enabled_ = true;          // 默认启用，若配置存在则以配置为准
  std::string tcp_host_ = "0.0.0.0";
  int tcp_port_ = 52088;
  std::atomic<bool> tcp_running_{false};
  std::thread tcp_thread_;
  int tcp_server_fd_ = -1;

  // 设置CAD解析配置
  CADParserConfig cad_parser_config;
  // 设置栅格地图配置
  GridMapConfig grid_map_config;
  // 设置路径规划配置
  PathPlannerConfig path_planner_config;
  // 设置路径偏移配置
  PathOffsetConfig offset_config;
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
