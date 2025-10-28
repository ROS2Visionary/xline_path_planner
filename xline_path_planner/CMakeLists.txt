cmake_minimum_required(VERSION 3.8)
project(xline_path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
# OpenCV is required by the planner and visualization utilities
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 使用 main.cpp 作为程序入口，并编译所需的源文件
add_executable(planner_node
  src/main.cpp
  src/cad_parser.cpp
  src/grid_map_generator.cpp
  src/path_planner.cpp
  src/trajectory_generator.cpp
  src/output_formatter.cpp
)
target_include_directories(planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(planner_node
  ${OpenCV_LIBS}
  yaml-cpp
)
target_compile_features(planner_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  planner_node
  "rclcpp"
  "geometry_msgs"
  "ament_index_cpp"
)

install(TARGETS planner_node
  DESTINATION lib/${PROJECT_NAME})

# Install configuration files (YAML)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 定义接口（仅 Service）
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanPath.srv"
  DEPENDENCIES builtin_interfaces
  SKIP_GROUP_MEMBERSHIP_CHECK
)

ament_export_dependencies(rosidl_default_runtime)

rosidl_target_interfaces(planner_node ${PROJECT_NAME} "rosidl_typesupport_cpp")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

## 已移除 CLI 过滤器以聚焦 ROS2 使用
