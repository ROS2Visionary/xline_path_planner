cmake_minimum_required(VERSION 3.8)
project(xline_path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# OpenCV is required by the planner and visualization utilities
find_package(OpenCV REQUIRED)

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
)
target_compile_features(planner_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  planner_node
  "rclcpp"
)

install(TARGETS planner_node
  DESTINATION lib/${PROJECT_NAME})

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
