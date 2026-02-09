cmake_minimum_required(VERSION 3.8)
project(fsm_decision)

# 设置 C++ 标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bt_interfaces REQUIRED)

# 包含头文件目录
include_directories(
  include
  3rdparty
  Actions
  States
)

# 创建可执行文件
add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/decision_executor.cpp
)

# 链接依赖
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  bt_interfaces
)

# 安装可执行文件
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件（可选）
install(DIRECTORY
  include/
  Actions/
  States/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

# 安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 测试配置（可选）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()