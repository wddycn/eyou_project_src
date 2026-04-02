cmake_minimum_required(VERSION 3.8)
project(eyou_test_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_control REQUIRED)

# 包含头文件目录
include_directories(
  include
  # 如果eyou.h不在系统路径，添加其路径
  # /path/to/eyou/header
)

# 编译硬件接口库
add_library(
  ${PROJECT_NAME}
  SHARED
  src/eyou_system.cpp
)

# 链接依赖
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  ros2_control
)

# 链接Eyou电机库（请替换为实际的库路径和名称）
# target_link_directories(${PROJECT_NAME} PRIVATE /path/to/eyou/lib)
# target_link_libraries(${PROJECT_NAME} PRIVATE eyou_motor)  # 例如libeyou_motor.so

# 导出插件
pluginlib_export_plugin_description_file(hardware_interface plugin_description.xml)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 安装库文件
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装配置文件
install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  ros2_control
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
