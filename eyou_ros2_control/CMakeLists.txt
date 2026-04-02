cmake_minimum_required(VERSION 3.8)
project(eyou_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)

# 包含目录
include_directories(
  include
  # 如果你的 eyou.h 不在 include 根目录，请添加具体路径
  # ${CMAKE_CURRENT_SOURCE_DIR}/path/to/eyou_lib/include
)

# 添加库目标
add_library(
  ${PROJECT_NAME}
  SHARED
  src/eyou_system.cpp
)

# 设置 C++ 标准
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# 链接依赖
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  std_msgs
)

# 【关键】链接你的第三方 eyou 库
# 假设你把 libeyou.so 放到了 lib/ 目录，或者系统里有 -leyou
# 这里需要根据你实际的库文件名和路径调整
# 方式 A: 如果库在系统路径
# target_link_libraries(${PROJECT_NAME} eyou) 

# 方式 B: 如果库在本地 (推荐)
# 假设你在项目根目录有个 lib/libeyou.so
# add_library(eyou_external SHARED IMPORTED)
# set_target_properties(eyou_external PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libeyou.so)
# target_link_libraries(${PROJECT_NAME} eyou_external)

# 临时方案：如果没有 .so 文件，确保编译时能找到头文件，链接时由用户指定
# 此处假设库名为 eyou_driver (请根据实际情况修改)
target_link_libraries(${PROJECT_NAME} pthread) # CAN 通常需要 pthread

# 导出插件
pluginlib_export_plugin_description_file(hardware_interface plugin_description.xml)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 安装库
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

# 安装其他资源 (URDF, Launch, Config)
install(
  DIRECTORY urdf launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
)

ament_package()