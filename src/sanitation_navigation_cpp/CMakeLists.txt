cmake_minimum_required(VERSION 3.16)
project(sanitation_navigation_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(console_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  ${console_bridge_INCLUDE_DIRS}
)

add_library(local_costmap_builder_component SHARED
  src/local_costmap_builder_component.cpp
)

ament_target_dependencies(local_costmap_builder_component
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
)

target_link_libraries(local_costmap_builder_component
  console_bridge::console_bridge
)

rclcpp_components_register_nodes(
  local_costmap_builder_component
  "sanitation_navigation_cpp::LocalCostmapBuilderComponent"
)

install(TARGETS local_costmap_builder_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(local_costmap_builder_component)
ament_package()
