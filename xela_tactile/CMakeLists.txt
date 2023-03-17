cmake_minimum_required(VERSION 3.8)
project(xela_tactile)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(xela_msgs REQUIRED)

find_package(websocketpp REQUIRED)
find_package(Boost REQUIRED)

include_directories(
  include
  ${websocket_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_component  SHARED
  src/xela_tactile.cpp
  src/websocket_client.cpp
)
ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  xela_msgs
)
target_link_libraries(${PROJECT_NAME}_component
  ${websocketpp_LIBRARIES}
  ${Boost_LIBRARIES}
)
rclcpp_components_register_nodes(${PROJECT_NAME}_component
  "xela_tactile::XelaTactile"
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(xela_msgs)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_component)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
