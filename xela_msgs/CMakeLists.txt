cmake_minimum_required(VERSION 3.8)
project(xela_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/SensData.msg"
  "msg/SensDataPoints.msg"
  "msg/SensName.msg"
  "msg/SensPoint.msg"
  "msg/SensStream.msg"
  "msg/XelaInfoMsg.msg"
  "msg/XelaSensorData.msg"
  "msg/XelaServerMsg.msg"
)

set(srv_files
  "srv/XelaSensorStream.srv"
  "srv/XelaSensorX.srv"
  "srv/XelaSensorXY.srv"
  "srv/XelaSensorXYZ.srv"
  "srv/XelaSensorY.srv"
  "srv/XelaSensorZ.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
