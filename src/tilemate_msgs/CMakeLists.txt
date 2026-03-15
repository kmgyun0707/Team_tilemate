cmake_minimum_required(VERSION 3.8)
project(tilemate_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# interface dependencies
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


set(action_files
  "action/ExecuteJob.action"
  "action/MeasureTilt.action"
  "action/PickTile.action"
  "action/PlaceTile.action"
  "action/Spread.action"
  "action/Cowork.action"
  "action/Inspect.action"
  "action/Press.action"
  "action/PatternInspect.action"
)

set(msg_files
  "msg/PatternScore.msg"
  "msg/PatternScoreArray.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${msg_files}
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()