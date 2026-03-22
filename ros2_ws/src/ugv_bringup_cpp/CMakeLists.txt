cmake_minimum_required(VERSION 3.8)
project(ugv_bringup_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

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

include_directories(include)

add_library(ugv_bringup_cpp_base
  src/base_controller.cpp
)
target_include_directories(ugv_bringup_cpp_base PUBLIC include)
ament_target_dependencies(ugv_bringup_cpp_base
  rclcpp
  nlohmann_json
)
target_link_libraries(ugv_bringup_cpp_base Boost::system)

add_executable(serial_node_cpp src/serial_node.cpp)
target_include_directories(serial_node_cpp PUBLIC include)
target_link_libraries(serial_node_cpp ugv_bringup_cpp_base)
ament_target_dependencies(serial_node_cpp
  rclcpp
  ament_index_cpp
  geometry_msgs
  sensor_msgs
  std_msgs
  nlohmann_json
)
target_link_libraries(serial_node_cpp Boost::system)

add_executable(joyparser_node_cpp src/joyparser_node.cpp)
ament_target_dependencies(joyparser_node_cpp
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

add_executable(sensordebug_node_cpp src/sensordebug_node.cpp)
ament_target_dependencies(sensordebug_node_cpp
  rclcpp
  sensor_msgs
  nlohmann_json
)

add_executable(camera_node_cpp src/camera_node.cpp)
ament_target_dependencies(camera_node_cpp
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(camera_node_cpp ${OpenCV_LIBS})

install(TARGETS
  ugv_bringup_cpp_base
  serial_node_cpp
  joyparser_node_cpp
  sensordebug_node_cpp
  camera_node_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
