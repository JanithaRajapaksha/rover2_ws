cmake_minimum_required(VERSION 3.8)
project(mecanum_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# executables
add_executable(mecanum_node src/mecanum_node.cpp)
ament_target_dependencies(mecanum_node rclcpp geometry_msgs)
install(TARGETS mecanum_node DESTINATION lib/${PROJECT_NAME})

add_executable(serial_scanner src/serial_scanner.cpp)
ament_target_dependencies(serial_scanner rclcpp)
install(TARGETS serial_scanner DESTINATION lib/${PROJECT_NAME})

add_executable(serial_mecanum_node src/serial_mecanum_node.cpp)
ament_target_dependencies(serial_mecanum_node rclcpp geometry_msgs)
install(TARGETS serial_mecanum_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
