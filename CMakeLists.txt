cmake_minimum_required(VERSION 3.8)
project(tong_thesis_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

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

install(
  DIRECTORY launch
  DESTINATION share/tong_thesis_1/
)
install(
  DIRECTORY maps
  DESTINATION share/tong_thesis_1/
)
install(
  DIRECTORY params
  DESTINATION share/tong_thesis_1/
)
install(
  DIRECTORY rviz
  DESTINATION share/tong_thesis_1/
)
install(
  DIRECTORY urdf
  DESTINATION share/tong_thesis_1/
)
install(
  DIRECTORY worlds
  DESTINATION share/tong_thesis_1/
)
install(
  DIRECTORY bags
  DESTINATION share/tong_thesis_1/
)
install(
  DIRECTORY cartographer_config
  DESTINATION share/tong_thesis_1/
)