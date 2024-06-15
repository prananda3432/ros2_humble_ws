cmake_minimum_required(VERSION 3.8)
project(project_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

##############################################################################

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(socket_communication REQUIRED)

add_executable(serial_opencv_trial src/serial_opencv_trial.cpp)
add_executable(serial_opencv src/serial_opencv.cpp)

ament_target_dependencies(serial_opencv_trial rclcpp std_msgs socket_communication)
ament_target_dependencies(serial_opencv rclcpp std_msgs socket_communication)

target_link_libraries(
  serial_opencv_trial
  serialport
)

target_link_libraries(
  serial_opencv
  serialport
)

install(TARGETS
  serial_opencv_trial
  serial_opencv
  DESTINATION lib/${PROJECT_NAME}
)

##############################################################################

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
