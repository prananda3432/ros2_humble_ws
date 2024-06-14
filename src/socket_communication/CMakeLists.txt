cmake_minimum_required(VERSION 3.8)
project(socket_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

######

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Opencv1.msg"
  DEPENDENCIES std_msgs
)

add_executable(get_data src/get_data.cpp)
add_executable(opencv_socket_1 src/opencv_socket_1.cpp)

ament_export_dependencies(rosidl_default_runtime)
ament_target_dependencies(get_data rclcpp std_msgs)
ament_target_dependencies(opencv_socket_1 rclcpp std_msgs)

rosidl_target_interfaces(opencv_socket_1
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS 
  get_data
  opencv_socket_1
  DESTINATION lib/${PROJECT_NAME})

######

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
