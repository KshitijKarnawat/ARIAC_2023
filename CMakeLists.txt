cmake_minimum_required(VERSION 3.8)
project(group3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ariac_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(OpenCV REQUIRED)

set(msg_files
  "msg/Part.msg"
  "msg/Parts.msg"
)

include_directories(include/group3)
install(DIRECTORY include config launch rviz DESTINATION share/${PROJECT_NAME}/)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  # ${srv_files}

  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs

  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(group3_exe src/ariac_competition.cpp src/tray_id_detect.cpp src/part_type_detect.cpp src/map_poses.cpp)
ament_target_dependencies(group3_exe rclcpp ariac_msgs std_srvs geometry_msgs std_msgs moveit_ros_planning_interface tf2 orocos_kdl tf2_ros tf2_geometry_msgs shape_msgs OpenCV cv_bridge image_transport)

rosidl_target_interfaces(group3_exe ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  group3_exe
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  nodes/part_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
