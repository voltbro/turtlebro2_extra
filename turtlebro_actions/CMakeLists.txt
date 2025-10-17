cmake_minimum_required(VERSION 3.10)
project(turtlebro_actions)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(ACTION_FILES
  "action/Move.action"
  "action/Rotation.action"
)

set(SRV_FILES
  "srv/Photo.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${ACTION_FILES}
  ${SRV_FILES}
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(NOT TARGET ament_cmake_python_symlink_${PROJECT_NAME})
  ament_python_install_package(${PROJECT_NAME})
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY Arduino stuff
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY action srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
