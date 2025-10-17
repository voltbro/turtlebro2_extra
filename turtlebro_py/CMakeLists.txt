cmake_minimum_required(VERSION 3.10)
project(turtlebro_py)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(NOT TARGET ament_cmake_python_symlink_${PROJECT_NAME})
  ament_python_install_package(${PROJECT_NAME})
endif()

install(DIRECTORY ino
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
