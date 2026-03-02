cmake_minimum_required(VERSION 3.10)
project(turtlebro_patrol)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME}_node
  PACKAGE_DIR ${PROJECT_NAME}_node
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PatrolPoint.msg"
  "srv/PatrolPointCallback.srv"
  "srv/PatrolControlCallback.srv"
  DEPENDENCIES std_msgs
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  turtlebro_patrol_node/patrol_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_add_pytest_test(test_patrol_node test/test_patrol_node.py)
  ament_copyright()
  ament_lint_cmake()
  ament_pep257()
  ament_xmllint()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
