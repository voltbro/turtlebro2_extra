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

ament_python_install_package(${PROJECT_NAME}_nodes
  PACKAGE_DIR turtlebro_actions_nodes
)

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

set(AMENT_LINT_AUTO_EXCLUDE ament_flake8)
set(ENV{AMENT_LINT_AUTO_EXCLUDE} "ament_flake8")

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_program(AMENT_FLAKE8_EXECUTABLE ament_flake8 REQUIRED)

  ament_add_pytest_test(test_imports test/test_imports.py)
  ament_copyright()
  ament_lint_cmake()
  ament_pep257()
  ament_xmllint()

  add_test(NAME flake8_custom
    COMMAND ${AMENT_FLAKE8_EXECUTABLE} --config ${CMAKE_CURRENT_SOURCE_DIR}/ament_flake8.cfg
  )
  set_tests_properties(flake8_custom PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
