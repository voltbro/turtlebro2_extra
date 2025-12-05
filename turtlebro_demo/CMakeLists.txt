cmake_minimum_required(VERSION 3.10)
project(turtlebro_demo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(turtlebro_py REQUIRED)
find_package(turtlebro_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR ${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/demo_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)

  ament_add_pytest_test(test_imports test/test_imports.py)
  ament_pep257()
  ament_lint_cmake()
  ament_xmllint()
  ament_copyright()
endif()

ament_package()
