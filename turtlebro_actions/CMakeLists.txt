cmake_minimum_required(VERSION 3.10)
project(turtlebro_actions)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR ${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY stuff
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

set(ACTIONS_SERVERS
  turtlebro_actions/servers/record_audio_service.py
  turtlebro_actions/servers/play_audio_service.py
  turtlebro_actions/servers/move_server.py
  turtlebro_actions/servers/photo_service.py
  turtlebro_actions/servers/rotate_server.py
  turtlebro_actions/servers/text_to_speech_server.py
)

set(EXAMPLE_CLIENTS
  turtlebro_actions/examples/move_client.py
  turtlebro_actions/examples/servo_client.py
  turtlebro_actions/examples/video_client.py
  turtlebro_actions/examples/rotate_client.py
  turtlebro_actions/examples/text_to_speech_client.py
)

install(PROGRAMS
  ${ACTIONS_SERVERS}
  ${EXAMPLE_CLIENTS}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_add_pytest_test(test_imports test/test_imports.py)
  ament_copyright()
  ament_lint_cmake()
  ament_pep257()
  ament_xmllint()

endif()

ament_package()
