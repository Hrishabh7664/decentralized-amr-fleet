cmake_minimum_required(VERSION 3.8)
project(amr_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  launch
  meshes
  scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/amr_spawner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
