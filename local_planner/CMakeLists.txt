cmake_minimum_required(VERSION 3.8)
project(local_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch scripts config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/obstacles.py
  scripts/ipc_controller.py
  scripts/waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
