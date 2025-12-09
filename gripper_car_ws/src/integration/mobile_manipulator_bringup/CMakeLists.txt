cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install launch, config directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/integrated_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
