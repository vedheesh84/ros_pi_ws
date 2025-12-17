cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install launch, config, and maps directories
install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
