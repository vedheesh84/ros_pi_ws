cmake_minimum_required(VERSION 3.8)
project(pick_and_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/pick_and_place_node.py
  scripts/qr_scanner_node.py
  scripts/arm_interface.py
  scripts/nav_interface.py
  scripts/location_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch directories
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
