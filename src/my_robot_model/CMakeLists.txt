cmake_minimum_required(VERSION 3.5)
project(my_robot_model)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install description files
install(
  DIRECTORY description
  DESTINATION share/${PROJECT_NAME}
)

# Install config files - ĐÃ SỬA
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install rviz files
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install worlds files - ĐÃ SỬA tên thư mục
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/wheel_odom_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

