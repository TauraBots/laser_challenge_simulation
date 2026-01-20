cmake_minimum_required(VERSION 3.8)
project(laser_challenge_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_sim REQUIRED)

# Instala os scripts Python
install(PROGRAMS
  scripts/base_spawner.py
  DESTINATION lib/${PROJECT_NAME}
)

# Instala as pastas de modelos e mundos para que fiquem acessíveis no install/
install(DIRECTORY
  models
  worlds
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()