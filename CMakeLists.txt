cmake_minimum_required(VERSION 3.8)
project(laser_challenge_simulation)

# Encontrar dependências
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY
  launch
  models
  worlds
  scripts
  DESTINATION share/${PROJECT_NAME}
)


install(PROGRAMS
  scripts/base_spawner.py
  scripts/takeoff_sdk.py  
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()