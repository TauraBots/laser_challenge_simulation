cmake_minimum_required(VERSION 3.8)
project(laser_challenge_simulation)

# Encontrar dependências
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(mavros REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

install(DIRECTORY
  launch
  models
  worlds
  scripts
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()