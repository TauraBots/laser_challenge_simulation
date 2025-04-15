# The LASER Challenge Simulation repository

This package contains the Gazebo files (models and worlds) related to the LASER UAV challenge.

## Models
- Arena
- Banner
- Barcode Shelf
- Cluttered Environment
- Landing Platform
- Measure Gas
- Pipe
- QR-Code Box
- Green Sensor
- Red Sensor
- Unit Box

## Worlds
- Challenge World

## How to run
```
ros2 run laser_challenge_simulation base_spawner.py --ros-args -p challenge_stage:=stage_{stage number}
```
