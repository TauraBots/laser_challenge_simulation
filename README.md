# The LASER Challenge Simulation repository

This package contains the Gazebo files (models and worlds) related to the LASER UAV challenge.

## Models
- Arena
- Banner
- Barcode Shelf
- Landing Platform
- Measure Gas
- Pipe
- QR-Code Box
- Green Sensor
- Red Sensor
- Unit Box

## Worlds
- Stage One
- Stage Two
- Stage Three
- Stage Four
- Test

## For the execution
```bash
ros2 launch laser_challenge_simulation challenge.launch.py stage:=stage_{task you want}
```