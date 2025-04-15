#!/bin/bash

cp -r ~/laser_uav_system_ws/src/laser_challenge_simulation/models/. ~/laser_uav_system_ws/src/laser_uav_simulation/core/models/

cp -r ~/laser_uav_system_ws/src/laser_challenge_simulation/worlds/. ~/laser_uav_system_ws/src/laser_uav_simulation/core/worlds/

cd ~/laser_uav_system_ws/src/laser_uav_simulation/core/scripts/

./link_models.sh

./link_worlds.sh
