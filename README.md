# VRX_riverSim
river_autonomy

Minimal ROS 2 Python package for VRX river experiments.

Build
  cd ~/vrx_ws
  colcon build --packages-select vrx_gz river_autonomy --merge-install

Source
  cd ~/vrx_ws
  source install/setup.bash

Launch the VRX river world with the default WAM-V
  cd ~/vrx_ws
  source install/setup.bash
  ros2 launch vrx_gz competition.launch.py \
    world:=river_world \
    config_file:=$HOME/vrx_ws/src/vrx/vrx_gz/config/river_world.yaml

The Current Implementaion only includes a Perception node that publishes odometry data. 