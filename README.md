# VRX_riverSim
ROS 2 workspace for VRX river experiments with a full Perceive-Plan-Control autonomy stack.

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

Launch the VRX obstacle world (large static columns)
  cd ~/vrx_ws
  source install/setup.bash
  ros2 launch vrx_gz competition.launch.py \
    world:=river_world_obstacles \
    config_file:=$HOME/vrx_ws/src/vrx/vrx_gz/config/river_world_obstacles.yaml

Launch full autonomy stack
  cd ~/vrx_ws
  source install/setup.bash
  ros2 launch river_autonomy autonomy_stack.launch.py

The current implementation includes Perception, Planning, and Control nodes.