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

Launch the simple autonomy node in a second terminal
  cd ~/vrx_ws
  source install/setup.bash
  ros2 launch river_autonomy simple_autonomy.launch.py

Current behavior
  The node reads IMU, lidar, and odometry topics from the WAM-V.
  If odometry is unavailable, it falls back to IMU heading-hold mode.
  The node drives upstream and steers away from close obstacles detected in the
  front lidar sector.

Useful notes
  Start the VRX simulation first, then launch river_autonomy in a second terminal.
  The river world spawn config places the WAM-V near x=-100, y=0, facing upstream.
  You can change the autonomy goal with:
    ros2 launch river_autonomy simple_autonomy.launch.py goal_x:=140.0 goal_y:=0.0
