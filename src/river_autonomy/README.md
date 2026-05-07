# River Autonomy - Perception + Planning + Control Layers

ROS 2 Python package implementing the full Perceive->Plan->Control architecture for autonomous boat control in VRX simulation.

## Architecture

```
Perception Layer (implemented)
├─ Subscribes to: Odometry, IMU, Lidar
├─ Publishes: Robot State, Obstacle Distances + Nearest Bearing
└─ Output → Planning Layer

Planning Layer (implemented)
├─ Subscribes to: Robot State, Obstacle Distances
├─ Publishes: Target Heading, Target Speed, Target Yaw Rate
└─ Output → Control Layer

Control Layer (implemented)
├─ Subscribes to: Target Heading, Target Speed, Target Yaw Rate
├─ Publishes: Left/Right Thrust, Left/Right Pod Angle
└─ Output → Thrusters
```

## Perception Node

**Subscriptions (Input Sensors):**
- `ground_truth_odometry`: Robot pose (x, y, yaw) and velocity
- `imu/data`: Acceleration and angular rates
- `lidar_scan`: 2D 16-beam lidar for obstacle detection

**Publications (Output to Planning):**
- `/perception/robot_state`: Aggregated state vector [x, y, yaw, vx, vy, vyaw, ax, ay, az, timestamp]
- `/perception/obstacles`: Obstacle summary [left, center, right, min_distance, nearest_bearing]

**Processing:**
- Sensor fusion: Combines odometry + IMU
- Lidar processing: Front-sector obstacle detection (configurable, default 60° cone)
- Filtering: Moving average buffers for noise reduction

## Planning Node

Subscriptions (Input from Perception):
- /perception/robot_state: Aggregated state vector [x, y, yaw, vx, vy, vyaw, ax, ay, az, timestamp]
- /perception/obstacles: Obstacle summary [left, center, right, min_distance, nearest_bearing]

Publications (Output to Control):
- /planning/target_heading: Desired global heading (rad)
- /planning/target_speed: Desired forward speed
- /planning/target_yaw_rate: Desired turn rate

Behavior:
- Tracks a configurable goal point (goal_x, goal_y)
- Uses a staged obstacle-pass state machine for smoother maneuvers (`track_goal`, `approach_slow`, `commit_turn`, `pass_obstacle`, `recover_to_goal`, `goal_reached`)
- Chooses and commits to a pass side, then recovers back to the goal heading

## Control Node

Subscriptions (Input from Planning and Perception):
- /planning/target_heading: Desired global heading (rad)
- /planning/target_speed: Desired forward speed
- /planning/target_yaw_rate: Desired turn rate
- /perception/robot_state: Current yaw and yaw rate feedback

Publications (Output to WAM-V):
- /wamv/thrusters/left/thrust
- /wamv/thrusters/right/thrust
- /wamv/thrusters/left/pos
- /wamv/thrusters/right/pos

Behavior:
- Tracks planner heading and yaw-rate targets with feedback control
- Converts target speed into forward thrust
- Adds differential thrust for turning
- Commands pod angles for steering

## Build

```bash
cd ~/vrx_ws
colcon build --packages-select vrx_gz river_autonomy --merge-install
source install/setup.bash
```

## Launch

### VRX River World (Terminal 1)
```bash
cd ~/vrx_ws
source install/setup.bash
ros2 launch vrx_gz competition.launch.py \
  world:=river_world \
  config_file:=$HOME/vrx_ws/src/vrx/vrx_gz/config/river_world.yaml \
  gui:=false
```

### VRX River World With Obstacles (Terminal 1)
```bash
cd ~/vrx_ws
source install/setup.bash
ros2 launch vrx_gz competition.launch.py \
  world:=river_world_obstacles \
  config_file:=$HOME/vrx_ws/src/vrx/vrx_gz/config/river_world_obstacles.yaml \
  gui:=false
```

Use this world to test obstacle detection and avoidance with large static obstacle columns.

### Full Autonomy Stack (Terminal 2)
```bash
cd ~/vrx_ws
source install/setup.bash
ros2 launch river_autonomy autonomy_stack.launch.py
```

This starts perception, planning, and control together.

### Perception Node (Terminal 2)
```bash
cd ~/vrx_ws
source install/setup.bash
ros2 launch river_autonomy perception.launch.py
```

### Planning Node (Terminal 3)
```bash
cd ~/vrx_ws
source install/setup.bash
ros2 launch river_autonomy planning.launch.py
```

### Control Node (Terminal 4)
```bash
cd ~/vrx_ws
source install/setup.bash
ros2 launch river_autonomy control.launch.py
```

### Monitor Output (Terminal 5)
```bash
# Watch perception state
ros2 topic echo /perception/robot_state

# Watch obstacle distances
ros2 topic echo /perception/obstacles

# Watch planner outputs
ros2 topic echo /planning/target_heading
ros2 topic echo /planning/target_speed
ros2 topic echo /planning/target_yaw_rate

# Watch control outputs
ros2 topic echo /wamv/thrusters/left/thrust
ros2 topic echo /wamv/thrusters/right/thrust
ros2 topic echo /wamv/thrusters/left/pos
ros2 topic echo /wamv/thrusters/right/pos
```

## Configuration

Edit parameters in launch/perception.launch.py, launch/planning.launch.py, and launch/control.launch.py:

- `odom_topic`: Odometry subscription topic
- `imu_topic`: IMU subscription topic  
- `scan_topic`: Lidar subscription topic
- `lidar_filter_range`: Max lidar range to process (meters)
- `lidar_sector_angle`: Front sector width for obstacle detection (degrees)
- `filter_buffer_size`: Moving average buffer size (samples)
- `publish_rate_hz`: State publishing frequency
- `turn_heading_offset_deg`: Heading offset used during committed turn phase
- `pass_heading_offset_deg`: Heading offset used while passing obstacle
- `commit_min_hold_sec`: Minimum duration of committed turn phase
- `pass_min_hold_sec`: Minimum duration of pass-obstacle phase
- `recover_heading_tolerance_deg`: Heading error required to complete recover phase
- `max_pod_rate`: Maximum pod angle rate (rad/s), limits steering jitter

Example with custom parameters:
```bash
ros2 launch river_autonomy perception.launch.py \
  lidar_sector_angle:=90.0 \
  filter_buffer_size:=10 \
  publish_rate_hz:=20.0

ros2 launch river_autonomy planning.launch.py \
  goal_x:=180.0 \
  goal_y:=0.0 \
  caution_distance:=14.0 \
  stop_distance:=6.0 \
  turn_heading_offset_deg:=34.0 \
  pass_heading_offset_deg:=18.0

ros2 launch river_autonomy control.launch.py \
  base_thrust_gain:=360.0 \
  heading_kp:=1.5 \
  yaw_rate_kp:=220.0 \
  max_pod_rate:=1.1
```

## Next Steps

1. Message Definitions: Replace Float32MultiArray with custom ROS messages for safer interfaces.
2. Goal Management: Add dynamic goal updates through a service or action.
3. Validation: Add integration tests for perception-to-planning-to-control handoff.
4. Safety: Add command timeout and watchdog behavior in control node.
