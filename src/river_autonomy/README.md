# River Autonomy - Perception + Planning Layers

ROS 2 Python package implementing the Perception and Planning layers of the Perceive->Plan->Control architecture for autonomous boat control in VRX simulation.

## Architecture

```
Perception Layer (implemented)
├─ Subscribes to: Odometry, IMU, Lidar
├─ Publishes: Robot State, Obstacle Distances
└─ Output → Planning Layer

Planning Layer (implemented)
├─ Subscribes to: Robot State, Obstacle Distances
├─ Publishes: Target Heading, Target Speed, Target Yaw Rate
└─ Output → Control Layer

Control Layer (future)
├─ Subscribes to: Navigation Goals
├─ Publishes: Thrust Commands
└─ Output → Thrusters
```

## Perception Node

**Subscriptions (Input Sensors):**
- `ground_truth_odometry`: Robot pose (x, y, yaw) and velocity
- `imu/data`: Acceleration and angular rates
- `lidar_scan`: 2D 16-beam lidar for obstacle detection

**Publications (Output to Planning):**
- `/perception/robot_state`: Aggregated state vector [x, y, yaw, vx, vy, vyaw, ax, ay, az, timestamp]
- `/perception/obstacles`: Obstacle distances [left, center, right, min_distance]

**Processing:**
- Sensor fusion: Combines odometry + IMU
- Lidar processing: Front-sector obstacle detection (configurable, default 60° cone)
- Filtering: Moving average buffers for noise reduction

## Planning Node

Subscriptions (Input from Perception):
- /perception/robot_state: Aggregated state vector [x, y, yaw, vx, vy, vyaw, ax, ay, az, timestamp]
- /perception/obstacles: Obstacle distances [left, center, right, min_distance]

Publications (Output to Control):
- /planning/target_heading: Desired global heading (rad)
- /planning/target_speed: Desired forward speed
- /planning/target_yaw_rate: Desired turn rate

Behavior:
- Tracks a configurable goal point (goal_x, goal_y)
- Slows down when obstacles are nearby
- Stops and turns away when obstacles are too close
- Holds position command when goal tolerance is reached

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

### Monitor Output (Terminal 4)
```bash
# Watch perception state
ros2 topic echo /perception/robot_state

# Watch obstacle distances
ros2 topic echo /perception/obstacles

# Watch planner outputs
ros2 topic echo /planning/target_heading
ros2 topic echo /planning/target_speed
ros2 topic echo /planning/target_yaw_rate
```

## Configuration

Edit parameters in launch/perception.launch.py and launch/planning.launch.py:

- `odom_topic`: Odometry subscription topic
- `imu_topic`: IMU subscription topic  
- `scan_topic`: Lidar subscription topic
- `lidar_filter_range`: Max lidar range to process (meters)
- `lidar_sector_angle`: Front sector width for obstacle detection (degrees)
- `filter_buffer_size`: Moving average buffer size (samples)
- `publish_rate_hz`: State publishing frequency

Example with custom parameters:
```bash
ros2 launch river_autonomy perception.launch.py \
  lidar_sector_angle:=90.0 \
  filter_buffer_size:=10 \
  publish_rate_hz:=20.0

ros2 launch river_autonomy planning.launch.py \
  goal_x:=180.0 \
  goal_y:=0.0 \
  caution_distance:=12.0 \
  stop_distance:=5.0
```

## Next Steps

1. Control Node: Subscribe to /planning/target_* topics and output thruster commands.
2. Message Definitions: Replace Float32MultiArray with custom ROS messages for safer interfaces.
3. Goal Management: Add dynamic goal updates through a service or action.
4. Validation: Add integration tests for perception-to-planning handoff.
