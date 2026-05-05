# River Autonomy - Perception Layer

ROS 2 Python package implementing the **Perception** layer of the Perceive→Plan→Control architecture for autonomous boat control in VRX simulation.

## Architecture

```
Perception Layer (this package)
├─ Subscribes to: Odometry, IMU, Lidar
├─ Publishes: Robot State, Obstacle Distances
└─ Output → Planning Layer

Planning Layer (future)
├─ Subscribes to: Robot State, Obstacle Distances
├─ Publishes: Navigation Goals
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

### Monitor Output (Terminal 3)
```bash
# Watch perception state
ros2 topic echo /perception/robot_state

# Watch obstacle distances
ros2 topic echo /perception/obstacles
```

## Configuration

Edit parameters in `launch/perception.launch.py`:

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
```

## Next Steps

1. **Planning Node**: Subscribe to `/perception/robot_state` and `/perception/obstacles` → compute waypoints/goals
2. **Control Node**: Subscribe to planning outputs → command thrusters
3. **Message Definitions**: Consider creating custom ROS messages for richer data exchange
4. **Sensor Validation**: Add covariance info to state messages
