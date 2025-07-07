# my_swarm_pkg

A ROS 2-based multi-UAV leader-follower swarm controller.  
One **Leader** publishes trajectory setpoints and formation targets, and multiple **Follower** nodes consume those targets, avoid each other with repulsion, and track the leader’s movement and formation commands.

---

## Table of Contents

1. [Prerequisites](#prerequisites)  
2. [Installation](#installation)  
3. [Package structure](#package-structure)  
4. [Launch file](#launch-file)  
5. [Leader node (`leader_node`)](#leader-node-leader_node)  
   1. [LeaderController class](#leadercontroller-class)  
   2. [Key methods & functions](#key-methods--functions)  
6. [Follower node (`follower_node`)](#follower-node-follower_node)  
   1. [FollowerController class](#followercontroller-class)  
   2. [Key methods & functions](#key-methods--functions-1)  
7. [Topics & message flow](#topics--message-flow)  
8. [Tuning parameters](#tuning-parameters)  

---

## Prerequisites

1. Ubuntu 20.04 / 22.04  
2. ROS 2 (e.g. Humble or Galactic) installed and sourced  
3. `px4_msgs` package  
4. Standard ROS 2 message packages:  
   - `std_msgs`  
   - `geometry_msgs`  
5. Python 3 dependencies (handled by `package.xml`/`setup.py`)  

You also need a working PX4 SITL or hardware setup for `/px4_N` namespaces.

---

## Installation

```bash
# 1. Source your ROS 2 installation:
source /opt/ros/<distro>/setup.bash

# 2. Create a workspace and clone this repo:
mkdir -p ~/swarm_ws/src
cd ~/swarm_ws/src
git clone git@github.com:<your-username>/my_swarm_pkg.git

# 3. Build:
cd ~/swarm_ws
colcon build --packages-select my_swarm_pkg

# 4. Source overlay:
source install/setup.bash

---

## Package structure


my_swarm_pkg/
├── package.xml
├── setup.py
├── launch/
│   └── swarm_launch.py
├── resource/
│   └── my_swarm_pkg  ← entry-point marker
├── my_swarm_pkg/
│   ├── leader_follower.py   ← leader & built-in follower classes
│   ├── follower_node.py     ← standalone follower node
│   └── constants.py         ← common constants (Z_CMD, KP_POS, etc.)
└── README.md

---

## Launch file

**`launch/swarm_launch.py`** brings up one leader node and four follower nodes:

python
def generate_launch_description():
return LaunchDescription([
Node(package='my_swarm_pkg', executable='leader_node', name='leader',  namespace='swarm'),
Node(package='my_swarm_pkg', executable='follower_node', name='follower_uav2', namespace='uav2'),
Node(package='my_swarm_pkg', executable='follower_node', name='follower_uav3', namespace='uav3'),
Node(package='my_swarm_pkg', executable='follower_node', name='follower_uav4', namespace='uav4'),
Node(package='my_swarm_pkg', executable='follower_node', name='follower_uav5', namespace='uav5'),
])

Usage:

bash
ros2 launch my_swarm_pkg swarm_launch.py

---

## Leader node (`leader_node`)

Implemented in `leader_follower.py` as class **`LeaderController`**.

### LeaderController class

- **Namespace**: `px4_1`  
- **Main responsibilities**  
  1. Subscribe to its own odometry (`/px4_1/fmu/out/vehicle_odometry`)  
  2. Accept external commands:  
- `/swarm/leader_cmd` (geometry_msgs/Twist)  
- `/swarm/waypoint_cmd` (geometry_msgs/Point)  
- `/swarm/formation_cmd` (std_msgs/String)  
- `/swarm/rotation_cmd` (geometry_msgs/Vector3)  
  3. Track each follower’s odometry: `/px4_2…px4_4/fmu/out/vehicle_odometry`  
  4. Publish:  
- Offboard mode commands (`OffboardControlMode`)  
- Trajectory setpoint (`TrajectorySetpoint`) for leader movement  
- VehicleCommand to arm & set mode  
- `/swarm/follower_targets` (std_msgs/String) listing each follower’s target position & arrival time  

- **Node timer** at 20 Hz runs `control_loop()`.

### Key methods & functions

1. **quaternion_to_yaw(q)**  
   Convert quaternion `[x,y,z,w]` to yaw angle.

2. **compute_formation_positions(cx, cy, formation, spacing, N)**  
   Returns a list of 2D offsets for `N` followers in:  
   - “line”  
   - “square”  
   - or custom OFFSETS fallback.

3. **send_cmd(command, p1=0, p2=0)**  
   Publish a `VehicleCommand` to arm/disarm or change mode.

4. **control_loop()**  
   - **Arm & set offboard** after 1 s (20 ticks).  
   - **Leader motion**  
- If `waypoint_cmd` active: P-control toward waypoint;  
- Else use last `/swarm/leader_cmd` Twist to dead-reckon.  
   - **Formation**  
- Compute formation points around current leader position.  
- Apply 3D rotations (roll, pitch, yaw) from `/swarm/rotation_cmd`.  
- Compute per-follower travel times (based on max distance & spacing speed).  
- Publish `/swarm/follower_targets` as semicolon-separated `idx,x,y,z,arrival_time`.  

---

## Follower node (`follower_node`)

Two variants exist:

1. **Built-in** follower in `leader_follower.py` under class `FollowerController`.  
2. **Standalone** in `follower_node.py` (very similar).

Below describes the standalone one in `follower_node.py`.

### FollowerController class

- Each instance runs under its PX4 namespace (`px4_{idx+2}`), e.g. `px4_2_follower`.  
- **Subscriptions**  
  - Leader odometry: `/px4_1/fmu/out/vehicle_odometry`  
  - Own odometry: `/<ns>/fmu/out/vehicle_odometry`  
  - Other followers’ odometry (for repulsion)  
  - `/swarm/follower_targets` (std_msgs/String) → updates offset, target z, arrival time  
  - `/swarm/waypoint_cmd` resets arrival flag  

- **Publishers**  
  - Offboard mode: `<ns>/fmu/in/offboard_control_mode`  
  - Trajectory setpoint: `<ns>/fmu/in/trajectory_setpoint`  
  - VehicleCommand (to arm/set mode)  
  - Follower error: `/px4_{i+2}/follower_err`  
  - Arrival notification: `/swarm/follower_arrived`  

- **Timer** at 20 Hz runs `control_loop()`.

### Key methods & functions

1. **target_cb(msg)**  
   Parses one entry `idx,x,y,z,t` from `/swarm/follower_targets`, picks the matching `idx` and updates:  
   - `offset = (x, y)`  
   - `z_target = z`  
   - `t_arrival = t`  
   - `arrived = False`

2. **_neighbor_cb(j, msg)**  
   Store neighbor `j`’s position for repulsive avoidance.

3. **send_cmd(command, p1=0, p2=0)**  
   Publish a `VehicleCommand` for arming or setting offboard mode.

4. **control_loop()**  
   - **Arm & set offboard** after 1 s.  
   - Compute **remaining time** to arrival.  
   - **Feed-forward** velocity = `(desired_position - current_position) / remaining_time` (zero if `<0.1 s`).  
   - **Proportional** velocity = `KP_POS * position_error`.  
   - **Repulsion**  
- If two followers closer than `REPULSION_THRESHOLD`, add a repulsive vector.  
   - **Velocity cap** at `REPULSION_THRESHOLD`.  
   - **Zero velocity & publish arrival** once error below `ARRIVAL_THRESHOLD`.  
   - Publish final `TrajectorySetpoint` with current pos, computed vel, and leader’s yaw.  

---

## Topics & message flow

| Topic                                    | Msg Type                | Publisher            | Subscriber(s)         |
|------------------------------------------|-------------------------|----------------------|-----------------------|
| /px4_1/fmu/out/vehicle_odometry          | px4_msgs/VehicleOdometry| Flight stack (leader)| Leader, followers     |
| /swarm/leader_cmd                        | geometry_msgs/Twist     | User / teleop node   | Leader                |
| /swarm/waypoint_cmd                      | geometry_msgs/Point     | User / planner       | Leader, followers     |
| /swarm/formation_cmd                     | std_msgs/String         | User                  | Leader                |
| /swarm/rotation_cmd                      | geometry_msgs/Vector3   | User                  | Leader                |
| /swarm/follower_targets                  | std_msgs/String         | Leader               | Followers             |
| /swarm/follower_arrived                  | std_msgs/String         | Followers            | Leader                |
| `<ns>/fmu/in/offboard_control_mode`      | px4_msgs/OffboardControlMode| Leader/Follower  | PX4 FMU               |
| `<ns>/fmu/in/trajectory_setpoint`        | px4_msgs/TrajectorySetpoint| Leader/Follower  | PX4 FMU               |
| `<ns>/fmu/in/vehicle_command`            | px4_msgs/VehicleCommand | Leader/Follower      | PX4 FMU               |

---

## Tuning parameters

Defined in `constants.py` (or top of scripts):

python
RATE_HZ = 20.0
Z_CMD = -1.0
OFFSETS = [(0,3),(0,-30),(-30,0)]
KP_POS = 0.6
ARRIVAL_THRESHOLD = 0.2
REPULSION_GAIN = 1.0
REPULSION_THRESHOLD = 1.5

Adjust spacing, gains, and thresholds to suit your UAV dynamics and formation scale.

---

With this setup, you can:

bash
# 1. Launch swarm
ros2 launch my_swarm_pkg swarm_launch.py

# 2. Send commands to leader, e.g.:
ros2 topic pub /swarm/waypoint_cmd geometry_msgs/Point "{x: 0.0, y: 5.0, z: -2.0}"
ros2 topic pub /swarm/formation_cmd std_msgs/String "data: 'square,4.0'"
ros2 topic pub /swarm/rotation_cmd geometry_msgs/Vector3 "{x: 0.0, y: 0.0, z: 1.57}"

Your followers will automatically receive updated targets, avoid collisions, and report arrival back to the leader.

Enjoy your ROS 2 swarm!
