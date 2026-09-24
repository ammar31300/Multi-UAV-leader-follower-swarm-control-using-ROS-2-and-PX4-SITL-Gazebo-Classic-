#Multi-UAV leader–follower swarm control using ROS 2 and PX4 SITL (Gazebo Classic)

---

## Table of Contents  
1. [Overview](#overview)  
2. [Prerequisites](#prerequisites)  
3. [Installation](#installation)  
4. [Usage / Launching](#usage--launching)  
5. [Package Structure](#package-structure)  
6. [Nodes & Classes](#nodes--classes)  
7. [Topics & Message Flow](#topics--message-flow)  
8. [Tuning Parameters](#tuning-parameters)  

---

## Overview  
This package implements a leader–follower formation control for a swarm of PX4-based VTOL/UAVs in Gazebo­ Classic using ROS 2. A single **LeaderController** node publishes setpoints and formation parameters; multiple **FollowerController** nodes subscribe, compute their local commands (including collision avoidance via repulsion), and send offboard commands back to PX4.

---

## Prerequisites  

1. **Operating System**  
   - Ubuntu 20.04 LTS or Ubuntu 22.04 LTS  

2. **ROS 2 (Humble / Galactic)**  
   - Installation guide:  
     https://docs.ros.org/en/rolling/Installation.html  
   - After install:  
     ```bash
     source /opt/ros/<distro>/setup.bash
     ```

3. **PX4 Autopilot Firmware & Gazebo Classic SITL**  
   - PX4 official instructions:  
     https://docs.px4.io/master/en/dev_setup/dev_env.html  
   - Build & run SITL with Gazebo Classic:  
     ```bash
     # clone PX4
     git clone https://github.com/PX4/PX4-Autopilot.git ~/px4_firmware
     cd ~/px4_firmware
     # update submodules
     git submodule update --init --recursive
     # build for SITL + Gazebo Classic
     make px4_sitl gazebo_classic
     ```
   - This will launch Gazebo Classic with the iris model in offboard-ready mode.

4. **PX4–ROS 2 Bridge (px4_ros_com / px4_msgs)**  
   - ROS 2 message definitions for PX4:  
     https://github.com/PX4/px4_ros_com  
   - Install via ROS 2 package index (if available) or build from source under your ROS 2 workspace:  
     ```bash
     cd ~/swarm_ws/src
     git clone https://github.com/PX4/px4_ros_com.git
     ```

5. **ROS 2 Build Tools & Utilities**  
   - `colcon`  
   - `ros2launch`, `rclpy`  
   - Standard ROS 2 message packages: `std_msgs`, `geometry_msgs`

6. **Python 3** (3.8 or later)  
   - Managed via `setup.py` / `package.xml`.  

---

## Installation  

```bash
# 1. Source your ROS 2 install (e.g., Humble or Galactic)
source /opt/ros/<distro>/setup.bash

# 2. Create your ROS 2 workspace & clone repositories
mkdir -p ~/swarm_ws/src
cd ~/swarm_ws/src

# 2a. Clone PX4–ROS bridge (px4_ros_com) if not installed via apt
git clone https://github.com/PX4/px4_ros_com.git

# 2b. Clone this swarm package
git clone git@github.com:<your-username>/my_swarm_pkg.git

# 3. Build the workspace
cd ~/swarm_ws
colcon build --packages-select px4_msgs my_swarm_pkg

# 4. Source the overlay
source install/setup.bash

---

## Usage / Launching  

1. **Start PX4 SITL + Gazebo Classic**  
   ```bash
   cd ~/px4_firmware
   make px4_sitl gazebo_classic
   ```
   – Wait until the PX4 SITL vehicle is armed and ready.

2. **In a new terminal**, source ROS 2 and your workspace:
   ```bash
   source /opt/ros/<distro>/setup.bash
   source ~/swarm_ws/install/setup.bash
   ```

3. **Launch the swarm**  
   ```bash
   ros2 launch my_swarm_pkg swarm_launch.py
   ```

4. **Send leader commands** (optional)  
   - Teleoperate the leader:  
     ```bash
     ros2 topic pub /swarm/leader_cmd geometry_msgs/Twist '{ linear: { x: 1.0, y: 0.0, z: 0.0 } }'
     ```  
   - Set a formation:  
     ```bash
     ros2 topic pub /swarm/formation_cmd std_msgs/String '{ data: "square,5.0" }'
     ```  
   - Issue a waypoint:  
     ```bash
     ros2 topic pub /swarm/waypoint_cmd geometry_msgs/Point '{ x: 0.0, y: 0.0, z: -2.0 }'
     ```  
   - Apply rotation:  
     ```bash
     ros2 topic pub /swarm/rotation_cmd geometry_msgs/Vector3 '{ x: 0.0, y: 0.0, z: 1.57 }'
     ```

---

## Package Structure  

my_swarm_pkg/
├── package.xml
├── setup.py
├── launch/
│   └── swarm_launch.py         # Launches leader & 4 followers
├── resource/
│   └── my_swarm_pkg            # Marker for setup.py
├── my_swarm_pkg/
│   ├── constants.py            # RATE_HZ, KP_POS, OFFSETS, …
│   ├── leader_follower.py      # LeaderController + embedded FollowerController
│   └── follower_node.py        # Standalone FollowerController executable
└── README.md

---

## Nodes & Classes  

### 1. LeaderController (leader_follower.py)  
- Namespace: `/px4_1`  
- Publishes:  
  - OffboardControlMode  
  - TrajectorySetpoint  
  - VehicleCommand (ARM & set mode)  
  - `/swarm/follower_targets` (String)  

- Subscribes:  
  - `/px4_1/fmu/out/vehicle_odometry` (VehicleOdometry)  
  - `/swarm/leader_cmd` (Twist)  
  - `/swarm/waypoint_cmd` (Point)  
  - `/swarm/formation_cmd` (String)  
  - `/swarm/rotation_cmd` (Vector3)  
  - `/swarm/follower_arrived` (String)  
  - `/px4_{i}/fmu/out/vehicle_odometry` for each follower  

- Core loop (`20 Hz`):  
  1. Publish offboard mode  
  2. Move leader (P-control to waypoint or dead-reckoning)  
  3. Compute formation offsets → apply roll/pitch/yaw rotations → build target list  
  4. Estimate travel time & publish `/swarm/follower_targets`  
  5. ARM & set mode once (after ~1 s)  

### 2. FollowerController (follower_node.py)  
- Namespace: `/uav2`, `/uav3`, `/uav4`, `/uav5` → translates to `/px4_2`…`/px4_5` internally  
- Publishes:  
  - OffboardControlMode  
  - TrajectorySetpoint  
  - VehicleCommand (ARM & set mode)  
  - `/swarm/follower_arrived` (String)  
  - `/px4_{i+2}/follower_err` (Float32)  

- Subscribes:  
  - Leader odometry `/px4_1/fmu/out/vehicle_odometry`  
  - Own odometry `/px4_{i}/fmu/out/vehicle_odometry`  
  - Neighbor odometries for repulsion  
  - `/swarm/follower_targets` (String)  
  - `/swarm/waypoint_cmd` (Point) to reset arrival flag  

- Core loop (`20 Hz`):  
  1. Publish offboard mode  
  2. Compute feed-forward & P-control to assigned offset  
  3. Add repulsion forces if neighbors < threshold  
  4. Cap velocity magnitude to REPULSION_THRESHOLD  
  5. Zero velocity & publish “arrived” once if within ARRIVAL_THRESHOLD  
  6. Publish final TrajectorySetpoint  
  7. ARM & set offboard once after ~1 s  

---

## Topics & Message Flow  
| Topic                                    | Msg Type                | Publisher            | Subscriber(s)         |
|------------------------------------------|-------------------------|----------------------|-----------------------|
| `/px4_1/fmu/out/vehicle_odometry`        | VehicleOdometry         | PX4 SITL (leader)    | Leader + all Followers |
| `/swarm/leader_cmd`                      | geometry_msgs/Twist     | Operator / Teleop    | Leader                |
| `/swarm/waypoint_cmd`                    | geometry_msgs/Point     | Planner / Operator   | Leader, Followers     |
| `/swarm/formation_cmd`                   | std_msgs/String         | Operator             | Leader                |
| `/swarm/rotation_cmd`                    | geometry_msgs/Vector3   | Operator             | Leader                |
| `/swarm/follower_targets`                | std_msgs/String         | Leader               | Followers             |
| `/swarm/follower_arrived`                | std_msgs/String         | Followers            | Leader                |
| `<ns>/fmu/in/offboard_control_mode`      | OffboardControlMode     | Leader / Followers   | PX4 FMU               |
| `<ns>/fmu/in/trajectory_setpoint`        | TrajectorySetpoint      | Leader / Followers   | PX4 FMU               |
| `<ns>/fmu/in/vehicle_command`            | VehicleCommand          | Leader / Followers   | PX4 FMU               |
| `/px4_{i+2}/follower_err`                | Float32                 | Followers            | Monitoring (optional) |

---

## Tuning Parameters  
Defined in **`constants.py`** (and top of scripts):

python
RATE_HZ             = 20.0        # control loop frequency (Hz)
Z_CMD               = -1.0        # default target altitude (m)
OFFSETS             = [(0.0,3.0), (0.0,-30.0), (-30.0,0.0)]
N_FOLLOW            = len(OFFSETS)
KP_POS              = 0.6         # P-gain for position controller
ARRIVAL_THRESHOLD   = 0.2         # [m] threshold to consider “arrived”
REPULSION_GAIN      = 1.0         # gain for inter-agent repulsion
REPULSION_THRESHOLD = 1.5         # [m] minimum repulsion distance

> **Note:** Adjust these parameters according to your UAV dynamics and desired formation size.

---

## References & Links  
- ROS 2 Installation: https://docs.ros.org/en/rolling/Installation.html  
- PX4 Development Environment: https://docs.px4.io/master/en/dev_setup/dev_env.html  
- PX4–ROS 2 Bridge (px4_ros_com): https://github.com/PX4/px4_ros_com  
- Gazebo Classic Installation: http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install  

---

Thank you for using **my_swarm_pkg**! Feel free to raise issues or contribute on GitHub.
