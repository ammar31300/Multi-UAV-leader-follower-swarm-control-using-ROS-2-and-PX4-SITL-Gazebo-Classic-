# my_swarm_pkg  
Multi-UAV leader–follower swarm control using ROS 2 and PX4 SITL (Gazebo Classic)

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



## 5. Package Structure

```
my_swarm_pkg/
├── launch/
│   └── swarm_launch.py         # Launch file for leader + N followers
│
├── my_swarm_pkg/
│   ├── constants.py            # RATE_HZ, N_FOLLOW, OFFSETS, QoS profiles, etc.
│   ├── leader_follower.py      # (unused – embedded version)
│   ├── follower_node.py        # Standalone FollowerController executable
│   ├── leader_node.py          # LeaderController executable
│   └── leader_manual.py        # Manual teleop & spawn logic
│
├── package.xml
├── setup.py
└── README.md                   # Original doc (superseded by this)
```

---

## 6. Constants

*All constants are defined in* `constants.py` *and some at the top of follower\_node.py / leader\_node.py.*

* `RATE_HZ`: 20.0
* `N_FOLLOW`: 4
* `LEADER_NS`: 'px4\_1'
* `OFFSETS`: Default unit vector offsets for formations
* `formation_qos`: QoSProfile for String formation messages
* `Z_CMD` / `SAFE_ALT`: 1.0
* `ARRIVAL_THRESHOLD`: 0.2
* `REPULSION_GAIN`: 1.0
* `REPULSION_THRESHOLD`: 1.5
* `HEARTBEAT_TIMEOUT`: 2.0
* `MAX_VZ`: 0.5
* `MIN_VZ`: -1.0
* `TAKEOFF_THRESHOLD`: 1.8
* `KP_POS`: 1.2
* `MIN_HORIZ_DELAY`: 0.3

---

## 7. Topics & Message Flow

**LeaderController** publishes:

* `/LEADER_NS/fmu/in/offboard_control_mode` (OffboardControlMode)
* `/LEADER_NS/fmu/in/trajectory_setpoint` (TrajectorySetpoint)
* `/LEADER_NS/fmu/in/vehicle_command` (VehicleCommand)
* `/swarm/follower_targets` (String)
* `/swarm/leader_heartbeat` (String)

**LeaderController** subscribes:

* `/LEADER_NS/fmu/out/vehicle_odometry` (VehicleOdometry)
* `/swarm/leader_cmd` (Twist)
* `/swarm/formation_cmd` (String)
* `/swarm/waypoint_cmd` (Point)
* `/swarm/follower_arrived` (String)
* `/swarm/rotation_cmd` (Vector3)
* `/px4_2..px4_{N+1}/fmu/out/vehicle_odometry` (VehicleOdometry)
* `/swarm/leader_disarm` (Empty)

**FollowerController** publishes:

* `/swarm/follower_arrived` (String)
* `/px4_{i+2}/fmu/in/offboard_control_mode` (OffboardControlMode)
* `/px4_{i+2}/fmu/in/trajectory_setpoint` (TrajectorySetpoint)
* `/px4_{i+2}/fmu/in/vehicle_command` (VehicleCommand)
* `/px4_{i+2}/follower_err` (Float32)

**FollowerController** subscribes:

* `/swarm/leader_heartbeat` (String)
* `/swarm/leader_disarm` (Empty)
* `/LEADER_NS/fmu/out/vehicle_odometry` (VehicleOdometry)
* `/px4_{i+2}/fmu/out/vehicle_odometry` (VehicleOdometry)
* `/px4_{j+2}/fmu/out/vehicle_odometry` (VehicleOdometry)
* `/swarm/follower_targets` (String)
* `/swarm/waypoint_cmd` (Point)

**LeaderManual** publishes:

* `/swarm/leader_cmd` (Twist)
* `/swarm/formation_cmd` (String)
* `/swarm/rotation_cmd` (Vector3)
* `/swarm/leader_disarm` (Empty)
* `/swarm/mission_cmd` (String)

**LeaderManual** subscribes:

* `/swarm/follower_targets` (String)
* `/swarm/leader_heartbeat` (String)

---

## 8. Nodes, Classes & Methods

\[See original document for full class and method details.]

---

## 9. Tuning Parameters

* `RATE_HZ`: 20
* `SAFE_ALT`: 1.0
* `ARRIVAL_THRESHOLD`: 0.2
* `REPULSION_THRESHOLD`: 1.5
* `REPULSION_GAIN`: 1.0
* `KP_POS`: 1.2 (followers), 0.6 (leader)
* `KD_POS`: from constants.py
* `MIN_VZ` / `MAX_VZ`: -1.0 to 0.5
* `HEARTBEAT_TIMEOUT`: 2.0
* `MIN_HORIZ_DELAY`: 0.3

---

## 10. Leader Election & Re-spawn Logic

* Heartbeat sent by leader on `/swarm/leader_heartbeat`
* Followers watch for loss of heartbeat
* If timeout occurs, all followers elect the lowest index still active
* The winner spawns a new `leader_node` via `subprocess.Popen()`
* All nodes that receive `/swarm/leader_disarm` mark that leader as permanently inactive
* `LeaderManual` can also spawn a new leader if it detects timeout

---

## 11. Formation Types & Computation

Supported formations (sent via `/swarm/formation_cmd`):

* `line`: UAVs aligned in x-y
* `square`: Grid-shaped square layout
* `triangle`: Triangular formation pattern
* `column`: UAVs vertically stacked along z

All are computed using offset vectors, spacing, and optional rotation commands (`/swarm/rotation_cmd`).

---

## 12. LeaderManual Keyboard Commands

* Arrow keys: Move leader in x/y
* PageUp/PageDown or `r`/`f`: vertical z motion
* `x`: Disarm leader (triggers re-election)
* `+` / `-`: Increase/decrease formation spacing
* `v` / `s` / `t` / `b`: Select line/square/triangle/column formation
* `w`, `a`, `d`: Adjust yaw and roll
* `m`: Upload mission (waypoint string)

---

## 13. License & Acknowledgements

Distributed under \[Your License]
Thanks to PX4, ROS 2 community, Gazebo Classic.

