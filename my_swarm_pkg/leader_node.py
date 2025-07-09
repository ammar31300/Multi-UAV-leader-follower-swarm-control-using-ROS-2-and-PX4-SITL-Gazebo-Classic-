
#!/usr/bin/env python3
# swarm_package/leader_controller.py

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
MIN_HORIZ_DELAY = 0.3   # ثانیه

from geometry_msgs.msg import Twist, Point, Vector3
from std_msgs.msg import String
from px4_msgs.msg import VehicleOdometry, OffboardControlMode, TrajectorySetpoint, VehicleCommand
from my_swarm_pkg.constants import *

def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q[0]*q[3] + q[1]*q[2]),
        1.0 - 2.0*(q[2]**2 + q[3]**2),
    )

def compute_formation_positions(cx, cy, formation, spacing, M):
    """
    Compute M = (#followers + 1) absolute positions in 2D:
     - The very first entry (index 0) is the leader (center).
     - The next entries are the followers.
    """

    offsets = []

    if formation == 'line':
        # Equally spaced along the x‐axis, centered on leader:
        # dx_i = (i - (M-1)/2)*spacing,  dy = 0
        for i in range(M):
            dx = (i - (M - 1)/2.0) * spacing
            offsets.append((dx, 0.0))

    elif formation == 'square':
        # Build an SxS grid that contains at least M points, then pick the M
        S = math.ceil(math.sqrt(M))
        mid = (S - 1)/2.0
        cells = []
        for row in range(S):
            for col in range(S):
                dx = (col - mid) * spacing
                dy = (mid - row) * spacing
                cells.append((dx, dy))
        # Sort by closeness to center so the leader ends up at the very first slot
        cells.sort(key=lambda p: abs(p[0]) + abs(p[1]))
        offsets = cells[:M]

    elif formation == 'triangle':
        # Triangular lattice: find r such that r*(r+1)/2 >= M
        r = 1
        while r*(r+1)//2 < M:
            r += 1
        vert_spacing = spacing * math.sqrt(3)/2.0
        cells = []
        # row = 1..r, each row has 'row' points
        for row in range(1, r+1):
            y = ((r-1)/2.0 - (row-1)) * vert_spacing
            mid = (row - 1)/2.0
            for j in range(row):
                dx = (j - mid) * spacing
                dy = y
                cells.append((dx, dy))
        # again, leader at first entry
        cells.sort(key=lambda p: p[0]*p[0] + p[1]*p[1])
        offsets = cells[:M]

    else:
        # default – use OFFSETS constant (unit‐vectors) scaled by spacing
        offsets.append((0.0, 0.0))  # leader
        for i in range(1, M):
            dx0, dy0 = OFFSETS[(i-1) % len(OFFSETS)]
            offsets.append((dx0 * spacing, dy0 * spacing))

    # Translate offsets into absolute positions
    pts = [[cx + dx, cy + dy] for dx, dy in offsets]
    return pts


class LeaderController(Node):
    def __init__(self):
        super().__init__(f"{LEADER_NS}_leader_controller")

        # --- internal state
        self.pose = None          # latest VehicleOdometry
        self.cmd_vel = None       # Twist from /swarm/leader_cmd
        self.waypoint = None      # geometry_msgs/Point
        self.formation = 'line'
        self.spacing = 2.0
        self.armed = False
        self.followers = {}       # idx->(x,y)
        self.arrived_followers = set()
        self.arm_wait_counter = 0
        # initial rotation angles (roll,pitch,yaw)
        self.rotation_angles = (0.0, 0.0, 0.0)

        # --- subscriptions
        self.create_subscription(
            VehicleOdometry,
            f"/{LEADER_NS}/fmu/out/vehicle_odometry",
            self.odom_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(Twist, "/swarm/leader_cmd", self.cmd_cb, 10)
        self.create_subscription(
            String, "/swarm/formation_cmd", self.formation_cb, formation_qos
        )
        self.create_subscription(Point, "/swarm/waypoint_cmd", self.waypoint_cb, 10)
        self.create_subscription(String, "/swarm/follower_arrived", self.arrived_cb, 10)
        self.create_subscription(
            Vector3, "/swarm/rotation_cmd", self.rotation_cb, 10
        )

        # follower status
        for i in range(N_FOLLOW):
            ns = f"px4_{i+2}"
            self.create_subscription(
                VehicleOdometry,
                f"/{ns}/fmu/out/vehicle_odometry",
                lambda msg, idx=i: self._follower_status_cb(idx, msg),
                qos_profile_sensor_data,
            )

        # --- publishers
        prefix = f"/{LEADER_NS}/fmu/in"
        self.mode_pub = self.create_publisher(
            OffboardControlMode, f"{prefix}/offboard_control_mode", 10
        )
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, f"{prefix}/trajectory_setpoint", 10
        )
        self.cmd_pub = self.create_publisher(
            VehicleCommand, f"{prefix}/vehicle_command", 10
        )
        self.target_pub = self.create_publisher(
            String, "/swarm/follower_targets", formation_qos
        )

        # control loop
        self.create_timer(1.0 / RATE_HZ, self.control_loop)
        self.get_logger().info("LeaderController initialized")

    # --- callbacks
    def odom_cb(self, msg):
        self.pose = msg

    def cmd_cb(self, msg):
        self.cmd_vel = msg

    def formation_cb(self, msg):
        parts = msg.data.split(',')
        self.formation = parts[0]
        if len(parts) > 1:
            self.spacing = float(parts[1])
        self.get_logger().info(f"Formation set to {self.formation}, spacing={self.spacing}")

    def waypoint_cb(self, msg):
        self.waypoint = msg
        self.arrived_followers.clear()
        self.get_logger().info(f"New waypoint: ({msg.x:.2f},{msg.y:.2f},{msg.z:.2f})")

    def arrived_cb(self, msg):
        idx = int(msg.data)
        self.arrived_followers.add(idx)

    def rotation_cb(self, msg):
        # x=roll, y=pitch, z=yaw in radians
        self.rotation_angles = (msg.x, msg.y, msg.z)
        self.get_logger().info(f"Rotation set to (r,p,y)=({msg.x:.2f},{msg.y:.2f},{msg.z:.2f})")

    def _follower_status_cb(self, idx, msg):
        p = msg.position
        self.followers[idx] = (p[0], p[1])

    def send_cmd(self, command, p1=0.0, p2=0.0):
        c = VehicleCommand()
        c.command = command
        c.param1 = p1
        c.param2 = p2
        c.target_system = int(LEADER_NS.split('_')[1]) + 1
        c.target_component = 1
        c.from_external = True
        self.cmd_pub.publish(c)

    # --- main control loop
    def control_loop(self):
        if not self.pose:
            return

        ts = int(self.get_clock().now().nanoseconds / 1e3)
        now = self.get_clock().now().nanoseconds / 1e9

        # Always publish Offboard(position=True)
        self.mode_pub.publish(OffboardControlMode(timestamp=ts, position=True))

        # 1) ARMED / TAKEOFF logic (once)
        self.arm_wait_counter += 1
        if not self.armed and self.arm_wait_counter > RATE_HZ:
            self.get_logger().info("Arming leader and switching to OFFBOARD")
            self.send_cmd(400, 1.0)       # ARM
            self.send_cmd(176, 1.0, 6.0)  # OFFBOARD
            self.armed = True
            return

        # wait for waypoint or cmd_vel
        if not self.waypoint:
            # no specific waypoint → manual cmd_vel is handled below
            pass

        # 2) FORMATION computation
        #   M = N_FOLLOW + 1 (leader + followers)
        M = N_FOLLOW + 1
        # center is either waypoint or current pose
        cx = self.waypoint.x if self.waypoint else self.pose.position[0]
        cy = self.waypoint.y if self.waypoint else self.pose.position[1]
        cz = self.waypoint.z if self.waypoint else self.pose.position[2]

        # get raw 2D positions (leader + followers)
        pts2d = compute_formation_positions(cx, cy, self.formation, self.spacing, M)

        # Apply rotation in 3D about leader center:
        rx, ry, rz = self.rotation_angles
        sx, cx1 = math.sin(rx), math.cos(rx)
        sy, cy1 = math.sin(ry), math.cos(ry)
        sz, cz1 = math.sin(rz), math.cos(rz)

        pts = []
        for (xi, yi) in pts2d:
            # local vector from center
            dx0 = xi - cx
            dy0 = yi - cy
            dz0 = 0.0
            # Rx
            x1, y1, z1 = dx0, cx1*dy0 - sx*dz0, sx*dy0 + cx1*dz0
            # Ry
            x2, y2, z2 = cy1*x1 + sy*z1, y1, -sy*x1 + cy1*z1
            # Rz
            x3, y3, z3 = cz1*x2 - sz*y2, sz*x2 + cz1*y2, z2
            pts.append([cx + x3, cy + y3, cz + z3])

        # 3) compute distances from current positions → each target
        dists = []
        # i=0 is leader
        xL, yL = self.pose.position[0], self.pose.position[1]
        for i, (xt, yt, zt) in enumerate(pts):
            if i == 0:
                d = math.hypot(xt - xL, yt - yL)
            else:
                fx, fy = self.followers.get(i-1, (xL, yL))
                d = math.hypot(xt - fx, yt - fy)
            dists.append(d)

        # 4) determine travel_time so everyone arrives together
        desired_speed = self.spacing
        max_dist = max(dists) if dists else 0.0
        travel_time = max_dist / desired_speed if desired_speed > 1e-6 else 0.0
        t_arrival = now + travel_time

        # 5) publish "follower_targets" (only followers, idx=0..N_FOLLOW-1)
        parts = []
        for i in range(1, M):
            xi, yi, zi = pts[i]
            parts.append(f"{i-1},{xi:.2f},{yi:.2f},{zi:.2f},{t_arrival:.3f}")
        self.target_pub.publish(String(data=";".join(parts)))

        # 6) Leader sets its own TrajectorySetpoint to move in sync
        rem = t_arrival - now
        # Feed‐forward velocity
        vx_ff = (pts[0][0] - xL) / rem if rem > MIN_HORIZ_DELAY else 0.0
        vy_ff = (pts[0][1] - yL) / rem if rem > MIN_HORIZ_DELAY else 0.0
        # P‐term
        vx_p = KP_POS * (pts[0][0] - xL)
        vy_p = KP_POS * (pts[0][1] - yL)
        vz_ff = (pts[0][2] - self.pose.position[2]) / rem if rem > MIN_HORIZ_DELAY else 0.0
        vz_p = KP_POS * (pts[0][2] - self.pose.position[2])

        vx = vx_ff + vx_p
        vy = vy_ff + vy_p
        vz = vz_ff + vz_p

        # publish leader setpoint
        sp = TrajectorySetpoint()
        sp.timestamp = ts
        # we can publish either the *current* position with velocity,
        # or directly the target position – here we keep current pos
        sp.position = [
            float(self.pose.position[0]),
            float(self.pose.position[1]),
            float(self.pose.position[2]),
        ]
        sp.velocity = [float(vx), float(vy), float(vz)]
        # apply yaw rotation
        base_yaw = quaternion_to_yaw(self.pose.q)
        sp.yaw = base_yaw + self.rotation_angles[2]

        self.traj_pub.publish(sp)

        # Done control loop