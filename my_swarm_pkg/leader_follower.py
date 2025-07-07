#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, Point, Vector3
from px4_msgs.msg import (
    VehicleOdometry,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
)

RATE_HZ = 20.0
Z_CMD = -1.0
LEADER_NS = 'px4_1'
OFFSETS = [(0.0, 3.0), (0.0, -30), (-30, 0.0)]
N_FOLLOW = len(OFFSETS)
KP_POS = 0.6
ARRIVAL_THRESHOLD = 0.2
REPULSION_GAIN = 1.0
REPULSION_THRESHOLD = 1.5
formation_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q[0] * q[3] + q[1] * q[2]),
        1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2),
    )

def compute_formation_positions(cx, cy, formation, spacing, N):
    pts = []
    r = spacing
    if formation == 'line':
        for i in range(N):
            offset = (i - (N - 1) / 2.0) * r
            pts.append([cx - offset, cy])
    elif formation == 'square':
        S = int(math.ceil(math.sqrt(N)))
        cells = []
        mid = (S - 1) / 2.0
        for row in range(S):
            for col in range(S):
                dx = (col - mid) * r
                dy = ((S - 1 - row) - mid) * r
                if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                    continue
                cells.append((dx, dy))
        pts = [[cx + dx, cy + dy] for dx, dy in cells[:N]]
    else:
        for i in range(N):
            dx, dy = OFFSETS[i % len(OFFSETS)]
            pts.append([cx + dx, cy + dy])
    return pts

class LeaderController(Node):
    def __init__(self):
        super().__init__(f"{LEADER_NS}_leader_controller")
        # حالت اولیه
        self.pose = None
        self.cmd_vel = None
        self.waypoint = None
        self.formation = 'line'
        self.spacing = 2.0
        self.armed = False
        self.followers = {}
        self.arrived_followers = set()
        self.arm_wait_counter = 0
        self.rotation_angles = (0.0, 0.0, 0.0)

        # Subscriptions
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
            Vector3,
            "/swarm/rotation_cmd",
            self.rotation_cb,
            10,
        )

        for i in range(N_FOLLOW):
            ns = f"px4_{i+2}"
            self.create_subscription(
                VehicleOdometry,
                f"/{ns}/fmu/out/vehicle_odometry",
                lambda msg, idx=i: self._follower_status_cb(idx, msg),
                qos_profile_sensor_data,
            )

        # Publishers
        self.mode_pub = self.create_publisher(
            OffboardControlMode,
            f"/{LEADER_NS}/fmu/in/offboard_control_mode",
            10,
        )
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint,
            f"/{LEADER_NS}/fmu/in/trajectory_setpoint",
            10,
        )
        self.cmd_pub = self.create_publisher(
            VehicleCommand, f"/{LEADER_NS}/fmu/in/vehicle_command", 10
        )
        self.target_pub = self.create_publisher(
            String, "/swarm/follower_targets", formation_qos
        )

        self.create_timer(1.0 / RATE_HZ, self.control_loop)
        self.get_logger().info("LeaderController initialized")

    def odom_cb(self, msg):
        self.pose = msg

    def cmd_cb(self, msg):
        self.cmd_vel = msg

    def formation_cb(self, msg):
        parts = msg.data.split(',')
        self.formation = parts[0]
        if len(parts) > 1:
            self.spacing = float(parts[1])

    def waypoint_cb(self, msg):
        self.waypoint = msg
        self.arrived_followers.clear()

    def arrived_cb(self, msg):
        idx = int(msg.data)
        self.arrived_followers.add(idx)

    def rotation_cb(self, msg):
        self.rotation_angles = (msg.x, msg.y, msg.z)

    def _follower_status_cb(self, idx, msg):
        pos = msg.position
        self.followers[idx] = (pos[0], pos[1])

    def send_cmd(self, command, p1=0.0, p2=0.0):
        c = VehicleCommand()
        c.command = command
        c.param1 = p1
        c.param2 = p2
        c.target_system = int(LEADER_NS.split('_')[1]) + 1
        c.target_component = 1
        c.from_external = True
        self.cmd_pub.publish(c)

    def control_loop(self):
        if not self.pose:
            return

        self.arm_wait_counter += 1
        ts = int(self.get_clock().now().nanoseconds / 1e3)
        self.mode_pub.publish(OffboardControlMode(timestamp=ts, position=True))

        # --- کنترل لیدر (مسیر یا cmd_vel) ---
        if self.waypoint:
            x0 = self.pose.position[0]
            y0 = self.pose.position[1]
            dx = self.waypoint.x - x0
            dy = self.waypoint.y - y0
            vx = KP_POS * dx
            vy = KP_POS * dy
            err = math.hypot(dx, dy)
            if err < ARRIVAL_THRESHOLD:
                vx = vy = 0.0
            sp = TrajectorySetpoint()
            sp.timestamp = ts
            sp.position = [
                self.waypoint.x,
                self.waypoint.y,
                self.waypoint.z,
            ]
            sp.velocity = [float(vx), float(vy), 0.0]
            self.traj_pub.publish(sp)
        else:
            dt = 1.0 / RATE_HZ
            x0 = self.pose.position[0]
            y0 = self.pose.position[1]
            vx = self.cmd_vel.linear.x if self.cmd_vel else 0.0
            vy = self.cmd_vel.linear.y if self.cmd_vel else 0.0
            x1 = x0 + vx * dt
            y1 = y0 + vy * dt
            sp = TrajectorySetpoint()
            sp.timestamp = ts
            sp.position = [float(x1), float(y1), float(Z_CMD)]
            sp.velocity = [0.0, 0.0, 0.0]
            self.traj_pub.publish(sp)

        # --- محاسبه و ارسال اهداف به فالورها با دوران ---
        cx, cy, cz = (
            (self.waypoint.x, self.waypoint.y, self.waypoint.z)
            if self.waypoint
            else (
                self.pose.position[0],
                self.pose.position[1],
                self.pose.position[2],
            )
        )
        pts2d = compute_formation_positions(
            cx, cy, self.formation, self.spacing, N_FOLLOW
        )
        rx, ry, rz = self.rotation_angles
        sx, cx1 = math.sin(rx), math.cos(rx)
        sy, cy1 = math.sin(ry), math.cos(ry)
        sz, cz1 = math.sin(rz), math.cos(rz)
        pts = []
        for (xi, yi) in pts2d:
            dx = xi - cx
            dy = yi - cy
            dz = 0.0
            # دوران X
            y1 = cx1 * dy - sx * dz
            z1 = sx * dy + cx1 * dz
            x1 = dx
            # دوران Y
            x2 = cy1 * x1 + sy * z1
            z2 = -sy * x1 + cy1 * z1
            y2 = y1
            # دوران Z
            x3 = cz1 * x2 - sz * y2
            y3 = sz * x2 + cz1 * y2
            z3 = z2
            pts.append([cx + x3, cy + y3, cz + z3])

        distances = []
        for i, (xi, yi, zi) in enumerate(pts):
            x_i, y_i = self.followers.get(i, (cx, cy))
            distances.append(math.hypot(xi - x_i, yi - y_i))

        desired_speed = self.spacing
        max_dist = max(distances) if distances else 0.0
        travel_time = max_dist / desired_speed if desired_speed > 0 else 0.0
        now = self.get_clock().now().nanoseconds / 1e9
        t_arrival = now + travel_time

        parts = [
            f"{i},{xi:.2f},{yi:.2f},{zi:.2f},{t_arrival:.3f}"
            for i, (xi, yi, zi) in enumerate(pts)
        ]
        self.target_pub.publish(String(data=";".join(parts)))

        # --- آرمیگ لیدر ---
        if not self.armed and self.arm_wait_counter > int(RATE_HZ * 1.0):
            self.send_cmd(400, 1.0)        # ARM
            self.send_cmd(176, 1.0, 6.0)   # OFFBOARD
            self.armed = True

class FollowerController(Node):
    def __init__(self, idx, offs):
        ns = f"px4_{idx+2}"
        super().__init__(f"{ns}_follower")
        self.idx = idx
        self.offset = offs
        self.z_target = Z_CMD
        self.t_arrival = 0.0
        self.leader = None
        self.odom = None
        self.armed = False
        self.arrived = False
        self.neighbors = {}
        self.arm_wait_counter = 0

        self._initial_takeoff_done = False
        self._takeoff_target_z = None

        # Subscriptions
        self.create_subscription(
            VehicleOdometry,
            f"/{LEADER_NS}/fmu/out/vehicle_odometry",
            lambda msg: setattr(self, "leader", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleOdometry,
            f"/{ns}/fmu/out/vehicle_odometry",
            lambda msg: setattr(self, "odom", msg),
            qos_profile_sensor_data,
        )
        for j in range(N_FOLLOW):
            if j == idx:
                continue
            other_ns = f"px4_{j+2}"
            self.create_subscription(
                VehicleOdometry,
                f"/{other_ns}/fmu/out/vehicle_odometry",
                lambda msg, j=j: self._neighbor_cb(j, msg),
                qos_profile_sensor_data,
            )

        self.create_subscription(
            String, "/swarm/follower_targets", self.target_cb, formation_qos
        )
        self.create_subscription(Point, "/swarm/waypoint_cmd", self._clear_arrival, 10)

        # Publishers
        self.arrival_pub = self.create_publisher(String, "/swarm/follower_arrived", 10)
        prefix = f"/{ns}/fmu/in"
        self.mode_pub = self.create_publisher(
            OffboardControlMode, f"{prefix}/offboard_control_mode", 10
        )
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, f"{prefix}/trajectory_setpoint", 10
        )
        self.cmd_pub = self.create_publisher(
            VehicleCommand, f"{prefix}/vehicle_command", 10
        )
        self.err_pub = self.create_publisher(
            Float32, f"/px4_{idx+2}/follower_err", 10
        )

        self.create_timer(1.0 / RATE_HZ, self.control_loop)
        self.get_logger().info(f"Follower {ns} ready")

    def _neighbor_cb(self, j, msg):
        self.neighbors[j] = msg.position

    def target_cb(self, msg):
        for p in msg.data.split(';'):
            fields = p.split(',')
            if len(fields) < 5:
                continue
            idx_s, x_s, y_s, z_s, t_s = fields
            if int(idx_s) == self.idx:
                self.offset = (float(x_s), float(y_s))
                self.z_target = float(z_s)
                self.t_arrival = float(t_s)
                self.arrived = False
                break

    def _clear_arrival(self, msg):
        self.arrived = False

    def send_cmd(self, command, p1=0.0, p2=0.0):
        c = VehicleCommand()
        c.command = command
        c.param1 = p1
        c.param2 = p2
        c.target_system = self.idx + 3
        c.target_component = 1
        c.from_external = True
        self.cmd_pub.publish(c)

    def control_loop(self):
        # نیاز به دادهٔ لیدر و ادم
        if not (self.leader and self.odom):
            return

        self.arm_wait_counter += 1
        ts = int(self.get_clock().now().nanoseconds / 1e3)
        self.mode_pub.publish(OffboardControlMode(timestamp=ts, position=True))

        # ---- فاز اوّلیه: pure vertical takeoff ----
        if not self._initial_takeoff_done:
            # اگر اولین باره، ارتفاع هدف را از لیدر بخوان
            if self._takeoff_target_z is None:
                # اطمینان از float بودن
                self._takeoff_target_z = float(self.leader.position[2])

            # موقعیت فعلی فالور
            x0, y0, _ = self.odom.position
            z_goal = self._takeoff_target_z

            # ساخت پیام Setpoint
            sp = TrajectorySetpoint()
            sp.timestamp = ts
            # حتما float ذخیره شود
            sp.position = [float(x0), float(y0), float(z_goal)]
            # سرعت‌ها صفر برای صرفا صعود عمودی
            sp.velocity = [0.0, 0.0, 0.0]
            # یاعکسگردش هم صفر
            sp.yaw = 0.0
            self.traj_pub.publish(sp)

            # وقتی به ارتفاع رسیدیم، فاز اوّلیه تمام می‌شود
            _, _, z_now = self.odom.position
            if abs(z_now - z_goal) < ARRIVAL_THRESHOLD:
                self._initial_takeoff_done = True
                self.get_logger().info(f"Follower {self.idx} initial takeoff done")
            return   # بدون اجرای بخش فرمیشنی

        # ---- پس از فاز اوّلیه: کنترل فرمیشنی (اصل کد) ----
        # هدف
        xd, yd = self.offset
        zd = self.z_target
        x0, y0, z0 = self.odom.position

        now = self.get_clock().now().nanoseconds / 1e9
        remaining = self.t_arrival - now

        if remaining > 0.1:
            vx_ff = (xd - x0) / remaining
            vy_ff = (yd - y0) / remaining
            vz_ff = (zd - z0) / remaining
        else:
            vx_ff = vy_ff = vz_ff = 0.0

        vx_p = KP_POS * (xd - x0)
        vy_p = KP_POS * (yd - y0)
        vz_p = KP_POS * (zd - z0)

        vx = vx_ff + vx_p
        vy = vy_ff + vy_p
        vz = vz_ff + vz_p

        # سقف سرعت
        v_mag = math.sqrt(vx**2 + vy**2 + vz**2)
        if v_mag > REPULSION_THRESHOLD:
            scale = REPULSION_THRESHOLD / v_mag
            vx *= scale
            vy *= scale
            vz *= scale

        # نیروی دافعه
        for pos in self.neighbors.values():
            dx = x0 - pos[0]
            dy = y0 - pos[1]
            dist = math.hypot(dx, dy)
            if 1e-3 < dist < REPULSION_THRESHOLD:
                rep = REPULSION_GAIN * (1.0/dist - 1.0/REPULSION_THRESHOLD)
                rep = max(rep, 0.0)
                vx += rep * (dx/dist)
                vy += rep * (dy/dist)

        err = math.sqrt((xd-x0)**2 + (yd-y0)**2 + (zd-z0)**2)
        self.err_pub.publish(Float32(data=err))

        if err < ARRIVAL_THRESHOLD:
            vx = vy = vz = 0.0
            if not self.arrived:
                self.arrival_pub.publish(String(data=str(self.idx)))
                self.arrived = True
                self.get_logger().info(f"Follower {self.idx} arrived")

        sp = TrajectorySetpoint()
        sp.timestamp = ts
        sp.position = [float(x0), float(y0), float(z0)]
        sp.velocity = [float(vx), float(vy), float(vz)]
        sp.yaw = quaternion_to_yaw(self.leader.q)
        self.traj_pub.publish(sp)

        # آرمیگ فالور
        if not self.armed and self.arm_wait_counter > int(RATE_HZ * 1.0):
            self.get_logger().info(f"Arming follower {self.idx}")
            self.send_cmd(400, 1.0)        # ARM
            self.send_cmd(176, 1.0, 6.0)   # OFFBOARD
            self.armed = True

def main(args=None):
    rclpy.init(args=args)
    execr = rclpy.executors.MultiThreadedExecutor()
    execr.add_node(LeaderController())
    for i, offs in enumerate(OFFSETS):
        execr.add_node(FollowerController(i, offs))
    try:
        execr.spin()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
