# swarm_package/leader_controller.py
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy

from geometry_msgs.msg import Twist, Point, Vector3
from std_msgs.msg import String
from px4_msgs.msg import VehicleOdometry, OffboardControlMode, TrajectorySetpoint, VehicleCommand
from my_swarm_pkg.constants import *

def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q[0]*q[3] + q[1]*q[2]),
        1.0 - 2.0*(q[2]**2 + q[3]**2),
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

# … (بقیه‌ی import و کدهای قبل) …
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
        # کانتر قبل از آرمیگ
        self.arm_wait_counter = 0
        self.takeoff_completed = True

        # ---- اضافه: نگهداری زوایای دوران (roll,pitch,yaw)
        self.rotation_angles = (0.0, 0.0, 0.0)

        # سابسکریپشن‌ها
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

        # ---- اضافه: سابسکریپشنی برای زاویه دوران
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

        # پابلیشرها
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
        # msg.x = roll, msg.y = pitch, msg.z = yaw (radians)
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

        # افزایش کانتر قبل از آرمیگ
        self.arm_wait_counter += 1

        ts = int(self.get_clock().now().nanoseconds / 1e3)
        # همیشه OffboardControlMode(position=True) بفرست
        self.mode_pub.publish(OffboardControlMode(timestamp=ts, position=True))

         # ۱) فاز پرواز اولیه (Takeoff): فقط صعود عمودی تا ارتفاع ۲ متر
        # اگر متغیر takeoff_completed در __init__ تعریف نشده، مقداردهی اولیه‌اش کن
        if not hasattr(self, 'takeoff_completed'):
            self.takeoff_completed = False

        if not self.takeoff_completed:
            # ارتفاع فعلی (از odom.z) و ارتفاع هدف (۲ متر)
            current_z = self.odom.position[2]
            target_z = Z_CMD * 2.0   # Z_CMD منفی است، پس -1.0 * 2.0 = -2.0

            # اگر در محدوده ±۵ سانتیمتر از هدف هستیم، فاز Takeoff تمام شد
            if abs(current_z - target_z) < 0.05:
                self.takeoff_completed = True
            else:
                # فقط فرمان عمودی بده؛ vx, vy = 0، vz مثبت برای صعود
                ts = int(self.get_clock().now().nanoseconds / 1e3)
                self.mode_pub.publish(
                    OffboardControlMode(timestamp=ts, position=True)
                )
                traj = TrajectorySetpoint()
                traj.timestamp = ts
                traj.position = [0.0, 0.0, float(target_z)]
                traj.velocity = [0.0, 0.0, 0.5]   # سرعت صعود (می‌توان تنظیم کرد)
                traj.yaw = quaternion_to_yaw(self.leader.q)
                self.traj_pub.publish(traj)
            return


        # ۱) کنترل رهبر: waypoint یا cmd_vel
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

        if not self.takeoff_completed:
            # فرض: Z_CMD = -1.0 معادل ارتفاع 1 متر مثبت
            # برای ارتفاع 2 متر، آستانه را -2.0 در نظر می‌گیریم
            if abs(self.pose.position[2] - (-2.0)) < 0.1:
                self.takeoff_completed = True
            else:
                return

        # ۲) محاسبهٔ Formation و ارسال target به followers، با دوران
        cx, cy, cz = (
            (self.waypoint.x, self.waypoint.y, self.waypoint.z)
            if self.waypoint
            else (
                self.pose.position[0],
                self.pose.position[1],
                self.pose.position[2],
            )
        )
        # اول محاسبهٔ بدون دوران (2D)
        pts2d = compute_formation_positions(
            cx, cy, self.formation, self.spacing, N_FOLLOW
        )

        # ماتریس‌های دوران سه‌محوره
        rx, ry, rz = self.rotation_angles
        sx, cx1 = math.sin(rx), math.cos(rx)
        sy, cy1 = math.sin(ry), math.cos(ry)
        sz, cz1 = math.sin(rz), math.cos(rz)
        # Rx, Ry, Rz
        # Rx = [[1, 0, 0], [0, cx1, -sx], [0, sx, cx1]]
        # Ry = [[cy1, 0, sy], [0, 1, 0], [-sy, 0, cy1]]
        # Rz = [[cz1, -sz, 0], [sz, cz1, 0], [0, 0, 1]]

        pts = []
        for (xi, yi) in pts2d:
            # نقطهٔ محلی نسبت به مرکز (dx,dy,0)
            dx = xi - cx
            dy = yi - cy
            dz = 0.0
            # دوران حول X
            y1 = cx1 * dy - sx * dz
            z1 = sx * dy + cx1 * dz
            x1 = dx
            # دوران حول Y
            x2 = cy1 * x1 + sy * z1
            z2 = -sy * x1 + cy1 * z1
            y2 = y1
            # دوران حول Z
            x3 = cz1 * x2 - sz * y2
            y3 = sz * x2 + cz1 * y2
            z3 = z2
            # نقطهٔ جهانی
            pts.append([cx + x3, cy + y3, cz + z3])

        # محاسبهٔ فاصله‌ها و ارسال
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

        # ۳) آرمیگ: فقط یک‌بار و بعد از حداقل ۲۰ حلقه
        if not self.armed and self.arm_wait_counter > int(RATE_HZ * 1.0):
            # ARM
            self.send_cmd(400, 1.0)
            # MODE -> OFFBOARD
            self.send_cmd(176, 1.0, 6.0)
            self.armed = True

