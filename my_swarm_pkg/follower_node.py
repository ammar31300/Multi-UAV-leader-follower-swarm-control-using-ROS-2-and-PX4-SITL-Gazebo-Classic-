# swarm_package/follower_controller.py
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point, Vector3
from px4_msgs.msg import VehicleOdometry, OffboardControlMode, TrajectorySetpoint, VehicleCommand
from my_swarm_pkg.constants import *
def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q[0]*q[3] + q[1]*q[2]),
        1.0 - 2.0*(q[2]**2 + q[3]**2),
    )
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
        # کانتر پیش از آرمیگ
        self.arm_wait_counter = 0
        self.enable_repulsion = True
        # سابسکریپشن‌ها
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

        # پابلیشرها
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
        if not (self.leader and self.odom):
            return

        # افزایش کانتر قبل از آرمیگ
        self.arm_wait_counter += 1

        ts = int(self.get_clock().now().nanoseconds / 1e3)
        self.mode_pub.publish(OffboardControlMode(timestamp=ts, position=True))

        # هدف
        xd, yd = self.offset
        zd = self.z_target
        x0, y0, z0 = self.odom.position

        now = self.get_clock().now().nanoseconds / 1e9
        remaining = self.t_arrival - now

        # محدود کردن Feed-forward برای remaining < 0.5s
        if remaining > 0.1:
            vx_ff = (xd - x0) / remaining
            vy_ff = (yd - y0) / remaining
            vz_ff = (zd - z0) / remaining
        else:
            vx_ff = vy_ff = vz_ff = 0.0

        # کنترل تناسبی
        vx_p = KP_POS * (xd - x0)
        vy_p = KP_POS * (yd - y0)
        vz_p = KP_POS * (zd - z0)

        vx = vx_ff + vx_p
        vy = vy_ff + vy_p
        vz = vz_ff + vz_p

        # سقف سرعت به آستانه دافعه
        v_mag = math.sqrt(vx ** 2 + vy ** 2 + vz ** 2)
        if v_mag > REPULSION_THRESHOLD:
            scale = REPULSION_THRESHOLD / v_mag
            vx *= scale
            vy *= scale
            vz *= scale

        # نیروی دافعه از همسایه‌ها
        for pos in self.neighbors.values():
            dx = x0 - pos[0]
            dy = y0 - pos[1]
            dist = math.hypot(dx, dy)
            if 1e-3 < dist < REPULSION_THRESHOLD:
                rep = REPULSION_GAIN * (1.0 / dist - 1.0 / REPULSION_THRESHOLD)
                rep = max(rep, 0.0)
                vx += rep * (dx / dist)
                vy += rep * (dy / dist)

        err = math.sqrt((xd - x0) ** 2 + (yd - y0) ** 2 + (zd - z0) ** 2)
        self.err_pub.publish(Float32(data=err))

        # وقتی در آستانه رسیدیم، سرعت‌‌ها را صفر کن
        if err < ARRIVAL_THRESHOLD:
            vx = vy = vz = 0.0

        # انتشار "arrived" فقط یک‌بار
        if not self.arrived and err < ARRIVAL_THRESHOLD:
            self.arrival_pub.publish(String(data=str(self.idx)))
            self.arrived = True
            self.get_logger().info(f"Follower {self.idx} arrived")

        # نشر TrajectorySetpoint نهایی
        sp = TrajectorySetpoint()
        sp.timestamp = ts
        sp.position = [float(x0), float(y0), float(z0)]
        sp.velocity = [float(vx), float(vy), float(vz)]
        sp.yaw = quaternion_to_yaw(self.leader.q)
        self.traj_pub.publish(sp)

        # آرمیگ: فقط یک‌بار و پس از ۲۰ حلقه
        if not self.armed and self.arm_wait_counter > int(RATE_HZ * 1.0):
            self.get_logger().info(f"Arming follower {self.idx}")
            # ARM
            self.send_cmd(400, 1.0)
            # MODE -> OFFBOARD
            self.send_cmd(176, 1.0, 6.0)
            self.armed = True

