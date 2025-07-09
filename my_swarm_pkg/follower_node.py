#!/usr/bin/env python3
# swarm_package/follower_controller.py

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point, Vector3
from px4_msgs.msg import (
    VehicleOdometry,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
)
from math import inf
from my_swarm_pkg.constants import *

# محدودیت‌های هوریزنتال و عمودی
MAX_VZ = 0.5
MIN_HORIZ_DELAY = 0.3  # تاخیر کوچک قبل از حرکت افقی [s]

def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q[0]*q[3] + q[1]*q[2]),
        1.0 - 2.0*(q[2]**2 + q[3]**2),
    )

class FollowerController(Node):
    def __init__(self, idx, offs):
        ns = f"px4_{idx+2}"
        super().__init__(f"{ns}_follower")

        # شناسه و آفست
        self.idx = idx
        self.offset = offs

        # مقادیر اولیه
        self.z_target = Z_CMD
        self.t_arrival = None
        self.target_received = False

        # وضعیت پهپاد
        self.leader = None
        self.odom = None
        self.armed = False
        self.arrived = False

        # همسایه‌ها و Repulsion
        self.neighbors = {}
        self.enable_repulsion = True

        # کانتر برای تأخیر پیش از آرمیگ
        self.arm_wait_counter = 0

        # subscriptions
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

        # publishers
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

        # timer
        self.create_timer(1.0 / RATE_HZ, self.control_loop)
        self.get_logger().info(f"Follower {ns} ready")

    def _neighbor_cb(self, j, msg):
        self.neighbors[j] = msg.position

    def target_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        for p in msg.data.split(';'):
            idx_s, x_s, y_s, z_s, t_s = p.split(',')
            if int(idx_s) == self.idx:
                self.offset = (float(x_s), float(y_s))
                self.z_target = float(z_s)
                self.t_arrival = float(t_s)
                self.target_received = True
                self.arrived = False
                self.get_logger().info(
                    f"Follower{self.idx}: got target, t_arrival={self.t_arrival:.3f}"
                )
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
        # نیاز به دسترسی به لیدر و اُدم
        if not (self.leader and self.odom):
            return

        ts = int(self.get_clock().now().nanoseconds / 1e3)
        now = self.get_clock().now().nanoseconds / 1e9

        # فاز ۱: Take-off با کنترل Velocity (تا زمانی که آرمیگ نشده‌ایم)
        if not self.armed:
            # ارسال Offboard Mode برای Velocity
            self.mode_pub.publish(
                OffboardControlMode(
                    timestamp=ts,
                    position=False,
                    velocity=True,
                    acceleration=False,
                    attitude=False,
                    body_rate=False,
                )
            )

            # محاسبه سرعت vz برای صعود
            z0 = self.odom.position[2]
            vz_p = KP_POS * (self.z_target - z0)
            vz = max(min(vz_p, MAX_VZ), -MAX_VZ)

            # انتشار Velocity Setpoint
            sp = TrajectorySetpoint()
            sp.timestamp = ts
            # در مد Velocity، فیلد position نادیده گرفته می‌شود
            sp.position = [0.0, 0.0, 0.0]
            sp.velocity = [0.0, 0.0, float(vz)]
            sp.yaw = quaternion_to_yaw(self.odom.q)
            self.traj_pub.publish(sp)

            # آرمیگ پس از ۱ ثانیه
            self.arm_wait_counter += 1
            if self.arm_wait_counter > int(RATE_HZ * 1.0):
                self.get_logger().info(f"Arming follower {self.idx}")
                self.send_cmd(400, 1.0)       # ARM
                self.send_cmd(176, 1.0, 6.0)  # OFFBOARD
                self.armed = True
            return

        # فاز ۲: پس از آرمیگ، تا دریافت Formation صبر کن
        if not self.target_received:
            # حین انتظار، Offboard Position را ارسال کن تا PX4 در offboard نماند
            self.mode_pub.publish(OffboardControlMode(timestamp=ts, position=True))
            return

        # فاز ۳: کنترل Formation (XY و Z)
        # ارسال Offboard Position
        self.mode_pub.publish(OffboardControlMode(timestamp=ts, position=True))

        xd, yd = self.offset
        zd = self.z_target
        x0, y0, z0 = self.odom.position

        # Feed-forward
        rem = self.t_arrival - now
        if rem > MIN_HORIZ_DELAY:
            vx_ff = (xd - x0) / rem
            vy_ff = (yd - y0) / rem
            vz_ff = (zd - z0) / rem
        else:
            vx_ff = vy_ff = vz_ff = 0.0

        # P-کنترل
        vx_p = KP_POS * (xd - x0)
        vy_p = KP_POS * (yd - y0)
        vz_p = KP_POS * (zd - z0)

        vx = vx_ff + vx_p
        vy = vy_ff + vy_p
        vz = vz_ff + vz_p

        # سقف سرعت کلی
        vmag = math.hypot(math.hypot(vx, vy), vz)
        if vmag > REPULSION_THRESHOLD:
            s = REPULSION_THRESHOLD / vmag
            vx *= s; vy *= s; vz *= s

        # نیروی دفع از همسایه‌ها
        for pos in self.neighbors.values():
            dx, dy = x0 - pos[0], y0 - pos[1]
            d = math.hypot(dx, dy)
            if 1e-3 < d < REPULSION_THRESHOLD:
                rep = REPULSION_GAIN * (1.0/d - 1.0/REPULSION_THRESHOLD)
                rep = max(rep, 0.0)
                vx += rep * (dx/d)
                vy += rep * (dy/d)

        # خطا و رسیدن
        err = math.sqrt((xd-x0)**2 + (yd-y0)**2 + (zd-z0)**2)
        self.err_pub.publish(Float32(data=err))

        if err < ARRIVAL_THRESHOLD:
            vx = vy = vz = 0.0
            if not self.arrived:
                self.arrival_pub.publish(String(data=str(self.idx)))
                self.arrived = True
                self.get_logger().info(f"Follower {self.idx} arrived")

        # انتشار Setpoint نهایی
        sp = TrajectorySetpoint()
        sp.timestamp = ts
        sp.position = [float(x0), float(y0), float(z0)]
        sp.velocity = [float(vx), float(vy), float(vz)]
        sp.yaw = quaternion_to_yaw(self.leader.q)
        self.traj_pub.publish(sp)

        return
