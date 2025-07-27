#!/usr/bin/env python3
# swarm_package/follower_controller.py

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
import time
import subprocess
from std_msgs.msg import String

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
MIN_VZ   = -1.0      # سرعت نزولی (همیشه منفی، مثلا -1.0 تا -1.2)
SAFE_ALT = 1.0       # متر هدف
TAKEOFF_THRESHOLD = 1.8    # برگشت به فرمیشن
KP_POS = 1.2         # گین کنترل تناسبی ارتفاع (بسته به سیستم تو)
MIN_HORIZ_DELAY = 0.3  # تاخیر کوچک قبل از حرکت افقی [s]

def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q[0]*q[3] + q[1]*q[2]),
        1.0 - 2.0*(q[2]**2 + q[3]**2),
    )

class FollowerController(Node):
    def __init__(self, idx, offs):
        ns = f"px4_{idx+2}"
        super().__init__(f"{ns}_follower",namespace=ns)

        # keep track of which leaders have been disarmed
        # once a drone is disarmed it will NEVER re-arm
        self.disarmed_leaders = set()
        # quick alias
        self._my_idx = idx

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
        self.takeoff_completed = False

        # همسایه‌ها و Repulsion
        self.neighbors = {}
        self.enable_repulsion = True

        self.last_leader_hb = self.get_clock().now().nanoseconds / 1e9
        self.elected = False           # آیا من خودم رو نامزد کردم؟
        # subscriber ضربان لیدر
        self.create_subscription(
            String, "/swarm/leader_heartbeat", self._hb_cb, 10
        )
        # یک تایمر هر ۰.۵ ثانیه برای چک کردن زنده بودن لیدر
        self.create_timer(0.5, self._check_leader)


        # Subscribe to disarm announcements
        # Whenever ANY leader disarms we will get an Empty
        from std_msgs.msg import Empty
        self.create_subscription(
            Empty,
            '/swarm/leader_disarm',
            self._on_any_leader_disarm,
            10
        )
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
    
    def _hb_cb(self, msg: String):
        # هر بار ضربان رسید، زمان اخیر را ذخیره کن
        try:
            self.last_leader_hb = float(msg.data)
        except:
            self.get_logger().warn("Malformed heartbeat")

    def _on_any_leader_disarm(self, msg):
        """
        When the current leader calls disarm(), they publish
        an Empty on /swarm/leader_disarm.  *All* followers hear it.
        We figure out which leader namespace has just disarmed
        (from LEADER_NS), compute its follower-index, and
        mark that leader permanently disarmed.  It will never
        re-arm, and it will be excluded from future elections.
        """
        # import the same LEADER_NS you used to subscribe to odom:
        from my_swarm_pkg.constants import LEADER_NS
        # LEADER_NS is e.g. 'px4_2' → strip off 'px4_' and subtract 2
        try:
            # follower‐idx = int( LEADER_NS.split('_')[1] ) - 2
            dis_idx = int(LEADER_NS.split('_')[1]) - 2
        except:
            self.get_logger().error("Cannot parse LEADER_NS to idx")
            return

        self.get_logger().warn(
            f">> Detected leader idx={dis_idx} DISARMED! "
            "That drone will never re-arm."
        )
        self.disarmed_leaders.add(dis_idx)

        # If *this* follower itself was the one disarmed,
        # force it into permanent “no-arm” mode:
        if dis_idx == self._my_idx:
            self.get_logger().warn(" I am permanently disarmed.  Standing down.")
            self.armed = False
            # we optionally reset t_arrival/offset so we never re-enter formation
            self.target_received = False
# follower_node

    def _check_leader(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_leader_hb <= 2.0:
            return

        # okay, the leader is dead.  We must elect a new one.

        # build list of eligible follower‐idxs
        all_idxs = list(range(N_FOLLOW))
        elig = [i for i in all_idxs if i not in self.disarmed_leaders]
        if not elig:
            self.get_logger().error("No eligible followers left to elect!")
            return

        # lowest idx wins
        winner = min(elig)
        if self._my_idx != winner:
            # not our turn to spawn
            return
        if self.elected:
            # we already spawned once
            return

        self.get_logger().warn(
            f"Heartbeat dead → I am next eligible idx={self._my_idx}, spawning new leader."
        )

        # build up the ROS2-launch command exactly as before:

        # Namespace فالور را بگیریم؛ اگر remap نشده بود، fallback:
        ns = self.get_namespace().lstrip('/')
        if not ns:
            ns = f"px4_{self.idx+2}"
        # اطمینان از leading slash:
        if not ns.startswith('/'):
            ns = '/' + ns

        # دستور کامل ros2 run … با --ros-args و -r
        cmd = [
            'ros2', 'run', 'my_swarm_pkg', 'main',   # یا اگر entry_point‌تان نام دیگریست آن را بنویسید
            '--ros-args',
            '-r', f'__ns:={ns}'
        ]
        try:
            subprocess.Popen(cmd)
            self.elected = True
            self.get_logger().info(f"Spawned main in namespace '{ns}'")
        except Exception as e:
            self.get_logger().error(f"Fail to spawn main in ns={ns}: {e}")

    def control_loop(self):

        # if *this* drone has previously been disarmed → do absolutely nothing
        if self.idx in self.disarmed_leaders:
            return
        
        # نیاز به دسترسی به لیدر و اُدم
        if not (self.leader and self.odom):
            return
        # --- بررسی زنده بودن لیدر ---
        
        
        has_leader = self.leader is not None and (self.get_clock().now().nanoseconds/1e9 - self.last_leader_hb) < LEADER_TIMEOUT
        if not has_leader:
            ######################################
            # حالت نگه‌داشت موقعیت (position hold)
            # ارسال Offboard تا PX4 لند نکند
            ######################################
            ts = int(self.get_clock().now().nanoseconds / 1e3)
            # ارسال مد موقعیت (position control)
            self.mode_pub.publish(
                OffboardControlMode(
                    timestamp=ts,
                    position=True,
                    velocity=False,
                    acceleration=False,
                    attitude=False,
                    body_rate=False,
                )
            )
            # نگه‌داشتن در همان جای فعلی
            sp = TrajectorySetpoint()
            sp.timestamp = ts
            sp.position = [
                float(self.odom.position[0]),
                float(self.odom.position[1]),
                float(self.odom.position[2]),
            ]
            sp.velocity = [0.0, 0.0, 0.0]
            sp.yaw = quaternion_to_yaw(self.odom.q)
            self.traj_pub.publish(sp)
            # دیگر ادامه ندهیم تا به محض election/leader جدید، از اول بیاییم تو فاز عادی
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
        
    # --- فاز ۱: ARM و OFFBOARD ---
        if not self.armed:
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
            z0 = self.odom.position[2]
            vz_cmd = max(MIN_VZ, -abs(KP_POS * (SAFE_ALT - z0)))
            sp = TrajectorySetpoint()
            sp.timestamp = ts
            sp.position = [0.0, 0.0, 0.0]
            sp.velocity = [0.0, 0.0, float(vz_cmd)]
            sp.yaw = quaternion_to_yaw(self.odom.q)
            self.traj_pub.publish(sp)

            self.arm_wait_counter += 1
            if self.arm_wait_counter > int(RATE_HZ * 1.0):
                self.get_logger().info(f"Arming follower {self.idx}")
                self.send_cmd(400, 1.0)       # ARM
                self.send_cmd(176, 1.0, 6.0)  # OFFBOARD
                self.armed = True
            return

        # --- فاز ۳: دریافت formation target ---
        if not self.target_received:
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
