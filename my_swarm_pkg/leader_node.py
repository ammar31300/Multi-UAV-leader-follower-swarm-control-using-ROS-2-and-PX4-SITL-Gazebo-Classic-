
#!/usr/bin/env python3
# swarm_package/leader_controller.py

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
MIN_HORIZ_DELAY = 0.3   # ثانیه
import time
from geometry_msgs.msg import Twist, Point, Vector3
from std_msgs.msg import String
from px4_msgs.msg import VehicleOdometry, OffboardControlMode, TrajectorySetpoint, VehicleCommand
from my_swarm_pkg.constants import *
from std_msgs.msg import String as StringMsg,Empty
MAX_SWARM_SPEED = 1.0 # متر بر ثانیه، یک سرعت ایمن و قابل قبول


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
    elif formation == 'column':
        offsets = []
        for i in range(M):
            dy = (i - (M - 1)/2.0) * spacing
            offsets.append((0.0, dy))

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

    def __init__(self, ns: str = None):
        # اگر ns داده نشده، بعد از init نود مقدار namespace واقعی رو بگیر
        if ns is None:
            # ساخت اولیه نود با نام موقت
            super().__init__('leader_controller')
            ns = self.get_namespace().lstrip('/') or LEADER_NS
            self.get_logger().info(f'Namespace auto-detected: {ns}')
        else:
            # ساخت نود با namespace مشخص‌شده
            super().__init__(f"{ns}_leader_controller", namespace='/' + ns)

        # حالا می‌تونی از ns استفاده کنی
        self.ns = ns
        self.get_logger().info(f"LeaderController node initialized with ns: {ns}")
        
        
        self.manual_active     = False     # آیا تاکنون ورودی دستی گرفته‌ایم؟
        self.manual_pos        = [0.0,0.0,0.0]      # نقطهٔ انتهایی جابجایی دستی
        self.prev_manual_time  = None      # برای محاسبه dt در integration
            # ماموریت چند نقطه‌ای

        self.mission_waypoints = []    # لیست geometry_msgs/Point
        self._mission_idx    = 0       # اندیس نقطه فعلی
        self._mission_active = False

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
        self.prev_error = (0.0, 0.0, 0.0)
        self.prev_time  = None
        # initial rotation angles (roll,pitch,yaw)
        self.rotation_angles = (0.0, 0.0, 0.0)
        self.takeoff_altitude = TAKEOFF_ALT  # Use constant from constants.py
        self.took_off = False
        self._disarm_requested = False
        self._timer_handle = self.create_timer(1.0 / RATE_HZ, self.control_loop)

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
        # سابسکرایب به یک تاپیک ساده برای فرمان ماموریت:
        self.create_subscription(
            String,
            '/swarm/mission_cmd',            # فرمان هدایت خودتان
            self._mission_cmd_callback, 
            10
        )
        self.get_logger().info("Subscribed to /swarm/mission_cmd for waypoint missions")
        # follower status
        for i in range(N_FOLLOW):
            ns = f"px4_{i+2}"
            self.create_subscription(
                VehicleOdometry,
                f"/{ns}/fmu/out/vehicle_odometry",
                lambda msg, idx=i: self._follower_status_cb(idx, msg),
                qos_profile_sensor_data,
            )

        self.create_subscription(
            Empty,
            '/swarm/leader_disarm',
            self.on_disarm_request,
            10
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
        # برای heartbeat
        self.hb_pub = self.create_publisher(
            String, "/swarm/leader_heartbeat", 10
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
        self.manual_active = False

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
        self.followers[idx] = (p[0], p[1], p[2])

    def send_cmd(self, command, p1=0.0, p2=0.0):
        # ایجاد پیام و timestamp
        ts = int(self.get_clock().now().nanoseconds / 1e3)
        c = VehicleCommand()
        c.timestamp        = ts
        c.command = command
        c.param1 = p1
        c.param2 = p2
        c.target_system = int(LEADER_NS.split('_')[1]) + 1
        c.target_component = 1
        # فرستنده هم باید مشخص شود
        c.source_system    = c.target_system
        c.source_component = 1
        c.from_external    = True
        c.confirmation     = 0
        self.cmd_pub.publish(c)
   
    def on_disarm_request(self, msg: Empty):
        self.get_logger().warn("Disarm requested → LAND کردن")
        self.send_cmd(MAV_CMD_NAV_LAND)

        # یک‌بار ۵ ثانیه بعد DISARM نهایی کنیم
        disarm_timer = None
        def _final_cb():
            self.final_disarm()
            # پس از نهایی‌کردن دیس‌آرم، Timer کنترل‌لوپ را لغو کن
            self._timer_handle.cancel()
            # دیگر ضربان قلب نده
            self.get_logger().info("LeaderController stopping control loop and heartbeat")
            # در نهایت، می‌توانید نود را تخریب کنید:
            self.destroy_node()
        # ایجاد تایمر یک‌بار اجرا
        disarm_timer = self.create_timer(5.0, _final_cb)
        self._disarm_requested = True


    def final_disarm(self):
        self.get_logger().warn("Final DISARM")
        # ارسال COMPOENENT_ARM_DISARM با param1=0
        self.send_cmd(MAV_CMD_COMPONENT_ARM_DISARM, 0.0)
    

    def _mission_cmd_callback(self, msg: String):
        """
        msg.data یک رشته است به شکل:
        "x1,y1,z1; x2,y2,z2; x3,y3,z3;"
        یعنی مختصات هر waypoint جدا شده با ';'
        """
        if self._mission_active:
            self.get_logger().warn("Mission already running – ignoring new command")
            return
        
        pts = []
        for part in msg.data.split(';'):
            part = part.strip()
            if not part:
                continue
            try:
                x_s, y_s, z_s = part.split(',')
                p = Point()
                p.x = float(x_s)
                p.y = float(y_s)
                p.z = float(z_s)
                pts.append(p)
            except Exception as e:
                self.get_logger().warn(f"Malformed mission point '{part}': {e}")
        if not pts:
            self.get_logger().warn("Mission Cmd دریافت شد اما لیست نقاط خالی بود")
            return

        # تنظیم وضعیت ماموریت
        self.mission_waypoints = pts
        self._mission_idx    = 0
        self._mission_active = True
        self.get_logger().info(f"Mission loaded: {len(pts)} waypoints")
        # حرکت به نقطه اول
        self._goto_next_mission_waypoint()
        self.manual_active = False

    def _goto_next_mission_waypoint(self):
        """
        از ماموریت فعلی یک نقطه انتخاب می‌کند،
        آن را به self.waypoint می‌دهد و arrived set را
        پاک می‌کند تا کنترل‌لوپ به آن نقطه برود.
        """
        wp = self.mission_waypoints[self._mission_idx]
        self.waypoint = wp
        self.arrived_followers.clear()
        self.get_logger().info(
            f"→ Mission → goto waypoint #{self._mission_idx}: "
            f"({wp.x:.2f},{wp.y:.2f},{wp.z:.2f})"
        )

    
    # --- main control loop
    def control_loop(self):
        if self._disarm_requested:
            return
        
        if not self.pose:
            return
        

        # ۱) چکِ رسیدن لیدر (و اختیاری فالورها) به waypoint فعلی
        if self._mission_active and self.waypoint is not None:
            # فاصله ۳ بعدی
            dx = float(self.pose.position[0]) - self.waypoint.x
            dy = float(self.pose.position[1]) - self.waypoint.y
            dz = float(self.pose.position[2]) - self.waypoint.z
            dist3d = math.sqrt(dx*dx + dy*dy + dz*dz)

            # لاگ دیباگی
            self.get_logger().info(
                f"[Mission-CHECK] idx={self._mission_idx}  dist3d={dist3d:.2f}  "
                f"arrived_foll={len(self.arrived_followers)}/{N_FOLLOW}"
            )

            # صرفاً بر اساس لیدر (یا اگر می‌خواهید فالورها را هم اضافه کنید شرط بعدی را هم بگذارید)
            if dist3d < MISSION_TOLERANCE:
                self.get_logger().info(
                    f"[Mission] reached wp#{self._mission_idx}  → moving on"
                )
                self._mission_idx += 1
                if self._mission_idx < len(self.mission_waypoints):
                    self._goto_next_mission_waypoint()
                else:
                    self._mission_active = False
                    self.get_logger().info("✅ Mission completed.")
        # timestamp و زمان فعلی
        ts = int(self.get_clock().now().nanoseconds / 1e3)
        now = self.get_clock().now().nanoseconds / 1e9

        # Get current yaw for stability
        q = self.pose.q
        yaw_now = quaternion_to_yaw((q[0], q[1], q[2], q[3]))

        # Always send offboard control mode first
        self.mode_pub.publish(OffboardControlMode(timestamp=ts, position=True))
        
        # Check if we need takeoff phase (right after arming)
        if self.armed and not self.took_off:
            current_z = self.pose.position[2]
            z_error = abs(current_z - self.takeoff_altitude)
            
            # Stay at current XY position during takeoff
            sp = TrajectorySetpoint()
            sp.timestamp = ts
            sp.position = [
                float(self.pose.position[0]),  # Hold X position
                float(self.pose.position[1]),  # Hold Y position
                float(self.takeoff_altitude),  # Target takeoff altitude
            ]
            
            # Gradual ascent with controlled velocity
            if current_z > self.takeoff_altitude:  # Need to ascend (NED frame)
                vz = max(MAX_ASCENT_SPEED, -0.5)  # Controlled ascent speed
            else:
                vz = 0.0
            
            sp.velocity = [0.0, 0.0, vz]  # No horizontal movement during takeoff
            sp.yaw = float(yaw_now)
            self.traj_pub.publish(sp)
            
            # Check if takeoff is complete
            if z_error < TAKEOFF_HYSTERESIS:
                self.took_off = True
                self.get_logger().info("Leader takeoff completed, ready for formation control")
            return

        # ۳) اگر هنوز Armed نشده‌ایم، handshake OFFBOARD → ARM
        if not self.armed:
            self.arm_wait_counter += 1
            
            # Send setpoints for stability before switching to offboard
            if self.arm_wait_counter <= PRE_ARM_SETPOINT_COUNT:
                # Send stable setpoints at current position for better transition
                stable_sp = TrajectorySetpoint()
                stable_sp.timestamp = ts
                stable_sp.position = [
                    float(self.pose.position[0]),
                    float(self.pose.position[1]),
                    float(self.takeoff_altitude),  # Target takeoff altitude
                ]
                stable_sp.velocity = [0.0, 0.0, 0.0]
                stable_sp.yaw = float(yaw_now)
                self.traj_pub.publish(stable_sp)
                return
            
            # ۳a) ارسال MAV_CMD_DO_SET_MODE برای ورود به OFFBOARD یکبار
            if self.arm_wait_counter == PRE_ARM_SETPOINT_COUNT + 1:
                self.get_logger().info("Switching leader to OFFBOARD mode")
                self.send_cmd(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    1.0,    # custom mode
                    6.0     # 6 = OFFBOARD
                )
            
            # ۳b) بعد از ARM_DELAY_SECONDS ثانیه، فرمان ARM را بفرست
            if self.arm_wait_counter == int(RATE_HZ * ARM_DELAY_SECONDS):
                self.get_logger().info("Arming leader")
                self.send_cmd(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    1.0,    # arm = 1
                    0.0
                )
            
            # ۳c) بعد از ARM_DELAY_SECONDS + 0.5 ثانیه، وضعیت Armed را تغییر بده
            if self.arm_wait_counter > int(RATE_HZ * (ARM_DELAY_SECONDS + 0.5)):
                self.armed = True
                self.took_off = False
                self.get_logger().info(">> Leader armed and in OFFBOARD - starting takeoff")
            return
        
        # --- ۲) تعیین manual_active ---
        vx_cmd = self.cmd_vel.linear.x if self.cmd_vel else 0.0
        vy_cmd = self.cmd_vel.linear.y if self.cmd_vel else 0.0
        vz_cmd = self.cmd_vel.linear.z if self.cmd_vel else 0.0
        manual_active = False
    # ===== ادغام سرعت دستی =====
    # فقط وقتی سرعت واقعی (بزرگتر از آستانه) باشد
        if abs(vx_cmd) > 1e-3 or abs(vy_cmd) > 1e-3 or abs(vz_cmd) > 1e-3:
            # بار اول ورود به حالت دستی
            if not self.manual_active:
                self.manual_active = True
                # مقداردهی manual_pos از آخرین waypoint یا موقعیت فعلی
                if self.waypoint:
                    self.manual_pos = [self.waypoint.x,
                                    self.waypoint.y,
                                    self.waypoint.z]
                elif self.pose:
                    px, py, pz = self.pose.position
                    self.manual_pos = [px, py, pz]
                # تنظیم زمان اولیه انتگرال‌گیری
                self.prev_int_time = now

            # محاسبه Δt
            dt = now - (self.prev_int_time or now)
            # محافظت در برابر مقادیر نامناسب
            if dt <= 0.0 or dt > 1.0:
                dt = 1.0 / RATE_HZ
            self.prev_int_time = now

            # انتگرال ساده: x += v * dt
            self.manual_pos[0] += vx_cmd * dt
            self.manual_pos[1] += vy_cmd * dt
            self.manual_pos[2] += vz_cmd * dt

        # اگر قبلاً وارد manual شده‌ایم، در همین شاخه بمانیم
        manual_active = self.manual_active
        if self.cmd_vel:
            vx_cmd = self.cmd_vel.linear.x
            vy_cmd = self.cmd_vel.linear.y
            vz_cmd = self.cmd_vel.linear.z
            if abs(vx_cmd) > 1e-3 or abs(vy_cmd) > 1e-3 or abs(vz_cmd) > 1e-3:
                manual_active = True

        # --- ۳) محاسبه موقعیت‌های Formation (برای فالورها) ---
        cx = self.pose.position[0]
        cy = self.pose.position[1]
        cz = self.pose.position[2]
        M  = N_FOLLOW + 1

        # اگر فرميشن ستون است → چینش در ارتفاع (Z)
        if self.formation == 'column':
            mid = (M - 1) / 2.0
            pts = []
            for i in range(M):
                dz = (i - mid) * self.spacing
                pts.append((cx, cy, cz + dz))
        else:
            # حالت‌های line/square/triangle و چرخش در ۳D
            pts2d = compute_formation_positions(cx, cy, self.formation, self.spacing, M)
            rx, ry, rz = self.rotation_angles
            sx, cx1 = math.sin(rx), math.cos(rx)
            sy, cy1 = math.sin(ry), math.cos(ry)
            sz, cz1 = math.sin(rz), math.cos(rz)
            pts = []
            for (xi, yi) in pts2d:
                dx0, dy0, dz0 = xi - cx, yi - cy, 0.0
                # Rx
                x1 = dx0
                y1 = cx1*dy0 - sx*dz0
                z1 = sx*dy0 + cx1*dz0
                # Ry
                x2 = cy1*x1 + sy*z1
                y2 = y1
                z2 = -sy*x1 + cy1*z1
                # Rz
                x3 = cz1*x2 - sz*y2
                y3 = sz*x2 + cz1*y2
                z3 = z2
                pts.append((cx + x3, cy + y3, cz + z3))

        # تعیین زمان رسیدن همه (travel_time)
        dists = [math.hypot(px-cx, py-cy) for (px,py,_) in pts]
        # اگر manual_active، اجازه بده فالورها سریع از پس حرکت بربیایند:
        if manual_active:
            travel_time = 1.0    # زمان پاسخ سریع برای کنترل دستی
        else:
            # از سرعت حداکثری که در بالا تعریف کردیم استفاده می‌کنیم
            desired_speed = MAX_SWARM_SPEED

            maxd = max(dists) if dists else 0.0
            
            # اطمینان از یک حداقل زمان سفر برای جلوگیری از سرعت‌های لحظه‌ای بالا
            MIN_TRAVEL_TIME = 1.0 
            travel_time = (maxd / desired_speed) if desired_speed > 1e-6 else 0.0
            travel_time = max(travel_time, MIN_TRAVEL_TIME) # حرکت حداقل نیم ثانیه طول بکشد
            
        t_arrival = now + travel_time

        # ساخت و پابلیش follower_targets
        parts = []
        for i in range(1, M):
            xi, yi, zi = pts[i]
            parts.append(f"{i-1},{xi:.2f},{yi:.2f},{zi:.2f},{t_arrival:.3f}")
        self.target_pub.publish(String(data=";".join(parts)))

        # --- ۴) ساخت TrajectorySetpoint برای خودِ لیدر ---
        sp = TrajectorySetpoint()
        sp.timestamp = ts
        sp.position = [float(cx), float(cy), float(cz)]

        if manual_active:
            sp.velocity = [float(vx_cmd), float(vy_cmd), float(vz_cmd)]
            sp.yaw      = float(quaternion_to_yaw(self.pose.q))
            self.traj_pub.publish(sp)

        else:
            # 2) FORMATION computation
            #   M = N_FOLLOW + 1 (leader + followers)
            M = N_FOLLOW + 1
            # center is either waypoint or current pose
            cx = self.pose.position[0]
            cy = self.pose.position[1]
            cz = self.pose.position[2]
            # --- محاسبه ستون یا حالات ۲D مثل قبلی ---
            if self.formation == 'column':
                # ستون عمودی
                mid = (M - 1) / 2.0
                pts = []
                for i in range(M):
                    dz = (i - mid) * self.spacing
                    pts.append([cx, cy, cz + dz])
            else:
                # line/square/triangle + چرخش ۳D
                pts2d = compute_formation_positions(cx, cy, self.formation, self.spacing, M)
                rx, ry, rz = self.rotation_angles
                sx, cx1 = math.sin(rx), math.cos(rx)
                sy, cy1 = math.sin(ry), math.cos(ry)
                sz, cz1 = math.sin(rz), math.cos(rz)
                pts = []
                for (xi, yi) in pts2d:
                    dx0, dy0, dz0 = xi - cx, yi - cy, 0.0
                    # Rx
                    x1 = dx0
                    y1 = cx1*dy0 - sx*dz0
                    z1 = sx*dy0 + cx1*dz0
                    # Ry
                    x2 = cy1*x1 + sy*z1
                    y2 = y1
                    z2 = -sy*x1 + cy1*z1
                    # Rz
                    x3 = cz1*x2 - sz*y2
                    y3 = sz*x2 + cz1*y2
                    z3 = z2
                    pts.append([cx + x3, cy + y3, cz + z3])

            # 3) compute distances from current positions → each target
            dists = []
            # i=0 is leader
            xL, yL = self.pose.position[0], self.pose.position[1]
            for i, (xt, yt, zt) in enumerate(pts):
                if i == 0:
                    d = math.hypot(xt - xL, yt - yL)
                else:
                    pos = self.followers.get(i-1, (xL, yL))
                    fx, fy = pos[0], pos[1]    
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

            # 6) Leader moves in sync with PD‑control to reduce oscillations
            rem = t_arrival - now

            # موقعیت فعلی لیدر
            xL, yL, zL = self.pose.position

            # هدف لیدر (pts[0])
            xt_L, yt_L, zt_L = pts[0]

            # Feed‑forward base velocity
            if rem > MIN_HORIZ_DELAY:
                vff_x = (xt_L - xL) / rem
                vff_y = (yt_L - yL) / rem
                vff_z = (zt_L - zL) / rem
            else:
                vff_x = vff_y = vff_z = 0.0

            # P‑term: خطای موقعیت
            err_x = xt_L - xL
            err_y = yt_L - yL
            err_z = zt_L - zL

            # D‑term: مشتق خطا
            if self.prev_time is None:
                # بار اول مشتق نداریم
                derr_x = derr_y = derr_z = 0.0
                dt = rem
            else:
                dt = now - self.prev_time
                dt = max(dt, 1e-6)
                prev_ex, prev_ey, prev_ez = self.prev_error
                derr_x = (err_x - prev_ex) / dt
                derr_y = (err_y - prev_ey) / dt
                derr_z = (err_z - prev_ez) / dt

            # ترکیب PD + FF
            # ترکیب PD + FF
            vx = vff_x + KP_POS * err_x + KD_POS * derr_x
            vy = vff_y + KP_POS * err_y + KD_POS * derr_y
            vz = vff_z + KP_POS * err_z + KD_POS * derr_z
            
            # محدود کردن سرعت افقی لیدر
            h_speed = math.sqrt(vx**2 + vy**2)
            if h_speed > MAX_LEADER_SPEED:
                scale = MAX_LEADER_SPEED / h_speed
                vx *= scale
                vy *= scale

            # محدود کردن سرعت عمودی با مقادیر نامتقارن
            if vz < 0:  # اگر در حال صعود هستیم (vz منفی است)
                vz = max(vz, MAX_ASCENT_SPEED)  # استفاده از سقف سرعت صعود
            else:  # اگر در حال فرود هستیم (vz مثبت است)
                vz = min(vz, MAX_DESCENT_SPEED) # استفاده از سقف سرعت فرود

            # ذخیره خطا و زمان برای دوره بعد
            self.prev_error = (err_x, err_y, err_z)
            self.prev_time = now

            # ساخت TrajectorySetpoint برای لیدر
            sp = TrajectorySetpoint()
            # موقعیت فعلی یا هدف
            sp.position = [float(xL), float(yL), float(zL)]
            sp.velocity = [float(vx), float(vy), float(vz)]
            sp.yaw = quaternion_to_yaw(self.pose.q) + self.rotation_angles[2]
            self.traj_pub.publish(sp)
            # ۶) ضربان → هر follower می‌فهمد لیدر هنوز زنده است
            now = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec*1e-9
            self.hb_pub.publish(String(data=f"{now:.3f}"))

                # --- پس از منطق اصلی کنترل لوپ:
        

        
    