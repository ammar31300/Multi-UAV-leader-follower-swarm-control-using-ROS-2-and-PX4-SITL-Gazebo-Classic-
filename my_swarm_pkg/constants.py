# swarm_package/constants.py
import math
from rclpy.qos import QoSProfile, DurabilityPolicy

MAX_SWARM_SPEED = 1.0 
RATE_HZ = 20.0
Z_CMD = -1.0
LEADER_NS = 'px4_1'
OFFSETS = [(0.0, 2.0), (0.0, 3.0), (0.0, 4.0)]
N_FOLLOW = len(OFFSETS)
KP_POS = 0.006
ARRIVAL_THRESHOLD = 0.2
REPULSION_GAIN = 1.0
REPULSION_THRESHOLD = 1.5
formation_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
MIN_HORIZ_DELAY = 0.3   # ثانیه
# در انتهای constants.py
TAKEOFF_ALT = 2.0          # ارتفاع هدف برای Hold
TAKEOFF_HYSTERESIS = 0.05  # برای جلوگیری از نوسان روی مرز
KD_POS = 0.02            # <— ضریب مشتق برای PD‑کنترل
MAV_CMD_NAV_LAND = 21
MAV_CMD_COMPONENT_ARM_DISARM = 400
# حداکثر زمان مجاز نبودن heartbeat رهبر (بر حسب ثانیه)
LEADER_TIMEOUT = 2.0
CHECK_PERIOD      = 0.5   # [s] دوره‌ی چک کردن
HEARTBEAT_TIMEOUT = 2.0
MISSION_TOLERANCE = 3  # متر
