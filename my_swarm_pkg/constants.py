# swarm_package/constants.py
import math
from rclpy.qos import QoSProfile, DurabilityPolicy

MAX_SWARM_SPEED = 1.7 # متر بر ثانیه: حداکثر سرعت حرکت گروه. می‌توانید این مقدار را کم یا زیاد کنید.
MAX_LEADER_SPEED = 2
RATE_HZ = 20.0
Z_CMD = -1.0
LEADER_NS = 'px4_1'
OFFSETS = [(0.0, 2.0), (0.0, 3.0), (0.0, 4.0)]
N_FOLLOW = len(OFFSETS)
KP_POS = 0.09
KD_POS = 0.03
ARRIVAL_THRESHOLD = 0.2
REPULSION_GAIN = 1.0
REPULSION_THRESHOLD = 1.5
formation_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
MIN_HORIZ_DELAY = 0.3   # ثانیه
# در انتهای constants.py
TAKEOFF_ALT = 2.0          # ارتفاع هدف برای Hold
TAKEOFF_HYSTERESIS = 0.05  # برای جلوگیری از نوسان روی مرز
MAV_CMD_NAV_LAND = 21
MAV_CMD_COMPONENT_ARM_DISARM = 400
# حداکثر زمان مجاز نبودن heartbeat رهبر (بر حسب ثانیه)
LEADER_TIMEOUT = 3.0
CHECK_PERIOD      = 0.5   # [s] دوره‌ی چک کردن
HEARTBEAT_TIMEOUT = 2.0
MISSION_TOLERANCE = 3  # متر
MAX_ASCENT_SPEED = -3.0  # متر بر ثانیه (مقدار منفی)

# سرعت فرود (باید مثبت و کوچک باشد تا فرود نرم باشد)
MAX_DESCENT_SPEED = 0.8 

TARGET_ALTITUDE = -3.0
FORMATION = "triangle"
FORMATION_SPACING = 4.0
MOVE_X_DIST = 10.0
MOVE_Y_DIST = 10.0
ROTATION_DEG = 360