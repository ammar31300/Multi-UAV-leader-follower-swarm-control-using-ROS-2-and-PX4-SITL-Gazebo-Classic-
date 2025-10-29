# swarm_package/constants.py
import math
from rclpy.qos import QoSProfile, DurabilityPolicy

MAX_SWARM_SPEED = 2.7 # متر بر ثانیه: حداکثر سرعت حرکت گروه. می‌توانید این مقدار را کم یا زیاد کنید.
MAX_LEADER_SPEED = 2
RATE_HZ = 20.0

# === ALTITUDE SETTINGS (ALL IN NED FRAME - NEGATIVE VALUES ARE UP) ===
Z_CMD = -2.0                # Default target altitude for formation flight (2m above ground)
TARGET_ALTITUDE = -2.0      # Mission target altitude (same as Z_CMD for consistency)
TAKEOFF_ALT = -2.0          # Takeoff target altitude (2m above ground)
MIN_SAFE_ALTITUDE = -0.5    # Minimum safe altitude (0.5m above ground)
MAX_SAFE_ALTITUDE = -10.0   # Maximum safe altitude (10m above ground)
EMERGENCY_LAND_ALT = -0.3   # Emergency landing altitude (0.3m above ground)

LEADER_NS = 'px4_1'
OFFSETS = [(0.0, 2.0), (0.0, 3.0), (0.0, 4.0)]
N_FOLLOW = len(OFFSETS)
KP_POS = 0.6
KD_POS = 0.1
ARRIVAL_THRESHOLD = 0.2
REPULSION_GAIN = 1.0
REPULSION_THRESHOLD = 1.5
formation_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
MIN_HORIZ_DELAY = 0.3   # ثانیه

# === TAKEOFF AND SAFETY SETTINGS ===
TAKEOFF_HYSTERESIS = 0.05  # برای جلوگیری از نوسان روی مرز
SAFE_ALT = -1.5            # Safe altitude for formation operations (1.5m above ground)
TAKEOFF_THRESHOLD = -1.8   # Threshold for considering takeoff complete (1.8m above ground)
# === MAVLINK COMMANDS ===
MAV_CMD_NAV_LAND = 21
MAV_CMD_COMPONENT_ARM_DISARM = 400

# === COMMUNICATION AND TIMEOUT SETTINGS ===
LEADER_TIMEOUT = 3.0       # حداکثر زمان مجاز نبودن heartbeat رهبر (بر حسب ثانیه)
CHECK_PERIOD = 0.5         # [s] دوره‌ی چک کردن
HEARTBEAT_TIMEOUT = 2.0
MISSION_TOLERANCE = 3      # متر

# === VELOCITY LIMITS (NED FRAME - NEGATIVE IS UP) ===
MAX_ASCENT_SPEED = -1.5    # Maximum ascent speed (1.5 m/s upward) - NEGATIVE value
MAX_DESCENT_SPEED = 1.0    # Maximum descent speed (1.0 m/s downward) - POSITIVE value
MAX_VZ = 1.0              # Maximum vertical velocity for followers
MIN_VZ = -1.5             # Minimum vertical velocity for followers (ascent)

# === FORMATION AND MISSION SETTINGS ===
FORMATION = "triangle"
FORMATION_SPACING = 4.0
MOVE_X_DIST = 10.0
MOVE_Y_DIST = 10.0
ROTATION_DEG = 360

# === SAFETY THRESHOLDS ===
LOW_BATTERY_THRESHOLD = 20.0      # Percentage
CRITICAL_BATTERY_THRESHOLD = 10.0 # Percentage
MAX_WIND_SPEED = 5.0              # m/s
GEOFENCE_RADIUS = 100.0           # meters from home position

# === ALTITUDE SAFETY LIMITS ===
ALTITUDE_SAFETY_MARGIN = 0.2      # meters - safety margin from limits
GROUND_CLEARANCE_MIN = 0.3        # meters - minimum clearance from ground
CEILING_LIMIT = -15.0             # meters - maximum altitude limit (15m above ground)

# === EMERGENCY PROCEDURES ===
EMERGENCY_DESCENT_RATE = 0.5      # m/s - controlled emergency descent
AUTO_LAND_TIMEOUT = 30.0          # seconds - auto land if no commands
FAILSAFE_ALTITUDE = -0.5          # meters - failsafe landing altitude

# === SAFETY CHECK FUNCTIONS ===
def is_altitude_safe(altitude):
    """Check if the given altitude is within safe limits"""
    return MAX_SAFE_ALTITUDE <= altitude <= MIN_SAFE_ALTITUDE

def clamp_altitude(altitude):
    """Clamp altitude to safe limits"""
    if altitude > MIN_SAFE_ALTITUDE:
        return MIN_SAFE_ALTITUDE
    elif altitude < MAX_SAFE_ALTITUDE:
        return MAX_SAFE_ALTITUDE
    return altitude

def is_emergency_landing_needed(altitude, battery_level=100.0):
    """Check if emergency landing is needed based on altitude or battery"""
    altitude_critical = altitude > (MIN_SAFE_ALTITUDE + ALTITUDE_SAFETY_MARGIN)
    battery_critical = battery_level < CRITICAL_BATTERY_THRESHOLD
    return altitude_critical or battery_critical