# swarm_package/constants.py
import math
from rclpy.qos import QoSProfile, DurabilityPolicy

RATE_HZ = 20.0
Z_CMD = -1.0
LEADER_NS = 'px4_1'
OFFSETS = [(0.0, 3.0), (0.0, -3.0), (-3.0, 0.0)]
N_FOLLOW = len(OFFSETS)
KP_POS = 0.6
ARRIVAL_THRESHOLD = 0.2
REPULSION_GAIN = 1.0
REPULSION_THRESHOLD = 1.5
formation_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
