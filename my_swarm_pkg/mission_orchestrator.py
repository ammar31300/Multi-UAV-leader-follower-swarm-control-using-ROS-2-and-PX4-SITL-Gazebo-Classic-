import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Point, Vector3
import math
import time

# --- ثابت‌های مأموریت ---
TARGET_ALTITUDE = -5.0           # ارتفاع پرواز: ۵ متر (منفی برای NED)
FORMATION = "triangle"           # نوع آرایش: "triangle" یا "square"
FORMATION_SPACING = 5.0          # فاصله بین پهپادها در آرایش
MOVE_DISTANCE_Y = 10.0           # فاصله حرکت در جهت Y
MOVE_DISTANCE_X = 10.0           # فاصله حرکت در جهت X
ROTATION_DEGREES = 90          # زاویه چرخش گروه (به درجه)

class MissionOrchestrator(Node):
    """
    این نود با استفاده از QoS صحیح، ابتدا دستور آرایش را روی زمین صادر می‌کند،
    سپس تیک‌آف کرده و مراحل بعدی مأموریت را با تاخیرهای مناسب اجرا می‌کند.
    """
    def __init__(self):
        super().__init__('mission_orchestrator')
        
        # --- تعریف QoS Profile صحیح ---
        # این پروفایل باید با Subscriber در leader_node مطابقت داشته باشد.
        # TRANSIENT_LOCAL باعث می‌شود آخرین پیام "latch" یا ذخیره شود.
        latching_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        # --- Publishers ---
        self.waypoint_pub = self.create_publisher(Point, '/swarm/waypoint_cmd', 10)
        self.rotation_pub = self.create_publisher(Vector3, '/swarm/rotation_cmd', 10)
        # Publisher for orbital rotation command
        self.orbital_pub = self.create_publisher(Vector3, '/swarm/orbital_cmd', 10)
        
        # --- ناشر اصلی با QoS اصلاح شده ---
        self.formation_pub = self.create_publisher(String, '/swarm/formation_cmd', latching_qos)
        
        self.get_logger().info("Mission Orchestrator با QoS صحیح آماده است.")
        self.get_logger().info("اجرای مأموریت تا چند لحظه دیگر...")
        
        # برای اطمینان از برقراری ارتباط کامل بین ناشر و مشترک، یک تأخیر کوتاه اضافه می‌کنیم
        time.sleep(2) 
        
        self.run_mission_sequence()

    def run_mission_sequence(self):
        """اجرای توالی جدید: ابتدا آرایش، سپس تیک‌آف و حرکت"""

        # --- مرحله ۱: ارسال دستور آرایش (روی زمین) ---
        self.get_logger().info(f"مرحله ۱: ارسال دستور آرایش '{FORMATION}' با فاصله {FORMATION_SPACING} متر.")
        formation_str = f"{FORMATION},{FORMATION_SPACING}"
        self.formation_pub.publish(String(data=formation_str))
        self.get_logger().info("دستور آرایش ارسال شد. ۱۰ ثانیه تأخیر برای پردازش...")
        time.sleep(3)

        # --- مرحله ۲: تیک‌آف به ارتفاع هدف ---
        self.get_logger().info(f"مرحله ۲: دستور تیک‌آف به ارتفاع {abs(TARGET_ALTITUDE)} متر.")
        waypoint = Point(x=0.0, y=0.0, z=TARGET_ALTITUDE)
        self.waypoint_pub.publish(waypoint)
        self.get_logger().info("۱۵ ثانیه تأخیر برای رسیدن به ارتفاع...")
        time.sleep(5)

        # --- مرحله ۳: حرکت در جهت Y ---
        self.get_logger().info(f"مرحله ۳: حرکت به اندازه {MOVE_DISTANCE_Y} متر در جهت Y.")
        waypoint = Point(x=0.0, y=MOVE_DISTANCE_Y, z=TARGET_ALTITUDE)
        self.waypoint_pub.publish(waypoint)
        time.sleep(5)

        # --- مرحله ۴: چرخش مداری گروه ---
        self.get_logger().info(f"مرحله ۴: چرخش مداری گروه با شعاع {FORMATION_SPACING} متر.")
        # x=radius, y=angular_velocity (rad/s), z=duration (0=infinite)
        orbital_cmd = Vector3(x=FORMATION_SPACING, y=math.radians(45.0), z=8.0)  # 45 deg/s for 8 seconds
        self.orbital_pub.publish(orbital_cmd)
        time.sleep(10)  # Wait for orbital rotation to complete

        # --- مرحله ۵: حرکت در جهت X ---
        self.get_logger().info(f"مرحله ۵: حرکت به اندازه {MOVE_DISTANCE_X} متر در جهت X.")
        waypoint = Point(x=MOVE_DISTANCE_X, y=MOVE_DISTANCE_Y, z=TARGET_ALTITUDE)
        self.waypoint_pub.publish(waypoint)
        time.sleep(5)

        # --- مرحله ۶: توقف چرخش مداری ---
        self.get_logger().info("مرحله ۶: توقف چرخش مداری.")
        stop_orbital_cmd = Vector3(x=0.0, y=0.0, z=0.0)  # x=0 means stop
        self.orbital_pub.publish(stop_orbital_cmd)
        time.sleep(2)

        self.get_logger().info("✅ مأموریت با موفقیت به پایان رسید.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    orchestrator_node = MissionOrchestrator()
    # نود به صورت خودکار پس از اتمام مأموریت خاموش می‌شود
    # بنابراین نیازی به spin طولانی نیست.
    # rclpy.spin(orchestrator_node) # این خط دیگر لازم نیست
    orchestrator_node.destroy_node()

if __name__ == '__main__':
    main()
