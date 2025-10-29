# my_swarm_pkg/mission_orchestrator.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Vector3
import math
import time

# Import constants including the QoS profile
from my_swarm_pkg.constants import (
    TARGET_ALTITUDE,
    FORMATION,
    FORMATION_SPACING,
    MOVE_X_DIST,
    MOVE_Y_DIST,
    ROTATION_DEG,
    formation_qos
)

class FinalMissionOrchestrator(Node):
    """
    Orchestrates the final mission scenario with the user-specified sequence:
    Formation -> Takeoff -> Move Y -> Rotate -> Move X -> Land.
    """
    def __init__(self):
        super().__init__('final_mission_orchestrator')

        # Publishers
        self.waypoint_pub = self.create_publisher(Point, '/swarm/waypoint_cmd', 10)
        self.formation_pub = self.create_publisher(String, '/swarm/formation_cmd', qos_profile=formation_qos)
        self.rotation_pub = self.create_publisher(Vector3, '/swarm/rotation_cmd', 10)

        # Keep track of the current commanded target position for the leader
        self.current_target_pos = Point(x=0.0, y=0.0, z=0.0)

        self.get_logger().info("Final Mission Orchestrator ready. Starting mission in 5 seconds...")
        self.create_timer(5.0, self.run_final_mission)

    def run_final_mission(self, timer=None):
        if timer: timer.cancel()

        # --- STEP 1: Form up on the ground (spacing 6m) ---
        self.get_logger().info(f"STEP 1: Commanding '{FORMATION}' formation with {FORMATION_SPACING}m spacing (on ground).")
        self.formation_pub.publish(String(data=f"{FORMATION},{FORMATION_SPACING}"))
        self.get_logger().info("   >> Waiting 10s for formation command to be processed.")
        time.sleep(10)

        # --- STEP 2: Takeoff to 5m altitude ---
        self.get_logger().info(f"STEP 2: Commanding takeoff to {abs(TARGET_ALTITUDE)}m altitude.")
        self.current_target_pos.z = TARGET_ALTITUDE
        self.waypoint_pub.publish(self.current_target_pos)
        self.get_logger().info("   >> Waiting 15s to reach altitude and stabilize.")
        time.sleep(15)

        # --- STEP 3: Move 10m in Y direction ---
        self.get_logger().info(f"STEP 3: Commanding {MOVE_Y_DIST}m move along Y-axis.")
        self.current_target_pos.y += MOVE_Y_DIST
        self.waypoint_pub.publish(self.current_target_pos)
        self.get_logger().info("   >> Waiting 20s to complete Y-axis travel and stabilize.")
        time.sleep(20)

        # --- STEP 4: Rotate 90 degrees ---
        self.get_logger().info(f"STEP 4: Commanding {ROTATION_DEG}-degree group rotation.")
        rotation_cmd = Vector3(z=math.radians(ROTATION_DEG))
        self.rotation_pub.publish(rotation_cmd)
        # Increased delay for a "clean" rotation, allowing drones to settle.
        self.get_logger().info("   >> Waiting 15s for a clean rotation and stabilization.")
        time.sleep(15)

        # --- STEP 5: Move 10m in X direction ---
        self.get_logger().info(f"STEP 5: Commanding {MOVE_X_DIST}m move along X-axis.")
        self.current_target_pos.x += MOVE_X_DIST
        self.waypoint_pub.publish(self.current_target_pos)
        self.get_logger().info("   >> Waiting 20s to complete X-axis travel and stabilize.")
        time.sleep(20)

        # --- STEP 6: Land ---
        self.get_logger().info("STEP 6: Commanding land at the current position.")
        land_point = Point(x=self.current_target_pos.x, y=self.current_target_pos.y, z=0.0)
        self.waypoint_pub.publish(land_point)
        self.get_logger().info("   >> Waiting 20s for a complete landing.")
        time.sleep(20)

        self.get_logger().info("✅✅✅ Final mission scenario completed successfully.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    orchestrator_node = FinalMissionOrchestrator()
    try:
        rclpy.spin(orchestrator_node)
    except (KeyboardInterrupt, SystemExit):
        orchestrator_node.destroy_node()

if __name__ == '__main__':
    main()
