#!/usr/bin/env python3
"""
Test script for orbital rotation functionality.
This script publishes orbital rotation commands to test the implementation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import math
import time

class OrbitalRotationTester(Node):
    def __init__(self):
        super().__init__('orbital_rotation_tester')
        
        # Publisher for orbital rotation command
        self.orbital_pub = self.create_publisher(Vector3, '/swarm/orbital_cmd', 10)
        
        self.get_logger().info("Orbital Rotation Tester initialized")
        
        # Wait a moment for connection
        time.sleep(2)
        
        # Test orbital rotation
        self.test_orbital_rotation()
    
    def test_orbital_rotation(self):
        """Test orbital rotation with different parameters"""
        
        # Test 1: Start orbital rotation
        self.get_logger().info("Test 1: Starting orbital rotation (radius=3m, 30deg/s, 10s duration)")
        orbital_cmd = Vector3()
        orbital_cmd.x = 3.0  # radius in meters
        orbital_cmd.y = math.radians(30.0)  # angular velocity in rad/s (30 deg/s)
        orbital_cmd.z = 10.0  # duration in seconds
        self.orbital_pub.publish(orbital_cmd)
        
        # Wait for the rotation to complete
        time.sleep(12)
        
        # Test 2: Start another orbital rotation with different parameters
        self.get_logger().info("Test 2: Starting orbital rotation (radius=5m, 45deg/s, 8s duration)")
        orbital_cmd.x = 5.0  # radius in meters
        orbital_cmd.y = math.radians(45.0)  # angular velocity in rad/s (45 deg/s)
        orbital_cmd.z = 8.0  # duration in seconds
        self.orbital_pub.publish(orbital_cmd)
        
        # Wait for the rotation to complete
        time.sleep(10)
        
        # Test 3: Stop orbital rotation manually
        self.get_logger().info("Test 3: Manually stopping orbital rotation")
        orbital_cmd.x = 0.0  # x=0 means stop
        orbital_cmd.y = 0.0
        orbital_cmd.z = 0.0
        self.orbital_pub.publish(orbital_cmd)
        
        self.get_logger().info("✅ All orbital rotation tests completed")

def main(args=None):
    rclpy.init(args=args)
    
    tester = OrbitalRotationTester()
    
    # Keep the node alive
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
