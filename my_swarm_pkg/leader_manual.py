#!/usr/bin/env python3
# leader_manual.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String,Empty
from rclpy.qos import QoSProfile, DurabilityPolicy
from my_swarm_pkg.constants import *
# اگر می‌خواهید با pygame کلیدها را بخوانید:
import pygame
from std_msgs.msg import String as StringMsg
import time
from my_swarm_pkg.constants import *
import subprocess

class LeaderManual(Node):
    def __init__(self):
        super().__init__('leader_manual')
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/swarm/leader_cmd', 10)
        self.form_pub = self.create_publisher(String, '/swarm/formation_cmd', formation_qos)
        self.rot_pub = self.create_publisher(Vector3, '/swarm/rotation_cmd', 10)
                # — publisher برای Disarm کردن لیدر فعلی —
        self.disarm_pub = self.create_publisher(Empty,   '/swarm/leader_disarm', 10)
        self.spacing = 4.0  # مقدار دلخواه (مثلا ۲ متر)
        
                # برای تایم‌بندی ضربان
        self.last_leader_hb = time.time()
        self.elected = False
        self.last_hb = time.time()

        # برای دریافت زمان اجرای فرميشن
        self.last_arrival = None
        self.create_subscription(
            StringMsg,
            '/swarm/follower_targets',
            self._targets_cb,
            formation_qos
        )
        
                # Subscriber ضربان لیدر
        self.create_subscription(
            String,
            '/swarm/leader_heartbeat',
            self._hb_cb,
            10
        )

        # تنظیم pygame برای خواندن صفحه‌کلید
        pygame.init()
        self.screen = pygame.display.set_mode((200,200))
        pygame.display.set_caption("Leader Manual Control")

        # مقادیر جاری
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0              # سرعت عمودی
        self.roll_cmd = 0.0
        self.pitch_cmd = 0.0

        # timer
        self.create_timer(0.05, self.timer_callback)
        self.create_timer(0.5, self._check_leader)
    def _targets_cb(self, msg: StringMsg):
        # msg.data مثل: "0,1.23,4.56,2.00,1633308.123;1, ... "
        parts = msg.data.split(';')
        if not parts:
            return
        # استخراج t_arrival از اولین follower
        try:
            _, _, _, _, t_s = parts[0].split(',')
            self.last_arrival = float(t_s)
            now = time.time()
            eta = self.last_arrival - now
            self.get_logger().info(f"Formation ETA ≃ {eta:.2f} s")
        except Exception as e:
            self.get_logger().warn(f"cannot parse arrival_time: {e}")
    
    def _hb_cb(self, msg: String):
        """هر بار ضربان رسید، زمان ذخیره شود"""
        try:
            self.last_leader_hb = float(msg.data)
        except:
            self.get_logger().warn("Malformed heartbeat")
    
    
    def _check_leader(self):
        # اگر بیش از timeout از آخرین ضربان گذشت → spawn مجدد LeaderController
        if time.time() - self.last_hb > HEARTBEAT_TIMEOUT and not self.elected:
            self.get_logger().warn("Leader heartbeat timeout → spawning new LeaderController")
            # پرچم رو بردار تا spawn فقط یک‌بار بشه
            self.elected = True
            # namespace ای که follower#0 درش قرار داره
            ns = '/px4_2'
            try:
                subprocess.Popen([
                    'ros2', 'run', 'my_swarm_pkg', 'leader_node',
                    '--ros-args', '-r', f'__ns:={ns}'
                ])
                self.get_logger().info(f"Spawned LeaderController in '{ns}'")
            except Exception as e:
                self.get_logger().error(f"Failed to spawn: {e}")


    def timer_callback(self):
        # دریافت رویدادهای pygame
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rclpy.shutdown()
            elif event.type == pygame.KEYDOWN:
                # ← کلید X برای disarm لیدر فعلی
                if event.key == pygame.K_x:
                    self.get_logger().warn("Manual: sending DISARM to LeaderController")
                    self.disarm_pub.publish(Empty())
                    # آماده spawn مجدد
                    self.elected = False
                    continue
                if event.key == pygame.K_UP:
                    self.vx = 1.0
                elif event.key == pygame.K_DOWN:
                    self.vx = -1.0
                elif event.key == pygame.K_LEFT:
                    self.vy = 1.0
                elif event.key == pygame.K_RIGHT:
                    self.vy = -1.0
                # حرکت عمودی: PageUp/PageDown یا R/F
                elif event.key == pygame.K_PAGEUP or event.key == pygame.K_r:
                    self.vz = 1.0
                elif event.key == pygame.K_PAGEDOWN or event.key == pygame.K_f:
                    self.vz = -1.0
                elif event.key == pygame.K_EQUALS or event.key == pygame.K_KP_PLUS:  # + key
                    self.spacing += 0.5
                    print(f"Spacing increased: {self.spacing}")
                elif event.key == pygame.K_MINUS or event.key == pygame.K_KP_MINUS:  # - key
                    self.spacing = max(0.5, self.spacing - 0.5)
                    print(f"Spacing decreased: {self.spacing}")
                
                               # ===== v: فرميشن ردیفی =====
                if event.key == pygame.K_v:
                    self.formation = 'line'
                    data = f"{self.formation},{self.spacing}"
                    self.form_pub.publish(String(data=data))
                    self.get_logger().info(f"[Man] set formation → {data}")

                # ===== b: فرميشن ستونی =====
                # (جدید) ستونی در ارتفاع → کلید b
                elif event.key == pygame.K_b:
                    data = f"column,{self.spacing}"
                    self.form_pub.publish(String(data=data))
                    self.get_logger().info(f"[Man] set formation → {data}")

                # تغییر فرمیشن
                elif event.key == pygame.K_l:
                    data = f"line,{self.spacing}"
                    self.form_pub.publish(String(data=data))
                    self.get_logger().info(f"[Man] published form_cmd → {data}")

                elif event.key == pygame.K_s:
                    data = f"square,{self.spacing}"
                    self.form_pub.publish(String(data=data))
                    self.get_logger().info(f"[Man] published form_cmd → {data}")

                elif event.key == pygame.K_t:
                    data = f"triangle,{self.spacing}"
                    self.form_pub.publish(String(data=data))
                    self.get_logger().info(f"[Man] published form_cmd → {data}")

                # چرخش (yaw) یا roll/pitch
                elif event.key == pygame.K_w:  # چرخش مثبت
                    self.rot_pub.publish(Vector3(x=0.0, y=0.0, z=0.2))
                elif event.key == pygame.K_a:  # roll چپ
                    self.rot_pub.publish(Vector3(x=0.2, y=0.0, z=0.0))
                elif event.key == pygame.K_d:  # roll راست
                    self.rot_pub.publish(Vector3(x=-0.2, y=0.0, z=0.0))

            elif event.type == pygame.KEYUP:
                # وقتی کلید رها شد سرعت را صفر می‌کنیم
                if event.key in (pygame.K_UP, pygame.K_DOWN):
                    self.vx = 0.0
                if event.key in (pygame.K_LEFT, pygame.K_RIGHT):
                    self.vy = 0.0
                if event.key in (pygame.K_PAGEUP, pygame.K_PAGEDOWN,
                                 pygame.K_r, pygame.K_f):
                    self.vz = 0.0
        # پابلیش Twist کنونی
        twist = Twist()
        twist.linear.x = self.vx
        twist.linear.y = self.vy
        twist.linear.z = self.vz
        twist.angular.z = 0.0  # چون yawRate را در rotation_cmd جدا می‌فرستیم
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderManual()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
