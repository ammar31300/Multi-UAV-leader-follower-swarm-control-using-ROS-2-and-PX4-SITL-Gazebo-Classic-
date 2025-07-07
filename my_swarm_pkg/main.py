# swarm_package/main.py
#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
from my_swarm_pkg.leader_node import LeaderController
from my_swarm_pkg.follower_node import FollowerController
from my_swarm_pkg.constants import OFFSETS

def main(args=None):
    rclpy.init(args=args)
    exe = MultiThreadedExecutor()
    exe.add_node(LeaderController())
    for i, offs in enumerate(OFFSETS):
        exe.add_node(FollowerController(i, offs))
    try:
        exe.spin()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()