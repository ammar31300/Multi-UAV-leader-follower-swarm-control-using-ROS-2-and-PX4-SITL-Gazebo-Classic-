import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    ld = LaunchDescription()

    # 1) Spawn Gazebo SITL
    ld.add_action(
        ExecuteProcess(
            cmd=[
                'gnome-terminal', '--', 'bash', '-c',
                "~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m iris -n 4; exec bash"
            ],
            output='screen'
        )
    )

    # 2) Delay 0.5s then start Micro XRCE Agent
    ld.add_action(
        TimerAction(
            period=7.0,
            actions=[ExecuteProcess(
                cmd=[
                    'gnome-terminal', '--', 'bash', '-c',
                    "MicroXRCEAgent udp4 -p 8888; exec bash"
                ],
                output='screen'
            )]
        )
    )

    # 3) Delay another 0.5s then launch all px4_ros_com in one terminal
    ld.add_action(
        TimerAction(
            period=10.0,
            actions=[ExecuteProcess(
                cmd=[
                    'gnome-terminal', '--', 'bash', '-c',
                    "source ~/swarm_ws/install/setup.bash; \
                    for i in 0 1 2 3; do \
                      ros2 run px4_ros_com offboard_control --ros-args \
                        -p transport:=udp -p address:=127.0.0.1 -p port:=8888 \
                        -p instance:=${i} -p namespace:=px4_${i} & \
                    done; wait; exec bash"
                ],
                output='screen'
            )]
        )
    )

    # 4) Delay until everything up, then launch leader_follower node
    ld.add_action(
        TimerAction(
            period=14.0,
            actions=[ExecuteProcess(
                cmd=[
                    'gnome-terminal', '--', 'bash', '-c',
                    "source ~/swarm_ws/install/setup.bash; \
                     ros2 run my_swarm_pkg main; exec bash"
                ],
                output='screen'
            )]
        )
    )
    ld.add_action(
        TimerAction(
            period=17.0,
            actions=[ExecuteProcess(
                cmd=[
                    'gnome-terminal', '--', 'bash', '-c',
                    "source ~/swarm_ws/install/setup.bash; \
                     ros2 topic pub /swarm/waypoint_cmd geometry_msgs/Point '{ x: 0.0, y: 0.0, z: -2.0 }'"
                ],
                output='screen'
            )]
        )
    )
    return ld
