from setuptools import setup

package_name = 'my_swarm_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/swarm_launch.py']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'px4_msgs',
        'px4_ros_com'

    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Leader-Follower UAV swarm simulation using PX4, Gazebo Classic, and ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'leader_node = my_swarm_pkg.leader_node:main',
            'follower_node = my_swarm_pkg.follower_node:main',
            'leader_follower = my_swarm_pkg.leader_follower:main',
            'main = my_swarm_pkg.main:main',
            'constants = my_swarm_pkg.constants:main',
            'leader_manual = my_swarm_pkg.leader_manual:main',

        ],
    },
)

