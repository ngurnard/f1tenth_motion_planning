from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pure_pursuit',
            executable='waypoint_node',
            name='waypoint_node'
        ),
        Node(
            package='pure_pursuit',
            executable='pose_fake_pub_node',
            name='pose_fake_pub_node',
        ),
        # Node(
        #     package='lab6_pkg',
        #     executable='rrt_node',
        #     name='rrt_node'
        # ),
    ])