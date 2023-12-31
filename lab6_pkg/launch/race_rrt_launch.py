from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pure_pursuit',
            executable='waypoint_node',
            name='waypoint_node',
            parameters=[
            {"local_frame": "laser"}
            ]
        ),
        IncludeLaunchDescription
        (
                PythonLaunchDescriptionSource
                ([
                    FindPackageShare("particle_filter"),
                    '/launch',
                    'localize_launch.py'
                ])
        ),
    ])