from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('pure_pursuit_controller')
    # path to the installed waypoints.csv
    wp_file = os.path.join(pkg_share, 'waypoints', 'waypoints.csv')

    return LaunchDescription([
        Node(
            package='pid_velocity_controller',
            executable='pid_velocity_node',
            name='pid_velocity',
            parameters=[{
                'target_speed': 0.5,
                'Kp': 1.0,
                'Ki': 0.0,
                'Kd': 0.1,
            }],
        ),
        Node(
            package='pure_pursuit_controller',
            executable='pure_pursuit_node',
            name='pure_pursuit',
            parameters=[{
                'waypoint_file': wp_file,
                'lookahead_dist': 2.0,
                'wheelbase': 0.5,
            }],
        ),
    ])
