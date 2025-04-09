from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_file = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(':')[0],
        'share', 'dip_sim', 'urdf', 'double_pendulum.urdf'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
