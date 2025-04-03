from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('qube_description'),
        'urdf',
        'qube.urdf.xacro'
    )

    robot_description = Command(['xacro ', urdf_file])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[],
            output='screen'
        )
    ])
