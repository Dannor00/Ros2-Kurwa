import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch-parametre for PID
    kp_arg = DeclareLaunchArgument('kp', default_value='1.0')
    ki_arg = DeclareLaunchArgument('ki', default_value='0.0')
    kd_arg = DeclareLaunchArgument('kd', default_value='0.0')

    kp = LaunchConfiguration('kp')
    ki = LaunchConfiguration('ki')
    kd = LaunchConfiguration('kd')

    # Last inn YAML-konfigurasjonsfilen
    config = os.path.join(
        get_package_share_directory('pid_controller'),
        'config',
        'parameters.yaml'
    )

    return LaunchDescription([
        kp_arg,
        ki_arg,
        kd_arg,

        Node(
            package='pid_controller',
            executable='pid_controller_node',
            name='pid_controller_node',
            output='screen',
            parameters=[{
                'kp': kp,
                'ki': ki,
                'kd': kd,
            }]
        ),

        # ❌ reference_input_node er utelatt for manuell oppstart

        Node(
            package='joint_simulator',
            executable='joint_simulator_node',
            name='joint_simulator_node',
            output='screen',
            parameters=[config]
        ),
    ])

