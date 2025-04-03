from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    pkg_bringup = get_package_share_directory('qube_bringup')
    urdf_path = os.path.join(pkg_bringup, 'urdf', 'controlled_qube.urdf.xacro')

    # Launch arguments
    baud_rate_arg = DeclareLaunchArgument('baud_rate', default_value='115200')
    device_arg = DeclareLaunchArgument('device', default_value='/dev/ttyACM0')
    simulation_arg = DeclareLaunchArgument('simulation', default_value='false')

    robot_description = Command(['xacro ', urdf_path,
                                  ' baud_rate:=', LaunchConfiguration('baud_rate'),
                                  ' device:=', LaunchConfiguration('device'),
                                  ' simulation:=', LaunchConfiguration('simulation')])

    return LaunchDescription([
        baud_rate_arg,
        device_arg,
        simulation_arg,

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['velocity_controller', '--controller-manager', '/controller_manager'],
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[],
            output='screen',
            condition=None
        ),
    ])
