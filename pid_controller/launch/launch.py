import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # Hent launch-parametere
    kp = LaunchConfiguration('kp').perform(context)
    ki = LaunchConfiguration('ki').perform(context)
    kd = LaunchConfiguration('kd').perform(context)

    # Konfigurasjonsfil for simulator
    config_path = os.path.join(
        get_package_share_directory('pid_controller'),
        'config',
        'parameters.yaml'
    )

    # 🔧 PID-kontroller kommando
    pid_cmd_str = (
        f'source ~/ros2_ws/install/setup.bash && '
        f'ros2 run pid_controller pid_controller_node '
        f'--ros-args -p kp:={kp} -p ki:={ki} -p kd:={kd}; exec bash'
    )
    pid_cmd = ['gnome-terminal', '--', 'bash', '-c', pid_cmd_str]

    # 🔧 Simulator-kommando
    sim_cmd_str = (
        f'source ~/ros2_ws/install/setup.bash && '
        f'ros2 run joint_simulator joint_simulator_node '
        f'--ros-args --params-file {config_path}; exec bash'
    )
    sim_cmd = ['gnome-terminal', '--', 'bash', '-c', sim_cmd_str]

    # 🔧 Input-node
    input_cmd_str = (
        'source ~/ros2_ws/install/setup.bash && '
        'ros2 run pid_controller reference_input_node; exec bash'
    )
    input_cmd = ['gnome-terminal', '--', 'bash', '-c', input_cmd_str]

    # 🔍 Logg kommandoene (for feilsøking)
    print("⭐ PID kommando:", pid_cmd_str)
    print("⭐ SIM kommando:", sim_cmd_str)
    print("⭐ INPUT kommando:", input_cmd_str)

    return [
        ExecuteProcess(cmd=pid_cmd, shell=False),
        ExecuteProcess(cmd=sim_cmd, shell=False),
        ExecuteProcess(cmd=input_cmd, shell=False),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('kp', default_value='1.0'),
        DeclareLaunchArgument('ki', default_value='0.0'),
        DeclareLaunchArgument('kd', default_value='0.0'),

        OpaqueFunction(function=launch_setup)
    ])

