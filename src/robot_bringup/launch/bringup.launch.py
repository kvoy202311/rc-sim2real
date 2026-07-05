import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    params_override_file = LaunchConfiguration("params_override_file")
    policies_file = LaunchConfiguration("policies_file")
    enable_restart_supervisor = LaunchConfiguration("enable_restart_supervisor")
    enable_startup_sound = LaunchConfiguration("enable_startup_sound")

    default_params = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "robot_params.yaml",
    )
    default_policies = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "policies.yaml",
    )

    def launch_setup(context, *args, **kwargs):
        runtime_params = [params_file]
        override_path = params_override_file.perform(context).strip()
        if override_path:
            runtime_params.append(params_override_file)

        return [
            Node(
                package="serial_comm",
                executable="serial_comm_node",
                name="serial_comm_node",
                parameters=runtime_params,
            ),
            Node(
                package="imu_driver",
                executable="imu_driver_node",
                name="imu_driver_node",
                parameters=runtime_params,
            ),
            Node(
                package="gamepad_input",
                executable="gamepad_input_node",
                name="gamepad_input_node",
                parameters=runtime_params,
            ),
            Node(
                package="robot_fsm",
                executable="robot_fsm_node",
                name="robot_fsm_node",
                parameters=runtime_params + [policies_file],
            ),
            Node(
                package="robot_bringup",
                executable="bringup_restart_supervisor.py",
                name="bringup_restart_supervisor",
                parameters=runtime_params,
                condition=IfCondition(enable_restart_supervisor),
            ),
            Node(
                package="robot_bringup",
                executable="startup_sound_node.py",
                name="startup_sound_node",
                parameters=runtime_params,
                condition=IfCondition(enable_startup_sound),
            ),
        ]

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("params_override_file", default_value=""),
        DeclareLaunchArgument("policies_file", default_value=default_policies),
        DeclareLaunchArgument("enable_restart_supervisor", default_value="true"),
        DeclareLaunchArgument("enable_startup_sound", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
