import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    params_override_file = LaunchConfiguration("params_override_file")
    policies_file = LaunchConfiguration("policies_file")

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
        ]

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("params_override_file", default_value=""),
        DeclareLaunchArgument("policies_file", default_value=default_policies),
        OpaqueFunction(function=launch_setup),
    ])
