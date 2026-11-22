import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("proyecto_rm_bringup")
    params_file = os.path.join(bringup_share, "config", "trajectory.yaml")

    trajectory = LaunchConfiguration("trajectory")
    max_speed = LaunchConfiguration("max_speed")
    num_laps = LaunchConfiguration("num_laps")
    use_sim_time = LaunchConfiguration("use_sim_time")

    tracker = Node(
        package="proyecto_rm_control",
        executable="trajectory_tracker",
        name="trajectory_tracker",
        output="screen",
        parameters=[
            params_file,
            {
                "trajectory_type": trajectory,
                "max_speed": ParameterValue(max_speed, value_type=float),
                "num_laps": ParameterValue(num_laps, value_type=int),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("trajectory", default_value="figure8"),
            DeclareLaunchArgument("max_speed", default_value="0.35"),
            DeclareLaunchArgument("num_laps", default_value="3"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            tracker,
        ]
    )
