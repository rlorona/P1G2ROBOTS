import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("proyecto_rm_bringup")
    description_share = get_package_share_directory("proyecto_rm_description")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "simulation.launch.py")
        ),
        launch_arguments={"headless": "false"}.items(),
    )

    trajectory = TimerAction(
        period=13.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, "launch", "trajectory.launch.py")
                )
            )
        ],
    )

    rviz = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(description_share, "rviz", "display.rviz"),
                ],
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    return LaunchDescription([simulation, rviz, trajectory])
