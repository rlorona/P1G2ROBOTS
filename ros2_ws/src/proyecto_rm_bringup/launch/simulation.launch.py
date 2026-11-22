import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory("proyecto_rm_description")
    bringup_share = get_package_share_directory("proyecto_rm_bringup")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    xacro_file = os.path.join(description_share, "urdf", "proyecto_rm.urdf.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()

    bridge_config = os.path.join(bringup_share, "config", "bridge.yaml")
    default_world = os.path.join(bringup_share, "worlds", "ackermann_track.sdf")

    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-s -r -v 4 ", world],
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(headless),
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -v 4 ", world],
            "on_exit_shutdown": "true",
        }.items(),
        condition=UnlessCondition(headless),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_proyecto_rm",
                output="screen",
                arguments=[
                    "-topic", "/robot_description",
                    "-name", "proyecto_rm",
                    "-allow_renaming", "false",
                    "-x", "0.0",
                    "-y", "0.0",
                    "-z", "0.01",
                    "-Y", "0.0",
                ],
            )
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[
            {
                "config_file": bridge_config,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="ros_gz_image",
        output="screen",
        arguments=["/top_camera/image_raw"],
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
    )

    def spawner(controller, delay):
        return TimerAction(
            period=delay,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    name=f"spawner_{controller}",
                    output="screen",
                    arguments=[
                        controller,
                        "--controller-manager", "/controller_manager",
                        "--controller-manager-timeout", "60",
                    ],
                )
            ],
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            gz_sim,
            gz_sim_gui,
            robot_state_publisher,
            bridge,
            spawn_robot,
            spawner("joint_state_broadcaster", 6.0),
            spawner("steering_controller", 8.0),
            spawner("rear_drive_controller", 10.0),
            image_bridge,
        ]
    )
