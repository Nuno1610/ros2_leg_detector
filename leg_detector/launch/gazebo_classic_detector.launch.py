#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("leg_detector")
    gazebo_share = get_package_share_directory("gazebo_ros")
    world_path = os.path.join(package_share, "worlds", "classic_detector.world")
    robot_path = os.path.join(package_share, "description", "classic_detector_bot.urdf")
    rviz_path = os.path.join(
        package_share,
        "rosbag",
        "demos",
        "rviz",
        "demo_stationary_simple_environment.rviz",
    )

    use_sim_time = {"use_sim_time": True}
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world_path, "verbose": "false"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": Command(["cat ", robot_path])}, use_sim_time],
        output="screen",
    )
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "classic_detector_bot", "-topic", "robot_description"],
        output="screen",
    )

    detector = Node(
        package="leg_detector",
        executable="detect_leg_clusters",
        name="detect_leg_clusters",
        parameters=[
            use_sim_time,
            {"forest_file": os.path.join(package_share, "config", "trained_leg_detector_res=0.33.yaml")},
            {"scan_topic": "/scan", "fixed_frame": "laser"},
        ],
        output="screen",
    )
    tracker = Node(
        package="leg_detector",
        executable="joint_leg_tracker.py",
        name="joint_leg_tracker",
        parameters=[use_sim_time, {"scan_topic": "/scan", "fixed_frame": "laser", "scan_frequency": 10.0}],
        output="screen",
    )
    mapping = Node(
        package="leg_detector",
        executable="local_occupancy_grid_mapping",
        name="local_occupancy_grid_mapping",
        parameters=[use_sim_time, {"scan_topic": "/scan", "fixed_frame": "laser"}],
        output="screen",
    )
    inflated_scan = Node(
        package="leg_detector",
        executable="inflated_human_scan",
        name="inflated_human_scan",
        parameters=[use_sim_time, {"inflation_radius": 1.0}],
        output="screen",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
        parameters=[use_sim_time],
        additional_env={
            "GTK_PATH": "",
            "GTK_EXE_PREFIX": "",
            "GIO_MODULE_DIR": "",
            "GSETTINGS_SCHEMA_DIR": "",
            "LOCPATH": "",
            "GTK_IM_MODULE_FILE": "",
            "GTK_MODULES": "",
        },
        output="screen",
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.environ.get("GAZEBO_MODEL_PATH", "")),
            DeclareLaunchArgument("world", default_value=world_path),
            gazebo,
            robot_state_publisher,
            spawn_robot,
            detector,
            tracker,
            mapping,
            inflated_scan,
            rviz,
        ]
    )
