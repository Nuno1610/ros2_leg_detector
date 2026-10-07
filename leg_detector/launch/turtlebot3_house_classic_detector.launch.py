#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    turtlebot3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    leg_detector_share = get_package_share_directory("leg_detector")
    turtlebot3_house = os.path.join(
        turtlebot3_gazebo_share, "launch", "turtlebot3_house.launch.py"
    )
    forest_file = os.path.join(
        leg_detector_share, "config", "trained_leg_detector_res=0.33.yaml"
    )
    use_sim_time = {"use_sim_time": True}
    fixed_frame = "base_scan"

    house = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(turtlebot3_house),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    detector = Node(
        package="leg_detector",
        executable="detect_leg_clusters",
        name="detect_leg_clusters",
        parameters=[
            use_sim_time,
            {"forest_file": forest_file},
            {"scan_topic": "/scan", "fixed_frame": fixed_frame},
        ],
        output="screen",
    )
    tracker = Node(
        package="leg_detector",
        executable="joint_leg_tracker.py",
        name="joint_leg_tracker",
        parameters=[
            use_sim_time,
            {
                "scan_topic": "/scan",
                "fixed_frame": fixed_frame,
                "publish_people_frame": fixed_frame,
                "scan_frequency": 5.0,
            },
        ],
        output="screen",
    )
    mapping = Node(
        package="leg_detector",
        executable="local_occupancy_grid_mapping",
        name="local_occupancy_grid_mapping",
        parameters=[
            use_sim_time,
            {"scan_topic": "/scan", "fixed_frame": fixed_frame},
        ],
        output="screen",
    )
    inflated_scan = Node(
        package="leg_detector",
        executable="inflated_human_scan",
        name="inflated_human_scan",
        parameters=[use_sim_time, {"inflation_radius": 1.0}],
        output="screen",
    )

    return LaunchDescription([house, detector, tracker, mapping, inflated_scan])
