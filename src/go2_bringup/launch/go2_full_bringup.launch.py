#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 1) RealSense camera launch (rs_launch.py with args)
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'color_width': '848',
            'color_height': '480',
            'depth_width': '848',
            'depth_height': '480',
            'align_depth': 'true',
        }.items(),
    )

    # 2) Vision node (pingpong_node)
    pingpong_node = Node(
        package='vision_node',
        executable='pingpong_node',
        name='pingpong_node',
        output='screen',
    )

    # 3) ArUco node -- DOUBLE CHECK
    aruco_node = Node( 
        package='aruco_node',
        executable='aruco',
        name='aruco_node',
        output='screen',
    )

    # 4) Base driver (Go2W low-level driver) -- DOUBLE CHECK
    base_driver_node = Node(
        package='base_driver',
        executable='base_driver',
        name='base_driver',
        output='screen',
    )

    # 5) Navigation node (navigation_node_three)
    navigator_node = Node(
        package='navigation_node_three',
        executable='navigator',
        name='navigator',
        output='screen',
    )

    # 6) Arm driver launch (includes its own launch file) -- DOUBLE CHECK
    arm_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('arm_driver'),
                'launch',
                'arm_driver.launch.py'
            )
        )
    )

    # 7) Arm control node -- DOUBLE CHECK
    arm_control_node = Node(
        package='arm_control',
        executable='arm_node',
        name='arm_node',
        output='screen',
    )

    # 8) Return navigation node
    return_nav_node = Node(
        package='navigation_return_node',
        executable='return_navigator',
        name='return_navigator',
        output='screen',
    )

    return LaunchDescription([
        realsense_launch,
        arm_driver_launch,
        pingpong_node,
        aruco_node,
        base_driver_node,
        navigator_node,
        arm_control_node,
        return_nav_node,
    ])

