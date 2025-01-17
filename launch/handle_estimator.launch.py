#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # 引数の宣言
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    rviz_cfg_arg = DeclareLaunchArgument(
        'rviz_cfg',
        default_value=os.path.join(
            get_package_share_directory('bag_handle_estimator'),
            'config',
            'rviz',
            'bag_handle.rviz'
        ),
        description='Path to the RViz config file'
    )

    # bag_handle_estimator のノード
    handle_estimator_node = Node(
        package='bag_handle_estimator',
        executable='handle_estimator',
        name='handle_estimator',
        output='screen',
        parameters=[{
            'execute_default': True,
            'pub_plane_cloud': True,
            'sub_point_topic_name': '/camera/camera/depth/color/points',
            'base_frame_name': 'base_footprint',
            # depth range
            'depth_range_min_x': 0.0,
            'depth_range_max_x': 0.5,
            # width range
            'depth_range_min_y': -0.3,
            'depth_range_max_y': 0.3,
            # height range
            'depth_range_min_z': 0.0,
            'depth_range_max_z': 0.5
        }]
    )

    # RViz のノード (条件付き)
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_cfg')],
        output='screen'
    )

    return LaunchDescription([
        rviz_arg,
        rviz_cfg_arg,
        handle_estimator_node,
        rviz_node
    ])
