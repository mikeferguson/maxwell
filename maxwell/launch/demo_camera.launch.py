#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer, Node
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    return LaunchDescription([
        # Arguments first
        DeclareLaunchArgument(
            'namespace', default_value='/demo_camera'
        ),
        DeclareLaunchArgument(
            'rgb_frame_id', default_value='demo_camera_rgb_optical_frame'
        ),
        DeclareLaunchArgument(
            'depth_frame_id', default_value='demo_camera_rgb_depth_optical_frame'
        ),
        ComposableNodeContainer(
            name='container',
            namespace=LaunchConfiguration('namespace'),
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                ComposableNode(
                    package='openni2_camera',
                    plugin='openni2_wrapper::OpenNI2Driver',
                    name='driver',
                    namespace=LaunchConfiguration('namespace'),
                    parameters=[{'rgb_frame_id':
                                 LaunchConfiguration('rgb_frame_id'),
                                 'depth_frame_id':
                                 LaunchConfiguration('depth_frame_id'),
                                 'depth_registration': True,
                                 'data_skip': 1,  # throttle to 15hz
                                 'use_device_time': False}, ],
                    remappings=[('depth/image', 'depth_registered/image_raw')],
                ),
            ],
            output='screen',
        ),
        Node(
            name='static_tf',
            namespace=LaunchConfiguration('namespace'),
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'demo_camera_rgb_optical_frame']
        )
    ])
