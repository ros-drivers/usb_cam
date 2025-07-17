# Copyright 2025 CyberBus Project
#
# Test launch file for YAML configuration loading

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from pathlib import Path
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    # Get path to config file
    pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    config_file = os.path.join(pkg_dir, 'config', 'params_1.yaml')
    
    print(f"Loading config from: {config_file}")
    
    # Create composable node with YAML config
    composable_node = ComposableNode(
        package='v4l2_cam',
        plugin='Cyberbus::V4l2CamComponent',
        name='test_camera',
        parameters=[{'config_file_path': config_file}],
        remappings=[
            ('image_raw', 'test_camera/image_raw'),
            ('camera_info', 'test_camera/camera_info'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    
    # Create component container
    container = ComposableNodeContainer(
        name='test_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[composable_node],
        output='screen',
        parameters=[{'use_intra_process_comms': True}]
    )
    
    ld.add_action(container)
    return ld
