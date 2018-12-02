import os

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    return LaunchDescription([
        Node(package='usb_cam', node_executable='usb_cam_node',
             output='screen', arguments=[['__params:=', ThisLaunchFileDir(), '/test_params.yaml']]),
        Node(package='image_tools', node_executable='showimage',
             output='screen', arguments=['-t', '/image_raw', '-r', '0'])
])
