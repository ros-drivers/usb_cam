import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='usb_cam', node_executable='usb_cam_node',
             output='screen'),
        Node(package='image_tools', node_executable='showimage',
             output='screen', arguments=['-t', '/image_raw', '-r', '0'])
])
