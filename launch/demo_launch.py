# Copyright 2018 Lucas Walter

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    image_manip_dir = get_package_share_directory('usb_cam')
    print('usb_cam dir ' + image_manip_dir)
    launches = []
    launches.append(launch_ros.actions.Node(
            package='usb_cam', node_executable='usb_cam_node', output='screen',
            ))
    launches.append(launch_ros.actions.Node(
            package='usb_cam', node_executable='show_image.py', output='screen',
            # arguments=[image_manip_dir + "/data/mosaic.jpg"])
            # remappings=[('image_in', 'image_raw')]
            ))

    return launch.LaunchDescription(launches)
