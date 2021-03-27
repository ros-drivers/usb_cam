# Copyright 2018 Lucas Walter

import argparse
import launch
import launch_ros.actions
import os
import sys
import time
import yaml

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    parser = argparse.ArgumentParser(description='usb_cam demo')
    parser.add_argument('-d', '--device', dest='device', type=str,
            help='video device', default='/dev/video0')
    parser.add_argument('-wd', '--width', dest='width', type=int,
            help='image width', default=640)
    parser.add_argument('-ht', '--height', dest='height', type=int,
            help='image height', default=480)
    parser.add_argument('-f', '--fps', dest='frame_rate', type=float,
            help='frame rate', default=5)
    args, unknown = parser.parse_known_args(sys.argv[4:])

    usb_cam_dir = get_package_share_directory('usb_cam')
    print('usb_cam dir ' + usb_cam_dir)
    launches = []

    prefix = "/tmp/ros2/" + str(int(time.time())) + "/"
    if not os.path.exists(prefix):
        os.makedirs(prefix)

    # TODO(lucasw) get from commandline
    # TODO(lucasw) if these are an invalid combination usb_cam just dies-
    # need a more helpfull error message.
    device = args.device
    framerate = int(args.frame_rate)
    width = args.width
    height = args.height

    # usb camera
    params = prefix + "demo_usb_cam.yaml"
    ns = 'demo'
    node_name = 'usb_cam'
    with open(params, 'w') as outfile:
        print("opened " + params + " for yaml writing")
        data = {}
        data[ns] = {}
        data[ns][node_name] = {}
        data[ns][node_name]['ros__parameters'] = dict(
                video_device = device,
                framerate = framerate,
                io_method = "mmap",
                frame_id = "camera",
                pixel_format = "yuyv",
                image_width = width,
                image_height = height,
                camera_name = "camera",
                )
        yaml.dump(data, outfile, default_flow_style=False)

    launches.append(launch_ros.actions.Node(
            package='usb_cam', node_executable='usb_cam_node', output='screen',
            node_name=node_name,
            node_namespace=ns,
            arguments=["__params:=" + params],
            # arguments=["__params:=" + usb_cam_dir + "/config/params.yaml"]
            ))
    launches.append(launch_ros.actions.Node(
            package='usb_cam', node_executable='show_image.py', output='screen',
            node_namespace=ns,
            # arguments=[image_manip_dir + "/data/mosaic.jpg"])
            # remappings=[('image_in', 'image_raw')]
            ))

    return launch.LaunchDescription(launches)
