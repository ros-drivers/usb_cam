# Copyright 2018 Lucas Walter

import argparse
from launch import LaunchDescription
from launch_ros.actions import Node

import os
import sys
import time
import yaml

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    parser = argparse.ArgumentParser(description='usb_cam demo')
    parser.add_argument('-n', '--node-name', dest='node_name', type=str,
            help='name for device', default='usb_cam')
    # parser.add_argument('-d', '--device', dest='device', type=str,
            # help='video device', default='/dev/video0')
    # parser.add_argument('-wd', '--width', dest='width', type=int,
            # help='image width', default=640)
    # parser.add_argument('-ht', '--height', dest='height', type=int,
            # help='image height', default=480)
    # parser.add_argument('-f', '--fps', dest='frame_rate', type=float,
            # help='frame rate', default=5)
    args, unknown = parser.parse_known_args(sys.argv[4:])

    usb_cam_dir = get_package_share_directory('usb_cam')
    # print('usb_cam dir ' + usb_cam_dir)
    launches = []

    # get path to params file
    params_path= os.path.join(
        usb_cam_dir,
        'config',
        'params.yaml'
    )

    # if not os.path.exists(params_path):
        # os.makedirs(params_path)

    # TODO(lucasw) get from commandline
    # TODO(lucasw) if these are an invalid combination usb_cam just dies-
    # need a more helpfull error message.
    # device = args.device
    node_name = args.node_name
    # frame_rate = int(args.frame_rate)
    # width = args.width
    # height = args.height

    # usb camera
    # path = prefix + "demo_usb_cam.yaml"
    # ns = 'demo'
    # params = {}

    # with open(path, 'w') as outfile:
        # print("opened " + path + " for yaml writing")
        # params[node_name] = {}
        # params[node_name]['ros__parameters'] = dict(
                # video_device = device,
                # framerate = frame_rate,
                # io_method = "mmap",
                # frame_id = "camera",
                # pixel_format = "yuyv",
                # image_width = width,
                # image_height = height,
                # camera_name = name,
                # )
        # yaml.dump(params, outfile, default_flow_style=False)
    print(params_path)
    ld.add_action(Node(
            package='usb_cam', executable='usb_cam_node', output='screen',
            name=node_name,
            # namespace=ns,
            parameters=[params_path]
            ))
    ld.add_action(Node(
            package='usb_cam', executable='show_image.py', output='screen',
            # namespace=ns,
            # arguments=[image_manip_dir + "/data/mosaic.jpg"])
            # remappings=[('image_in', 'image_raw')]
            ))

    return ld
