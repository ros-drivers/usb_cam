import argparse
import os
from pathlib import Path  
import sys

print("Starting Webcam Launch File")

# Hack to get relative import of .camera_config file working
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)

from camera_config import CameraConfig
from launch import LaunchDescription  
from launch.actions import GroupAction  
from launch_ros.actions import Node  
#get this package path
USB_CAM_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
print("Finished importing modules")


CAMERAS = []
CAMERAS.append(
    CameraConfig(
        name='webcam_node',
        namespace='rosa',
        param_path=Path(USB_CAM_DIR, 'config', 'rosa_webcam.yaml')
    )
)
CAMERAS.append(
    CameraConfig(
        name='webcam_node',
        namespace='rondor',
        param_path=Path(USB_CAM_DIR, 'config', 'rondor_webcam.yaml')
    )
    # Add more Camera's here and they will automatically be launched below
)

print("Finished configuring cameras")

def generate_launch_description():
    ld = LaunchDescription()

    parser = argparse.ArgumentParser(description='usb_cam demo')
    parser.add_argument('-n', '--node-name', dest='node_name', type=str,
                        help='name for device', default='usb_cam')

    camera_nodes = [
        Node(
            package=__package__ if __package__ else 'usb_cam',
            executable='usb_cam_node_exe', output='screen',
            name=camera.name,
            namespace=camera.namespace,
            parameters=[camera.param_path],
            remappings=camera.remappings
        )
        for camera in CAMERAS
    ]

    camera_group = GroupAction(camera_nodes)

    ld.add_action(camera_group)
    return ld

if __name__ == '__main__':
    print("Running generate_launch_description()")
    launch_description = generate_launch_description()
    print("Launch description generated")
    
    # Normally you would use a ROS 2 launch system to run this, but for testing:
    from launch import LaunchService
    ls = LaunchService()
    ls.include_launch_description(launch_description)
    ls.run()
