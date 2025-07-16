# Copyright 2023 usb_cam Authors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the usb_cam Authors nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import os
from pathlib import Path
import sys

# Hack to get relative import of .camera_config file working
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)

from camera_config import CameraConfig, V4L2_CAM_DIR

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


# 定义多个相机配置
CAMERAS = []
CAMERAS.append(
    CameraConfig(
        name='camera1',
        param_path=Path(V4L2_CAM_DIR, 'config', 'params_1.yaml'),
        remappings= [
                        ('image_raw', 'camera1/image_raw'),
                        ('image_raw/compressed', 'camera1/image_compressed'),
                        ('image_raw/compressedDepth', 'camera1/compressedDepth'),
                        ('image_raw/theora', 'camera1/image_raw/theora'),
                        ('camera_info', 'camera1/camera_info'),
                    ]
    )
)
CAMERAS.append(
    CameraConfig(
        name='camera2', 
        param_path=Path(V4L2_CAM_DIR, 'config', 'params_2.yaml'),
        remappings= [
                        ('image_raw', 'camera2/image_raw'),
                        ('image_raw/compressed', 'camera2/image_compressed'),
                        ('image_raw/compressedDepth', 'camera2/compressedDepth'),
                        ('image_raw/theora', 'camera2/image_raw/theora'),
                        ('camera_info', 'camera2/camera_info'),
                    ]
    )
)
# 可以继续添加更多相机...


def generate_launch_description():
    ld = LaunchDescription()

    parser = argparse.ArgumentParser(description='usb_cam multi-camera demo with intra-process communication')
    parser.add_argument('-n', '--container-name', dest='container_name', type=str,
                        help='name for component container', default='usb_cam_container')

    # 创建组合节点列表
    composable_nodes = []
    
    for camera in CAMERAS:
        if not camera.param_path.exists():
            print(f"Warning: Parameter file {camera.param_path} not found, skipping camera {camera.name}")
            continue

        composable_node = ComposableNode(
            package='v4l2_camera',
            plugin='usb_cam::UsbCamNode',
            name=camera.name,
            namespace=camera.namespace,
            parameters=[camera.param_path],
            remappings=camera.remappings if camera.remappings else [],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        composable_nodes.append(composable_node)

    # 创建组件容器
    container = ComposableNodeContainer(
        name='usb_cam_container',
        namespace='cyberbus',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
        parameters=[{'use_intra_process_comms': True}]
    )

    ld.add_action(container)
    return ld
