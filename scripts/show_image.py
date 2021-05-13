#!/usr/bin/env python3

# Copyright 2021 Evan Flynn, Lucas Walter
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
#    * Neither the name of the Evan Flynn nor the names of its
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


import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image


class ExamineImage(Node):
    def __init__(self):
        super().__init__('examine_image')

        self.mat = None
        self.sub = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            100)

    def image_callback(self, msg):
        sz = (msg.height, msg.width)
        # print(msg.header.stamp)
        if False:
            print("{encoding} {width} {height} {step} {data_size}".format(
                encoding=msg.encoding, width=msg.width, height=msg.height,
                step=msg.step, data_size=len(msg.data)))
        if msg.step * msg.height != len(msg.data):
            print("bad step/height/data size")
            return

        if msg.encoding == "rgb8":
            dirty = (self.mat is None or msg.width != self.mat.shape[1] or
                     msg.height != self.mat.shape[0] or len(self.mat.shape) < 2 or
                     self.mat.shape[2] != 3)
            if dirty:
                self.mat = np.zeros([msg.height, msg.width, 3], dtype=np.uint8)
            self.mat[:, :, 2] = np.array(msg.data[0::3]).reshape(sz)
            self.mat[:, :, 1] = np.array(msg.data[1::3]).reshape(sz)
            self.mat[:, :, 0] = np.array(msg.data[2::3]).reshape(sz)
        elif msg.encoding == "mono8":
            self.mat = np.array(msg.data).reshape(sz)
        else:
            print("unsupported encoding {}".format(msg.encoding))
            return
        if self.mat is not None:
            cv2.imshow("image", self.mat)
            cv2.waitKey(5)


def main(args=None):
    rclpy.init(args=args)

    examine_image = ExamineImage()

    try:
        rclpy.spin(examine_image)
    except KeyboardInterrupt:
        pass

    examine_image.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
