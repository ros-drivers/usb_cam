#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
import time

from rclpy.node import Node
from sensor_msgs.msg import Image


class ExamineImage(Node):
    def __init__(self):
        super().__init__('examine_image')

        self.mat = None
        self.sub = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback)
        # print("subscribed to " + self.sub.getTopic())

    def image_callback(self, msg):
        sz = (msg.height, msg.width)
        print(msg.header.stamp)
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
    # TODO(lucasw) this is taking 100% cpu
    # rclpy.spin(examine_image)
    # this is also taking 100% cpu
    while rclpy.ok():
        rclpy.spin_once(examine_image)
        time.sleep(0.1)

    examine_image.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
