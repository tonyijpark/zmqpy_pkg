#!/usr/bin/python

import rospy
import base64
import cv2
import zmq
import json
import logging
import threading
import numpy as np
import math
import tf
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import pcl_ros
import ros_numpy


class ImageSub :
    def __init__(self) :
        connect_to = "tcp://localhost:5454"

        ctx = zmq.Context()
        self.socket = ctx.socket(zmq.SUB)
        self.socket.setsockopt(zmq.SUBSCRIBE, "")
        self.socket.connect(connect_to)


    def subscribe(self) :
        while not rospy.is_shutdown() :
            try :
                (rgb_str, depth_str) = self.socket.recv_multipart()
                rgb_image = self.decode_image(rgb_str)
                depth_image = self.decode_depth_image(depth_str)

                # (rgb_image, depth_image) = self.socket.recv_multipart()

                # print np.max(depth_image)

                cv2.imshow("rgb", rgb_image)
                #cv2.imshow("depth", depth_image)
                cv2.waitKey(10)
            except KeyboardInterrupt as e:
                print (e)
                break

    def decode_image(self, image_str) :
        img = base64.b64decode(image_str)
        npimg = np.fromstring(img, dtype=np.uint8)
        source = cv2.imdecode(npimg, 1)
        return source

    def decode_depth_image(self, image_str) :
        img = base64.b64decode(image_str)
        npimg = np.fromstring(img, dtype=np.uint8)
        source = cv2.imdecode(npimg, 1)
        return source


if __name__ == "__main__":
    rospy.init_node('image_sub', anonymous=True)
    image_sub = ImageSub()
    image_sub.subscribe()

    rospy.spin()
