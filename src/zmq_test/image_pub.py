#!/usr/bin/python
import rospy
import base64
import cv2
import zmq
import json
import logging
import threading
import numpy as np
import tf
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import ros_numpy

class ImagePublisher :
    def __init__(self) : 

        self.rgb_image = None
        self.depth_image = None
        self.xyz_image = None

        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.points_topic = rospy.get_param("~points_topic", "/camera/depth_registered/points")
        
        self.bridge = CvBridge()

        self.rgb_sub = rospy.Subscriber(self.rgb_topic, Image, self.image_callback)
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_callback)
        self.pointcloud_sub = rospy.Subscriber(self.points_topic, PointCloud2, self.pointcloud_callback)


        self.pub_thread = threading.Thread(target=self.pub_loop, args=())
        self.pub_thread.start()


        self.topics = [
            "image.rgb",
            "image.depth"
        ]

        self.ctx = zmq.Context()
        self.socket = self.ctx.socket(zmq.PUB)
        self.socket.bind('tcp://*:5454')

    def image_callback(self, msg):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # rgb_image = cv2.resize(rgb_image, dsize=(480, 360), interpolation=cv2.INTER_AREA)
            rgb_image = cv2.resize(rgb_image, dsize=(640, 480), interpolation=cv2.INTER_AREA)
            self.rgb_image = rgb_image

        except CvBridgeError as e:
            print(e)

    def depth_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            # rgb_image = cv2.resize(rgb_image, dsize=(480, 360), interpolation=cv2.INTER_AREA)
            self.depth_image = cv_image
        except CvBridgeError as e:
            print(e)

    def pointcloud_callback(self, msg):
        self.xyz_image = ros_numpy.point_cloud2.pointcloud2_to_array(msg)

    def pub_loop(self) :

        '''
        wait image available
        '''
        while self.rgb_image is None or self.depth_image is None :
            rospy.loginfo("wait image is available")
            rospy.sleep(0.1)

        rospy.loginfo("image is available")

        rate = rospy.Rate(30)
        while not rospy.is_shutdown() :
            try :
                rgb_encoded = self.encode_image(self.rgb_image)
                depth_encoded = self.encode_image(self.depth_image)
                self.socket.send_string(rgb_encoded, flags=zmq.SNDMORE)
                self.socket.send_string(depth_encoded)
                rate.sleep()

            except Exception as e :
                print(e)
                break

    def encode_image(self, image) :
        encoded, buffer = cv2.imencode('.jpg', image)
        jpg_as_text = base64.b64encode(buffer)
        return jpg_as_text

if __name__ == "__main__":
    rospy.init_node("zmq_image_pub", anonymous=False)
    ip = ImagePublisher()
    rospy.spin()
