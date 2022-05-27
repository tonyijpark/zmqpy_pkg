#!/usr/bin/python2
import time
import rospy
import cv2
import zmq
import base64
import json
import logging as log
import numpy as np
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageSender :
    def __init__(self) :
        context = zmq.Context()
        self.socket = context.socket(zmq.REQ)
        self.socket.connect('tcp://localhost:2345')

        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_rect_raw")
        # self.rgb_topic = rospy.get_param("~rgb_topic", "/usb_cam/image_raw")
        self.rgb_sub = rospy.Subscriber(self.rgb_topic, Image, self.image_callback)
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_callback)

    def image_callback(self, msg):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # rgb_image = cv2.resize(rgb_image, dsize=(480, 360), interpolation=cv2.INTER_AREA)
            rgb_image = cv2.resize(rgb_image, dsize=(640, 480), interpolation=cv2.INTER_AREA)
            self.rgb_image = rgb_image
            # rospy.loginfo("new image assigned")
        except CvBridgeError as e:
            print(e)

    def depth_callback(self, msg) :
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            # print cv_image
            # rgb_image = cv2.resize(rgb_image, dsize=(480, 360), interpolation=cv2.INTER_AREA)
            self.depth_image = cv_image

            # print self.depth_image.dtype

            # cv2.imshow("source", self.depth_image)
            # cv2.waitKey(10)

        except CvBridgeError as e:
            print(e)


    def pub_loop(self) :

        '''
        wait image available
        '''
        while self.rgb_image is None :
            rospy.loginfo("wait image is available")
            rospy.sleep(0.1)

        rospy.loginfo("image is available")
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() :
            try :
                self.send_rgb()
                self.send_depth()
                rate.sleep()
            except KeyboardInterrupt as e :
                print (e)
                break
            # rgb_encoded = self.encode_image(self.rgb_image)
            # self.socket.send_string(rgb_encoded, flags=zmq.SNDMORE)
            # self.socket.send_string(depth_encoded)
            # rate.sleep()

    def send_rgb(self) :
        b64image = base64.b64encode(self.rgb_image)
        j = {
            'shape' : self.rgb_image.shape,
            'matrix' : b64image
        }

        self.socket.send_json(j)
        res = self.socket.recv()

    def send_depth(self) :
        b64image = base64.b64encode(self.depth_image)
        j = {
            'shape' : self.depth_image.shape,
            'matrix' : b64image
        }

        self.socket.send_json(j)
        res = self.socket.recv()

if __name__ == "__main__":
    rospy.init_node("image_sender", anonymous=True)
    s = ImageSender()
    s.pub_loop()

    rospy.spin()