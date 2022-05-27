#!/usr/bin/python2
import time
import cv2
import zmq
import base64
import json
import logging as log
import numpy as np
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

log.basicConfig(
  format = '%(asctime)s:%(levelname)s:%(message)s',
  datefmt = '%m/%d/%Y %I:%M:%S %p',
  level = log.DEBUG
)


class Sender :
    def __init__(self) :
        context = zmq.Context()
        self.socket = context.socket(zmq.REQ)
        self.socket.connect('tcp://localhost:1234')

        # self.bridge = CvBridge()
        # self.rgb_image = None
        # # self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/color/image_raw")
        # self.rgb_sub = rospy.Subscriber(self.rgb_topic, Image, self.image_callback)

    # def image_callback(self, msg):
    #     try:
    #         rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    #         # rgb_image = cv2.resize(rgb_image, dsize=(480, 360), interpolation=cv2.INTER_AREA)
    #         rgb_image = cv2.resize(rgb_image, dsize=(640, 480), interpolation=cv2.INTER_AREA)
    #         self.rgb_image = rgb_image

    #     except CvBridgeError as e:
    #         print(e)

    def pub_loop(self) :

        '''
        wait image available
        '''
        # while self.rgb_image is None :
        #     rospy.loginfo("wait image is available")
        #     rospy.sleep(0.1)

        # rospy.loginfo("image is available")
        t = [ 
            [1000, 2000, 3000],
            [1000, 2000, 3000],
            [1000, 2000, 3000],
        ]
        a = np.array(t, dtype=np.int16)



        while True :
            try :
                # flatten = self.rgb_image.flatten()
                # print flatten.shape
                s = base64.b64encode(a)
                j = {
                    'shape' : a.shape,
                    'matrix' : s
                }
                self.socket.send_json(j)
                log.debug("array sent : " + str(a))
                # rospy.loginfo("array sent")
                res = self.socket.recv()
                log.debug(res)
                time.sleep(0.1)
            except KeyboardInterrupt as e :
                print (e)
                break
            # rgb_encoded = self.encode_image(self.rgb_image)
            # self.socket.send_string(rgb_encoded, flags=zmq.SNDMORE)
            # self.socket.send_string(depth_encoded)
            # rate.sleep()

if __name__ == "__main__":
    s = Sender()
    s.pub_loop()

    # rospy.spin()