
import rospy
import base64
import cv2
import zmq
import json
import logging
import threading
import numpy as np
import math
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import pcl_ros
import ros_numpy


logging.basicConfig(
  format = '%(asctime)s:%(levelname)s:%(message)s',
  datefmt = '%m/%d/%Y %I:%M:%S %p',
  level = logging.DEBUG
)

class ImageTester :
    def __init__(self) :
        context = zmq.Context()
        self.socket = context.socket(zmq.REQ)
        self.socket.connect('tcp://localhost:7777')


        self.rgb_image = None
        self.depth_image = None
        self.xyz_image = None

        self.bridge = CvBridge()
        self.pub_info = rospy.Publisher('/face_mask/info', String, queue_size=10)
        self.pub_image_rgb = rospy.Publisher('/face_mask/image_raw', Image, queue_size=10)
        self.pub_image_depth = rospy.Publisher('/face_mask/depth/image_raw', Image, queue_size=10)
        self.rgb_sub = rospy.Subscriber("/rgb/rgb/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/rgb/depth/image_raw", Image, self.depth_callback)        
        self.pointcloud_sub = rospy.Subscriber("/rgb/depth/points", PointCloud2, self.pointcloud_callback)        

        self.frgb_thr = threading.Thread(target=self.front_loop, args=() )
        self.frgb_thr.start()

    def front_loop(self) :
        logging.info("frgb loop started")

        rate = rospy.Rate(25)

        while rospy.is_shutdown() is not True :
            if self.rgb_image is not None and self.depth_image is not None:
                # (height, width) = self.rgb_image.shape[:2]
                rgb_frame = self.rgb_image.copy()
                depth_frame = self.depth_image.copy()
                self.rgb_image = None
                self.depth_image = None
                encoded, buffer = cv2.imencode('.jpg', rgb_frame)
                jpg_as_text = base64.b64encode(buffer)
                self.socket.send(jpg_as_text)
                j = self.socket.recv_json()
                objects = j['persons']
                self.calculate_distance(depth_frame, objects)
                self.draw_bbox(rgb_frame, objects)
                self.draw_bbox(depth_frame, objects)
                self.publish_info(objects)
                self.publish_image(rgb_frame, depth_frame)
                # print(str(objects))
                # cv2.imshow("image", rgb_frame)
                # cv2.imshow("depth", depth_frame)
                # cv2.waitKey(10)

                # rospy.loginfo(str(objects))
                rate.sleep()


    def image_callback(self, msg) :
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # rgb_image = cv2.resize(rgb_image, dsize=(480, 360), interpolation=cv2.INTER_AREA)
            self.rgb_image = rgb_image

        except CvBridgeError as e:
            print(e)

    def depth_callback(self, msg) :
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            # rgb_image = cv2.resize(rgb_image, dsize=(480, 360), interpolation=cv2.INTER_AREA)
            self.depth_image = cv_image
        except CvBridgeError as e:
            print(e)


    def pointcloud_callback(self, msg) :
        ''' Converts a rospy PointCloud2 message to a numpy recordarray

        Reshapes the returned array to have shape (height, width), even if the height is 1.

        The reason for using np.fromstring rather than struct.unpack is speed... especially
        for large point clouds, this will be <much> faster.
        '''
        # construct a numpy record type equivalent to the point type of this cloud
        # dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)
        # print (msg)
        self.xyz_image = ros_numpy.point_cloud2.pointcloud2_to_array(msg)

        
    def calculate_distance(self, depth_image, objects) : 

        for obj in objects :
            (xmin, ymin, xmax, ymax) = obj[1:5]
            if (xmax - xmin) > 10 and (ymax - ymin) > 10 :
                cx = xmin + (xmax - xmin)/2
                cy = ymin + (ymax - ymin)/2

                (x, y, z) = self.xyz_image[cy, cx]


                u = [1, 0, 0]
                v = [x, y, z]

                v1_u = u / np.linalg.norm(u)
                v2_u = v / np.linalg.norm(v)
                dot_product = np.dot(v1_u, v2_u)
                angle = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
                degree = (angle * 180 / 3.14159) - 90.0
                distance = math.sqrt(x**2 + z**2)

                if math.isnan(degree) : degree = 0.0
                if math.isnan(distance)  : distance = 0.0

                obj.append(degree)
                obj.append(distance)

                # center_arr = depth_image[cx-5 : cx+5, cy-5 : cy +5]
                # average = np.average(depth_image[cx-5 : cx+5, cy-5 : cy +5])
                # obj.append(str(average))
                

    def draw_bbox(self, image, objects) :
        for object in objects :
            try :
                class_id = object[0]
                (xmin, ymin, xmax, ymax) = object[1:5]
                color = (0, 255, 255)
                # if class_id == 15:
                #     color = (0, 255, 255)  # bgr
                # else:
                #     color = (min(class_id * 12.5, 255), min(class_id * 7, 255), min(class_id * 5, 255))
                # cv2.rectangle(image, (xmin, ymin), (xmax, ymax), color, 2)
                cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (0, 255, 255), 2)
                cx = xmin + (xmax - xmin)/2
                cy = ymin + (ymax - ymin)/2
                cv2.rectangle(image, (cx-5, cy-5), (cx+5, cy+5), color, 2)
                text =  str(class_id) +"(" + str(xmax - xmin) +"), (" + str(ymax - ymin) +"), " + str(object[5])
                cv2.putText(image, text, (xmin, ymin - 7), cv2.FONT_HERSHEY_COMPLEX, 0.5, color, 1)

            except KeyError as e :
                logging.error(str(e))


    def publish_info(self, objects) :
        j = {
            "persons" : []

        }
        
        for obj in objects :
            mask_info = obj[5]
            bbox = obj[1:5]
            degree = obj[6]
            distance = obj[7]

            j_dic = {
                "mask" : mask_info,
                "bbox" : bbox,
                "degree" : degree,
                "distance" : distance
            }

            j['persons'].append(j_dic)
        
        j_str = json.dumps(j)
        # rospy.loginfo(json.dumps(j, indent=2))
        rospy.loginfo(j_str)
        self.pub_info.publish(j_str)

    def publish_image(self, rgb, depth) :
        self.pub_image_rgb.publish(self.bridge.cv2_to_imgmsg(rgb, "bgr8"))
        # self.pub_image_depth.publish(self.bridge.cv2_to_imgmsg(rgb, "bgr8"))


if __name__ == "__main__":

    rospy.init_node('image_sender', anonymous=True)
    sender = ImageTester()

    rospy.spin()

