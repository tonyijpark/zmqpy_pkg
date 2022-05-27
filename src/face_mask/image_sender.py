
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


logging.basicConfig(
    format='%(asctime)s:%(levelname)s:%(message)s',
    datefmt='%m/%d/%Y %I:%M:%S %p',
    level=logging.DEBUG
)


class ImageSender:
    def __init__(self):
        context = zmq.Context()
        self.socket = context.socket(zmq.REQ)
        self.socket.connect('tcp://localhost:7777')

        self.rgb_image = None
        self.depth_image = None
        self.xyz_image = None

        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/rgb/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.points_topic = rospy.get_param("~points_topic", "/camera/depth/points")
        

        self.bridge = CvBridge()
        self.pub_info = rospy.Publisher(
            '/face_mask/info', String, queue_size=10)
        self.pub_image_rgb = rospy.Publisher(
            '/face_mask/image_raw', Image, queue_size=10)
        self.pub_image_depth = rospy.Publisher(
            '/face_mask/depth/image_raw', Image, queue_size=10)


        self.pub_marker = rospy.Publisher('/face_mask/marker', MarkerArray, queue_size=10)

        self.rgb_sub = rospy.Subscriber(
            self.rgb_topic, Image, self.image_callback)
        self.depth_sub = rospy.Subscriber(
            self.depth_topic, Image, self.depth_callback)
        self.pointcloud_sub = rospy.Subscriber(
            self.points_topic, PointCloud2, self.pointcloud_callback)

        self.frgb_thr = threading.Thread(target=self.front_loop, args=())
        self.frgb_thr.start()

        self.no_maskcnt = 0
        self.MASK_COUNT_LIMIT = 5
        self.WITH_MASK = "with_mask"
        self.NO_MASK = "no_mask"
        self.WINDOW_SIZE = 10

    def front_loop(self):
        # logging.info("frgb loop started")
        rospy.loginfo("frgb loop started")

        rate = rospy.Rate(30)

        while rospy.is_shutdown() is not True:
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
                self.publish_marker(objects)
                # print(str(objects))
                cv2.imshow("image", rgb_frame)
                cv2.imshow("depth", depth_frame)
                cv2.waitKey(10)

                # rospy.loginfo(str(objects))
                rate.sleep()

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
        ''' Converts a rospy PointCloud2 message to a numpy recordarray

        Reshapes the returned array to have shape (height, width), even if the height is 1.

        The reason for using np.fromstring rather than struct.unpack is speed... especially
        for large point clouds, this will be <much> faster.
        '''
        # construct a numpy record type equivalent to the point type of this cloud
        # dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)
        # print (msg)
        self.xyz_image = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
        # print self.xyz_image.shape
        # print self.xyz_image
        # self.xyz_image = np.reshape(self.xyz_image, (msg.height, msg.width, 3))
        # print self.xyz_image
        # print "------\n\n"
        # print self.xyz_image

    def calculate_distance(self, depth_image, objects):
        (height, width) = depth_image.shape[:2]
        for obj in objects:
            (xmin, ymin, xmax, ymax) = obj[1:5]
            if (xmax - xmin) > 10 and (ymax - ymin) > 10:

                # grab center
                cx = xmin + (xmax - xmin)/2
                cy = ymin + (ymax - ymin)/2

                # small box calculation
                (s_lx, s_rx) = (cx - self.WINDOW_SIZE / 2, cx + self.WINDOW_SIZE / 2)
                (s_ly, s_ry) = (cy - self.WINDOW_SIZE / 2, cy + self.WINDOW_SIZE / 2)

                # array index cutting
                if s_lx < 0 : s_lx = 0
                if s_ly < 0 : s_ly = 0
                if s_rx > width : s_ry = width
                if s_ry > height : s_ry = height

                (x, y, z, rgb) = self.xyz_image[cy, cx]

                u = [1, 0, 0]
                v = [x, y, z]

                # calculate angle
                v1_u = u / np.linalg.norm(u)
                v2_u = v / np.linalg.norm(v)
                angle = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
                degree = (angle * 180 / 3.14159) - 90.0
                
                sliced = depth_image[s_ly : s_ry, s_lx : s_rx]
                target_array = sliced.flatten()

                distance = np.average(target_array)

                # sum = 0
                # count = 0
                # for d in target_array :
                #     if d != 0 :
                #         sum += d
                #         count+=1
                
                # if count > 0 : distance = sum / count
                # else : distance = 0

                # distance = np.average(depth_image[ymin:ymax, xmin:xmax])

                if math.isnan(degree):      degree = 0.0
                if math.isnan(distance):    distance = 0.0

                obj.append(degree)
                obj.append(distance)
                obj.append((x, y, z))

                # center_arr = depth_image[cx-5 : cx+5, cy-5 : cy +5]
                # average = np.average(depth_image[cx-5 : cx+5, cy-5 : cy +5])
                # obj.append(str(average))

    def draw_bbox(self, image, objects):
        for object in objects:
            try:
                class_id = object[0]
                (xmin, ymin, xmax, ymax) = object[1:5]
                mask_info = object[5]
                degree = int(object[6])
                disance = int(object[7])

                thickness = 2
                if mask_info == self.WITH_MASK :
                    color = (0, 255, 255)
                else :
                    color = (0, 0, 255)
                    thickness = 3
                # if class_id == 15:
                #     color = (0, 255, 255)  # bgr
                # else:
                #     color = (min(class_id * 12.5, 255), min(class_id * 7, 255), min(class_id * 5, 255))
                # cv2.rectangle(image, (xmin, ymin), (xmax, ymax), color, 2)
                
                cv2.rectangle(image, (xmin, ymin),
                              (xmax, ymax), color, thickness=thickness)
                cx = xmin + (xmax - xmin)/2
                cy = ymin + (ymax - ymin)/2
                cv2.rectangle(image, (cx-5, cy-5), (cx+5, cy+5), (255, 0, 0), thickness=thickness)
                text = str(disance) +", " + str(degree)
                cv2.putText(image, text, (xmin, ymin - 7),
                            cv2.FONT_HERSHEY_COMPLEX, 1, color, thickness=thickness)

            except KeyError as e:
                logging.error(str(e))

    def publish_info(self, objects):
        j = {
            "persons": []
        }

        for obj in objects:
            mask_info = obj[5]
            bbox = obj[1:5]
            degree = str(obj[6])
            distance = str(obj[7])

            # if mask_info == self.NO_MASK:
            #     if self.no_maskcnt < self.MASK_COUNT_LIMIT:
            #         mask_info = "with_mask"
            #         self.no_maskcnt = self.no_maskcnt + 1
            #     elif self.no_maskcnt >= self.MASK_COUNT_LIMIT:
            #         self.no_maskcnt = 0
            #         mask_info = self.NO_MASK

            rospy.loginfo("mask_info :" + mask_info)

            j_dic = {
                "mask": mask_info,
                "bbox": bbox,
                "degree": degree,
                "distance": distance
            }

            j['persons'].append(j_dic)

        j_str = json.dumps(j)
        # rospy.loginfo(json.dumps(j, indent=2))
        rospy.loginfo(j_str)
        self.pub_info.publish(j_str)

    def publish_image(self, rgb, depth):
        self.pub_image_rgb.publish(self.bridge.cv2_to_imgmsg(rgb, "bgr8"))
        # self.pub_image_depth.publish(self.bridge.cv2_to_imgmsg(rgb, "bgr8"))


    def publish_marker(self, objects) :
        marker_array = MarkerArray()
        for object in objects:
            marker = Marker()
            marker.header.frame_id = "camera_depth_frame"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.a = 1.0

            degree = float(object[6])
            angle = degree / 180 * 3.14159
            (roll, pitch, yaw) = (0.0, 0.0, angle)
            quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
            marker.pose.orientation.x = quaternion[0]
            marker.pose.orientation.y = quaternion[1]
            marker.pose.orientation.z = quaternion[2]
            marker.pose.orientation.w = quaternion[3]
            (x, y, z) = object[8]
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker_array.markers.append(marker)

        self.pub_marker.publish(marker_array)



if __name__ == "__main__":
    rospy.init_node('face_client', anonymous=False)
    sender = ImageSender()
    rospy.spin()
