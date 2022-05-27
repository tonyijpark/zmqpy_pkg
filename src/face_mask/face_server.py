#!/usr/bin/python3
import time
import cv2
import zmq
import base64
import json
import logging
import numpy as np
from face_detector import FaceDetector
from mask_detector import MaskDetector

logging.basicConfig(
  format = '%(asctime)s:%(levelname)s:%(message)s',
  datefmt = '%m/%d/%Y %I:%M:%S %p',
  level = logging.DEBUG
)

'''

recognition request server

'''

class FaceServer : 
    def __init__(self) :
        context = zmq.Context()
        self.socket = context.socket(zmq.REP)
        # socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind('tcp://*:7777')


        face_xml = "../model/face-detection-adas-0001.xml"
        face_bin = "../model/face-detection-adas-0001.bin"

        mask_xml =  "../model/face_mask.xml"
        mask_bin =  "../model/face_mask.bin"


        open_vino_device = 'CPU'
        open_vino_threshold = 0.8
        self.face_detector = FaceDetector(face_xml, face_bin, open_vino_device, open_vino_threshold)
        self.mask_detection = MaskDetector(mask_xml, mask_bin, open_vino_device, open_vino_threshold)

    def subscribe(self) :
        while True:
            #  Wait for next request from client
            
            try :
                frame = self.socket.recv_string()
                img = base64.b64decode(frame)
                npimg = np.fromstring(img, dtype=np.uint8)
                source = cv2.imdecode(npimg, 1)

                j_result = {
                    'persons' : []
                }
                results = self.recognize(source)
                # results = self.face_detector.recognize(source)
                for result in results :
                    j_result['persons'].append(result)

                self.socket.send_json(j_result)

            except KeyboardInterrupt as e:
                print (e)
                break

    
    def recognize(self, image) :
        initial_h, initial_w = image.shape[:2]
        results = self.face_detector.recognize(image)
        
        for obj in results :
            class_id = obj[0]
            (xmin_f, ymin_f, xmax_f, ymax_f) = obj[1:]

            margin = 20
            xmin_m = xmin_f - margin if xmin_f - margin >= 0 else 0
            ymin_m = ymin_f - margin if ymin_f - margin >= 0 else 0
            xmax_m = xmax_f + margin if xmax_f + margin <= initial_w else initial_w
            ymax_m = ymax_f + margin if ymax_f + margin <= initial_h else initial_h
            (xmin_f, ymin_f, xmax_f, ymax_f) = (xmin_m, ymin_m, xmax_m, ymax_m)

            # print(xmin_m, ", ", ymin_m, ", ", xmax_m, ", ", ymax_m)

            face = image[ymin_m : ymax_m, xmin_m : xmax_m]
            (face_height, face_width) = face.shape[:2]
            if face_height < 20 or face_width < 20 : 
                logging.warn("detected face too small")
                return None
            mask_result = self.mask_detection.recognize(face)
            # mask_result = mask_detection.recognize(frame)
            if mask_result > 0.0 :
                obj.append("with_mask")
            else :
                obj.append("no_mask")

        return results

        #     cv2.rectangle(frame_copy, (xmin_f, ymin_f), (xmax_f, ymax_f), (0, 255, 255), 2)
        #     cv2.putText(frame_copy, mask_text, (xmin_f, ymin_f - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
        # cv2.imshow("Frame", frame_copy)


if __name__ == "__main__":
    server = FaceServer()
    server.subscribe()

