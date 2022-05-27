#!/usr/bin/python3
import time
import cv2
import zmq
import base64
import json
import logging
import numpy as np
from person_detector import PersonDetector

logging.basicConfig(
  format = '%(asctime)s:%(levelname)s:%(message)s',
  datefmt = '%m/%d/%Y %I:%M:%S %p',
  level = logging.DEBUG
)

class PeopleServer : 
    def __init__(self) :
        context = zmq.Context()
        self.socket = context.socket(zmq.REP)
        # socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind('tcp://*:3333')
        logging.info("server started on 3333")


        model_xml = "../../model/mobilenet-ssd/mobilenet-ssd.xml"
        model_bin = "../../model/mobilenet-ssd/mobilenet-ssd.bin"

        open_vino_device = 'CPU'
        open_vino_threshold = 0.8
        self.face_detector = PersonDetector(model_xml, model_bin, open_vino_device, open_vino_threshold)

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
        return results

        #     cv2.rectangle(frame_copy, (xmin_f, ymin_f), (xmax_f, ymax_f), (0, 255, 255), 2)
        #     cv2.putText(frame_copy, mask_text, (xmin_f, ymin_f - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
        # cv2.imshow("Frame", frame_copy)


if __name__ == "__main__":
    server = PeopleServer()
    server.subscribe()

