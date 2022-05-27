#!/usr/bin/python3
import time
import cv2
import zmq
import base64
import numpy as np
import json
from person_detector import PersonDetector

class Server : 
    def __init__(self) :
        context = zmq.Context()
        self.socket = context.socket(zmq.REP)
        # socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind('tcp://*:5555')


        model_xml = "../model/mobilenet-ssd/mobilenet-ssd.xml"
        model_bin = "../model/mobilenet-ssd/mobilenet-ssd.bin"
        self.detector = PersonDetector(model_xml, model_bin, "CPU", 0.95)

    def subscribe(self) :
        while True:
            #  Wait for next request from client
            try :
                frame = self.socket.recv_string()
                img = base64.b64decode(frame)
                npimg = np.fromstring(img, dtype=np.uint8)
                source = cv2.imdecode(npimg, 1)

                j_result = {
                    'results' : []
                }
                results = self.detector.recognize(source)
                for result in results :
                    j_result['results'].append(result)

                self.socket.send_json(j_result)

            except KeyboardInterrupt as e:
                print (e)
                break


if __name__ == "__main__":
    server = Server()
    server.subscribe()

