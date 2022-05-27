
import rospy
import time
import cv2
import zmq
import base64
import json
import logging as log
import numpy as np

log.basicConfig(
  format = '%(asctime)s:%(levelname)s:%(message)s',
  datefmt = '%m/%d/%Y %I:%M:%S %p',
  level = log.DEBUG
)


if __name__ == "__main__":
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    # socket.setsockopt(zmq.LINGER, 0)
    socket.bind('tcp://*:1234')
    
    while True :
        try : 
            msg = socket.recv_json()
            shape = msg['shape']
            mat = msg['matrix']
            mat = base64.b64decode(mat)
            mat = np.frombuffer(mat, dtype=np.int16)
            mat = mat.reshape(shape)
            msg['matrix'] = mat
            log.debug(str(msg))
            # np.reshape(480, 640)
            # b.reshape(480, 640)
            # print np.shape(b)
            socket.send("yes")
        except KeyboardInterrupt as e :
            print (e)
            break
