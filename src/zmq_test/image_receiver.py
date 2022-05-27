
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

def decode_rgb(msg) :
    shape = msg['shape']
    mat = msg['matrix']
    mat = base64.b64decode(mat)
    mat = np.frombuffer(mat, dtype=np.uint8)
    mat = mat.reshape(shape)
    return mat


def decode_depth(msg) :
    shape = msg['shape']
    mat = msg['matrix']
    mat = base64.b64decode(mat)
    mat = np.frombuffer(mat, dtype=np.float32)
    mat = mat.reshape(shape)
    return mat

if __name__ == "__main__":
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    # socket.setsockopt(zmq.LINGER, 0)
    socket.bind('tcp://*:2345')
    
    while True :
        try : 
            msg = socket.recv_json()
            rgb_image = decode_rgb(msg)
            socket.send("rgb_ok")

            msg = socket.recv_json()
            depth_image = decode_depth(msg)
            print np.max(depth_image)
            socket.send("depth_ok")
            # print mat
            # log.debug(str(msg))

            cv2.imshow('transfereed_image', rgb_image)
            cv2.imshow('transfereed_image_depth', depth_image)
            cv2.waitKey(10)

            log.debug("loop")

            # np.reshape(480, 640)
            # b.reshape(480, 640)
            # print np.shape(b)
        except KeyboardInterrupt as e :
            print (e)
            break
