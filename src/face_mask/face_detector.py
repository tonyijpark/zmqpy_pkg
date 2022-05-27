import cv2
import time
import sys
# from inference import Network, IECore
from openvino.inference_engine import IENetwork, IECore
import numpy as np


class FaceDetector :
    def __init__(self, model_xml, model_bin, device, threshold) :
        open_vino_model = model_xml #'./face-detection-adas-0001.xml'
        open_vino_model_bin = model_bin #'./face-detection-adas-0001.bin'
        open_vino_device = device #'CPU'
        self.open_vino_threshold = threshold #0.8
        # Instantiating the Network clas

        ie = IECore()
        self.input_blob = None
        # Calling the load_model function
        # It returns the plugin and the shape of the input layer
        # n,c,h,w = infer_network.load_model(open_vino_model,open_vino_device,1,1,2,open_vino_library)[1]
        self.input_blob = None
        self.infer_network = ie.read_network(open_vino_model, open_vino_model_bin)
        for blob_name in self.infer_network.inputs :
            self.input_blob = blob_name


        self.out_blob = next(iter(self.infer_network.outputs))

        print(self.out_blob)

        self.exec_net = ie.load_network(network = self.infer_network, device_name = open_vino_device)
        self.n, self.c, self.h, self.w = self.infer_network.inputs[self.input_blob].shape

    def recognize(self, frame) :
        feed_dict = {}
        frame_copy = frame.copy()
        initial_h, initial_w = frame.shape[:2]

        in_frame = cv2.resize(frame_copy, (self.w, self.h))
        in_frame = in_frame.transpose((2,0,1))
        in_frame = in_frame.reshape((self.n, self.c, self.h, self.w))

        feed_dict[self.input_blob] = in_frame
        res = self.exec_net.infer(inputs = feed_dict)

        # if infer_network.wait(cur_request_id) == 0:
        #     res = infer_network.get_output(cur_request_id)
        result = []
        res = res[self.out_blob]
            # Parsing the result
        for obj in res[0][0]:
            if obj[2] > self.open_vino_threshold:
                # The result obtained is normalised, hence it is being multiplied with the original width and height.
                xmin = int(obj[3] * initial_w)
                ymin = int(obj[4] * initial_h)
                xmax = int(obj[5] * initial_w)
                ymax = int(obj[6] * initial_h)
                class_id = int(obj[1])
                
                confidence = int(obj[2])
                result.append( [class_id, xmin, ymin, xmax, ymax] )

        return result
                # # cropped_image = frame_copy[ymin:ymax,xmin:xmax]
                # cv2.rectangle(frame_copy, (xmin, ymin), (xmax, ymax), (0, 255, 255), 2)
                # # cv2.imshow('Face',cropped_image)
                # cv2.imshow("Frame", frame_copy)
                # print('FPS : {:.2f}'.format(fps))

