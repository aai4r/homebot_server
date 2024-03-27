#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import sys
#----------------------------------------------------------------
# local 테스트 진행 시
# 현재위치의 상대경로를 시스템에서 인식시켜주기 위한 작업
path = os.path.abspath(os.path.dirname(sys.argv[0]))
sys.path.append(path[:path.rfind('/')])
#----------------------------------------------------------------

import rclpy
from rclpy.node import Node
import rclpy.qos as qos
import std_msgs.msg as std_msg
from sensor_msgs.msg import CompressedImage
#from sensor_msgs.msg import Image
import cv2
import numpy as np
from constant.constant import *


class RosTest(Node):

    def __init__(self):
        super().__init__('RosTest')
        self.__window_name = "ETRI EDGE DEBUG IMG"
        self.img_sub = None
        self._setup()

    def _setup(self):

        _profile = qos.QoSProfile(history=qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL)
        self.img_sub = self.create_subscription(CompressedImage,
                                                TOPIC_EDGE_SEND_IMAGE,
                                                self.msg_callback,
                                                qos_profile=_profile)

    def msg_callback(self, msg):

        #self.print_header_info(msg.header)
        #np_img = np.reshape(msg.data, (msg.height, msg.width, 3)).astype(np.uint8)
        self.display(msg.data)

    def print_header_info(self, m : std_msg.Header):
        print("received image stamp {}".format(m.stamp))

    #def display(self, img : np.ndarray):
    def display(self, param_data):

        _encoded_img = np.frombuffer(param_data, dtype=np.uint8)
        _cv_image = cv2.imdecode(_encoded_img, cv2.IMREAD_COLOR)
        cv2.imshow(self.__window_name, _cv_image)
        cv2.waitKey(1)

        # cv2.imshow(self.__window_name, param_data)
        # cv2.waitKey(1)


if __name__ == '__main__':

    try:
        rclpy.init(args=None)
        _test = RosTest()
        rclpy.spin(_test)
        _test.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt as error:
        print("robot __main__ : {0}".format(error))

