#!/usr/bin/python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import rclpy.qos as qos
from sensor_msgs.msg import Image

class RosTest(Node):

    def __init__(self):
        super().__init__('RosTest')
        self.__window_name = "ETRI EDGE DEBUG IMG"
        self.img_sub = None
        self._setup()

    def _setup(self):

        _profile = qos.QoSProfile(history=qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL)
        self.img_sub = self.create_subscription(Image,
                                                "/camera/rgb/image_raw",
                                                self.msg_callback,
                                                qos_profile=_profile)

    def msg_callback(self, msg):

        print("received")

if __name__ == '__main__':

    try:
        print("ok")
        rclpy.init(args=None)
        _test = RosTest()
        print("create rostest..")
        rclpy.spin(_test)
        _test.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt as error:
        print("robot __main__ : {0}".format(error))