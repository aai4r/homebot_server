#!/usr/bin/python
# -*- coding: utf-8 -*-

###
# @file edge_transfer_exampled.py
#
# @brief edge 서버의 broker 에서 구독 및 발행되는 메시지를 다룬다.
#
# @section
# MqttManager
# : MQTT 메시지 구독 및 발행
#
# ImageSubscriber
# : 이미지 구독
# : 데이터 검증(Debug)
# UserReactionAnalyer
# : 사용자 반응 분석결과 발행(Debug)

import os
import sys
#----------------------------------------------------------------
# local 테스트 진행 시
# 현재위치의 상대경로를 시스템에서 인식시켜주기 위한 작업
path = os.path.abspath(os.path.dirname(sys.argv[0]))
sys.path.append(path[:path.rfind('/')])
#----------------------------------------------------------------

import time
import logging
import random
import signal
import asyncio
from mqtt.mqtt_client import MqttAgent
from mqtt.mqtt_data import *

# gmqtt also compatibility with uvloop
import uvloop
asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())

# MQTT Properties
ETRI_MQTT_SERVER_URL='127.0.0.1'
ETRI_MQTT_CONN_ID='clobot'
ETRI_MQTT_CONN_PASSWORD='ansdufdj1*'
SERVER_ID='edge_monitor'
FILE_SAVE_PATH='/tmp/_test.jpg'


class MqttManager:
    """MQTT client service class """
    def __init__(self, param_callback, param_queue):
        self.image_pub_func = param_callback
        self._queue = param_queue
        self._logger = logging.getLogger("mqtt")
        self._previous_time = time.time()
        self._is_image_save = False
        self._is_image_view = True
        self._setup()
        self._start()

    def _setup(self):

        self.MQTT_SERVER_IP = ETRI_MQTT_SERVER_URL
        self.MQTT_SERVER_PORT = 1883  # 5685
        self.MQTT_CONN_ID = ETRI_MQTT_CONN_ID
        self.MQTT_CONN_PASSWORD = ETRI_MQTT_CONN_PASSWORD
        self.agent_id = self._get_agent_id()
        self.agent_info_list = []
        self.heartbeat_times = 0
        self.agent_handler = None
        self.loop = None
        self.STOP = asyncio.Event()
        self._logger.debug("setup completed.")

    def _get_agent_id(self):
        return SERVER_ID

    def ask_exit(self, *args):
        self.STOP.set()

    def _start(self):

        try:
            self.loop = asyncio.get_event_loop()
            self.loop.add_signal_handler(signal.SIGINT, self.ask_exit)
            self.loop.add_signal_handler(signal.SIGTERM, self.ask_exit)
            #self.loop.create_task(self.consume(self._queue))
            self.loop.run_until_complete(self.ready())

        except Exception as e:
            self._logger.error("start exception coroutine. reason [{}]".format(e))

        self.loop.close()

    async def consume(self, param_queue):
        """ Queue """
        while True:
            _msg = await param_queue.get()
            if _msg is not None:
                print("queue get [{}]".format(_msg))
                self.send_recognition_result(_msg)

            await asyncio.sleep(random.random())

    async def ready(self):
        """ 준비과정, 신규등록 여부 확인 및 MQTT 서버 접속 """

        self._logger.debug('connect mqtt.')

        self.agent_handler = MqttAgent(self.MQTT_SERVER_IP, self.MQTT_SERVER_PORT, self.agent_id)
        self.agent_id = self.agent_handler.get_agent_id() if self.agent_id is None else self.agent_id
        self._logger.debug("ready set agent_handler [{}]".format(self.agent_id))

        self.agent_handler.on('request', self.on_request)
        self.agent_handler.on('response', self.on_response)
        self.agent_handler.on('disconnect', self.on_disconnect)

        await self.agent_handler.connect(self.MQTT_CONN_ID, self.MQTT_CONN_PASSWORD)
        self._logger.debug("ready success connect mqtt server.")

        await self.STOP.wait()
        self._logger.info("stopped event handler.")
        await self.agent_handler.disconnect()
        self._logger.info("disconnected mqtt client.")

    def on_request(self, client, cmd, req, properties):
        """ 로봇으로부터 전달받은 메시지 """
        self._logger.debug("cmd[{}] req[{}]".format(cmd, req))

        if cmd == EtriCloudMessage.SENDIMAGE:
            self.parse_send_image_info(req, properties)

    def on_response(self, client, cmd, res, properties):
        """ Edge server가 보낸 메시지에 대한 로봇의 응답을 처리 """
        self._logger.debug("cmd[{}] properties[{}]".format(cmd, properties))

    def on_disconnect(self):
        """ 접속 끊겼을 시에 대한 후처리 """
        self._logger.debug("mqtt client! ID [{}]".format(self.agent_id))

    def parse_send_image_info(self, raw_data, properties):
        """ 데이터 검증 및 이미지 바이너리를 파일로 저장 """
        _dict_props = dict(properties['user_property'])
        _seq = int(_dict_props['seq'])
        if _seq <= 0:
            self._logger.error("wrong client seq")
            return

        # 데이터가 존재하면,
        if _dict_props is not None:
            # 객체가 없으면, 최초 전달받은 AgentInfo 를 삽입
            if not self.agent_info_list:
                self.agent_info_list.append(AgentInfo(_dict_props, self._is_image_save, self._is_image_view))
            else:
                # 기존 에이전트 리스트가 존재할 시, agent_id로 검색
                for _info in self.agent_info_list:
                    if _dict_props['agent_id'] != _info.get_agent_id():
                        # 없으면, 리스트에 추가
                        self.agent_info_list.append(AgentInfo(_dict_props, self._is_image_save, self._is_image_view))
                    else:
                        # 존재하면, 데이터 업데이트
                        _info.update(_dict_props, raw_data)
                        _info.print()
                        _info.ros_callback(self.image_pub_func)

    def send_recognition_result(self, param_personal_vo):
        """ 인식결과 정보 발행 """

        # user properties
        _push_props = (
            ('id', param_personal_vo.id),
            ('mask', param_personal_vo.mask)
        )
        print("send msg [{}]".format(_push_props))
        self.agent_handler.push(data=None, custom_topic=SEND_SITUATION_RESULT_TOPIC, qos=0, properties=_push_props)

## Agent information

import cv2
import numpy as np


class AgentInfo:
    """ 에이전트 정보 분석 """
    def __init__(self, param_user_properties, param_is_image_save, param_is_image_view):

        self.user_properties = param_user_properties
        self.is_image_save = param_is_image_save
        self.is_image_view = param_is_image_view
        self._file_save_path = FILE_SAVE_PATH
        self._loop_count = 0
        self._previous_time = time.time()
        self.raw_data = b""

    def get_agent_id(self):
        return self.user_properties['agent_id']

    def update(self, param_user_properties, param_raw_data):

        self.user_properties = param_user_properties
        self.raw_data = param_raw_data
        self._loop_count += 1

    def print(self):

        _elapsed_time = time.time()
        if self._loop_count % 20 == 0:
            print("time interval :[{}] agent id: [{}] seq: [{}] recv size: [{}]"
                  .format(_elapsed_time - self._previous_time,
                          self.user_properties['agent_id'],
                          self.user_properties['seq'],
                          len(self.raw_data)))

        self._previous_time = _elapsed_time

        if self.is_image_view is True:
            self._view_image(self.raw_data)

        if self.is_image_save is True:
            self._save_image(self.raw_data)

    def _save_image(self, param_b64_data):

        with open(self._file_save_path, "wb") as f:
            f.write(param_b64_data)

    def _view_image(self, param_b64_data):

        _np_array_img = np.frombuffer(param_b64_data, dtype=np.uint8)
        _cv2_image = cv2.imdecode(_np_array_img, cv2.IMREAD_COLOR)

        if _cv2_image is None:
            print("image not found")
            return

        cv2.imshow('robot camera_view', _cv2_image)
        cv2.waitKey(1)

    def ros_callback(self, param_call_func):
        """ Queue image rawdata """
        param_call_func(self.user_properties, self.raw_data)


## ROS

import rclpy
import json
from rclpy.node import Node
import rclpy.qos as qos
#from sensor_msgs.msg import Image
from constant.constant import *
#from sensor_msgs.msg import CompressedImage
from aai4r_edge_interfaces.msg import RobotImageInfo
#from cv_bridge import CvBridge


class RosManager(Node):

    def __init__(self, param_queue):
        super().__init__('RosManager')
        self._logger = logging.getLogger('ros2')
        self._queue = param_queue
        self._is_image_show = False
        self.image_publisher = None
        self.result_subscriber = None
        self._setup()

    def _setup(self):

        _profile = qos.QoSProfile(history=qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL)
        self.image_publisher = self.create_publisher(
            RobotImageInfo,
            "/camera/robot_image_info",
            _profile
        )

    def callback_image_publish(self, param_user_properties, param_image):

        #print("compressed image published")
        #_image_data = self.bridge.cv2_to_compressed_imgmsg(np.array(param_cv_image))
        #print(self.get_clock().now().to_msg())

        # _image_msg = CompressedImage()
        # _image_msg.header.stamp = self.get_clock().now().to_msg()
        # _image_msg.format = "jpeg"
        # _image_msg.data = param_image
        _msg = RobotImageInfo()
        _msg.agent_id = param_user_properties["agent_id"]
        _msg.format = param_user_properties["format"]
        _msg.hash = ""
        _msg.seq_id = int(param_user_properties["seq"])
        _msg.height = int(param_user_properties["height"])
        _msg.width = int(param_user_properties["width"])
        _msg.distance = 0.0
        _msg.zone = 1
        _msg.data = param_image

        #print(param_user_properties)

        # _image_msg = Image()
        # _image_msg.header.stamp = self.get_clock().now().to_msg()
        # _image_msg.height = int(param_user_properties["height"])
        # _image_msg.width = int(param_user_properties["width"])
        # _image_msg.encoding = "bgr8"
        # _image_msg.is_bigendian = False
        # _image_msg.step = _image_msg.width * 3
        # _image_msg.data = param_image

        self.image_publisher.publish(_msg)

    def show_image(self, param_cv_image):

        cv2.imshow('robot camera_view', param_cv_image)
        cv2.waitKey(1)


if __name__ == '__main__':

    try:
        rclpy.init(args=None)
        _situation_result_queue = asyncio.Queue()
        _ros_manager = RosManager(param_queue=_situation_result_queue)
        _mqtt = MqttManager(param_callback=_ros_manager.callback_image_publish, param_queue=_situation_result_queue)
        _ros_manager.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt as error:
        print("robot __main__ : {0}".format(error))







