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

import time
import json
import signal
import asyncio
from mqtt.mqtt_client import MqttAgent
from mqtt.mqtt_data import *
from constant.constant import *

# gmqtt also compatibility with uvloop
import uvloop
asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())


class MqttManager:
    """MQTT client service class """
    def __init__(self, param_image_cb, param_stt_cb):
        self.image_pub_func = param_image_cb
        self.stt_pub_func = param_stt_cb
        self._previous_time = time.time()
        self.agent_handler = None
        self.agent_id = None
        self.loop = None
        self.agent_info_list = []
        self.heartbeat_times = 0
        self.STOP = asyncio.Event()
        self._setup()
        self._start()

    def _setup(self):

        self.agent_id = self._get_agent_id()
        print("setup completed. {}".format(self.agent_id), flush=True)

    def _get_agent_id(self):
        return EDGE_SERVER_SUB_ID

    def ask_exit(self, *args):
        self.STOP.set()

    def _start(self):

        try:
            self.loop = asyncio.get_event_loop()
            self.loop.add_signal_handler(signal.SIGINT, self.ask_exit)
            self.loop.add_signal_handler(signal.SIGTERM, self.ask_exit)
            self.loop.run_until_complete(self.ready())

        except Exception as e:
            print("start exception coroutine. reason [{}]".format(e))

        self.loop.close()

    async def ready(self):
        """ 준비과정, 신규등록 여부 확인 및 MQTT 서버 접속 """

        print("connect mqtt. [{}][{}][{}]".format(ETRI_MQTT_SERVER_URL,
                                                  ETRI_MQTT_SERVER_PORT,
                                                  self.agent_id), flush=True)

        self.agent_handler = MqttAgent(ETRI_MQTT_SERVER_URL, ETRI_MQTT_SERVER_PORT, self.agent_id)
        self.agent_id = self.agent_handler.get_agent_id() if self.agent_id is None else self.agent_id
        print("ready set agent_handler [{}]".format(self.agent_id), flush=True)

        self.agent_handler.on('request', self.on_request)
        self.agent_handler.on('response', self.on_response)
        self.agent_handler.on('disconnect', self.on_disconnect)

        await self.agent_handler.connect(ETRI_MQTT_CONN_ID, ETRI_MQTT_CONN_PASSWORD)
        print("ready success connect mqtt server.", flush=True)

        await self.STOP.wait()
        print("stopped event handler.", flush=True)
        await self.agent_handler.disconnect()
        print("disconnected mqtt client.", flush=True)

    def on_request(self, client, cmd, req, properties):
        """ 로봇으로부터 전달받은 메시지 """
        if cmd == "video/image_info":
            self.parse_send_image_info(req, properties)

        elif cmd == "stt/result":
            self.send_stt_result(str(req))

    def on_response(self, client, cmd, res, properties):
        """ Edge server가 보낸 메시지에 대한 로봇의 응답을 처리 """
        print("cmd[{}] properties[{}]".format(cmd, properties))

    def on_disconnect(self):
        """ 접속 끊겼을 시에 대한 후처리 """
        print("mqtt client! ID [{}]".format(self.agent_id))

    def parse_send_image_info(self, raw_data, properties):
        """ 데이터 검증 및 이미지 바이너리를 파일로 저장 """

        _dict_props = dict(properties['user_property'])
        _seq = int(_dict_props['seq'])
        if _seq <= 0:
            print("wrong client seq")
            return

        # 데이터가 존재하면,
        if _dict_props is not None:
            # 객체가 없으면, 최초 전달받은 AgentInfo 를 삽입
            if not self.agent_info_list:
                self.agent_info_list.append(AgentInfo(_dict_props))
            else:
                # 기존 에이전트 리스트가 존재할 시, agent_id로 검색
                for _info in self.agent_info_list:
                    if _dict_props['agent_id'] != _info.get_agent_id():
                        # 없으면, 리스트에 추가
                        self.agent_info_list.append(AgentInfo(_dict_props))
                    else:
                        # 존재하면, 데이터 업데이트
                        _info.update(_dict_props, raw_data)
                        _info.print()
                        _info.ros_callback(self.image_pub_func)

    def send_stt_result(self, result: str):
        """ STT 결과 전달 """
        print(f"stt result: {result}", flush=True)
        self.stt_pub_func(result)


## Agent information
import os
from datetime import datetime


class AgentInfo:
    """ 에이전트 정보 분석 """
    def __init__(self, param_user_properties):

        self.user_properties = param_user_properties
        self._previous_time = time.time()
        self.is_image_save = False
        self._local_saved_path = "/data1/backup/images"
        self.raw_data = b""
        self._loop_count = 0
        self._setup()

    def _setup(self):
        _str_agent_id = self.get_agent_id()
        _today = datetime.today().strftime("%Y%m%d")
        # 더이상 저장역할을 하지 않으므로, 주석처리
        # self._local_saved_path = "/data1/backup/images/"+_str_agent_id+"/"+_today
        # self.mkdir()

    def mkdir(self):
        print("create dir [{}]".format(self._local_saved_path), flush=True)
        os.makedirs(self._local_saved_path, exist_ok=True)

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
                          len(self.raw_data)), flush=True)

        self._previous_time = _elapsed_time

        if self.is_image_save is True:
            self._save_image(self.raw_data)

    def _save_image(self, param_b64_data):

        self.check_date()

        (dt, micro) = datetime.utcnow().strftime('%Y%m%d%H%M%S.%f').split('.')
        dt = "%s%03d.jpeg" % (dt, int(micro) / 1000)
        _file_path = self._local_saved_path+"/"+dt

        with open(_file_path, "wb") as f:
            f.write(param_b64_data)

    def check_date(self):

        _str_agent_id = self.get_agent_id()
        _today = datetime.today().strftime("%Y%m%d")
        self._local_saved_path = "/data1/backup/images/"+_str_agent_id+"/"+_today

        if os.path.isdir(self._local_saved_path) is False:
            self.mkdir()

    def ros_callback(self, param_call_func):
        """ Queue image rawdata """
        param_call_func(self.user_properties, self.raw_data)


## ROS
import rclpy
from rclpy.node import Node
from aai4r_edge_interfaces.msg import RobotImageInfo
from std_msgs.msg import String


class RosManager(Node):

    def __init__(self):
        super().__init__('RosManager')
        self._is_image_show = False
        self.image_publisher = None
        self.result_subscriber = None
        self._setup()

    def _setup(self):
        self.image_publisher = self.create_publisher(RobotImageInfo, ROS_TOPIC_EDGE_SEND_IMAGE, 1)
        self.stt_result_publisher = self.create_publisher(String, ROS_TOPIC_EDGE_STT_RESULT, 10)

    def callback_image_publish(self, param_user_properties, param_image):

        _msg = RobotImageInfo()
        _msg.stamp = self.get_clock().now().to_msg()
        _msg.agent_id = param_user_properties["agent_id"]
        _msg.format = param_user_properties["format"]
        _msg.hash = ""
        _msg.seq_id = int(param_user_properties["seq"])
        _msg.height = int(param_user_properties["height"])
        _msg.width = int(param_user_properties["width"])
        _msg.distance = float(param_user_properties["distance"])
        _msg.zone = 1
        _msg.data = param_image
        _msg.params = json.dumps({'meal_start_time': time.time()})

        self.image_publisher.publish(_msg)

    def send_stt_result(self, param_result: str):
        msg = String()
        msg.data = param_result
        self.stt_result_publisher.publish(msg)


if __name__ == '__main__':

    try:
        rclpy.init(args=None)
        _ros_manager = RosManager()
        _mqtt = MqttManager(param_image_cb=_ros_manager.callback_image_publish,
                            param_stt_cb=_ros_manager.send_stt_result)
        _ros_manager.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt as error:
        print("robot __main__ : {0}".format(error))







