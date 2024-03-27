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

## Common
import time
import os
from datetime import datetime
import threading
import rclpy
import json
import csv
from rclpy.node import Node
import rclpy.qos as qos
from std_msgs.msg import String
from std_msgs.msg import Empty
from constant.constant import *


class AppMain(threading.Thread):
    def __init__(self, param_mqtt_client):
        threading.Thread.__init__(self)
        self._ros_manager = RosManager(param_mqtt_client)

    def run(self):
        rclpy.spin(self._ros_manager)
        self._ros_manager.destroy_node()


class RosManager(Node):

    def __init__(self, param_mqtt_client):
        super().__init__('RosManager')
        self._mqtt_client = param_mqtt_client
        self._is_image_show = False
        self.talk_trigger = None
        self.result_subscriber = None
        self._facial_attribute = ""
        self._fashion_attribute = ""
        self._local_saved_path = "/data1/backup/log"
        self._list_human_detect_id = []
        self._face_detect_count = 0
        self._setup()

    def _setup(self):

        _qos_profile = qos.QoSProfile(
            reliability=qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(String,
                                                     ROS_TOPIC_EDGE_SPEECH,
                                                     self.callback_speech,
                                                     qos_profile=_qos_profile)
        self.subscription

        self.subscription = self.create_subscription(String,
                                                     ROS_TOPIC_EDGE_EXPRESSION,
                                                     self.callback_expression,
                                                     qos_profile=_qos_profile)
        self.subscription

        self.subscription = self.create_subscription(String,
                                                     ROS_TOPIC_EDGE_MOTION_TILT,
                                                     self.callback_motion_tilt,
                                                     qos_profile=_qos_profile)
        self.subscription

        self.subscription = self.create_subscription(String,
                                                     ROS_TOPIC_EDGE_MOTION_PAN,
                                                     self.callback_motion_pan,
                                                     qos_profile=_qos_profile)
        self.subscription

        # self.talk_trigger = self.create_publisher(Empty, TOPIC_EDGE_AAI4R_TALK_TRIGGER, 1)
        # self.reset_human_detected()

        # if os.path.isdir(self._local_saved_path) is False:
        #    self.mkdir()

    def callback_situation_result(self, msg):
        """ 인식결과 반환 """

        if self._mqtt_client is None or \
                self._mqtt_client.is_connected() is False:
            return

        _dict_situation_result = {}

        try:
            _dict_situation_result = json.loads(msg.data)
        except Exception as error:
            print("parse error. reason [{}]".format(error))
            return

        _str_timestamp = ""
        _str_face_detected = "no"
        _dict_personal_context = None

        if "time" in _dict_situation_result:
            _str_timestamp = _dict_situation_result["time"]

        if "face_detected" in _dict_situation_result:
            _str_face_detected = _dict_situation_result["face_detected"]

        if "personal_context" in _dict_situation_result:
            _list_situation_info = _dict_situation_result["personal_context"]
            if len(_list_situation_info) > 0:
                _dict_personal_context = _dict_situation_result["personal_context"][0]

        # 얼굴인식이 된 경우, 유의미한 정보
        #print("[{}] face [{}] situation result [{}]".format(_str_timestamp, _str_face_detected, _dict_personal_context), flush=True)
        self._mqtt_client.send_recognition_result(_str_timestamp, _str_face_detected, _dict_personal_context)

    def callback_facial(self, msg):

        if self._mqtt_client is None or self._mqtt_client.is_connected() is False:
            return

        info = json.loads(msg.data)

        if isinstance(info, list):
            if len(info) <= 0: return
            info = info[0]

        if isinstance(info, dict):
            if "facial" in info:
                if len(info["facial"]) <= 0: return
                #print(f"information: {info['facial']}", flush=True)
                self._face_detect_count += 1
                # 하나의 인식결과에 따른 결과만 산출한다.
                # 비슷한 시기의 데이터를 중복으로 보내면 talk result 결과 속도에 영향을 끼칠 것으로 판단
                if len(self._list_human_detect_id) == 0:
                    print(f"fired talk triggering..", flush=True)
                    self._facial_attribute = str(msg.data)
                    self.talk_trigger.publish(Empty())
                    self._list_human_detect_id.append(info['facial'])

                if self._face_detect_count == 5:
                    self.reset_human_detected()

    def callback_talk_result(self, msg):

        if self._mqtt_client is None or self._mqtt_client.is_connected() is False:
            return

        print(f"fashion talk result : {msg.data}", flush=True)
        self._mqtt_client.send_fashion_talk(msg.data)
        try:
            self._save_log(msg.data)
        except Exception as error:
            print(f"error reason: {error}")
        self._list_human_detect_id.clear()

    def callback_fashion_attribute(self, msg):
        self._fashion_attribute = str(msg.data)

    def reset_human_detected(self):
        self._list_human_detect_id.clear()
        self._face_detect_count = 0

    def _save_log(self, sentence):

        _today = datetime.today().strftime("%Y%m%d%H:%M:%S")

        _data = {
            "timestamp": _today,
            "facial": self._facial_attribute,
            "fashion": self._fashion_attribute,
            "talk": sentence
        }

        _file_name = self._local_saved_path + "/analysis_" + _today + ".log"
        with open(_file_name, "w", encoding="utf-8") as f:
            f.write(json.dumps(_data, ensure_ascii=True))
            # writer = csv.DictWriter(f, delimiter='\t')
            # writer.writerows(_data)

    def mkdir(self):
        print("create dir [{}]".format(self._local_saved_path), flush=True)
        os.makedirs(self._local_saved_path, exist_ok=True)

    def callback_speech(self, msg):

        if self._mqtt_client is None or self._mqtt_client.is_connected() is False:
            return

        print(f"request speech : {msg.data}", flush=True)
        self._mqtt_client.send_event("speech", msg.data)

    def callback_expression(self, msg):

        if self._mqtt_client is None or self._mqtt_client.is_connected() is False:
            return

        print(f"request expression : {msg.data}", flush=True)
        self._mqtt_client.send_event("expression", msg.data)

    def callback_motion_tilt(self, msg):

        if self._mqtt_client is None or self._mqtt_client.is_connected() is False:
            return

        print(f"request tilt motion : {msg.data}", flush=True)
        self._mqtt_client.send_event("motion_tilt", msg.data)
    def callback_motion_pan(self, msg):

        if self._mqtt_client is None or self._mqtt_client.is_connected() is False:
            return

        print(f"request pan motion : {msg.data}", flush=True)
        self._mqtt_client.send_event("motion_pan", msg.data)

## MQTT
import time
import signal
import asyncio
from mqtt.mqtt_client import MqttAgent

# uvloop 사용하지 않음. 메시지 전달주기를 무너뜨리는 현상 확인
# gmqtt also compatibility with uvloop
# import uvloop
# asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())


class MqttManager:
    """MQTT client service class """

    def __init__(self):
        self._previous_time = time.time()
        self.agent_handler = None
        self._is_connected = False
        self.agent_id = self._get_agent_id()
        self.agent_info_list = []
        self._event_loop_stopper = asyncio.Event()
        self.heartbeat_times = 0
        self.loop_count = 0

    def _get_agent_id(self):
        return EDGE_SERVER_PUB_ID

    def exit_event_loop(self, *args):
        self._event_loop_stopper.set()

    def is_connected(self):
        return self._is_connected

    async def close(self):
        await self.agent_handler.disconnect()
        self._is_connected = False

    def run(self):
        """ MQTT 접속 """

        print("startup mqtt..")

        try:
            _event_loop = asyncio.get_event_loop()
            _event_loop.add_signal_handler(signal.SIGINT, self.exit_event_loop)
            _event_loop.add_signal_handler(signal.SIGTERM, self.exit_event_loop)
            _event_loop.run_until_complete(self.ready())
        except Exception as e:
            print("start exception coroutine. reason [{}]".format(e))
        finally:
            _event_loop.close()
            print("close event loop.")

    async def ready(self):
        """ MQTT 서버 접속 """

        print('connect mqtt. [{}][{}][{}]'.format(ETRI_MQTT_SERVER_URL,
                                                  ETRI_MQTT_SERVER_PORT,
                                                  self.agent_id))

        self.agent_handler = MqttAgent(ETRI_MQTT_SERVER_URL,
                                       ETRI_MQTT_SERVER_PORT,
                                       self.agent_id)

        self.agent_id = self.agent_handler.get_agent_id() if self.agent_id is None else self.agent_id
        print("ready set agent_handler [{}]".format(self.agent_id))

        self.agent_handler.on('request', self.on_request)
        self.agent_handler.on('response', self.on_response)
        self.agent_handler.on('disconnect', self.on_disconnect)

        await self.agent_handler.connect(ETRI_MQTT_CONN_ID, ETRI_MQTT_CONN_PASSWORD)
        print("ready success connect mqtt server.")
        self._is_connected = True

        print("wait event loop..")
        await self._event_loop_stopper.wait()
        print("stopped event loop.")
        await self.agent_handler.disconnect()
        print("disconnected mqtt client.")

    def on_request(self, client, cmd, req, properties):
        """ 로봇으로부터 전달받은 메시지 """
        pass

    def on_response(self, client, cmd, res, properties):
        """ Edge server가 보낸 메시지에 대한 로봇의 응답을 처리 """
        print("cmd[{}] properties[{}]".format(cmd, properties))

    def on_disconnect(self):
        """ 접속 끊겼을 시에 대한 후처리 """
        print("disconnect mqtt client. ID [{}]".format(self.agent_id))

    def send_recognition_result(self, param_timestamp, param_face_detect, param_dict_personal_context):
        """ 인식결과 정보 발행 """

        _mask = ""
        _cup = ""

        if param_face_detect == "no":
            pass
        elif param_dict_personal_context is None:
            pass
        else:
            if "mask" in param_dict_personal_context:
                _mask = str(param_dict_personal_context["mask"])
            if "has_cup" in param_dict_personal_context:
                _cup = str(param_dict_personal_context["has_cup"])

        _push_props = (
            ("time", param_timestamp),
            ("face_detect", param_face_detect),
            ("mask", _mask),
            ("cup", _cup)
        )

        print(_push_props, flush=True)
        self.agent_handler.push(data="", custom_topic=RECOGNITION_RESULT, qos=0, properties=_push_props)

    def send_fashion_talk(self, message):
        _push_props = (("time",""))
        self.agent_handler.push(data=message, custom_topic=FASHION_TALK, qos=0, properties=_push_props)

    def send_event(self, service_type: str, message: str):

        payload = {}
        if service_type == 'speech': payload = {'cmd': 'request', 'tts': message}
        elif service_type == 'expression': payload = {'cmd': 'request', 'expression': message}
        elif service_type == 'motion_tilt': payload = {'direction': 'TILT', 'degree': message}
        elif service_type == 'motion_pan': payload = {'direction': 'PAN', 'degree': message}

        _request = {
            'msg_id': '',
            'robot_id': 'robot_mode',
            'send_from': 'tm',
            'service': service_type,
            'msg_type': '',
            'payload': json.dumps(payload, ensure_ascii=False)
        }

        self.agent_handler.push(data=_request, custom_topic=REQUEST_EVENT, qos=0, properties=("time", ""))

# -----------------------------------------------------------------------------------------------------------------------
# global
# -----------------------------------------------------------------------------------------------------------------------

_event_loop_stopper = asyncio.Event()

def exit_event_loop(*args):
    _event_loop_stopper.set()

def launch_mqtt_module(param_mqtt_manager):
    """ MQTT 접속 """

    print("startup mqtt...")

    try:
        _event_loop = asyncio.get_event_loop()
        _event_loop.add_signal_handler(signal.SIGINT, exit_event_loop)
        _event_loop.add_signal_handler(signal.SIGTERM, exit_event_loop)
        _event_loop.run_until_complete(param_mqtt_manager.ready())

    except Exception as e:
        print("start exception coroutine. reason [{}]".format(e))
    finally:
        _event_loop.close()
        print("close event loop..")


if __name__ == '__main__':

    try:
        rclpy.init(args=None)
        _mqtt = MqttManager()
        _app_thread = AppMain(_mqtt)
        _app_thread.daemon = True
        _app_thread.start()
        _mqtt.run()
        rclpy.shutdown()

    except KeyboardInterrupt as error:
        print("robot __main__ : {0}".format(error))
