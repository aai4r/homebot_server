#!/usr/bin/python
# -*- coding: utf-8 -*-

###
# @file broadcast_publisher.py
#
# @brief message broadcast 실험
#
# @section
# Broadcaster
# : MQTT 메시지 구독 및 발행

import os
import sys
#----------------------------------------------------------------
# local 테스트 진행 시
# 현재위치의 상대경로를 시스템에서 인식시켜주기 위한 작업
path = os.path.abspath(os.path.dirname(sys.argv[0]))
sys.path.append(path[:path.rfind('/')])
#----------------------------------------------------------------

import time
import signal
import asyncio
import logging
from constant.constant import *
from mqtt.mqtt_client import MqttAgent
from mqtt.mqtt_data import *

# uvloop 사용하지 않음. 메시지 전달주기를 무너뜨리는 현상 확인
# gmqtt also compatibility with uvloop
# import uvloop
# asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())


class MqttHandler(object):
    """ MQTT client service class """

    def __init__(self):
        self._logger = logging.getLogger("my")
        self._logger.setLevel(logging.DEBUG)
        stream_hander = logging.StreamHandler()
        self._logger.addHandler(stream_hander)
        self._previous_time = time.time()
        self.agent_handler = None
        self._is_connected = False
        self.agent_id = self._get_agent_id()
        self.agent_info_list = []
        self.heartbeat_times = 0
        self.loop_count = 0

    def _get_agent_id(self):
        return EtriComponent.EDGE_MONITOR.value

    def is_connected(self):
        return self._is_connected

    async def close(self):
        await self.agent_handler.disconnect()
        self._is_connected = False

    async def run(self):
        """ MQTT 접속 """

        self._logger.debug("startup mqtt..")

        try:
            await asyncio.create_task(self.ready())
            await asyncio.create_task(self.tick_tok())

        except Exception as e:
            self._logger.exception("start exception coroutine. reason [{}]".format(e))
        finally:
            self._logger.debug("close event loop.")

    async def ready(self):
        """ MQTT 서버 접속 """

        self._logger.debug('connect mqtt. [{}][{}][{}]'.format("192.168.112.230",
                                                               ETRI_MQTT_SERVER_PORT,
                                                               self.agent_id))

        self.agent_handler = MqttAgent("192.168.112.230",
                                       ETRI_MQTT_SERVER_PORT,
                                       self.agent_id)

        self.agent_id = self.agent_handler.get_agent_id() if self.agent_id is None else self.agent_id
        self._logger.debug("ready set agent_handler [{}]".format(self.agent_id))

        self.agent_handler.on('request', self.on_request)
        self.agent_handler.on('response', self.on_response)
        self.agent_handler.on('disconnect', self.on_disconnect)

        await self.agent_handler.connect(ETRI_MQTT_CONN_ID, ETRI_MQTT_CONN_PASSWORD)
        self._logger.debug("ready success connect mqtt server.")
        self._is_connected = True

    async def tick_tok(self):

        _seq = 0
        while True:
            _push_props = (('seq', str(_seq)), ('data', '1'))
            self.agent_handler._publish(topic="/edge_broadcaster/stat",
                                        data=None,
                                        qos=0,
                                        properties=_push_props)
            self._logger.debug("tick {}".format(_push_props))
            await asyncio.sleep(1.0)
            _seq = _seq+1

    def on_request(self, client, cmd, req, properties):
        """ 로봇으로부터 전달받은 메시지 """
        pass

    def on_response(self, client, cmd, res, properties):
        """ Edge server가 보낸 메시지에 대한 로봇의 응답을 처리 """
        self._logger.debug("cmd[{}] properties[{}]".format(cmd, properties))

    def on_disconnect(self):
        """ 접속 끊겼을 시에 대한 후처리 """
        self._logger.debug("disconnect mqtt client. ID [{}]".format(self.agent_id))


_event_loop_stopper = asyncio.Event()


def exit_event_loop(*args):
    _event_loop_stopper.set()


if __name__ == '__main__':

    try:
        _event_loop = asyncio.get_event_loop()
        _event_loop.add_signal_handler(signal.SIGINT, exit_event_loop)
        _event_loop.add_signal_handler(signal.SIGTERM, exit_event_loop)

        _pub_task = MqttHandler()
        _event_loop.run_until_complete(_pub_task.run())

        _event_loop_stopper.wait()
        print("stopped event loop.")
        _pub_task.close()
        print("disconnected mqtt client.")
        _event_loop.close()

    except KeyboardInterrupt as error:
        print("robot __main__ : {0}".format(error))