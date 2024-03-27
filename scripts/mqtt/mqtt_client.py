# -*- coding:utf-8 -*-

import time
import asyncio
import json
import uuid
import datetime
import traceback
import logging

from gmqtt import Client as MQTTClient
from .mqtt_data import *


class MqttAgent:
    """
    Description
    -----------
    Mqtt Client 클래스
    생성자에 agent_id는 Cloud에서 할당받는 agentId값을 사용
    초기 등록시에는 할당받는 ID가 없기에 입력받은 nickname을 사용해서 등록 (uuid)
    """
    def __init__(self, host, port, agent_id):

        self.host = host
        self.port = port
        self.agent_id = None
        self.client_id = None
        self.my_topic = None
        self.set_agent_id(agent_id)
        self._logger = logging.getLogger("mqtt")
        self._logger.setLevel(logging.INFO)
        self.requests = {}
        self.client = MQTTClient(client_id=self.client_id,
                                 clean_session=True,
                                 optimistic_acknowledgement=True,
                                 will_message=None,
                                 maximum_packet_size=2000000,
                                 receive_maximum=24000)

        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect
        self.client.on_subscribe = self._on_subscribe

    def is_connected(self):
        if self.client is None:
            return False

        return self.client.is_connected()

    # 에이전트 ID 설정
    def set_agent_id(self, agent_id):

        self.agent_id = str(agent_id)
        self.client_id = '{}'.format(self.agent_id)
        self.my_topic = '{}'.format(self.agent_id)
        print("my topic is {}".format(self.my_topic), flush=True)

    # 에이전트 ID 획득
    def get_agent_id(self):
        return self.agent_id

    # 서버 접속
    async def connect(self, id=None, pwd=None):

        self._logger.info("host[{}] port[{}]".format(self.host, self.port))
        if pwd:
            self.client.set_auth_credentials(id if id else self.client_id, pwd)
        await self.client.connect(host=self.host, port=self.port, ssl=False, keepalive=10)
        self._logger.info("done.")

    # 서버접속 해제
    async def disconnect(self):
        await self.client.disconnect()

    # Callback function 등록
    def on(self, event, handler):

        if 'request' == event: self.on_request = handler
        if 'response' == event: self.on_response = handler
        if 'disconnect' == event: self.on_disconnect = handler
        if 'error' == event: self.on_error = handler

    # 요청과 응답이 있는 메시지
    # 반환되는 값은 메시지 식별 ID로 on_response에 의해서 응답된 메시지 식별용
    def send(self, data, wait_sec=10, qos=1):

        cmd = mqtt_get_command(data)

        if not cmd:
            print("failed - wrong cmd: type[{}]".format(type(data).__name__))
            return None

        try:
            reply_to = '/{}/response'.format(self.my_topic)
            corr_id = str(uuid.uuid4())
            if hasattr(data, 'agentId'):
                data.agentId = self.agent_id
            payload = Request(reply_to, corr_id, data)
            self._push_raw(cmd, payload, qos)
            now = datetime.datetime.now()
            self.requests[corr_id] = {'name': cmd, 'timeout': now + datetime.timedelta(seconds=wait_sec)}
            return corr_id
        except ValueError as ex:
            print("failed({}): {}".format(cmd, ex))

        return None

    # 응답이 없는 이벤트성 메시지
    def push(self, data, custom_topic=None, qos=1, properties=None):

        cmd = mqtt_get_command(data) if custom_topic is None else custom_topic
        if not cmd:
            print("failed - wrong cmd: type[{}]".format(type(data).__name__))
            return

        self._push_raw(cmd, data, qos, properties)

    # 성공 응답 보냄
    def reply_success(self, req, data='', qos=1):

        res = Response(req.correlationId, 0, '', data)
        self._publish(req.replyTo, res, qos)

    # 실패 응답 보냄
    def reply_fail(self, req, status, message, qos=1):

        res = Response(req.correlationId, status, message, None)
        self._publish(req.replyTo, res, qos)

    # Topic 구독
    def subscribe(self, topic, qos=1):
        self.client.subscribe(topic, qos)

    def _push_raw(self, cmd, data, qos=1, properties=None):

        server = mqttComponentMap[EtriCloudMessage(cmd)]
        topic = '/{}/{}'.format(server.value, cmd)
        self._publish(topic, data, qos, properties)

    # Topic 발행
    def _publish(self, topic, data, qos=1, properties=None):
        self.client.publish(topic, data, qos, user_property=properties)

    # 서버 접속 시 반환되는 콜백함수
    def _on_connect(self, client, flags, rc, properties):
        self._logger.info("client [{}]".format(client._client_id))
        client.subscribe('/{}/#'.format(self.my_topic), qos=0)

    # Topic 데이터 분석
    def _on_message(self, client, topic, payload, qos, properties):
        try:
            #recv_json = json.loads(payload)
            idx = topic.find('/', 1)
            if 0 > idx:
                self._logger.error("invalid data: topic[{}]".format(topic))
                return

            cmd = topic[idx + 1:]

            if 'response' == cmd:
                pass
                # if hasattr(self, 'on_response') and callable(self.on_response):
                #     recv_data = Response(**recv_json)
                #     corr_id = recv_data.correlationId
                #     if corr_id in self.requests:
                #         cmd = EtriAgentMessage(self.requests[corr_id]['name'])
                #         self._logger.debug("MqttAgent message conv: cmd[{}] data[{}]".format(cmd, recv_data.response))
                #         recv_data.response = mqtt_message_convert(cmd, recv_data.response)
                #         self.on_response(self, cmd, recv_data, properties)
            else:
                if hasattr(self, 'on_request') and callable(self.on_request):
                    #cmd = EtriCloudMessage(cmd)
                    #recv_data = mqtt_message_convert(cmd, recv_json)
                    #self._logger.debug("MqttAgent message conv: cmd[{}] data[{}]".format(cmd, recv_data))
                    self.on_request(self, cmd, payload, properties)
                    #print('recv_data : [{}/{}/{}]'.format(recv_data.seq, recv_data.format, len(recv_data.data)))

            # else:
            #     if hasattr(self, 'on_request') and callable(self.on_request):
            #         cmd = EtriCloudMessage(cmd)
            #         recv_data = mqtt_request_convert(cmd, recv_json)
            #         self.on_request(self, cmd, recv_data)

        except ValueError as ex:
            pass
            #self._logger.error("MqttAgent on_message - internal error: cmd[{}] error[{}]".format(topic, ex))
            #print(traceback.format_exc())

    # 서버 접속 해제 시 반환되는 콜백함수
    def _on_disconnect(self, client, packet, exc=None):
        if hasattr(self, 'on_disconnect') and callable(self.on_disconnect): self.on_disconnect()

    #def _on_subscribe(self, client, mid, qos):
    def _on_subscribe(self, client, mid, qos, properties):
        pass

    def _error(self, event, data):

        if not hasattr(self, 'on_error') or not callable(self.on_error): return
        self.on_error(event, data)
