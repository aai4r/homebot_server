# -*- coding:utf-8 -*-

import enum
import re

from dataclasses import *
from typing import Any, List

## Common Enum


class EtriComponent(enum.Enum):
    EDGE_BROADCASTER = 'edge_broadcaster'
    EDGE_MONITOR = 'edge_monitor'
    EDGE_PUBLISHER = 'edge_publisher'
    ROBOT_HANCOM_QI_1 = 'hancom_qi_1'
    AGNET_LG_HOMEBOT_1 = 'lg_homebot_1'
    AGNET_LG_HOMEBOT_2 = 'lg_homebot_2'
    AGENT_LG_HOMEBOT = 'server_manager'


class EtriCloudMessage(enum.Enum):
    SENDIMAGE = 'video/image_info'
    RECOGNITION_RESULT = 'video/result'
    STAT = 'stat'
    FASHION_TALK = 'fashion/talk'
    STT_RESULT = 'stt/result'
    REQUEST_EVENT = 'event'


class EtriAgentMessage(enum.Enum):
    HEARTBEAT = 'heartbeat'
    STARTED = 'started'
    MONITORING_DATA = 'monitoringData'
    LOG = 'log'
    STAT = 'stat'  # pageView, naviHistory 통합. category 영역을 보고 판단해야 함
    SENDIMAGE = 'video/image_info'
    RECOGNITION_RESULT = 'video/result'
    STT_RESULT = 'stt/result'


mqttComponentMap = {
    EtriCloudMessage.RECOGNITION_RESULT: EtriComponent.ROBOT_HANCOM_QI_1,
    EtriCloudMessage.STAT: EtriComponent.EDGE_BROADCASTER,
    EtriCloudMessage.FASHION_TALK: EtriComponent.ROBOT_HANCOM_QI_1,
    EtriCloudMessage.REQUEST_EVENT: EtriComponent.AGENT_LG_HOMEBOT
}


# 데이터 클래스 명칭 정의시 유의사항
# * 클래스명은 반드시 카멜 표기법으로된 명령어로 사용하며 파스칼표기법으로 변환해서 사용
# * 클래스명 뒤에는 2글자로 클래스의 속성을 표시함
#    - RQ : 요청용 데이터 클래스
#    - RS : 응답용 데이터 클래스
#    - EV : 이벤트용 데이터 클래스
# 예) 명령어 : RequestRegistration --> 클래스명: RequestRegistrationRQ, RequestRegistrationRS

## Common Dataclass
@dataclass
class Request:
    replyTo: str
    correlationId: str
    request: Any = None


@dataclass
class Response:
    correlationId: str
    status: int
    message: str = None
    response: Any = None


## 서버 측에서 모니터링 시작 요청 시, 비디오 서버 정보를 전달
@dataclass
class MonitoringCameraRQ:
    host: str
    port: int
    auth: str


## 서버 측에서 모니터링 시작 요청
@dataclass
class StartControlRQ:
    userId: str
    interval: int
    camera: List[MonitoringCameraRQ] = field(default_factory=list)


@dataclass
class PositionRS:
    x: float
    y: float
    degree: float


@dataclass
class PoiRS:
    markerName: str
    markerId: str
    x: float
    y: float
    degree: float


@dataclass
class MapRS:
    image: str
    poi: List[PoiRS]


@dataclass
class ContentVersion:
    contentId: str
    type: str
    version: str


@dataclass
class ServiceModeRS:
    selectedMode: str


@dataclass
class ScheduleEnvRS:
    serviceMode: str
    scheduleType: str


@dataclass
class SpeedRS:
    defaultValue: float
    ranges: List[float]


@dataclass
class ActiveContent:
    contentId: str
    version: str


@dataclass
class Content:
    contentId: str
    type: str
    version: str
    fileUrl: str
    fileHash: str
    fileSize: int
    ext: str
    pageId: str
    fileVersion: str


## 서버 측에 모니터링 시작 시 필요한 데이터 응답
@dataclass
class StartControlRS:
    monitoringId: str
    serviceStartDate: str
    batteryStatus: str
    map: MapRS
    activeContent: ActiveContent
    contents: List[Content]
    serviceMode: ServiceModeRS
    speed: SpeedRS


@dataclass
class StopControlRS:
    status: int
    message: str


## 서버측 제어상황의 사용자정보 전달, 서버 입장의 Heartbeat 로 사용
@dataclass
class ControllingRQ:
    userId: str


## 응답없는 이벤트 메시지
## RMS_Client ->> RMS_Server

# Taskmanager 시작알림
@dataclass
class StartedEV:
    agentId: str
    version: str
    osName: str
    osVersion: str
    ip: str


# Taskmanager 운용 중임을 주기적으로 알림
@dataclass
class HeartbeatEV:
    agentId: str
    timestamp: str


# 페이지 전환 정보 로그
@dataclass
class PageViewEV:
    agentId: str
    mac: str
    robotLocation: str
    customer: str
    pageId: str
    pageName: str
    pageType: str
    pageLang: str
    pageStaySeconds: str
    robotBootingYmdhms: str
    robotShutdownYmdhms: str
    robotBatteryRate: int
    robotTenMinBeforeBatteryRate: int
    gender: str
    age: str
    maskYn: str


# 자율주행 관련 로그
@dataclass
class NavHistoryEV:
    agentId: str
    mac: str
    robotLocation: str
    goalId: str
    goalName: str
    goalType: str
    currentMoveStatus: str
    retryCurrentCnt: int


# 챗봇 로그
# content 에 json 구조의 string 값이 유입되므로 주의
@dataclass
class Log:
    source: str
    timestamp: str
    category: str
    level: str
    target: str
    content: str


@dataclass
class LogEV:
    agentId: str
    branchId: str
    ip: str
    timestamp: str
    logs: List[Log] = field(default_factory=list)


@dataclass
class StatEV:
    agentId: str
    branchId: str
    ip: str
    timestamp: str
    logs: List[Log] = field(default_factory=list)


@dataclass
class ImuRS:
    roll: float
    pitch: float
    yaw: float


@dataclass
class InfraredRS:
    front: int
    back: int


@dataclass
class WeatherRS:
    temperature: int
    pressure: int
    humid: int


@dataclass
class LidarRS:
    center: float
    centerL: float
    centerR: float
    sideL: float
    sideR: float
    left: float
    right: float


## 각종 센서 및 모니터링 정보를 주기적으로 전달
@dataclass
class MonitoringDataEV:
    agentId: str
    floorId: str
    emergencyStatus: str
    moveStatus: str
    robotStatus: str
    batteryStatus: str
    screenshot: str
    position: PositionRS
    imu: ImuRS
    infrared: InfraredRS
    pressure: float
    weather: WeatherRS
    lidar: LidarRS


## CROMS로부터 제공받은 로봇 ID 등록
@dataclass
class Profile:
    key: str
    value: str


@dataclass
class RegisterRQ:
    agentId: str
    nonce: str
    profiles: List[Profile] = field(default_factory=list)


## CROMS로부터 로봇 ID 해제

# @dataclass
# class UnregisterRQ:
# Empty

@dataclass
class GetControlStatusRS:
    status: str
    userId: str


@dataclass
class DoExpressionSetRQ:
    pageId: str
    emotion: str
    motion: str
    led: str
    tts: str


@dataclass
class MoveToRQ:
    markerName: str
    markerId: str
    x: float
    y: float
    degree: float
    speed: float


@dataclass
class SpeechRQ:
    text: str


@dataclass
class DriveRQ:
    direction: str
    rotate: str


@dataclass
class ContentVersion:
    contentId: str
    type: str
    version: str


@dataclass
class GetLastContentsVersionRQ:
    agentId: str
    contentId: str
    contents: List[ContentVersion]


@dataclass
class ContentVersionRS:
    type: str
    version: str
    fileUrl: str


@dataclass
class GetLastContentsVersionRS:
    contentId: str
    contents: List[ContentVersionRS]


@dataclass
class ComebackRQ:
    missionFinYn: bool


@dataclass
class SendImageEV:
    agent_id: str
    seq: str
    format: str
    height: str
    width: str
    data: str

# 사람 인식정보
@dataclass
class PersonalContextRQ:
    id: str
    mask: int

# 인식결과 정보 구독
@dataclass
class RecognitionResultRQ:
    personalContexts : List[PersonalContextRQ]

_mqttDataclassMap = {
    # Client Request

    # Client Response
    EtriCloudMessage.SENDIMAGE: SendImageEV

    # 이벤트 메시지 제외
}

# Function
def mqtt_request_convert(cmd, json_data):

    #print("cmd [{}] json [{}]".format(cmd, json_data))
    ret = Request(**json_data)
    ret.request = mqtt_message_convert(cmd, ret.request)
    return ret


def mqtt_message_convert(cmd, data):
    if not cmd in _mqttDataclassMap: return None
    conv = _mqttDataclassMap[cmd]
    #print("cmd [{}] conv [{}]".format(cmd, conv))
    return conv(**data) if conv else data


name_re = re.compile("(\w+)(RS|RQ|EV)$")


def mqtt_get_command(data):
    name = type(data).__name__
    matches = name_re.match(name)
    if not matches: return None

    cmd = matches.group(1)
    return cmd[0:1].lower() + cmd[1:]
