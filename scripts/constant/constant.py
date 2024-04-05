#!/usr/bin/python
# -*- coding: utf-8 -*-

###
# @file constant.py
# @brief Enum class 및 상수 선언
# @section

import enum
from dataclasses import *


#-------------------------------------------------------------
# ROS Topic 정의
#-------------------------------------------------------------

TOPIC_TEST_SCENARIO_EVENT = "/clobot_tm/scenario/event"
TOPIC_UI_CHANGE_REQUEST = "/clobot_tm/topic/ui_to_robot/change_request"
TOPIC_UI_ROBOT_EFFECT = "/clobot_tm/service/ui_to_robot/robot_effect"
TOPIC_UI_CONFIG_SAVE = "/clobot_tm/service/ui_to_robot/config_save"
TOPIC_UI_CONFIG_LOAD = "/clobot_tm/service/ui_to_robot/config_load"
TOPIC_COMPRESSED_IMAGE = "/cv_camera/camera_head/image_raw/compressed" # for mqtt
TOPIC_RAW_IMAGE = "/cv_camera/camera_head/image_raw"
TOPIC_EDGE_SEND_IMAGE = "/camera/robot_image_info"#"/camera/rgb/image_compressed"
TOPIC_EDGE_AAI4R_SITUATION = "/aai4r/situation"
TOPIC_EDGE_AAI4R_FACIAL = "/aai4r/facial"
TOPIC_EDGE_AAI4R_TALK_TRIGGER = "/aai4r/fashion/talk_trigger"
TOPIC_EDGE_AAI4R_TALK = "/aai4r/fashion/talk"
TOPIC_EDGE_AAI4R_FASHION_ATTR = "/aai4r/fashion"

# 홈봇의 부가서비스 대응
# MQTT
MQTT_TOPIC_EVENT = "/server_manager/event" # service = 'speech', 'expression', 'motion'
MQTT_TOPIC_SEND_IMAGE = "/homebot/camera_event"
MQTT_TOPIC_STT_RESULT = "robot_mode/request" # service = 'stt' payload = 'sentence'

# ROS
ROS_TOPIC_EDGE_SEND_IMAGE = "/camera/robot_image_info"
ROS_TOPIC_EDGE_STT_RESULT = "/stt/result"
ROS_TOPIC_EDGE_SPEECH = "/server_manager/speech"
ROS_TOPIC_EDGE_EXPRESSION = "/server_manager/expression"
ROS_TOPIC_EDGE_MOTION_TILT = "/server_manager/motion_tilt"
ROS_TOPIC_EDGE_MOTION_PAN = "/server_manager/motion_pan"
#----------------------------------------------------------------
# MQTT Properties
#----------------------------------------------------------------#

EDGE_SERVER_SUB_ID = "robot_mode"
EDGE_SERVER_PUB_ID = "edge_publisher"
ETRI_MQTT_SERVER_URL = "172.39.0.2"
ETRI_MQTT_SERVER_PORT = 1883
ETRI_MQTT_CONN_ID = "clobot"
ETRI_MQTT_CONN_PASSWORD = "ansdufdj1*"
IMG_URL_FORMAT = "data:image/jpeg,"
FILE_SAVE_PATH = "/tmp/cv_camera.jpg"
SEND_IMAGE_TOPIC = "video/image_info"
RECOGNITION_RESULT = "video/result"
FASHION_TALK = "fashion/talk"
REQUEST_EVENT = "event"

#-------------------------------------------------------------
# Enum class
#-------------------------------------------------------------


## datetime
class ConstantDataFormat(enum.Enum):
    DATE_DEFAULT_FORMAT = "%Y-%m-%d %H:%M:%S"
    DATE_DEFAULT_MILLISECONDS_FORMAT = "%Y-%m-%d %H:%M:%S.%f"
    DATE_INTERFACE_FORMAT = "%Y/%m/%d %H:%M:%S"
    DATE_INSERTDB_FORMAT = "%Y%m%d%H%M%S"
    DATE_TIME_FORMAT = "%H:%M:%S"
    DATE_HH_FORMAT = "%H"
    DATE_MM_FORMAT = "%M"
    DATE_SS_FORMAT = "%S"
    DATE_YMD_FORMAT = "%Y%m%d"
    DATE_HHMM_FORMAT = "%H%M"
    DATE_TIMEDELTA_FORMAT = ""


## service mode
class ServiceModeType(enum.Enum):
    INIT = 0            # boot check
    E_STOP = 1          # empergency button pressed.
    ERROR = 2           # error
    ADMIN = 3           # admin page
    FORCE_CHARGE = 4    # admin : force charge evnet
    FOLLOW = 5          # follow
    CAUTION = 6         # egde event : caution
    GREET = 7           # egde event : greet
    SCHEDULE = 8        # move schedule
    INFORMATION = 9     # user page
    STANBY = 10         # stanby


## command
class EventCommand(enum.Enum):
    MQTT_RESPONSE = "mqtt_response"
    ACTION_MOVETO = "action_moveto"
    ACTION_DOCKING = "action_docking"
    ACTION_DOCKING_FRONT = "action_docking_front"
    ACTION_UNDOCKING = "action_undocking"
    ACTION_CANCEL = "action_cancel"
    PAGE_FOLLOW = "page_follow"


## event type
class EventType(enum.Enum):
    MQTT = "mqtt"
    ACTION = "action"
    UI = "ui"


## action
class ActionComponents(enum.Enum):
    DOCKING = "DOCKING"
    DOCKING_FRONT = "DOCKING_FRONT"
    UNDOCKING = "UNDOCKING"
    MOVETO = "MOVETO"
    INIT_POSE = "INIT_POSE"
    CANCELED = "CANCELED"
    START_POI = "START_POI"
    NEXT_POI = "START_POI"
    END_POI = "START_POI"


# tb_robot_move_mission : action step
class EtriActionStep(enum.Enum):
    STANBY = 0
    REQUEST = 1
    RESPONSE = 2
    COMPLETE = 3


# tb_robot_move_mission : action state
class EtriActionState(enum.Enum):
    NO_ERROR = 0
    GOAL_REACHED = 1
    ERROR_PLANNER=2
    ERROR_CONTROLLER=3
    ERROR_ETC=4


# tb_robot_move_mission : driver state
class EtriDriverState(enum.Enum):
    NO_ERROR = 0
    ERROR_FAULT = 1
    ERROR_PREEMPTED = 2
    ERROR_CONDITION = 3
    ERROR_TIMEOUT = 4
    ERROR_DRIVER = 5


# tb_robot_status : current_move_status
class MoveStatusType(enum.Enum):
    INIT = 0
    DOCKING = 1
    UNDOCIKING = 2
    SCHEDULE = 3
    ERROR = 9


# service mode priority
class Priority(enum.Enum):
    HIGH = 0
    MEDIUM = 1
    LOW = 2


# sound mode
class SoundMode(enum.Enum):
    NONE = 0
    ONLY_EFFECT = 1
    ONLY_TTS = 2
    BOTH = 3


# sound mask type
class SoundMaskType(enum.Enum):
    COMPLETED = 0
    ONLY_REQUEST_COMPLETED = 1
    ALL = 2 # start / pause / complete


# sound state
class SoundState(enum.Enum):
    NONE = 1
    EFFECT_START = 2
    EFFECT_PLAYING = 3
    EFFECT_STOP = 4
    EFFECT_END = 5
    TTS_START = 6
    TTS_MAKE = 7
    TTS_PLAYING = 8
    TTS_STOP = 9
    TTS_END = 10
    TTS_PART_END = 11
    ERROR = 12


class RobotSoundFiles(enum.Enum):
    NONE = 0
    ROBOT_ERROR = 1
    TTS = 2
    ROBOT_VOLUME = 3


# ui request type (string)
class UiRequestType(enum.Enum):
    TTS_SUBTITLE = "TTS_SUBTITLE"
    CHANGE_LANG = "CHANGE_LANG"
    CHANGE_PAGE = "CHANGE_PAGE"
    SCREEN_TOUCH = "SCREEN_TOUCH"


# UI topic 요청자
class UiRequestCallerType(enum.Enum):
    WEB = "WEB"
    ROBOT = "ROBOT"
    RMS = "RMS"
    NAVI = "NAVI"


# UI Effect type
class UiEffectType(enum.Enum):
    VOLUME_UP = "VOLUME_UP"
    VOLUME_DOWN = "VOLUME_DOWN"
    SHUTDOWN_TM = "SHUTDOWN_TM"
    SHUTDOWN_PC = "SHUTDOWN_PC"
    FORCE_CHARGE = "FORCE_CHARGE"

#-------------------------------------------------------------
# Dataclass
#-------------------------------------------------------------

# 데이터 클래스 명칭 정의시 유의사항
# * 클래스명은 반드시 카멜 표기법으로된 명령어로 사용하며 파스칼표기법으로 변환해서 사용
# * 클래스명 뒤에는 2글자로 클래스의 속성을 표시함
#    - RQ : 요청용 데이터 클래스
#    - RS : 응답용 데이터 클래스
#    - EV : 이벤트용 데이터 클래스
# 예) 명령어 : RequestRegistration --> 클래스명: RequestRegistrationRQ, RequestRegistrationRS

# @dataclass
# class RobotEventRS:
#     type : str
#     message : str
