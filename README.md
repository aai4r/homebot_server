
# ETRI 지능형 로봇 개발

- 홈봇과 Docker container (ROS2) 간의 메시지 중계 역할을 담당 합니다.
- 하기 내용에는 MQTT broker 설치도 포함되어 있습니다.
- 소스코드 내에 edge subscriber 와 publisher 가 함께 포함되어 있습니다.
- 각각의 dockerfile 을 기반으로 빌드와 실행을 진행 합니다.

## ```준비물```

- Docker 설치 (https://docs.docker.com/engine/install/ubuntu)

## ```Docker none-root 권한 부여```
```bash
# docker group 생성
sudo groupadd docker

# 현재 사용자를 docker group에 추가
sudo usermod -aG docker $USER

newgrp docker

# docker CLI 실행 시 하기의 오류 발생 시
WARNING: Error loading config file: /home/user/.docker/config.json -
stat /home/user/.docker/config.json: permission denied

# 권한 부여
sudo chown "$USER":"$USER" /home/"$USER"/.docker -R
sudo chmod g+rwx "$HOME/.docker" -R
``` 

## ```docker 가상 네트워크 만들기```
- 시스템 재부팅 시, 가상 host IP 체계가 변경
- 이를 고정하기 위해 subnet mask 를 적용한 임의의 gateway 생성
- 이후 컨테이너 실행 옵션에 해당 network로 설정하여 사용
```bash
docker network ls

NETWORK ID| NAME   | DRIVER | SCOPE
a9123712  | bridge | bridge | local

docker network create --gateway 172.39.0.1 --subnet 172.39.0.0/21 --driver bridge homebot_virtual_bridge
``` 

## ```엣지서버 - emqx```
- MQTT broker 이며, 로봇과 엣지서버 간의 통신 중계를 담당 합니다.
- docker 가상 네트워크 만든 이후에 진행 하셔야 합니다.
- no EE 버전 입니다. (enterprise version)
- 설치 과정은 아래 내용을 참고해 주세요.
```bash
# docker image download
# emqx 의 경우, 4.2.9 버전을 지정하여 사용 (emqx 에서 제공하는 plugin 이 특정 version에서만 정상동작하는 문제)
docker pull emqx/emqx:4.2.9

# run container
# plugin : emqx_management, emqx_recon, emqx_retainer, emqx_dashboard, emqx_auth_username
# emqx_dashboard 의 경우, 웹 브라우저에서 emqx 상태를 모니터링 할 수 있음

docker run -d --name emqx_429 --net homebot_virtual_bridge --ip 172.39.0.2 --restart=always \
-p 18083:18083 -p 1883:1883 -p 8083:8083 -p 8084:8084 -p 8883:8883 \
-e EMQX_LOADED_PLUGINS="emqx_management, emqx_recon, emqx_retainer, emqx_dashboard, emqx_auth_username" \
-e EMQX_LISTENER__TCP__EXTERNAL=1883 \
-e EMQX_MQTT__MAX_PACKET_SIZE=1MB \
-e EMQX_ALLOW_ANONYMOUS=false \
-e EMQX_AUTH__USER__PASSWORD_HASH=plain \
-e EMQX_AUTH__USER__1__USERNAME="clobot" \
-e EMQX_AUTH__USER__1__PASSWORD="ansdufdj1*" \
-e EMQX_DASHBOARD__DEFAULT_USER__LOGIN="clobot" \
-e EMQX_DASHBOARD__DEFAULT_USER__PASSWORD="ansdufdj1*" \
-e EMQX_LOG__LEVEL=warning \
emqx/emqx:4.2.9

# 실행 과정 중에 port bind 오류 발생 시, emqx가 사용하는 port를 먼저 점유하고 있던 프로세스를 찾아 종료한다.
netstat -atn | grep :1883

### Docker build
```bash
mkdir -p ~/clobot_ws/src/etri_inteli_robot

# git clone
https://github.com/aai4r/homebot_server.git

# docker build : edge_sub
# -t : image tag name
# -f : 특정 Dockerfile 설정, 설정이 없다면 Dockerfile 이름을 찾는다.
docker build -t edge_sub -f Dockerfile.sub .

# docker build : edge_pub
docker build -t edge_pub -f Dockerfile.pub .
```

### Docker run
```bash
# docker run : edge_sub
# -d : daemon, background
# --name : docker container name
# --restart : 재시작 옵션. always 의 경우, 항시 재시작. no 의 경우 재시작하지 않음 
docker run -d --name edge_sub --restart=always --net homebot_virtual_bridge --ip 172.39.0.4 edge_sub

# docker run : edge_pub
docker run -d --name edge_pub --restart=always --net homebot_virtual_bridge --ip 172.39.0.5 edge_pub
```

### 메시지 확인
- SERVER to ROBOT
    - speech
    - expression
    - motion

```bash
# PC host와 container
xhost+
docker run --net homebot_virtual_bridge -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --rm -it aai4r/dummybot bash

# ROS2 환경변수 등록
source /opt/ros/foxy/setup.bash
# Custom msg (RobotImageInfo) 환경변수 등록
source install/setup.bash

ros2 topic list

/camera/robot_image_info
/server_manager/expression
/server_manager/motion_pan
/server_manager/motion_tilt
/server_manager/speech
/stt/result

ros2 topic pub -1 /server_manager/speech std_msgs/String "data: hello"
ros2 topic pub -1 /server_manager/expression std_msgs/String "data: FACE_WAKE_UP"
ros2 topic pub -1 /server_manager/motion_pan std_msgs/String "data: {direction: 'pan', degree: '10'}"
ros2 topic pub -1 /server_manager/motion_tilt std_msgs/String "data: {direction: 'tilt', degree: '10'}"
```

- ROBOT to SERVER
    - send image
    - stt result

```bash
xhost+
docker run --net homebot_virtual_bridge -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --rm -it aai4r/dummybot bash

# ROS2 환경변수 등록
source /opt/ros/foxy/setup.bash
# Custom msg (RobotImageInfo) 환경변수 등록
source install/setup.bash

ros2 topic echo /camera/robot_image_info
ros2 topic echo /stt/result 
```
