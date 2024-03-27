
# ETRI 지능형 로봇 개발

- 홈봇과 Docker container (ROS2) 간의 메시지 중계 역할을 담당 합니다.

## ```준비물```

- Docker 설치 (https://docs.docker.com/engine/install/ubuntu)

### Docker build
```bash
mkdir -p ~/clobot_ws/src/etri_inteli_robot

# git clone
https://github.com/aai4r/{}.git

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
