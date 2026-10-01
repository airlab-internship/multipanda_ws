#!/usr/bin/env bash
# Usage:
#   ./run.sh                 -> 컨테이너 안 터미널로 진입
#   ./run.sh <command...>    -> 컨테이너 안에서 명령 실행 (예: ./run.sh ros2 topic list)
#   ./run.sh '<cmd | cmd>'   -> 따옴표로 감싼 한 문자열은 셸 명령으로 실행 (파이프 등)
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="airlab-sim:humble"
NAME="airlab-sim"

xhost +local:docker > /dev/null 2>&1 || true

GPU=()
if command -v nvidia-smi > /dev/null 2>&1 && docker info 2> /dev/null | grep -q nvidia; then
    GPU=(--gpus 'all,"capabilities=compute,utility,graphics,display"')
fi

if [ -z "$(docker ps -aq -f name="^${NAME}$")" ]; then
    docker run -d --name "$NAME" \
        --user "$(id -u):$(id -g)" \
        --network host --ipc host --pid host --privileged \
        --tty --interactive \
        --env "DISPLAY=$DISPLAY" \
        --env "ROS_DOMAIN_ID=1" \
        --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
        --volume /dev:/dev \
        --volume "$WS:/home/developer/multipanda_ws" \
        --ulimit rtprio=99 --ulimit rttime=-1 --ulimit memlock=-1 \
        --cap-add=sys_nice \
        "${GPU[@]}" \
        "$IMAGE" > /dev/null
elif [ -z "$(docker ps -q -f name="^${NAME}$")" ]; then
    docker start "$NAME" > /dev/null
fi

# 워크스페이스 setup.bash를 .bashrc에 한 번만 등록 (튜토리얼 2.4)
docker exec --user developer "$NAME" bash -c \
    "grep -q 'multipanda_ws/install/setup.bash' ~/.bashrc || echo '[ -f ~/multipanda_ws/install/setup.bash ] && source ~/multipanda_ws/install/setup.bash' >> ~/.bashrc"

if [ $# -eq 0 ]; then
    docker exec -it --user developer "$NAME" bash
else
    # 인자 1개 = 셸 명령 문자열 그대로 (파이프 등), 여러 개 = 각 인자의 따옴표 유지
    if [ $# -eq 1 ]; then CMD="$1"; else printf -v CMD '%q ' "$@"; fi
    TTY=(-i); [ -t 0 ] && TTY=(-it)
    docker exec "${TTY[@]}" --user developer "$NAME" bash -ic "cd ~/multipanda_ws && $CMD"
fi
