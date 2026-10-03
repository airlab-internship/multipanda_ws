#!/usr/bin/env bash
# 튜토리얼 2장(컨테이너 구성)을 한 번에 실행: 베이스 이미지 → 실습 이미지 → 컨테이너 → colcon build
# Docker 설치(1장)는 먼저 끝내고, 로그아웃/로그인해서 docker 그룹이 적용된 상태여야 함.
set -eo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! docker info > /dev/null 2>&1; then
    echo "docker에 접근할 수 없음. 튜토리얼 1장(Docker 설치) 후 로그아웃 → 로그인하고 다시 실행." >&2
    exit 1
fi
if [ "$(uname -m)" != "x86_64" ]; then
    echo "경고: docker/Dockerfile은 amd64 베이스 이미지(build-env:multipanda_ros2-amd64)를 사용함." >&2
fi

# 2.2 (1) 베이스 이미지 (네트워크 오류 시 최대 3회 재시도, 완료된 단계는 캐시됨)
if ! docker image inspect build-env:multipanda_ros2-amd64 > /dev/null 2>&1; then
    for i in 1 2 3; do
        "$WS/src/multipanda_ros2/tools/setup_env" && break
        [ "$i" -eq 3 ] && exit 1
        echo "setup_env 실패 → 재시도 ($i/3)"
    done
fi

# 2.2 (3) 실습 이미지: 통신 패키지만 포함 (데이터·모델·학습 환경은 제외)
tar -C "$WS" --exclude='__pycache__' --exclude='*.pyc' --exclude='.venv' \
    -cf - docker/Dockerfile src/openpi/packages/openpi-client \
    | docker build -f docker/Dockerfile -t airlab-sim:humble -

# 기존 컨테이너는 이미지 재빌드만으로 업데이트되지 않는다. 임의 삭제하지 않는다.
if docker container inspect airlab-sim > /dev/null 2>&1; then
    BUILT_IMAGE_ID="$(docker image inspect --format '{{.Id}}' airlab-sim:humble)"
    CONTAINER_IMAGE_ID="$(docker container inspect --format '{{.Image}}' airlab-sim)"
    if [ "$BUILT_IMAGE_ID" != "$CONTAINER_IMAGE_ID" ]; then
        echo "이미지 빌드 완료. 기존 airlab-sim 컨테이너는 이전 이미지를 사용 중입니다."
        echo "기존 컨테이너를 중지하고 다른 이름으로 보관한 뒤 ./setup.sh를 다시 실행하세요."
        echo "워크스페이스는 호스트에 그대로 있으며, 기존 컨테이너는 삭제하지 않았습니다."
        exit 1
    fi
fi

# 2.3 + 2.4 컨테이너 실행 후 워크스페이스 빌드
"$WS/run.sh" colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

echo
echo "완료. 시뮬레이터 실행:"
echo "  $WS/run.sh ros2 launch franka_bringup franka_sim_with_camera.launch.py"
echo "Pick & Place 시뮬레이터 (Part B):"
echo "  $WS/run.sh ros2 launch airlab_pick_place sim.launch.py"
