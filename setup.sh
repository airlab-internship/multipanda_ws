#!/usr/bin/env bash
# 튜토리얼 2장(컨테이너 구성)을 한 번에 실행: 베이스 이미지 → 실습 이미지 → 컨테이너 → colcon build
# Docker 설치(1장)는 먼저 끝내고, 로그아웃/로그인해서 docker 그룹이 적용된 상태여야 함.
set -e
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

# 2.2 (3) 실습 이미지
docker build -t airlab-sim:humble "$WS/docker"

# 2.3 + 2.4 컨테이너 실행 후 워크스페이스 빌드
"$WS/run.sh" colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

echo
echo "완료. 시뮬레이터 실행:"
echo "  $WS/run.sh ros2 launch franka_bringup franka_sim_with_camera.launch.py"
