# multipanda_ws

MuJoCo 카메라, pick-and-place 튜토리얼을 포함한 워크스페이스입니다.

# Packages & Files

| 패키지 | 내용 |
|---|---|
| `src/multipanda_ros2` | 튜토리얼 내용이 추가된 [tenfoldpaper/multipanda_ros2](https://github.com/tenfoldpaper/multipanda_ros2) fork |
| `src/camera_subscriber_pkg` | 카메라 이미지 저장 노드 |
| `src/moveit2/moveit_py` | planning을 위한 [moveit/moveit2](https://github.com/moveit/moveit2) fork |
| `src/airlab_msgs` | `PickPlace` 액션 정의 |
| `src/airlab_pick_place` | pick & place 서버 및 에피소드 수집기 |
| `src/openpi` | H5 → LeRobot 변환 및 π0.5 학습·추론용 Python 프로젝트 ([사용 방법](src/openpi/README.md)) |
| `docker/Dockerfile`, `run.sh` | 실습 이미지 / 컨테이너 실행 스크립트 |
| `setup.sh` | 실습에 필요한 환경을 구축하는 데 사용하는 환경 셋업 스크립트 |

## 설치 (Ubuntu 22.04, x86_64)

1. Docker 설치
2. 클론 및 설정

```bash
git clone https://github.com/airlab-internship/multipanda_ws.git ~/multipanda_ws
~/multipanda_ws/setup.sh
```

## 실행

ROS·MuJoCo 빌드 및 실행은 컨테이너 안에서(워크스페이스 루트에서 `./run.sh`로 진입) 진행해 주세요.
호스트에서 빌드하면 `The build time path "/home/developer/..." doesn't exist` 에러가 발생합니다.

## π0.5 데이터 변환·학습·추론

현재 호스트 워크스페이스는 `~/airlab-internship/multipanda_ws`이며, Docker 안에서는 `/home/developer/multipanda_ws`로 마운트됩니다. `data/episodes`의 H5를 `data/lerobot`으로 변환하고, 별도 GPU 서버에서 학습합니다.

OpenPI는 ROS 패키지가 아니며 `COLCON_IGNORE`로 ROS 빌드에서 제외됩니다. 호스트 또는 GPU 서버에서 uv 환경으로 실행합니다. uv 설치부터 변환·학습·추론까지는 [OpenPI README](src/openpi/README.md)를 따라가세요. Docker와 uv를 함께 사용하는 방법도 해당 문서에 정리되어 있습니다.
