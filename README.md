# multipanda_ws

AIRLAB · Simulation with MuJoCo · Part A (MuJoCo 씬 · 카메라 토픽 설정) 튜토리얼을 끝까지 적용한 ROS 2 Humble 워크스페이스.

- `src/multipanda_ros2` — [tenfoldpaper/multipanda_ros2](https://github.com/tenfoldpaper/multipanda_ros2) `46cccdc` (submodule `mujoco_ros_pkgs` `b3a65f3` 포함) 사본 + 튜토리얼 수정사항
- `src/camera_subscriber_pkg` — 카메라 이미지 저장 노드 (튜토리얼 7장)
- `docker/Dockerfile`, `run.sh` — 실습 이미지 / 컨테이너 실행 스크립트 (튜토리얼 2장)
- `setup.sh` — 2장 전체(이미지 빌드 → 컨테이너 → `colcon build`)를 한 번에 실행

## 설치 (Ubuntu 22.04, x86_64)

1. Docker 설치 (튜토리얼 1장) 후 **로그아웃 → 로그인**
2. 클론 및 설정 (베이스 이미지 빌드에 수십 분 소요)

```bash
git clone https://github.com/airlab-internship/multipanda_ws.git ~/multipanda_ws
~/multipanda_ws/setup.sh
```

## 실행

컨테이너 안 터미널 진입 (새 터미널마다):

```bash
~/multipanda_ws/run.sh
```

```bash
ros2 launch franka_bringup franka_sim_with_camera.launch.py                    # 시뮬레이터
ros2 launch franka_bringup franka_sim_with_camera.launch.py save_images:=true  # + hand_cam PNG 저장 → ~/multipanda_ws/data/hand_cam/
ros2 run rqt_image_view rqt_image_view                                         # /mujoco_server/cameras/{test_cam,hand_cam}/rgb/image_raw
```

호스트에서 바로 실행도 가능 (컨테이너 안 `~/multipanda_ws`에서 실행됨):

```bash
~/multipanda_ws/run.sh ros2 launch franka_bringup franka_sim_with_camera.launch.py
```

파일 수정 후 재빌드: `colcon build --packages-select franka_bringup franka_description camera_subscriber_pkg`

## 원본 대비 수정 파일

| 파일 | 튜토리얼 |
|---|---|
| `franka_description/mujoco/franka/objects_airlab.xml` (신규) | 3.1 |
| `franka_description/mujoco/franka/scene_add_camera.xml` (신규, `scene.xml` 기반) | 3.2, 5.1, 6.2 |
| `franka_description/mujoco/franka/panda_with_hand_camera.xml` (신규, `panda.xml` 기반) | 6.1 |
| `franka_bringup/config/sim/single_sim_controllers_add_camera.yaml` (신규) | 5.2, 6.3 |
| `franka_bringup/launch/sim/franka_sim_with_camera.launch.py` (신규) | 3.3, 5.3, 7.6 |
| `tools/setup_env` — `REPO_PATH`를 스크립트 위치 기준으로 변경 | 사본으로 포함되어 `git rev-parse`가 워크스페이스 루트를 가리키므로 |
