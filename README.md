# multipanda_ws

AIRLAB · Simulation with MuJoCo 튜토리얼을 끝까지 적용한 ROS 2 Humble 워크스페이스.

- Part A — MuJoCo 씬 · 카메라 토픽 설정
- Part B — MuJoCo Pick & Place · Episode 수집

| 경로 | 내용 |
|---|---|
| `src/multipanda_ros2` | [tenfoldpaper/multipanda_ros2](https://github.com/tenfoldpaper/multipanda_ros2) `46cccdc` (submodule `mujoco_ros_pkgs` `b3a65f3` 포함) 사본 + Part A 수정사항 |
| `src/camera_subscriber_pkg` | 카메라 이미지 저장 노드 (Part A 7장) |
| `src/moveit2/moveit_py` | [moveit/moveit2](https://github.com/moveit/moveit2) `humble` `c283a36` (2.5.10)의 `moveit_py`만 sparse checkout한 사본 (Part B 2장) |
| `src/airlab_msgs` | `PickPlace` 액션 정의 (Part B 3.1) |
| `src/airlab_pick_place` | pick & place 서버 · 에피소드 수집기 (Part B 3~9장) |
| `docker/Dockerfile`, `run.sh` | 실습 이미지 / 컨테이너 실행 스크립트 (Part A 2장) |
| `setup.sh` | Part A 2장 전체(이미지 빌드 → 컨테이너 → `colcon build`)를 한 번에 실행 |

## 설치 (Ubuntu 22.04, x86_64)

1. Docker 설치 (Part A 1장) 후 **로그아웃 → 로그인**
2. 클론 및 설정 (베이스 이미지 빌드에 수십 분 소요)

```bash
git clone https://github.com/airlab-internship/multipanda_ws.git ~/multipanda_ws
~/multipanda_ws/setup.sh
```

## 실행

빌드·실행은 모두 컨테이너 안에서 (새 터미널마다 `~/multipanda_ws/run.sh`로 진입).
호스트에서 빌드하면 `The build time path "/home/developer/..." doesn't exist` 에러.

### Part A: 씬 · 카메라

```bash
ros2 launch franka_bringup franka_sim_with_camera.launch.py                    # 시뮬레이터
ros2 launch franka_bringup franka_sim_with_camera.launch.py save_images:=true  # + hand_cam PNG 저장 → ~/multipanda_ws/data/hand_cam/
ros2 run rqt_image_view rqt_image_view                                         # /mujoco_server/cameras/{test_cam,hand_cam}/rgb/image_raw
```

### Part B: Pick & Place · Episode 수집 (터미널 3개)

```bash
ros2 launch airlab_pick_place sim.launch.py              # 터미널 1: 시뮬레이터 + 팔 컨트롤러
ros2 run airlab_pick_place pick_place_server             # 터미널 2: "pick_place_server ready" 확인 후
ros2 run airlab_pick_place collect_episodes              # 터미널 3: 에피소드 수집 → ~/multipanda_ws/data/episodes/ep_0000.h5 …
```

- 컨트롤러 확인: `ros2 control list_controllers` (`panda_arm_controller`, `joint_state_broadcaster` 모두 active)
- pre-grasp 테스트: `ros2 run airlab_pick_place pregrasp`
- 단일 요청: `ros2 action send_goal /pick_place airlab_msgs/action/PickPlace "{place_x: -0.5, place_y: 0.15}"`
- 에피소드 수 변경: `ros2 run airlab_pick_place collect_episodes --ros-args -p num_episodes:=30 -p start_index:=30`
- 튜닝 값: `src/airlab_pick_place/config/pick_place.yaml` 수정 → `colcon build --packages-select airlab_pick_place` → 노드 재실행

호스트에서 바로 실행도 가능 (컨테이너 안 `~/multipanda_ws`에서 실행됨):

```bash
~/multipanda_ws/run.sh ros2 launch airlab_pick_place sim.launch.py
```

## 튜토리얼과 다른 점

- 소스를 `git clone` 대신 사본으로 포함 (`multipanda_ros2`, `moveit2/moveit_py`). 튜토리얼이 받는 것과 같은 커밋.
- `ros2 pkg create`로 만드는 패키지(`camera_subscriber_pkg`, `airlab_msgs`, `airlab_pick_place`)는 Humble `ros2pkg` 템플릿 그대로 생성 (maintainer = 컨테이너 사용자 `developer`).
- `multipanda_ros2/tools/setup_env` — `REPO_PATH`를 스크립트 위치 기준으로 변경 (사본으로 포함되어 `git rev-parse`가 워크스페이스 루트를 가리키므로).
- `run.sh` — 명령 인자 실행 지원, `install/setup.bash` source 줄을 `.bashrc`에 자동 등록.
- `setup.sh`, `README.md`, `.gitignore` 추가.

## multipanda_ros2 원본 대비 수정 파일 (Part A)

| 파일 | 튜토리얼 |
|---|---|
| `franka_description/mujoco/franka/objects_airlab.xml` (신규) | 3.1 |
| `franka_description/mujoco/franka/scene_add_camera.xml` (신규, `scene.xml` 기반) | 3.2, 5.1, 6.2 |
| `franka_description/mujoco/franka/panda_with_hand_camera.xml` (신규, `panda.xml` 기반) | 6.1 |
| `franka_bringup/config/sim/single_sim_controllers_add_camera.yaml` (신규) | 5.2, 6.3 |
| `franka_bringup/launch/sim/franka_sim_with_camera.launch.py` (신규) | 3.3, 5.3, 7.6 |
| `tools/setup_env` | 위 "튜토리얼과 다른 점" 참고 |
