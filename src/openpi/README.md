# π0.5 데이터 변환·학습·추론

로컬에서 H5를 LeRobot으로 변환하고, 서버에서 non-idle 전처리 → 정규화 → 학습 순서로 실행한다.
실행 파일과 학습 설정은 `src/openpi/airlab/`에 모여 있다.
아래 경로는 `~/multipanda_ws` 기준이며, 다른 곳에 설치했다면 `cd` 경로만 바꾼다.

- 설정: `pi05_droid_finetune` (π0.5 Base, custom velocity, LoRA, batch 1, 20,000 steps)

## 1. uv 설치

uv가 없으면 로컬과 서버에서 각각 한 번 설치한다.

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source "$HOME/.local/bin/env"
```

## 2. 로컬 환경 설치·데이터 변환

```bash
cd ~/multipanda_ws/src/openpi
GIT_LFS_SKIP_SMUDGE=1 uv sync --locked

uv run src/openpi/airlab/convert_dataset.py --data-dir ../../data/episodes
```

입력은 `multipanda_ws/data/episodes/*.h5`, 출력은 `multipanda_ws/data/lerobot/`이다.
원본 H5는 유지하며, 출력 폴더가 이미 있으면 덮어쓰지 않고 중단한다.

Action은 팔 7축의 `(q[t+1] - q[t]) / dt` [rad/s]와 그리퍼 1차원이다.
그리퍼는 `0=열림, 1=닫힘`이며 다음 관측이 없는 마지막 프레임은 제외한다.

## 3. 서버 환경 설치
코드와 `data/lerobot/` 전체를 서버에 옮긴 뒤 실행한다. `.venv`는 옮기지 않는다.

```bash
cd ~/multipanda_ws/src/openpi
GIT_LFS_SKIP_SMUDGE=1 uv sync --locked
```
`uv sync --locked`가 LeRobot·h5py와 프로젝트를 함께 설치한다..

## 4. 서버 Non-idle 전처리

이 단계부터 정규화·학습까지 같은 터미널에서 순서대로 실행한다.

```bash
cd ~/multipanda_ws/src/openpi
export AIRLAB_LEROBOT_ROOT="$HOME/multipanda_ws/data/lerobot"

uv run src/openpi/airlab/compute_nonidle_indices.py \
  --data-dir "$AIRLAB_LEROBOT_ROOT"

export AIRLAB_NONIDLE_INDICES="$AIRLAB_LEROBOT_ROOT/nonidle_indices.npy"
```

`export`는 이후 실행할 프로그램에 설정을 전달한다. 파일을 옮기거나 생성하는 명령이 아니다.

- `$HOME`: 현재 로그인한 계정의 홈 폴더.
- `AIRLAB_LEROBOT_ROOT`: 데이터 폴더 위치. `realpath`로 절대 경로를 지정한다.
- `AIRLAB_NONIDLE_INDICES`: 정규화·학습에 사용할 프레임 인덱스 파일 위치.

새 터미널이나 SSH 재접속 후에는 두 `export`를 다시 실행한다.
필터 없이 실행할 때만 `unset AIRLAB_NONIDLE_INDICES`로 설정을 해제한다. 파일은 삭제되지 않는다.

결과는 `data/lerobot/nonidle_indices.npy`이며 재실행하면 덮어쓴다. 원본 에피소드는 유지한다.
현재 idle 기준은 7축 모두 속도 절댓값 `0.001 rad/s` 미만이다. 느린 이동도 제외될 수 있다.

## 5. 서버 정규화 계산 — GPU

GPU 번호는 본인이 사용할 번호로 바꾼다.

```bash
CUDA_VISIBLE_DEVICES={?}\
XLA_PYTHON_CLIENT_PREALLOCATE=false \
uv run src/openpi/airlab/compute_norm_stats.py \
  --config-name pi05_droid_finetune
```
데이터나 non-idle 설정을 바꾸면 학습 전에 다시 계산한다.

## 6. 서버 학습 — GPU
정규화가 완료된 뒤 같은 non-idle 환경변수 설정을 유지해서 실행한다.

```bash
CUDA_VISIBLE_DEVICES={?}\
uv run src/openpi/airlab/train.py pi05_droid_finetune \
  --exp-name=pick_place
```

`--exp-name`은 실험 폴더 이름이며, 이미 사용한 이름이면 새 이름으로 바꾼다.

체크포인트: `checkpoints/pi05_droid_finetune/<exp-name>/<step>/`

## 7. 서버 추론 — GPU

`<step>`을 저장된 체크포인트 번호로 바꾸고, 학습 때 실험명을 바꿨다면 경로도 맞춘다.

```bash
cd ~/multipanda_ws/src/openpi

CUDA_VISIBLE_DEVICES={?}
uv run src/openpi/airlab/serve_policy.py --port=? \
  --policy.config=pi05_droid_finetune \
  --policy.dir="checkpoints/pi05_droid_finetune/<exp-name>/<step>"
```

## 8. 로컬 ROS 추론 클라이언트

호스트에서 `./setup.sh`를 실행하면 통신용 `openpi-client` 설치와 ROS 빌드가 함께 진행된다. Docker 안에서 별도 `pip install`이나 `uv sync`는 필요 없다. 기존 컨테이너가 이전 이미지를 쓰고 있으면 `setup.sh` 안내에 따라 컨테이너를 교체한다.

```bash
cd /home/developer/multipanda_ws
source install/setup.bash
```

시뮬레이터가 실행 중이 아니면 컨테이너 터미널 하나에서 실행한다.

```bash
ros2 launch airlab_pick_place sim.launch.py
```

다른 컨테이너 터미널에서 실행한다. 서버의 추론 정책을 먼저 켜고 `<GPU_SERVER_IP>`를 실제 서버 주소로 바꾼다.

```bash
cd /home/developer/multipanda_ws
source install/setup.bash
ros2 run airlab_pick_place pi05_client \
  --ros-args -p server_ip:="<GPU_SERVER_IP>" -p server_port:=<PORT> \
  -p open_loop_horizon:=12
```

서버 주소·포트·H는 `airlab_pick_place/config/pi05_client.yaml`에서 바꾼다. 일회성 변경은 위처럼 `--ros-args -p 이름:=값`으로 지정한다. 예전 `--server-ip`, `--open-loop-horizon`, `--arm-action-mode` 옵션은 사용하지 않는다.

주기·작업 문장·그리퍼 폭과 힘은 기존 `config/pick_place.yaml`에서 읽는다. YAML을 바꾼 뒤 `colcon build --packages-select airlab_pick_place`로 다시 빌드한다. 기본 15Hz로 `q_target = q_current + velocity / 15`를 적용하며, 그리퍼 비동기 처리는 `pi05_client.py` 안에 있다. 기존 수집 코드는 수정하지 않는다.

카메라는 수집과 같은 `test_cam`, `hand_cam`을 사용한다. 클라이언트는 큐브·로봇을 자동 초기화하지 않으므로 장면을 준비한 뒤 실행한다. 추론 중에는 `collect_episodes`나 다른 팔·그리퍼 제어 작업을 동시에 실행하지 않는다. 종료는 `Ctrl+C`다.
