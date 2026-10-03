# OpenPI: AIRLAB 학습·추론용 사본

`~/pi0.5`에서 π0.5 학습과 WebSocket 추론에 필요한 소스를 가져온 폴더다.
원본 커밋: `83a35c9d75ae2fa52bc952b400156960fd340535`.
홈 원본과 기존 `airlab-internship/openpi`는 변경하지 않았다.

## 폴더 구성

```text
openpi/
├── pyproject.toml            # Python 의존성 및 uv workspace
├── uv.lock                   # 원본의 고정된 의존성 버전
├── .python-version           # Python 3.11
├── COLCON_IGNORE             # ROS colcon 빌드에서 제외
├── src/openpi/
│   ├── airlab/               # 직접 실행·수정하는 파일
│   │   ├── config.py        # pi05_droid_finetune 학습 설정
│   │   ├── train.py         # JAX/LoRA 학습
│   │   ├── serve_policy.py  # GPU 추론 서버
│   │   ├── convert_dataset.py # H5 → LeRobot 변환 (custom velocity)
│   │   ├── compute_norm_stats.py # 정규화 통계 계산
│   │   └── compute_nonidle_indices.py # non-idle 전처리
│   ├── training/            # 데이터 로더·체크포인트 등 공통 학습 코드
│   ├── models/               # 모델 구현
│   ├── policies/             # 입력 변환과 정책 실행
│   ├── serving/              # WebSocket 서버
│   └── shared/               # 정규화·다운로드 등 공통 기능
└── packages/openpi-client/   # 추론 요청·응답과 영상 처리 라이브러리
```

소스는 **JAX π0.5 LoRA + 로컬 LeRobot + WebSocket 추론** 경로로 정리했다. ALOHA/LIBERO 정책·설정, FAST 모델·토크나이저, RLDS 로더, PyTorch 모델, 다른 환경의 기본 서버 설정, 사용하지 않는 client runtime 코드는 제거했다. 학습 설정은 기존 `pi05_droid_finetune` 하나만 제공한다. `pi0.py`와 `pi0_config.py`는 π0.5도 사용하는 공통 모델 구현이므로 유지한다. 이 사본은 standalone Python 프로젝트이며 ROS 패키지가 아니다.

`uv` 실행 파일은 복사 대상이 아니다. 각 실행 컴퓨터에 uv를 설치하고 환경 파일로 `.venv`를 생성한다. 요청에 따라 `pyproject.toml`, `uv.lock`, `.python-version`, 클라이언트의 `pyproject.toml`은 원본 그대로 유지했다. 소스를 줄였지만 설치 의존성 목록은 축소하지 않았다. PyTorch는 LeRobot 데이터 로더에서도 사용하므로 JAX 학습에서도 필요하다.

## 1. uv 설치 및 Python 환경 구성

아래 기본 순서는 **호스트 터미널**에서 실행한다. `uv sync`는 uv 자체를 설치하는 명령이 아니다. `uv: command not found`가 나오면 먼저 uv를 설치한다. 이미 `uv --version`이 되면 설치 단계는 건너뛴다.

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source "$HOME/.local/bin/env"
uv --version
```

공식 설치 안내: <https://docs.astral.sh/uv/getting-started/installation/>.
새 터미널에서 명령을 찾지 못하면 `source "$HOME/.local/bin/env"`를 다시 실행한다. `sudo`는 필요 없다.

프로젝트 환경 설치:

```bash
cd ~/airlab-internship/multipanda_ws/src/openpi
GIT_LFS_SKIP_SMUDGE=1 uv sync --locked
```

- `uv sync`: 프로젝트의 `.venv`에 의존성과 현재 프로젝트를 editable 방식으로 설치한다. 별도의 `uv pip install -e .`는 필요 없다.
- `--locked`: 기존 `uv.lock`을 변경하지 않는다. 프로젝트 설정과 lock이 맞지 않으면 중단한다.
- `GIT_LFS_SKIP_SMUDGE=1`: Git LFS 파일 자동 다운로드를 건너뛴다. uv 설치와는 별개다.
- LeRobot은 프로젝트에 고정된 Git revision으로 설치되고 `h5py`도 의존성으로 함께 설치된다. 별도로 최신 LeRobot을 `pip install`하지 않는다.
- 이 명령은 변환용 패키지 두 개만이 아니라 JAX·PyTorch 등을 포함한 전체 환경을 설치하므로 다운로드 용량과 시간이 클 수 있다.
- 이 OpenPI 사본에는 submodule이 없으므로 이 폴더에서 `git submodule update --init --recursive`는 필요 없다. ROS 워크스페이스의 설치 과정과는 별개다.

설치 확인:

```bash
uv run python -c "import h5py; from lerobot.common.datasets.lerobot_dataset import LeRobotDataset; print('설치 완료')"
```

이후 `uv run` 명령은 모두 `pyproject.toml`이 있는 `multipanda_ws/src/openpi`에서 실행한다. 별도 GPU 서버도 OpenPI 폴더를 복사한 뒤 그 서버에서 uv와 환경을 설치한다. `.venv` 자체를 복사하지 않는다.

## 2. 데이터 위치

기본 데이터 경로는 이 폴더 위치를 기준으로 `../../data/lerobot`이다.

```text
multipanda_ws/data/
├── episodes/              # 기존 원본 H5
└── lerobot/
    ├── data/
    ├── meta/
    └── nonidle_indices.npy  # 전처리 후 생성
```

서버의 배치 위치가 다르면 실제 경로를 지정한다. 로컬 경로 예시:

```bash
export AIRLAB_LEROBOT_ROOT="$HOME/airlab-internship/multipanda_ws/data/lerobot"
```

학습 설정의 `repo_id="airlab/pick_place"`는 식별자다. 데이터 로더가 `root`를 직접 지정하므로 실제 디렉터리 아래에 `airlab/pick_place` 폴더가 추가로 생기지 않는다.

## 3. H5 → LeRobot 변환

현재 터미널이 `~/airlab-internship/multipanda_ws`라면 먼저 `cd src/openpi`로 이동한다. 이미 OpenPI 폴더라면 바로 실행한다.

```bash
uv run src/openpi/airlab/convert_dataset.py --data-dir ../../data/episodes
```

기본 출력은 입력 `episodes/` 옆의 `lerobot/`이다. 다른 위치는 `--output-dir /path/to/lerobot`으로 지정한다. 기존 출력이 있으면 삭제하지 않고 중단한다. 별도 업로드는 하지 않는다.

`../../data/episodes`는 `multipanda_ws/data/episodes`를 뜻한다. 기존 `ep_0000.h5`, `ep_0001.h5`, `ep_0002.h5`는 읽기 전용으로 사용하며 변경하지 않는다. 변환 결과는 `multipanda_ws/data/lerobot/{data,meta}`에 생성된다. `nonidle_indices.npy`는 다음 전처리 단계에서 생성된다.

원본 H5 **재수집**은 변환과 다르다. 수집기는 같은 출력 폴더와 번호이면 기존 H5를 덮어쓴다. 세 에피소드 뒤에 추가하려면 수집기의 `start_index`를 3부터 지정한다.

- DROID 예제와 같은 필드 이름을 사용하지만 **custom action만** 지원한다. `--action-mode` 옵션은 없다.
- `joint_names`에서 Panda 팔 7축을 선택하고 `actions[:7] = (q[t+1] - q[t]) / (timestamps[t+1] - timestamps[t])` [rad/s]로 계산한다. DROID 명령 스케일로 변환하지 않는다.
- 관측 그리퍼는 `gripper_width`, action 그리퍼는 `action_gripper_width`를 사용한다. 둘 다 `clip(1 - width / 0.08, 0, 1)`로 변환한다.
- 다음 위치가 없는 마지막 프레임은 제외한다. 언어 지시문은 H5의 `instruction` 속성을 사용한다.
- BGR 이미지는 RGB로 변환한다. 기존 H5의 640×480 해상도를 유지하고, 모델 입력 크기 조정은 기존 OpenPI 전처리에 맡긴다.
- LeRobot의 명목 FPS는 기본 15이며 속도 차분에는 실제 timestamp 간격을 사용한다. 시간 재샘플링은 하지 않으므로 기록 주기가 다르면 `--fps`를 맞춰야 한다.

## 4. Non-idle 인덱스 생성

변환이 완료된 뒤 같은 OpenPI 폴더에서 실행한다.

```bash
export AIRLAB_LEROBOT_ROOT="$(realpath ../../data/lerobot)"

uv run src/openpi/airlab/compute_nonidle_indices.py \
  --data-dir "$AIRLAB_LEROBOT_ROOT"

export AIRLAB_NONIDLE_INDICES="$AIRLAB_LEROBOT_ROOT/nonidle_indices.npy"
```

인덱스는 학습 시작점으로 사용할 프레임 목록이다. 원본 H5나 변환된 에피소드 자체를 잘라내지는 않는다. 다시 실행하면 `nonidle_indices.npy`는 덮어쓴다.

현재 `compute_nonidle_indices.py`의 `VELOCITY_EPS`는 **0**이다. 판정식이 `abs(joint_vel) < VELOCITY_EPS`여서 현재 값으로는 arm idle 구간을 검출하지 않는다. 다만 구간 최소 길이 조건과 마지막 `FILTER_LAST_N=10` 프레임 제외는 적용된다. 실제 idle 제거를 사용할 때는 이 임계값을 데이터에 맞게 양수로 정해야 한다. 이번 문서 수정에서는 코드 설정값을 변경하지 않았다.

`AIRLAB_NONIDLE_INDICES`를 설정하지 않으면 전체 프레임을 학습 시작점으로 사용한다. 필터 없이 실행하려면 `unset AIRLAB_NONIDLE_INDICES`를 사용한다. 데이터나 전처리 설정이 바뀌면 인덱스와 정규화 통계를 다시 만든다.

## 5. 별도 GPU 서버에서 정규화 계산·학습

OpenPI 폴더와 변환된 `lerobot/` 전체를 서버에 복사하고 1절처럼 환경을 설치한다. 아래 경로는 서버에 실제로 복사한 위치로 바꾼다. 로컬의 경로·환경변수는 서버로 자동 전달되지 않는다.

```bash
# 서버의 OpenPI 프로젝트 루트에서 실행
export AIRLAB_LEROBOT_ROOT="/실제/서버/경로/lerobot"
export AIRLAB_NONIDLE_INDICES="$AIRLAB_LEROBOT_ROOT/nonidle_indices.npy"

CUDA_VISIBLE_DEVICES=0 uv run src/openpi/airlab/compute_norm_stats.py \
  --config-name pi05_droid_finetune

CUDA_VISIBLE_DEVICES=0 uv run src/openpi/airlab/train.py pi05_droid_finetune \
  --exp-name=pick_place
```

인덱스를 사용한다면 서버에도 `nonidle_indices.npy`를 함께 복사하거나 서버에서 다시 생성한다. 정규화 계산과 학습에서 동일한 환경변수를 사용한다. `CUDA_VISIBLE_DEVICES=0`은 사용할 GPU 번호로 바꾼다.

현재 `airlab/config.py`의 설정은 `pi05_droid_finetune`이며 실제 시작 가중치는 **pi05_base**, LoRA·batch size 1·**20,000 steps**·action horizon 16이다. 이름이 DROID여도 action은 custom rad/s이며 정규화는 위 명령으로 계산한 데이터 통계를 사용한다. WandB는 기본 활성화되어 있다. 계정을 연결하지 않으려면 학습 명령에 `--no-wandb-enabled`를 추가한다. API 키를 README나 코드에 적지 않는다.

## 6. GPU 서버에서 추론 서버 실행

학습이 완료된 후 실제 체크포인트 번호를 넣는다.

```bash
CUDA_VISIBLE_DEVICES=0 XLA_PYTHON_CLIENT_PREALLOCATE=false \
uv run src/openpi/airlab/serve_policy.py --port=8152 \
  --policy.config=pi05_droid_finetune \
  --policy.dir="checkpoints/pi05_droid_finetune/pick_place/<step>"
```

위 `<step>`은 자리표시자이므로 실제 숫자로 바꿔 실행한다. 서버는 학습과 같은 설정 및 체크포인트의 정규화 통계를 사용한다. 이제 체크포인트 방식만 지원하므로 이전 명령의 `policy:checkpoint` 토큰은 생략한다. PyTorch `model.safetensors` 대신 JAX `params/` 체크포인트를 사용한다.

추론 요청을 보내는 Python 라이브러리는 `packages/openpi-client`에 포함했다. 로봇을 움직이는 ROS 노드는 기존 `airlab_pick_place/airlab_pick_place/pi05_client.py`에서 관리한다. 그 노드의 토픽·그리퍼 액션과 현재 시뮬레이터의 일치 여부는 이번 파일 복사에서 수정하거나 검증하지 않았다.

## 7. Docker와 함께 사용하기

Docker는 ROS·시스템 환경을, uv는 OpenPI Python 환경을 관리하므로 함께 사용할 수 있다. 기본 작업 분담은 다음과 같다.

- ROS·MuJoCo·H5 수집: 기존 `multipanda_ws/run.sh`로 진입하는 컨테이너.
- H5 변환·전처리: 호스트의 OpenPI uv 환경.
- 정규화 계산·학습·추론 서버: 별도 GPU 서버의 OpenPI uv 환경.

현재 `run.sh`는 워크스페이스 전체를 컨테이너의 `/home/developer/multipanda_ws`에 마운트한다. 따라서 호스트 `~/airlab-internship/multipanda_ws/data`와 컨테이너 `/home/developer/multipanda_ws/data`는 같은 데이터를 본다. 복사할 필요는 없다.

호스트에 설치한 uv가 컨테이너에 자동 설치되지는 않는다. 기존 Dockerfile에는 ROS 수집용 시스템 Python의 `h5py` 설치가 있지만, 이는 호스트나 OpenPI `.venv`의 설치와 별개다.

OpenPI도 컨테이너 안에서 실행하려면 컨테이너 안에 uv를 설치하고 환경을 별도로 만든다. 현재 마운트는 `.venv`도 공유하므로 호스트의 `.venv`를 그대로 사용하지 않도록 아래 변수를 **컨테이너 터미널마다** 설정한다.

```bash
# 컨테이너 안에서만 실행
cd /home/developer/multipanda_ws/src/openpi
export UV_PROJECT_ENVIRONMENT="$HOME/.venvs/openpi"
GIT_LFS_SKIP_SMUDGE=1 uv sync --locked
uv run src/openpi/airlab/convert_dataset.py --data-dir ../../data/episodes
```

컨테이너에서도 1절의 uv 설치를 먼저 완료해야 한다. ROS 실행에는 ROS Python 환경을 유지하고, OpenPI 실행에만 `uv run`을 사용한다. Docker 안에서 GPU 학습·추론까지 한다면 컨테이너에 GPU가 전달되는지도 확인해야 한다. ROS 컨테이너의 클라이언트는 GPU 서버 주소와 포트 8152로 연결하며, 서버에 대한 네트워크 접근이 필요하다.

## 검증 범위

- 데이터 경로를 개인 서버의 절대 경로에서 workspace 기본값 + 환경변수로 변경.
- non-idle 필터는 환경변수로 명시적으로 활성화.
- GPU 번호를 실행 명령에서 선택하도록 변경.
- π0.5 모델의 핵심 연산, 학습 루프, LeRobot/non-idle 로딩을 유지하고 미사용 경로를 제거.
- 환경 파일과 의존성 버전은 유지.
- 파일 구성, Python 문법, 내부 import 대상 존재 여부를 확인.
- H5 수치 데이터와 custom 속도·그리퍼 변환, RGB 변환 및 기존 출력 보호 로직을 검사.
- 전체 LeRobot 파일 저장, 의존성 설치 완료, GPU 학습·추론 실행은 아직 검증하지 않음.
