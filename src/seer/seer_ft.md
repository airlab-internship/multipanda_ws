# Seer Fine-Tuning 가이드

[🤔 Seer란?](#seer란?) <br/>
[💾 학습 데이터](#학습-데이터) <br/>
[📚 데이터 수집](#데이터-수집) <br/>
[🏋️ 학습](#학습) <br/>
[🔌 노드 구성](#노드-구성) <br/>
[🤖 추론](#추론) <br/>
[⚠️ 공동 작업 시 맞춰야 하는 것들](#맞춰야-하는-것들) <br/>
[⭐️ 수정 내역](#수정-내역) <br/>

## Seer란?

![Seer 아키텍처](https://github.com/InternRobotics/Seer/raw/main/assets/seer_method.jpg)

카메라 영상(이미지 시퀀스), 로봇 상태, 언어 지시를 입력 받아 로봇 팔의 end-effector 동작을 출력하는 policy 모델로, 미래 관측을 먼저 예측하고, 그 예측을 조건으로 행동을 추론하는 방식으로 작동합니다. 자세한 사항은 [논문](https://arxiv.org/html/2412.15109v1)을 참고해 주세요.

## 학습 데이터

[공식 레포지토리](https://github.com/InternRobotics/Seer)를 확인해 보면, 모델을 fine-tuning하는 데 필요한 데이터를 소개하는 [문서](https://github.com/InternRobotics/Seer/blob/main/docs/REAL-WORLD_POSTPROCESS.md)가 있습니다. 

### 필요한 데이터

| 데이터                    | 설명                                 | 관련 토픽                                       |
| ----------------------- | ----------------------------------- | --------------------------------------------- |
| image_primary.jpg       | 제3자 시점 RGB 이미지 (640x480 권장)     | /mujoco_server/cameras/test_cam/rgb/image_raw |
| image_wrist.jpg         | Gripper 카메라 RGB 이미지 (640x480 권장) | /mujoco_server/cameras/hand_cam/rgb/image_raw |
| gripper_pose            | Gripper pose                        | /tf                                           |
| gripper_open_state      | Gripper 상태 (-1: 닫힘, 1: 열림)        | /panda_gripper_sim_node/joint_states          |
| joints                  | 관절 qpos                             | /joint_states                                 |
| language_instruction    | 태스크 명령                            |                                               |
| action_gripper_pose     | Gripper 목표 pose                     |                                               |
| delta_cur_2_last_action | 이전 gripper pose로부터의 delta         |                                               |

- `gripper_pose`: [x, y, z, rx, ry, rz] 형태이며, x, y, z는 m, rx, ry, rz는 rad 단위입니다.
- `action_gripper_pose`: [x, y, z, rx, ry, rz, gripper] 형태이며, x, y, z는 m, rx, ry, rz는 rad, gripper는 -1이면 닫힘, 1이면 열림을 의미합니다.
- `delta_cur_2_last_action`: [dx, dy, dz, drx, dry, drz, gripper] 형태이며, gripper_pose, action_gripper_pose를 사용해 계산합니다.


### 데이터셋 구조

```markdown
0000 실험 ID
|—— 000000 에피소드 ID
    |—— steps 
        |—— 0000 타임스텝 ID (시작 스텝)
            |—— image_primary.jpg 제3자 시점 RGB 이미지
            |—— image_wrist.jpg   그리퍼 시점 RGB 이미지
            └── other.npz         로봇 상태, 명령, 행동
        |—— ......
        └── xxxx 타임스텝 ID (종료 스텝)
|—— 000001 에피소드 ID
    |—— steps
        |—— ......
|—— ......
└── 000099 에피소드 ID
    |—— steps
        |—— ......
```

공식적으로 추천하는 데이터셋 구조입니다. 데이터 수집 단계에서는 구조가 달라도 문제가 되진 않지만, 이 구조를 사용하도록 하드코딩이 되어 있기 때문에 맞추는 것을 추천합니다.


## 데이터 수집

`seer_collect_episodes.py`는 pick-and-place를 반복 실행하면서 데이터를 [데이터셋 구조](#데이터셋-구조)에 맞춰 바로 저장하는 노드입니다. 수집된 에피소드는 별도의 변환 과정 없이 학습에 사용할 수 있습니다.

### 동작 방식

1. 에피소드마다 pick-and-place 동작을 실행합니다(큐브 시작/놓기 위치를 랜덤으로 설정하려면 `--random`을 `true`로 설정해 주세요).
2. 실행되는 동안 15 Hz 주기로 이미지, 관절, gripper pose, 그리퍼 명령을 버퍼에 기록합니다. (`/episode_active == false`인 동안에는 기록하지 않습니다.)
3. 실행이 끝나면 버퍼를 [후처리](#후처리)한 뒤 step 폴더로 저장합니다.
4. 실행이 실패하거나 너무 짧게 끝난 에피소드는 버리고, 같은 번호로 다시 수집합니다.

### 구독 토픽

| 토픽                                                           | 타입                     | 용도                                         |
| ------------------------------------------------------------ | ------------------------ | ------------------------------------------ |
| /mujoco_server/cameras/test_cam/rgb/image_raw                | sensor_msgs/Image        | image_primary.jpg                          |
| /mujoco_server/cameras/hand_cam/rgb/image_raw                | sensor_msgs/Image        | image_wrist.jpg                            |
| /joint_states                                                | sensor_msgs/JointState   | joints (panda_joint1 ~ 7)                  |
| /gripper_command                                             | sensor_msgs/JointState   | gripper_open_state (`position[0] > 0`이면 1) |
| /episode_active                                              | std_msgs/Bool            | 기록 구간                                      |
| /tf                                                          | tf2_msgs/TFMessage       | gripper_pose (panda_link0 → panda_hand_tcp) |


### 후처리

- `action_gripper_pose`: 다음 프레임의 gripper pose와 그리퍼 상태를 현재 프레임의 목표로 사용합니다. 마지막 프레임은 목표가 없으므로 제외합니다.
- 놓은 뒤 구간 정리: 마지막으로 그리퍼가 닫힘 → 열림으로 바뀐 뒤 `--post_release`(기본 5) 프레임까지만 남깁니다.
- 정지 프레임 제거: 위치 변화가 `--idle_thresh`(기본 0.5 mm) 미만이고 그리퍼 상태도 그대로인 프레임을 제거합니다. Seer의 `filter_real_data`와 같은 기준이며, 제거 후 목표를 다시 계산합니다.
- `delta_cur_2_last_action`: Seer의 `compute_delta_action`과 같은 방식으로 계산합니다. 첫 스텝은 현재 pose, 이후 스텝은 이전 스텝의 목표 pose 기준 상대 변환입니다.
- 학습 window가 10이므로 남은 스텝이 `--min_steps`(기본 15)보다 적으면 저장하지 않습니다.
- `language_instruction`은 UTF-8 바이트(`uint8` 배열)로 저장합니다. Seer가 `.tobytes().decode("utf-8")`로 읽기 때문에, 문자열 배열로 저장하면 깨집니다.

### 실행

시뮬레이터를 먼저 실행한 뒤, 컨테이너 안에서 실행합니다.

```bash
~/multipanda_ws/run.sh
python3 src/airlab_pick_place/airlab_pick_place/seer_collect_episodes.py \
    --episodes 100 \
    --root ~/multipanda_ws/data/seer \
    --primary_topic /mujoco_server/cameras/test_cam/rgb/image_raw \
    --wrist_topic /mujoco_server/cameras/hand_cam/rgb/image_raw
```


### 주요 파라미터

| 파라미터            | 기본값                                  | 설명                                    |
| ----------------- | ------------------------------------- | ------------------------------------- |
| `--episodes`      | 1                                     | 수집할 에피소드 수                            |
| `--start_from`    | 0                                     | 시작 에피소드 번호 (이어서 수집할 때)                |
| `--root`          | ~/seer_data                           | 데이터 저장 위치 (Seer의 `root_dir`)          |
| `--dataset_name`  | pick_place                            | 데이터셋 이름 (Seer의 `real_dataset_names`)  |
| `--exp_id`        | 0                                     | 실험 ID                                 |
| `--instruction`   | Pick up the blue cube, and place it.  | 언어 지시                                 |
| `--hz`            | 15.0                                  | 기록 주기 (Hz)                            |
| `--cube_body`     | cube                                  | 큐브의 MuJoCo body 이름                    |
| `--cube_range`    | 0.30 0.60 -0.30 0.30                  | 큐브 시작 범위 (x_min x_max y_min y_max, panda_link0 기준) |
| `--place_range`   | 0.30 0.60 -0.30 0.30                  | 놓을 위치 범위 (panda_link0 기준)              |
| `--start_jitter`  | 0.05                                  | 시작 관절 무작위 오프셋 (rad)                    |

### 저장 결과

```markdown
<root>
|—— panda_pick_place          데이터셋 이름
    └── 0000                  실험 ID
        |—— 000000            에피소드 ID
            |—— steps
                |—— ......
            └── episode_meta.json   큐브 위치, 놓을 위치, 시작 관절 등
        |—— ......
└── _meta                     mujoco_pick_place가 남기는 메타데이터 (임시)
```

## 학습

실제 학습은 공식 레포지토리에서 제공하는 [스크립트](https://github.com/InternRobotics/Seer/blob/main/scripts/REAL/single_node_ft.sh)를 활용합니다. 

### 1. 체크포인트 다운로드

| 체크포인트              | 링크                                                                                                      |
| ------------------ | ------------------------------------------------------------------------------------------------------- |
| MAE ViT            | [다운로드](https://drive.google.com/file/d/1bSsvRI4mDM3Gg51C6xO0l9CbojYw3OEt/view?usp=sharing)              |
| Seer (DROID 사전학습) | [다운로드](https://drive.google.com/drive/folders/1rT8JKLhJGIo97jfYUm2JiFUrogOq-dgJ?usp=drive_link)        |

### 2. 데이터 인덱스 생성

Seer는 학습할 때 `data_info/<데이터셋 이름>.json`을 읽기 때문에, 먼저 이 파일을 만들어야 합니다. 정지 프레임 제거는 수집 단계에서 이미 했으므로 `filter_real_data`는 생략해도 됩니다.

```python
from utils.real_ft_data import make_aug_short_real_dataset_info

make_aug_short_real_dataset_info(
    root_path="/abs/path/to/data/seer/panda_pick_place",  # ~ 대신 절대 경로
    root_info_path="./data_info",
    dataset_name="panda_pick_place",
    sequence_length=7,
    action_pred_steps=3,
)
```

### 3. 스크립트 수정

코드 상단에 “NEED TO CHANGE”라고 쓰여 있는 부분을 수정해 주세요.

| 변수                              | 값                                            |
| ------------------------------- | -------------------------------------------- |
| `save_checkpoint_path`          | 체크포인트를 저장할 경로                                |
| `root_dir`                      | 수집할 때 지정한 `--root`                           |
| `real_dataset_names`            | 수집할 때 지정한 `--dataset_name`                   |
| `finetune_from_pretrained_ckpt` | Seer (DROID 사전학습) 체크포인트 경로                    |
| `vit_checkpoint_path`           | MAE ViT 체크포인트 경로                             |

스크립트는 GPU 8개(`node_num=8`)를 기준으로 작성되어 있으므로, 사용하는 GPU 개수에 맞춰 수정해 주세요.

### 4. 실행

```bash
bash scripts/REAL/single_node_ft.sh
```

## 노드 구성

VLA는 CPU 환경에서 돌리기 어렵기 때문에, 추론은 GPU 서버에서 하고 시뮬레이터 쪽 노드가 SSH 포트 포워딩으로 서버와 통신하는 방식을 사용합니다.

```bash
ssh -N -L 5001:127.0.0.1:5001 user@host
```

```markdown
[시뮬레이터 PC]                                 [GPU 서버]
ROS 2 노드 ── 127.0.0.1:5001 ══ SSH 터널 ══▶ 127.0.0.1:5001  Seer 추론
```

## 추론

### 입력

Seer 모델이 입력으로 받는 값은 아래와 같습니다. 시뮬레이터 쪽 노드가 ROS 2 토픽에서 값을 모아 매 스텝 전달합니다.

| 입력          | 내용                                                    | 관련 토픽                                          |
| ----------- | ----------------------------------------------------- | ---------------------------------------------- |
| image       | 고정 3인칭 카메라(primary) + 손목 카메라(wrist) RGB 이미지            | /mujoco_server/cameras/test_cam, hand_cam      |
| states      | gripper pose [x, y, z, rx, ry, rz] + 그리퍼 상태 (-1: 닫힘, 1: 열림) | /tf, 그리퍼 상태                                    |
| instruction | 언어 지시, 예) "Pick up the blue cube."                   |                                                |

- 매 스텝 한 장씩 넣으면 모델이 **최근 7 스텝**을 내부 queue로 유지합니다.
- states는 수집할 때와 같은 좌표계와 규약을 사용해야 합니다. ([맞춰야 하는 것들](#맞춰야-하는-것들) 참고)

### 출력

Seer는 정규화된 delta pose와 그리퍼 명령 `[dx, dy, dz, drx, dry, drz, gripper]`를 출력합니다. 로봇에 보내기 전에 절대 목표 pose로 바꿔야 합니다.

```python
target_pos *= 0.02    # max_rel_pos (m), 학습과 동일
target_euler *= 0.05  # max_rel_orn (rad), 학습과 동일
last_target = last_target @ pose6d_to_mat(np.concatenate([target_pos, target_euler]))
target_pose = mat_to_pose6d(last_target)  # 로봇에 보낼 절대 목표 pose
```

- delta는 측정된 현재 pose가 아니라 **직전 목표 pose**에 누적합니다. 처음에는 현재 gripper pose로 시작합니다.
- gripper는 -1(닫힘) 또는 1(열림)입니다.
- 학습 데이터와 같은 **15 Hz**로 반복합니다.

## 맞춰야 하는 것들

공동 작업 시 아래 항목이 다르면 데이터를 합치거나 모델을 공유할 수 없습니다.

| 항목         | 값                                                    | 비고                                     |
| ---------- | ---------------------------------------------------- | -------------------------------------- |
| 좌표계        | panda_link0 기준, EE는 panda_hand_tcp                    | MuJoCo world와 x, y 부호가 반대               |
| 회전 표현      | Euler `"xyz"`, rad                                   | scipy `R.as_euler("xyz")`              |
| 그리퍼 상태     | 1: 열림, -1: 닫힘                                        |                                        |
| 주기         | 15 Hz                                                | 수집과 추론이 같아야 함                          |
| 카메라        | primary = test_cam, wrist = hand_cam, 640x480        | `scene_add_camera.xml`의 카메라 위치·화각 변경 금지 |
| 언어 지시      | 수집과 추론에서 같은 문장                                       |                                        |
| delta 정규화  | 위치 0.02 m, 회전 0.05 rad                               | Seer 기본값                               |
| 데이터 경로     | `<root>/<dataset_name>/<exp_id>/<demo_id>/steps/<step>` | 4자리 / 6자리 / 4자리                       |
| 에피소드 번호    | 000000부터 빈 번호 없이                                     | Seer가 폴더 개수만큼 번호를 순서대로 읽음              |
| 실험 ID      | 사람(또는 수집 세션)마다 다르게                                   | 데이터를 합칠 때 번호 충돌 방지                     |

## 수정 내역


| 날짜         | 내용    |
| ---------- | ----- |
| 2026.10.01 | 최초 생성 |


