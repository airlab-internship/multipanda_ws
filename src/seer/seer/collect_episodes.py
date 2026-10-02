"""
A data collector for Seer.

Dataset format:

0000 (exp_id)
|—— 000000 (episode_id)
    |—— steps
        |—— 0000 (timestep_id, start)
            |—— image_primary.jpg (Eye-on-Base camera rgb image)
            |—— image_wrist.jpg (Eye-on-Hand camera rgb image)
            └── other.npz (robot state, language, action)
        |—— ......
        └── xxxx (timestep_id, end)
|—— 000001 (episode_id)
    |—— steps
        |—— ......
|—— ......
└── 000099 (episode_id)
    |—— steps
        |—— ......

Output per step (matches Seer's documented format):

  - image_primary.jpg
  - image_wrist.jpg
  - other.npz
    - joints
    - gripper_pose
    - gripper_open_state,
    - action_gripper_pose
    - delta_cur_2_last_action
    - language_instruction
"""
import os
import cv2
import time
import math
import rclpy
import random
import numpy as np

from airlab_msgs.action import PickPlace
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation as R

from .params import declare_from_yaml
from .recorder import Recorder
from .sim_bodies import SimBodies


# ============================= Pose helpers =============================
def pose6d(position, quat_xyzw):
    """위치 + 쿼터니언 -> [x, y, z, rx, ry, rz].
    그리퍼가 아래를 볼 때 rx가 ±π 경계라 프레임마다 +π/-π로 튐 -> [0, 2π)로 옮겨 π 근처에서 연속이 되게 함.
    추론할 때 모델에 넣는 gripper pose에도 똑같이 적용해야 함."""
    rx, ry, rz = R.from_quat(quat_xyzw).as_euler('xyz')
    return np.array([*position, rx % (2 * math.pi), ry, rz])


def pose6d_to_mat(p):
    T = np.eye(4)
    T[:3, 3] = p[:3]
    T[:3, :3] = R.from_euler('xyz', p[3:6]).as_matrix()
    return T


def mat_to_pose6d(T):
    return np.concatenate([T[:3, 3], R.from_matrix(T[:3, :3]).as_euler('xyz')])


def compute_delta_actions(steps):
    """
    Seer의 compute_delta_action과 같은 방식: 직전 목표 pose(첫 스텝은 현재 pose) 기준 현재 목표 pose의 상대 변환.
    """
    for i, s in enumerate(steps):
        last = s['gripper_pose'] if i == 0 else steps[i - 1]['action_gripper_pose'][:6]
        cur2last = np.linalg.inv(pose6d_to_mat(last)) @ pose6d_to_mat(s['action_gripper_pose'][:6])
        s['delta'] = np.concatenate([mat_to_pose6d(cur2last), s['action_gripper_pose'][6:]])


def filter_idle(steps, thresh):
    """
    Seer의 filter_real_data와 같은 기준: 위치 delta가 thresh 이상이거나 그리퍼 명령이 바뀐 스텝만 남김.
    마지막으로 남긴 스텝 이후(에피소드 끝)는 그대로 둠.
    """
    keep, prev_g = [], steps[0]['action_gripper_pose'][6]
    for i, s in enumerate(steps):
        g = s['action_gripper_pose'][6]
        if np.any(np.abs(s['delta'][:3]) >= thresh) or g != prev_g:
            keep.append(i)
        prev_g = g
    if not keep:
        return []
    keep += range(keep[-1] + 1, len(steps))
    return [steps[i] for i in keep]


class SeerDataCollector(Node):
    def __init__(self):
        super().__init__('seer_data_collector')
        self.p = declare_from_yaml(self)  # config/pick_place.yaml의 seer_data_collector 블록
        # <output_dir>/<exp_id>/<episode_id>/steps/<step_id>  (output_dir = Seer의 root_dir/real_dataset_names)
        self.exp_dir = os.path.join(os.path.expanduser(self.p('output_dir')), f"{self.p('exp_id'):04d}")
        os.makedirs(self.exp_dir, exist_ok=True)
        zones = list(self.p('start_zones'))
        self.zones = list(zip(zones[0::2], zones[1::2]))
        self.marker_deck = []  # 마커용 칸: 무작위 순서로 한 번씩 사용, 다 쓰면 다시 섞음

        self.bodies = SimBodies(self)
        self.pick_place = ActionClient(self, PickPlace, 'pick_place')
        self.go_home = self.create_client(Trigger, 'go_home')
        if not self.pick_place.wait_for_server(timeout_sec=10.0) or not self.go_home.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('pick_place_server is not running')
        self.recorder = Recorder(self, self.p('record_rate'))

        # Seer는 에피소드 폴더를 000000부터 빈 번호 없이 읽음 -> 저장한 에피소드만 번호를 올림
        self.episode_id = self.p('start_index')


    def _wait(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()


    def _start_xy(self, index):
        cx, cy = self.zones[index % len(self.zones)]
        half = self.p('start_zone_half')
        return cx + random.uniform(-half, half), cy + random.uniform(-half, half)


    def _marker_xy(self, cube_xy):
        """
        섞인 칸 순서(marker_deck)에서 다음 칸을 꺼내 그 안에서 무작위.
        marker_range 밖이거나 큐브와 min_marker_dist 이내면 다시 뽑고, 칸 전체가 안 되면 그 칸은 미뤄 두고 다음 칸 사용.
        """
        x_min, x_max, y_min, y_max = self.p('marker_range')
        half, min_dist = self.p('start_zone_half'), self.p('min_marker_dist')
        skipped = []
        for _ in range(len(self.zones)):
            if not self.marker_deck:
                self.marker_deck = random.sample(self.zones, len(self.zones))
            cx, cy = self.marker_deck.pop()
            for _ in range(100):
                x, y = cx + random.uniform(-half, half), cy + random.uniform(-half, half)
                if x_min <= x <= x_max and y_min <= y <= y_max and \
                        math.hypot(x - cube_xy[0], y - cube_xy[1]) >= min_dist:
                    self.marker_deck[:0] = skipped  # 미뤄 둔 칸은 이번 순서의 마지막에 사용
                    return x, y
            skipped.append((cx, cy))
        raise RuntimeError('no marker position far enough from the cube')


    def run_episode(self, index):
        log = self.get_logger()
        obj, marker = self.p('object_body'), self.p('marker_body')
        start_x, start_y = self._start_xy(index)
        target_x, target_y = self._marker_xy((start_x, start_y))

        # 기록 밖에서 home 복귀 -> 마커·큐브 배치 (모든 에피소드가 같은 자세에서 시작)
        home = self._wait(self.go_home.call_async(Trigger.Request()), 30.0)
        if home is None or not home.success:
            log.error('go_home failed')
            return False
        if not self.bodies.place(marker, target_x, target_y, self.p('marker_z')):
            log.error(f'cannot move {marker}')
            return False
        if not self.bodies.place(obj, start_x, start_y, self.p('object_reset_z')):
            log.error(f'cannot move {obj}')
            return False
        time.sleep(1.0)  # 큐브가 테이블에 안착할 시간

        log.info(f'[episode {index}] start ({start_x:.3f}, {start_y:.3f}) -> target ({target_x:.3f}, {target_y:.3f})')
        self.recorder.start()
        goal = PickPlace.Goal(object_name=obj, place_x=target_x, place_y=target_y)
        handle = self._wait(self.pick_place.send_goal_async(goal), 15.0)
        wrapped = None if handle is None or not handle.accepted else \
            self._wait(handle.get_result_async(), self.p('result_timeout'))
        self.recorder.stop()

        if wrapped is None:
            if handle is not None and handle.accepted:
                # 취소하지 않으면 서버가 이전 goal을 계속 실행하는 중에 다음 에피소드가 큐브를 옮겨 버림
                self._wait(handle.cancel_goal_async(), 10.0)
            success, distance, message = False, -1.0, 'no result (rejected or timed out)'
        else:
            r = wrapped.result
            success, distance, message = r.success, r.final_distance, r.message

        # 실패한 시연은 학습 데이터로 쓰지 않음
        if not success:
            log.warn(f'[episode {index}] failed ({message}), not saved')
            return False
        if len(self.recorder) == 0:
            log.error(f'[episode {index}] no samples recorded, not saved')
            return False
        return self.save_episode(index)


    def build_steps(self, data):
        """
        recorder.data (필드별 리스트) -> Seer 스텝 리스트.
        action = 다음 스텝의 명령 pose·그리퍼 (마지막 프레임은 다음 목표가 없어서 제외).
        """
        open_thresh = self.p('open_thresh')
        steps = []
        for t in range(len(data['timestamps']) - 1):
            steps.append({
                'primary': data['agent_image'][t],
                'wrist': data['wrist_image'][t],
                'joints': np.asarray(data['joint_positions'][t][:7]),  # 손가락 2개 제외, Seer는 7개
                'gripper_pose': pose6d(data['ee_position'][t], data['ee_orientation'][t]),
                'gripper_open_state': 1.0 if data['gripper_width'][t] > open_thresh else -1.0,
                'action_gripper_pose': np.concatenate([
                    pose6d(data['action_ee_position'][t + 1], data['action_ee_orientation'][t + 1]),
                    [1.0 if data['action_gripper_width'][t + 1] > open_thresh else -1.0]]),
            })
        if not steps:
            return []
        compute_delta_actions(steps)
        steps = filter_idle(steps, self.p('idle_thresh'))
        compute_delta_actions(steps)  # 남은 스텝끼리 다시 이어지도록 재계산
        return steps


    def save_episode(self, index):
        log = self.get_logger()
        raw = len(self.recorder)
        steps = self.build_steps(self.recorder.data)
        if len(steps) < self.p('min_steps'):
            log.warn(f'[episode {index}] only {len(steps)} usable steps (raw {raw}), not saved')
            return False

        episode_dir = os.path.join(self.exp_dir, f'{self.episode_id:06d}')  # e.g. <output_dir>/0000/000000
        if os.path.exists(episode_dir):
            raise RuntimeError(f'{episode_dir} already exists (start_index를 저장된 에피소드 수로 지정)')

        language_instruction = np.frombuffer(self.p('instruction').encode('utf-8'), dtype=np.uint8)
        for t, s in enumerate(steps):
            step_dir = os.path.join(episode_dir, 'steps', f'{t:04d}')
            os.makedirs(step_dir)
            cv2.imwrite(os.path.join(step_dir, 'image_primary.jpg'), s['primary'])
            cv2.imwrite(os.path.join(step_dir, 'image_wrist.jpg'), s['wrist'])
            np.savez_compressed(
                os.path.join(step_dir, 'other.npz'),
                joints=s['joints'].astype(np.float32),
                gripper_pose=s['gripper_pose'].astype(np.float32),
                gripper_open_state=np.array(s['gripper_open_state'], dtype=np.float32),
                action_gripper_pose=s['action_gripper_pose'].astype(np.float32),
                delta_cur_2_last_action=s['delta'].astype(np.float32),
                language_instruction=language_instruction,
            )

        dpos = np.abs(np.array([s['delta'][:3] for s in steps]))
        log.info(f'[episode {index}] {len(steps)} steps (raw {raw}) -> {episode_dir} '
                 f'(|dpos| max {dpos.max():.4f} m, 학습 시 0.02로 정규화)')
        self.episode_id += 1
        return True


def main():
    rclpy.init()
    node = SeerDataCollector()
    try:
        first = node.p('start_index')
        for index in range(first, first + node.p('num_episodes')):
            if not node.run_episode(index) and node.p('stop_on_failure'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
