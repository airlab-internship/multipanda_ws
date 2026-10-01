"""에피소드 수집기: 에피소드마다 home 복귀 -> 큐브 배치 -> 기록 시작 -> PickPlace goal -> 기록 종료 -> 저장.

pick_place_server가 먼저 떠 있어야 함. 놓을 위치는 마커(place_marker)의 현재 위치를 읽어서 사용.
"""
import math
import os
import random
import time

import rclpy
from airlab_msgs.action import PickPlace
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from .params import declare_from_yaml
from .recorder import Recorder
from .sim_bodies import SimBodies

class EpisodeCollector(Node):
    def __init__(self):
        super().__init__('episode_collector')
        self.p = declare_from_yaml(self)  # config/pick_place.yaml
        self.output_dir = os.path.expanduser(self.p('output_dir'))
        os.makedirs(self.output_dir, exist_ok=True)
        zones = list(self.p('start_zones'))
        self.zones = list(zip(zones[0::2], zones[1::2]))

        self.bodies = SimBodies(self)
        self.pick_place = ActionClient(self, PickPlace, 'pick_place')
        self.go_home = self.create_client(Trigger, 'go_home')
        if not self.pick_place.wait_for_server(timeout_sec=10.0) or not self.go_home.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('pick_place_server is not running')
        self.recorder = Recorder(self, self.p('record_rate'))

    def _wait(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def _start_xy(self, index):
        cx, cy = self.zones[index % len(self.zones)]
        half = self.p('start_zone_half')
        return cx + random.uniform(-half, half), cy + random.uniform(-half, half)

    def run_episode(self, index):
        log = self.get_logger()
        obj = self.p('object_body')
        marker = self.bodies.position(self.p('marker_body'))
        if marker is None:
            log.error('cannot read marker position')
            return False
        target_x, target_y = marker[0], marker[1]
        start_x, start_y = self._start_xy(index)

        # 기록 밖에서 home 복귀 -> 큐브 배치 (모든 에피소드가 같은 자세에서 시작)
        home = self._wait(self.go_home.call_async(Trigger.Request()), 30.0)
        if home is None or not home.success:
            log.error('go_home failed')
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

        if len(self.recorder) == 0:
            log.error(f'[episode {index}] no samples recorded, not saved')
            return False
        path = os.path.join(self.output_dir, f'ep_{index:04d}.h5')
        self.recorder.save(path, {
            'instruction': self.p('instruction'), 'success': bool(success),
            'final_distance': float(distance), 'message': message, 'object_body': obj,
            'object_start_x': start_x, 'object_start_y': start_y,
            'object_start_z': self.p('object_reset_z'),
            'object_target_x': target_x, 'object_target_y': target_y, 'episode_index': index,
        })
        log.info(f'[episode {index}] {len(self.recorder)} frames -> {path} '
                 f'(success={success}, {distance:.3f} m)')
        return bool(success)


def main():
    rclpy.init()
    node = EpisodeCollector()
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
