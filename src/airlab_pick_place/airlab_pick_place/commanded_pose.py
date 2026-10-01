"""명령(commanded) EE pose 추적.

VLA 학습의 action 라벨은 로봇이 실제로 도달한 값이 아니라 명령한 값이어야 함.
Pilz는 실행 전에 시간이 매겨진 궤적을 만들므로, 그 궤적을 현재 시각으로 보간하고 FK를 계산하면
"지금 명령 중인 pose"가 됨. 이동 사이 대기 중에는 마지막 목표를 그대로 유지함.
"""
import threading
import time

from moveit.core.robot_state import RobotState

ARM_GROUP = 'panda_arm'


def _seconds(point):
    return point.time_from_start.sec + point.time_from_start.nanosec * 1e-9


def interpolate(trajectory, elapsed):
    points = trajectory.points
    if elapsed <= _seconds(points[0]):
        return list(points[0].positions)
    for prev, nxt in zip(points, points[1:]):
        t0, t1 = _seconds(prev), _seconds(nxt)
        if elapsed <= t1:
            a = 0.0 if t1 <= t0 else (elapsed - t0) / (t1 - t0)
            return [p + a * (q - p) for p, q in zip(prev.positions, nxt.positions)]
    return list(points[-1].positions)


class CommandedPose:
    def __init__(self, moveit, ee_link):
        self._state = RobotState(moveit.get_robot_model())
        self._ee_link = ee_link
        self._lock = threading.Lock()
        self._trajectory = None
        self._start = None

    def set_trajectory(self, robot_trajectory):
        """계획 직후·실행 직전에 호출 (motion.move_to_* 의 on_planned)."""
        trajectory = robot_trajectory.get_robot_trajectory_msg().joint_trajectory
        if not trajectory.points:
            return
        with self._lock:
            # 새 궤적의 시작점은 /joint_states에서 읽은 값이라 이전 목표와 미세하게 달라 한 틱 튐.
            # 기록용 사본의 첫 점을 지금까지 보고하던 값으로 맞춰 연속성을 유지함.
            if self._trajectory is not None:
                trajectory.points[0].positions = interpolate(self._trajectory, time.monotonic() - self._start)
            self._trajectory = trajectory
            self._start = time.monotonic()

    def pose(self):
        """geometry_msgs/Pose (panda_link0 기준) 또는 첫 궤적 전이면 None."""
        with self._lock:
            if self._trajectory is None:
                return None
            positions = interpolate(self._trajectory, time.monotonic() - self._start)
        self._state.set_joint_group_positions(ARM_GROUP, positions)
        self._state.update()
        return self._state.get_pose(self._ee_link)
