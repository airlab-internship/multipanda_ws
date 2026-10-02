"""수집·추론 공통 pose 규약: [x, y, z, rx, ry, rz], Euler "xyz", rad (Seer와 동일)."""
import math

import numpy as np
from scipy.spatial.transform import Rotation as R


def pose6d(position, quat_xyzw):
    """위치 + 쿼터니언 -> [x, y, z, rx, ry, rz].
    그리퍼가 아래를 볼 때 rx가 ±π 경계라 프레임마다 +π/-π로 튐 -> [0, 2π)로 옮겨 π 근처에서 연속이 되게 함.
    학습 데이터의 gripper_pose와 추론 때 모델에 넣는 state 모두 이 함수를 써야 함."""
    rx, ry, rz = R.from_quat(quat_xyzw).as_euler('xyz')
    return np.array([*position, rx % (2 * math.pi), ry, rz])


def pose6d_to_mat(p):
    T = np.eye(4)
    T[:3, 3] = p[:3]
    T[:3, :3] = R.from_euler('xyz', p[3:6]).as_matrix()
    return T


def mat_to_pose6d(T):
    return np.concatenate([T[:3, 3], R.from_matrix(T[:3, :3]).as_euler('xyz')])
