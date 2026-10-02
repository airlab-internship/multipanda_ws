"""에피소드 기록: 일정 주기로 두 카메라 영상, 관절, EE pose(도달값), 명령 pose·그리퍼 폭(action)을 모아 HDF5 1개로 저장.

모델별 형식이 아니라 원시 신호를 저장함 (RPY·이진 그리퍼 등은 나중에 변환 가능, 반대는 불가).
"""
import time

import h5py
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64

AGENT_TOPIC = '/mujoco_server/cameras/test_cam/rgb/image_raw'
WRIST_TOPIC = '/mujoco_server/cameras/hand_cam/rgb/image_raw'
BASE_FRAME, EE_FRAME = 'panda_link0', 'panda_hand_tcp'
JOINTS = [f'panda_joint{i}' for i in range(1, 8)] + ['panda_finger_joint1', 'panda_finger_joint2']
FIELDS = ['timestamps', 'agent_image', 'wrist_image', 'joint_positions', 'gripper_width',
          'ee_position', 'ee_orientation', 'action_ee_position', 'action_ee_orientation',
          'action_gripper_width']


class Recorder:
    def __init__(self, node, rate_hz):
        self.node = node
        self.rate_hz = rate_hz
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        self.latest = {}
        self.joints = {}  # /joint_states는 팔·그리퍼가 따로 들어오므로 이름별로 최신값 유지
        self.data = None
        self.timer = None
        self.dropped = 0

        def keep(key):
            return lambda msg: self.latest.__setitem__(key, msg)
        node.create_subscription(Image, AGENT_TOPIC, keep('agent'), 10)
        node.create_subscription(Image, WRIST_TOPIC, keep('wrist'), 10)
        node.create_subscription(PoseStamped, '/commanded_ee_pose', keep('cmd_pose'), 10)
        node.create_subscription(Float64, '/commanded_gripper_width', keep('cmd_width'), 10)
        node.create_subscription(JointState, '/joint_states', self._on_joints, 10)

    def _on_joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def start(self):
        self.data = {key: [] for key in FIELDS}
        self.dropped = 0
        self.timer = self.node.create_timer(1.0 / self.rate_hz, self._sample)

    def stop(self):
        if self.timer is not None:
            self.timer.cancel()
            self.node.destroy_timer(self.timer)
            self.timer = None
        if self.dropped:
            self.node.get_logger().warn(f'recorder dropped {self.dropped} samples (inputs not ready)')

    def __len__(self):
        return len(self.data['timestamps']) if self.data else 0

    def _sample(self):
        ready = all(k in self.latest for k in ('agent', 'wrist', 'cmd_pose', 'cmd_width')) \
            and all(j in self.joints for j in JOINTS)
        try:
            tf = self.tf_buffer.lookup_transform(BASE_FRAME, EE_FRAME, rclpy.time.Time()) if ready else None
        except tf2_ros.TransformException:
            tf = None
        if tf is None:
            self.dropped += 1
            return

        t, q = tf.transform.translation, tf.transform.rotation
        cmd = self.latest['cmd_pose'].pose
        d = self.data
        d['timestamps'].append(time.time())
        d['agent_image'].append(self.bridge.imgmsg_to_cv2(self.latest['agent'], 'bgr8'))
        d['wrist_image'].append(self.bridge.imgmsg_to_cv2(self.latest['wrist'], 'bgr8'))
        d['joint_positions'].append([self.joints[j] for j in JOINTS])
        d['gripper_width'].append(self.joints['panda_finger_joint1'] + self.joints['panda_finger_joint2'])
        d['ee_position'].append([t.x, t.y, t.z])
        d['ee_orientation'].append([q.x, q.y, q.z, q.w])
        d['action_ee_position'].append([cmd.position.x, cmd.position.y, cmd.position.z])
        d['action_ee_orientation'].append([cmd.orientation.x, cmd.orientation.y,
                                           cmd.orientation.z, cmd.orientation.w])
        d['action_gripper_width'].append(self.latest['cmd_width'].data)

    def save(self, path, metadata):
        with h5py.File(path, 'w') as f:
            for key, values in self.data.items():
                if key.endswith('image'):
                    f.create_dataset(key, data=np.stack(values), compression='gzip', compression_opts=4)
                else:
                    f.create_dataset(key, data=np.asarray(values, dtype=np.float64))
            f.attrs['joint_names'] = JOINTS
            f.attrs['ee_orientation_convention'] = 'xyzw'
            f.attrs['ee_parent_frame'] = BASE_FRAME
            f.attrs['image_encoding'] = 'bgr8'
            f.attrs['action_convention'] = 'action_* = commanded (planned trajectory + FK); ee_* = achieved'
            for key, value in metadata.items():
                f.attrs[key] = value
