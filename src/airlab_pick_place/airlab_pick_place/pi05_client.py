#!/usr/bin/env python3
"""Custom π0.5 client using shared collection defaults and pi05_client.yaml.

ros2 run airlab_pick_place pi05_client --ros-args -p server_ip:=127.0.0.1 -p open_loop_horizon:=12
Arm actions are joint-position differences divided by dt [rad/s], not DROID commands.
"""

from pathlib import Path
import threading
import time

import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from cv_bridge import CvBridge
from openpi_client import image_tools, websocket_client_policy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .gripper import ACTION_NAME as GRIPPER_ACTION
from .recorder import AGENT_TOPIC, JOINTS, WRIST_TOPIC

ARM_JOINT_NAMES, FINGER_JOINT_NAMES = JOINTS[:7], JOINTS[7:]
PANDA_LOWER = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
PANDA_UPPER = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])


class PandaOpenPIClient(Node):
    def __init__(self):
        super().__init__('panda_openpi_client')
        config_dir = Path(get_package_share_directory('airlab_pick_place')) / 'config'
        with (config_dir / 'pick_place.yaml').open() as f:
            shared = yaml.safe_load(f)
        collection = shared['episode_collector']['ros__parameters']
        gripper = shared['pick_place_server']['ros__parameters']
        defaults = {
            'control_hz': collection['record_rate'], 'instruction': collection['instruction'],
            'open_width': gripper['open_width'], 'grasp_width': gripper['grasp_width'],
            'grasp_effort': gripper['grasp_effort'],
        }
        with (config_dir / 'pi05_client.yaml').open() as f:
            defaults.update(yaml.safe_load(f)['panda_openpi_client']['ros__parameters'])
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        self.control_hz = float(p('control_hz'))
        self.open_loop_horizon = int(p('open_loop_horizon'))
        self.max_gripper_width = float(p('open_width'))
        self.grasp_width = float(p('grasp_width'))
        self.grasp_effort = float(p('grasp_effort'))
        self.instruction = ' '.join(p('instruction').split())
        if not np.isfinite(self.control_hz) or self.control_hz <= 0:
            raise ValueError('control_hz must be finite and > 0')
        if self.open_loop_horizon <= 0:
            raise ValueError('open_loop_horizon must be > 0')
        if not np.isfinite(self.max_gripper_width) or self.max_gripper_width <= 0:
            raise ValueError('open_width must be finite and > 0')
        if not np.isfinite(self.grasp_width) or not 0 <= self.grasp_width <= self.max_gripper_width:
            raise ValueError('grasp_width must be between 0 and open_width')
        if not np.isfinite(self.grasp_effort) or self.grasp_effort < 0:
            raise ValueError('grasp_effort must be finite and >= 0')
        if not self.instruction:
            raise ValueError('instruction must not be empty')
        self.dt = 1.0 / self.control_hz
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.bridge = CvBridge()
        self.images = {}
        self.joints = {}
        self.gripper_busy = False
        self.last_gripper_command = None
        group = ReentrantCallbackGroup()

        for name, topic in (('primary', AGENT_TOPIC), ('wrist', WRIST_TOPIC)):
            self.create_subscription(Image, topic, lambda msg, name=name: self.image_callback(msg, name),
                                     qos_profile_sensor_data, callback_group=group)
        for topic in ('/joint_states', '/panda_gripper_sim_node/joint_states'):
            self.create_subscription(JointState, topic, self.joint_callback, 20, callback_group=group)
        self.arm_pub = self.create_publisher(JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        self.gripper_client = ActionClient(self, GripperCommand, GRIPPER_ACTION, callback_group=group)
        if not self.gripper_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f'{GRIPPER_ACTION} not available')
        self.policy_client = websocket_client_policy.WebsocketClientPolicy(p('server_ip'), p('server_port'))
        self.worker = threading.Thread(target=self.control_worker, daemon=True)
        self.worker.start()

    def image_callback(self, msg, name):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as exc:
            self.get_logger().warn(f'{name} image conversion failed: {exc}')
            return
        with self.lock:
            self.images[name] = np.asarray(image).copy()

    def joint_callback(self, msg):
        # Arm and finger states may arrive separately; retain values by joint name.
        values = dict(zip(msg.name, msg.position))
        if not all(np.isfinite(value) for value in values.values()):
            return
        with self.lock:
            self.joints.update(values)

    def latest_snapshot(self):
        with self.lock:
            if not all(name in self.images for name in ('primary', 'wrist')):
                return None
            if not all(name in self.joints for name in JOINTS):
                return None
            return {
                'primary': self.images['primary'].copy(),
                'wrist': self.images['wrist'].copy(),
                'joints': np.array([self.joints[name] for name in ARM_JOINT_NAMES]),
                'fingers': np.array([self.joints[name] for name in FINGER_JOINT_NAMES]),
            }

    def make_policy_request(self, snapshot):
        gripper = np.clip(1.0 - np.sum(snapshot['fingers']) / self.max_gripper_width, 0.0, 1.0)
        return {
            'observation/exterior_image_1_left': image_tools.resize_with_pad(snapshot['primary'], 224, 224),
            'observation/wrist_image_left': image_tools.resize_with_pad(snapshot['wrist'], 224, 224),
            'observation/joint_position': snapshot['joints'].astype(np.float32),
            'observation/gripper_position': np.array([gripper], dtype=np.float32),
            'prompt': self.instruction,
        }

    def execute_arm_action(self, action):
        velocity = np.asarray(action, dtype=np.float64)
        if velocity.shape != (7,) or not np.isfinite(velocity).all():
            raise ValueError('arm action must contain 7 finite velocities [rad/s]')
        snapshot = self.latest_snapshot()
        if snapshot is None:
            raise RuntimeError('joint state unavailable')
        delta = velocity * self.dt
        target = np.clip(snapshot['joints'] + delta, PANDA_LOWER, PANDA_UPPER)
        point = JointTrajectoryPoint()
        point.positions = target.tolist()
        ns = int(self.dt * 1_000_000_000)
        point.time_from_start = Duration(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)
        self.arm_pub.publish(JointTrajectory(joint_names=ARM_JOINT_NAMES, points=[point]))
        return delta, target

    def command_gripper(self, value):
        if not np.isfinite(value):
            raise ValueError('gripper action must be finite')
        closed = bool(value > 0.5)
        with self.lock:
            if self.gripper_busy or self.last_gripper_command == closed:
                return
            self.gripper_busy = True

        def done(success):
            with self.lock:
                self.last_gripper_command = closed if success else None
                self.gripper_busy = False
            if not success:
                self.get_logger().warn('gripper action failed; command may be retried')

        def result_ready(future):
            try:
                wrapped = future.result()
                success = wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.reached_goal
            except Exception as exc:
                self.get_logger().error(f'gripper result error: {exc}')
                success = False
            done(bool(success))

        def goal_ready(future):
            try:
                handle = future.result()
                if handle is not None and handle.accepted:
                    handle.get_result_async().add_done_callback(result_ready)
                    return
            except Exception as exc:
                self.get_logger().error(f'gripper goal error: {exc}')
            done(False)

        # Non-blocking inference command; leave the collection helper unchanged.
        goal = GripperCommand.Goal()
        goal.command.position = (self.grasp_width if closed else self.max_gripper_width) / 2.0
        goal.command.max_effort = self.grasp_effort if closed else 10.0
        try:
            self.gripper_client.send_goal_async(goal).add_done_callback(goal_ready)
        except Exception as exc:
            self.get_logger().error(f'gripper send error: {exc}')
            done(False)

    def control_worker(self):
        log = self.get_logger()
        log.info('Waiting for camera and joint data...')
        chunk, index = None, 0
        try:
            while not self.stop_event.is_set() and rclpy.ok():
                started = time.perf_counter()
                snapshot = self.latest_snapshot()
                if snapshot is None:
                    self.stop_event.wait(0.02)
                    continue
                if chunk is None or index >= min(self.open_loop_horizon, len(chunk)):
                    result = self.policy_client.infer(self.make_policy_request(snapshot))
                    if self.stop_event.is_set() or not rclpy.ok():
                        break
                    chunk = np.asarray(result['actions'], dtype=np.float64)
                    if chunk.ndim != 2 or chunk.shape[1] != 8 or len(chunk) == 0:
                        raise ValueError(f'expected a non-empty action chunk [N, 8], got {chunk.shape}')
                    if not np.isfinite(chunk).all():
                        raise ValueError('policy returned non-finite actions')
                    index = 0
                    log.info(f'replan: chunk={chunk.shape}, infer={(time.perf_counter() - started) * 1000:.1f} ms')
                # Inference latency must not shorten the first action's execution interval.
                action_started = time.perf_counter()
                self.execute_arm_action(chunk[index, :7])
                self.command_gripper(float(chunk[index, 7]))
                index += 1
                self.stop_event.wait(max(0.0, self.dt - (time.perf_counter() - action_started)))
        except Exception as exc:
            log.error(f'OpenPI control stopped: {type(exc).__name__}: {exc}')

    def close(self):
        self.stop_event.set()
        self.worker.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node, executor = None, None
    try:
        node = PandaOpenPIClient()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
