"""작성 예정: 로컬 MuJoCo와 GPU 서버의 π0.5를 연결하는 ROS 클라이언트.

들어갈 내용:
- 외부/손목 카메라와 joint_states 구독.
- 학습과 같은 영상 전처리, 관절 순서, 그리퍼 상태 구성.
- 관측과 언어 지시문을 WebSocket으로 GPU 추론 서버에 전송.
- 반환된 관절 속도 7개(rad/s)와 그리퍼 행동 1개 해석.
- action chunk에서 앞 H개를 실행하고 최신 관측으로 재요청.
- 기존 MuJoCo 컨트롤러에 맞는 속도 적용 방식 구현.
- 단일 큐브/타겟 초기화, 실행 종료, 결과 기록.
- 연결 실패, 오래된 관측, 잘못된 행동 처리.
- 구현 후 setup.py 실행 항목과 package.xml 의존성 등록.

현재는 설명만 있으며 실행 코드는 없음.
"""
#!/usr/bin/env python3
# ruff: noqa
# droid
#   DROID normalized joint command
#   max(|action|) > 1 이면 전체 vector rescale
#   delta_q = action * 0.2

# custom
#   raw commanded joint velocity [rad/s]
#   delta_q = action * (1 / control_hz)

#모션에 움직임함수좀빼고 커스텀속도만사용으로
from __future__ import annotations

import argparse
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
from franka_msgs.action import Grasp, Move
from openpi_client import image_tools
from openpi_client import websocket_client_policy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ---------------------------------------------------------------------------
# ROS2 / MuJoCo configuration
# ---------------------------------------------------------------------------

PRIMARY_CAMERA_TOPIC = "/mujoco_server/cameras/pi_primary_cam/rgb/image_raw"
WRIST_CAMERA_TOPIC = "/mujoco_server/cameras/pi_wrist_cam/rgb/image_raw"
ARM_JOINT_TOPIC = "/joint_states"
GRIPPER_JOINT_TOPIC = "/panda_gripper_sim_node/joint_states"

ARM_COMMAND_TOPIC = "/panda_arm_controller/joint_trajectory"

GRIPPER_MOVE_ACTION = "/panda_gripper_sim_node/move"
GRIPPER_GRASP_ACTION = "/panda_gripper_sim_node/grasp"

ARM_JOINT_NAMES = [f"panda_joint{i}" for i in range(1, 8)]
FINGER_JOINT_NAMES = ["panda_finger_joint1", "panda_finger_joint2"]

#각 Panda joint의 최소 각도.
PANDA_LOWER = np.asarray(
    [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
    dtype=np.float64,
)
#각 joint 최대 각도.
PANDA_UPPER = np.asarray(
    [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973],
    dtype=np.float64,
)

DROID_MAX_JOINT_DELTA = 0.2
# DROID runs at 15 Hz.
DEFAULT_CONTROL_HZ = 15.0

# Franka Hand: two fingers, approximately 0.04 m each at full opening.
DEFAULT_MAX_GRIPPER_WIDTH = 0.08

DEFAULT_SERVER_IP = "127.0.0.1"
DEFAULT_SERVER_PORT = 8152
DEFAULT_INSTRUCTION = "pick up the blue box and place it in the red target zone"


class PandaOpenPIClient(Node):
    def __init__(
        self,
        server_ip: str,
        server_port: int,
        instruction: str,
        control_hz: float,
        open_loop_horizon: int,
        max_gripper_width: float,
        grasp_width: float,
        arm_action_mode: str,
    ) -> None:
        super().__init__("panda_openpi_client")

        if control_hz <= 0:
            raise ValueError("control_hz must be > 0")
        if open_loop_horizon <= 0:
            raise ValueError("open_loop_horizon must be > 0")
        if max_gripper_width <= 0:
            raise ValueError("max_gripper_width must be > 0")

        self.server_ip = server_ip
        self.server_port = server_port
        self.instruction = " ".join(instruction.split())
        self.control_hz = float(control_hz)
        self.dt = 1.0 / self.control_hz
        self.open_loop_horizon = int(open_loop_horizon)
        self.max_gripper_width = float(max_gripper_width)
        self.grasp_width = float(grasp_width)
        self.arm_action_mode = arm_action_mode

        self.running = True
        self.cb_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        # Latest sensor values.
        self.latest_primary: Optional[np.ndarray] = None
        self.latest_wrist: Optional[np.ndarray] = None
        self.latest_joints: Optional[np.ndarray] = None
        self.latest_fingers: Optional[np.ndarray] = None

        self.primary_seq = 0
        self.wrist_seq = 0

        # 현재 gripper action이 수행 중인지
        self.gripper_busy = False

        # 마지막으로 실제 실행한 gripper 명령
        self.last_gripper_command: Optional[float] = None

        self.create_subscription(
            Image,
            PRIMARY_CAMERA_TOPIC,
            self.primary_callback,
            1,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            Image,
            WRIST_CAMERA_TOPIC,
            self.wrist_callback,
            1,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            JointState,
            ARM_JOINT_TOPIC,
            self.arm_joint_callback,
            20,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            JointState,
            GRIPPER_JOINT_TOPIC,
            self.gripper_joint_callback,
            20,
            callback_group=self.cb_group,
        )

        # Existing MuJoCo Panda arm controller: position trajectory input.
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            ARM_COMMAND_TOPIC,
            10,
        )

        # Existing simulated Franka gripper action servers.
        self.gripper_move_client = ActionClient(
            self,
            Move,
            GRIPPER_MOVE_ACTION,
            callback_group=self.cb_group,
        )
        self.gripper_grasp_client = ActionClient(
            self,
            Grasp,
            GRIPPER_GRASP_ACTION,
            callback_group=self.cb_group,
        )


        # OpenPI's official websocket inference client.
        self.policy_client = websocket_client_policy.WebsocketClientPolicy(
            self.server_ip,
            self.server_port,
        )

        self.worker = threading.Thread(
            target=self.control_worker,
            daemon=True,
        )
        self.worker.start()

    # -----------------------------------------------------------------------
    # ROS callbacks
    # -----------------------------------------------------------------------

    @staticmethod
    def named_positions(
        msg: JointState,
        names: list[str],
    ) -> Optional[np.ndarray]:
        table = dict(zip(msg.name, msg.position))
        if not all(name in table for name in names):
            return None

        values = np.asarray(
            [table[name] for name in names],
            dtype=np.float64,
        )

        if not np.isfinite(values).all():
            return None

        return values

    def primary_callback(self, msg: Image) -> None:
        try:
            # OpenPI expects RGB.
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="rgb8",
            )
        except Exception as exc:
            self.get_logger().warn(
                f"primary image conversion failed: {exc}"
            )
            return

        with self.lock:
            self.latest_primary = np.asarray(image).copy()
            self.primary_seq += 1

    def wrist_callback(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="rgb8",
            )
        except Exception as exc:
            self.get_logger().warn(
                f"wrist image conversion failed: {exc}"
            )
            return

        with self.lock:
            self.latest_wrist = np.asarray(image).copy()
            self.wrist_seq += 1

    def arm_joint_callback(self, msg: JointState) -> None:
        joints = self.named_positions(msg, ARM_JOINT_NAMES)
        if joints is None:
            return

        with self.lock:
            self.latest_joints = joints

    def gripper_joint_callback(self, msg: JointState) -> None:
        fingers = self.named_positions(msg, FINGER_JOINT_NAMES)
        if fingers is None:
            return

        with self.lock:
            self.latest_fingers = fingers

    # -----------------------------------------------------------------------
    # Observation conversion
    # -----------------------------------------------------------------------

    def latest_snapshot(self) -> Optional[dict]:
        with self.lock:
            if (
                self.latest_primary is None
                or self.latest_wrist is None
                or self.latest_joints is None
                or self.latest_fingers is None
            ):
                return None

            return {
                "primary": self.latest_primary.copy(),
                "wrist": self.latest_wrist.copy(),
                "joints": self.latest_joints.copy(),
                "fingers": self.latest_fingers.copy(),
                "primary_seq": self.primary_seq,
                "wrist_seq": self.wrist_seq,
            }

    def droid_gripper_position(
        self,
        fingers: np.ndarray,
    ) -> float:
        """Convert MuJoCo finger joints to DROID normalized gripper position.

        Franka finger joints represent approximately half-width each:
            fully closed -> [0.0, 0.0]
            fully open   -> [0.04, 0.04]

        DROID convention:
            0.0 = fully open
            1.0 = fully closed

        Therefore:
            width = finger1 + finger2
            droid_position = 1 - width / max_width
        """
        width = float(np.sum(fingers))
        width = float(
            np.clip(
                width,
                0.0,
                self.max_gripper_width,
            )
        )

        position = 1.0 - (
            width / self.max_gripper_width
        )

        return float(np.clip(position, 0.0, 1.0))

    def make_policy_request(
        self,
        snapshot: dict,
    ) -> dict:
        gripper_position = self.droid_gripper_position(
            snapshot["fingers"]
        )

        return {
            "observation/exterior_image_1_left":
                image_tools.resize_with_pad(
                    snapshot["primary"],
                    224,
                    224,
                ),
            "observation/wrist_image_left":
                image_tools.resize_with_pad(
                    snapshot["wrist"],
                    224,
                    224,
                ),
            "observation/joint_position":
                snapshot["joints"].astype(np.float64),
            "observation/gripper_position":
                np.asarray(
                    [gripper_position],
                    dtype=np.float64,
                ),
            "prompt": self.instruction,
        }

    # -----------------------------------------------------------------------
    # Arm / gripper actuation
    # -----------------------------------------------------------------------

    def publish_arm_position(
        #모델의 joint velocity를 적분해서 만든 joint_target을 ROS2 JointTrajectory 메시지로 
        #만들어 Panda arm controller에 보내는 함수
        #publish_arm_position()은 전송 담당
        self,
        joint_target: np.ndarray,
        #joint_target은 Panda 7개 관절의 **목표 각도 [rad]**
    ) -> None:
        
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [float(q)for q in joint_target]

        ns = int(self.dt * 1_000_000_000)
        point.time_from_start = Duration(
            sec=ns // 1_000_000_000,
            nanosec=ns % 1_000_000_000,
        )#이 목표 joint position에 trajectory 시작 후 0.0667초 안에 도달해라

        msg.points = [point]
        self.arm_pub.publish(msg)

    def execute_arm_action(
        self,
        arm_action: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:

        arm_action = np.asarray(
            arm_action,
            dtype=np.float64,
        )

        if arm_action.shape != (7,):
            raise ValueError(
                f"arm_action must be shape (7,), got {arm_action.shape}"
            )

        if not np.isfinite(arm_action).all():
            raise ValueError(
                f"non-finite arm action: {arm_action}"
            )

        snapshot = self.latest_snapshot()

        if snapshot is None:
            raise RuntimeError("joint state unavailable")

        q_now = snapshot["joints"]

        # 수정
        if self.arm_action_mode == "droid":
            # DROID 공식 RobotIKSolver 방식
            max_norm = np.abs(arm_action).max()
            if max_norm > 1.0:
                arm_action = arm_action / max_norm

            joint_delta = arm_action * 0.2

        elif self.arm_action_mode == "custom":
            # custom commanded velocity [rad/s]
            joint_delta = arm_action * self.dt

        else:
            raise RuntimeError(
                f"Unknown arm_action_mode: {self.arm_action_mode}"
            )

        q_target = q_now + joint_delta

        q_target = np.clip(
            q_target,
            PANDA_LOWER,
            PANDA_UPPER,
        )

        self.publish_arm_position(q_target)

        return joint_delta, q_target

    def command_gripper(
        self,
        droid_gripper_position: float,
    ) -> None:

        # 모델 출력값을 0~1 범위로 제한
        value = float(
            np.clip(
                droid_gripper_position,
                0.0,
                1.0,
            )
        )

        # DROID:
        # 0 = OPEN
        # 1 = CLOSE
        binary = 1.0 if value > 0.5 else 0.0

        # 이전 OPEN/CLOSE action이 아직 수행 중이면
        # 새로운 gripper 명령을 보내지 않음
        if self.gripper_busy:
            return

        # 마지막으로 완료된 gripper 상태와 동일하면
        # 같은 명령을 다시 보내지 않음
        if self.last_gripper_command == binary:
            return

        # 지금부터 gripper action 수행 중
        self.gripper_busy = True

        if binary == 0.0:
            # -------------------------
            # DROID 0 = OPEN
            # -------------------------
            goal = Move.Goal()
            goal.width = self.max_gripper_width   # 0.08 m
            goal.speed = 0.05

            # send_goal_async()가 반환하는 Future 저장
            future = self.gripper_move_client.send_goal_async(
                goal
            )

            self.get_logger().info(
                "=============================="
            )
            self.get_logger().info(
                "== gripper: DROID=0 -> OPEN =="
            )
            self.get_logger().info(
                "=============================="
            )

        else:
            # -------------------------
            # DROID 1 = CLOSE
            # -------------------------
            goal = Grasp.Goal()
            goal.width = self.grasp_width
            goal.speed = 0.1
            goal.force = 40.0
            goal.epsilon.inner = 0.01
            goal.epsilon.outer = 0.01

            # send_goal_async()가 반환하는 Future 저장
            future = self.gripper_grasp_client.send_goal_async(
                goal
            )

            self.get_logger().info(
                "==============================="
            )
            self.get_logger().info(
                "== gripper: DROID=1 -> CLOSE =="
            )
            self.get_logger().info(
                "==============================="
            )

        # action server가 goal을 accept/reject 했는지 확인
        future.add_done_callback(
            lambda f, cmd=binary:
                self.gripper_goal_response_callback(
                    f,
                    cmd,
                )
        )
        
    def gripper_goal_response_callback(
        self,
        future,
        command: float,
    ) -> None:

        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"gripper goal error: {exc}"
            )
            self.gripper_busy = False
            return

        if not goal_handle.accepted:
            self.get_logger().warn(
                "gripper goal rejected"
            )
            self.gripper_busy = False
            return

        # 실제 gripper action이 완료될 때까지 기다리는 future
        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            lambda f, cmd=command:
                self.gripper_result_callback(f, cmd)
        )


    def gripper_result_callback(
        self,
        future,
        command: float,
    ) -> None:

        try:
            future.result()

            # 여기까지 왔을 때 비로소
            # OPEN/CLOSE가 완료된 것으로 기록
            self.last_gripper_command = command

            self.get_logger().info(
                f"gripper completed: "
                f"{'CLOSE' if command == 1.0 else 'OPEN'}"
            )

        except Exception as exc:
            self.get_logger().error(
                f"gripper result error: {exc}"
            )

        finally:
            self.gripper_busy = False

    # -----------------------------------------------------------------------
    # Main OpenPI control loop
    # -----------------------------------------------------------------------

    def control_worker(self) -> None:
        if not self.gripper_move_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"{GRIPPER_MOVE_ACTION} not available")
            return

        if not self.gripper_grasp_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"{GRIPPER_GRASP_ACTION} not available")
            return

        self.get_logger().info("waiting for camera/joint/gripper data...")

        while self.running and rclpy.ok():
            if self.latest_snapshot() is not None:
                break
            time.sleep(0.02)

        if not self.running or not rclpy.ok():
            return

        self.get_logger().info(
            f"starting OpenPI control: "
            f"{self.control_hz:.1f} Hz, "
            f"open_loop_horizon={self.open_loop_horizon}, "
            f"server={self.server_ip}:{self.server_port}"
        )

        pred_action_chunk: Optional[np.ndarray] = None
        action_index = 0
        step_index = 0

        try:
            while self.running and rclpy.ok():
                cycle_start = time.perf_counter()

                # Replan when the current open-loop segment has been consumed.
                if (
                    pred_action_chunk is None
                    or action_index >= self.open_loop_horizon
                    or action_index >= len(pred_action_chunk)
                ):
                    snapshot = self.latest_snapshot()
                    if snapshot is None:
                        time.sleep(0.01)
                        continue

                    request_data = self.make_policy_request(snapshot)

                    infer_start = time.perf_counter()
                    result = self.policy_client.infer(request_data
                                                      )
                    infer_ms = (time.perf_counter()- infer_start) * 1000.0

                    if "actions" not in result:
                        raise RuntimeError(
                            "policy response does not contain 'actions'"
                        )

                    pred_action_chunk = np.asarray(result["actions"],dtype=np.float64,)

                    if (
                        pred_action_chunk.ndim != 2
                        or pred_action_chunk.shape[1] != 8
                    ):
                        raise RuntimeError(
                            "expected action chunk [N, 8], "
                            f"got {pred_action_chunk.shape}"
                        )

                    if not np.isfinite(
                        pred_action_chunk
                    ).all():
                        raise RuntimeError(
                            "policy returned non-finite action values"
                        )

                    action_index = 0

                    obs_grip = float(
                        request_data[
                            "observation/gripper_position"
                        ][0]
                    )

                    self.get_logger().info(
                        f"replan: "
                        f"chunk={pred_action_chunk.shape}, "
                        f"infer={infer_ms:.1f} ms, "
                        f"obs_grip={obs_grip:.3f}"
                    )

                action = pred_action_chunk[action_index].copy()
                action_index += 1

                # Official OpenPI DROID example clips the full action to [-1, 1].
                # action = np.clip(action,-1.0,1.0,)

                arm_action = action[:7]

                # Gripper output is a POSITION command. Binarize exactly like
                # the official OpenPI DROID example.
                raw_gripper = float(action[7])
                gripper_cmd = (
                    1.0
                    if raw_gripper > 0.5
                    else 0.0
                )

                joint_delta, q_target = (self.execute_arm_action(arm_action))
                self.command_gripper(gripper_cmd)

                self.get_logger().info(
                    f"[{step_index:05d}] "
                    f"chunk_i={action_index - 1:02d} "
                    f"mode={self.arm_action_mode} "
                    f"action="
                    f"{np.array2string(arm_action, precision=3, suppress_small=True)} "
                    f"delta_q="
                    f"{np.array2string(joint_delta, precision=3, suppress_small=True)} "
                    f"grip_raw={raw_gripper:+.3f} "
                    f"grip_bin={gripper_cmd:.0f} "
                    f"q_target="
                    f"{np.array2string(q_target, precision=3, suppress_small=True)}"
                )

                step_index += 1

                # Maintain DROID control frequency.
                elapsed = (time.perf_counter()- cycle_start)
                remaining = self.dt - elapsed

                if remaining > 0:
                    time.sleep(remaining)

        except Exception as exc:
            self.get_logger().error(
                f"OpenPI control loop stopped: {type(exc).__name__}: {exc}"
            )

    def close(self) -> None:
        self.running = False

        if self.worker.is_alive():
            self.worker.join(
                timeout=2.0
            )


def main(args=None) -> None:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--server-ip",
        default=DEFAULT_SERVER_IP,
    )
    parser.add_argument(
        "--server-port",
        type=int,
        default=DEFAULT_SERVER_PORT,
    )
    parser.add_argument(
        "--instruction",
        default=DEFAULT_INSTRUCTION,
    )
    parser.add_argument(
        "--control-hz",
        type=float,
        default=DEFAULT_CONTROL_HZ,
    )
    parser.add_argument(
        "--open-loop-horizon",
        type=int,
        default=12,
        help=(
            "Number of predicted actions to execute "
            "before requesting a new action chunk."
        ),
    )
    parser.add_argument(
        "--max-gripper-width",
        type=float,
        default=DEFAULT_MAX_GRIPPER_WIDTH,
        help=(
            "Full Franka gripper width in meters. "
            "Default: 0.08."
        ),
    )
    parser.add_argument(
        "--grasp-width",
        type=float,
        default=0.0,
        help=(
            "Target width used for binary DROID close command. "
            "Use 0.0 for full close; increase if your MuJoCo object "
            "requires a non-zero grasp width."
        ),
    )
    parser.add_argument(
        "--arm-action-mode",
        choices=["droid", "custom"],
        required=True,
        help=(
        "'droid' = DROID normalized joint command, delta_q = action * 0.2; "
        "'custom' = raw commanded joint velocity [rad/s], delta_q = action / control_hz"),
    )

    parsed, ros_args = parser.parse_known_args(
        args
    )

    rclpy.init(args=ros_args)

    node = PandaOpenPIClient(
        server_ip=parsed.server_ip,
        server_port=parsed.server_port,
        instruction=parsed.instruction,
        control_hz=parsed.control_hz,
        open_loop_horizon=parsed.open_loop_horizon,
        max_gripper_width=parsed.max_gripper_width,
        grasp_width=parsed.grasp_width,
        arm_action_mode=parsed.arm_action_mode,
    )

    executor = MultiThreadedExecutor(
        num_threads=4
    )
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
