"""PickPlace 액션 서버: approach -> descend -> grasp -> lift -> place -> open -> retreat.

MoveItPy를 한 번만 만들고 여러 goal에 재사용함. 파라미터 기본값은 config/pick_place.yaml.
"""
import math
import random
import time

import rclpy
from airlab_msgs.action import PickPlace
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from .commanded_pose import CommandedPose
from .gripper import Gripper
from .motion import PLANNING_FRAME, current_ee_pose, move_to_named, move_to_pose, pilz_params, \
    target_pose, world_to_base
from .moveit_setup import create_moveit
from .params import declare_from_yaml
from .planning_scene import attach_box, detach_box
from .sim_bodies import SimBodies

class Failed(Exception):
    pass


class Canceled(Exception):
    pass


class PickPlaceServer(Node):
    def __init__(self):
        super().__init__('pick_place_server')
        p = self.p = declare_from_yaml(self)  # config/pick_place.yaml

        # 서비스·액션을 기다리는 호출은 executor에 넣지 않은 별도 노드로 처리.
        # 액션 서버와 같은 노드에서 spin_until_future_complete를 쓰면 결과가 클라이언트에 전달되지 않음.
        self.helper = rclpy.create_node('pick_place_server_helper')
        self.moveit = create_moveit(node_name='pick_place_server')
        self.arm = self.moveit.get_planning_component('panda_arm')
        self.gripper = Gripper(self.helper)
        self.bodies = SimBodies(self.helper)
        self._wait_for_arm_controller()

        self.ptp = pilz_params(self.moveit, 'PTP', p('velocity_scale'))
        self.lin = pilz_params(self.moveit, 'LIN', p('velocity_scale'))

        # 기록용 명령 pose/그리퍼 폭 발행 (동작 실행과 동시에 돌도록 별도 callback group)
        self.commanded = CommandedPose(self.moveit, p('ee_link'))
        self.commanded_width = p('open_width')
        self.pose_pub = self.create_publisher(PoseStamped, '/commanded_ee_pose', 10)
        self.width_pub = self.create_publisher(Float64, '/commanded_gripper_width', 10)
        self.create_timer(1.0 / p('commanded_rate'), self._publish_commanded,
                          callback_group=ReentrantCallbackGroup())

        # 액션과 go_home은 같은 MutuallyExclusive 그룹: 로봇을 움직이는 요청은 한 번에 하나만
        motion_group = MutuallyExclusiveCallbackGroup()
        self.action_server = ActionServer(self, PickPlace, 'pick_place', self._execute,
                                          callback_group=motion_group,
                                          cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.create_service(Trigger, 'go_home', self._go_home, callback_group=motion_group)
        self.get_logger().info('pick_place_server ready')

    def _wait_for_arm_controller(self):
        client = ActionClient(self.helper, FollowJointTrajectory,
                              'panda_arm_controller/follow_joint_trajectory')
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('panda_arm_controller not available')
        client.destroy()
        time.sleep(2.0)  # MoveIt 내부 실행 클라이언트 연결 대기 (첫 goal이 조용히 실패하는 것 방지)

    def _publish_commanded(self):
        pose = self.commanded.pose()
        if pose is None:
            return
        msg = PoseStamped(pose=pose)
        msg.header.frame_id = PLANNING_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(msg)
        self.width_pub.publish(Float64(data=self.commanded_width))

    def _set_gripper(self, width, effort=10.0):
        self.commanded_width = width
        return self.gripper.move(width, effort)

    def _release(self, object_name):
        """실패·취소·home 복귀 공통 정리: 그리퍼 열기 + planning scene에서 큐브 제거."""
        self._set_gripper(self.p('open_width'))
        detach_box(self.moveit, object_name)

    def _go_home(self, request, response):
        self._release(self.p('object_body'))
        response.success = move_to_named(self.moveit, self.arm, 'ready', self.ptp,
                                         self.commanded.set_trajectory)
        response.message = 'at ready' if response.success else 'failed to reach ready'
        return response

    def _execute(self, goal_handle):
        goal = goal_handle.request
        name = goal.object_name or self.p('object_body')
        try:
            result = self._pick_and_place(goal_handle, name, goal.place_x, goal.place_y)
            goal_handle.succeed()
            return result
        except Canceled:
            self._release(name)
            goal_handle.canceled()
            return PickPlace.Result(success=False, message='canceled')
        except Exception as error:  # Failed 포함, 어떤 오류든 정리 후 abort
            self._release(name)
            self.get_logger().error(str(error))
            goal_handle.abort()
            return PickPlace.Result(success=False, message=str(error))

    def _pick_and_place(self, goal_handle, name, place_x, place_y):
        p, log = self.p, self.get_logger()
        ee = p('ee_link')

        def stage(label):
            if goal_handle.is_cancel_requested:
                raise Canceled()
            goal_handle.publish_feedback(PickPlace.Feedback(stage=label))
            log.info(f'[{label}]')

        def move(label, xyz, params):
            stage(label)
            pose = target_pose(xyz, current_ee_pose(self.moveit, ee).orientation)  # 현재 방향 유지
            if not move_to_pose(self.moveit, self.arm, pose, ee, params, self.commanded.set_trajectory):
                raise Failed(f'{label} failed')
            time.sleep(p('stage_pause'))

        start = self.bodies.position(name)
        if start is None:
            raise Failed(f'cannot read {name}')
        lo, hi = p('grasp_offset_x_min'), p('grasp_offset_x_max')
        if hi < lo:
            raise Failed(f'grasp_offset_x_min ({lo}) > grasp_offset_x_max ({hi})')
        dx = -random.uniform(lo, hi)  # panda_link0 x 오프셋 -> world x (부호 반대)
        x, y, z = start
        log.info(f'{name} at ({x:.3f}, {y:.3f}, {z:.3f}) -> place ({place_x:.3f}, {place_y:.3f}), '
                 f'grasp offset {-dx * 1000:+.1f} mm')

        self._set_gripper(p('open_width'))
        move('approach', (x + dx, y, z + p('approach_height')), self.ptp)
        move('descend', (x + dx, y, z + p('grasp_height')), self.lin)

        stage('grasp')
        if not self._set_gripper(p('grasp_width'), p('grasp_effort')):
            raise Failed('gripper close failed')
        time.sleep(p('stage_pause'))
        held = self.bodies.pose(name)
        if held is not None:
            box = Pose(orientation=held.orientation)
            box.position.x, box.position.y, box.position.z = world_to_base(
                held.position.x, held.position.y, held.position.z)
            attach_box(self.moveit, name, box)

        move('lift', (x + dx, y, z + p('lift_height')), self.lin)
        move('place', (place_x, place_y, z + p('approach_height')), self.lin)
        move('place', (place_x, place_y, z + p('grasp_height')), self.lin)

        stage('open')
        detach_box(self.moveit, name)
        if not self._set_gripper(p('open_width')):
            raise Failed('gripper open failed')
        time.sleep(p('stage_pause'))

        stage('retreat')  # 실패해도 결과 판정에는 영향 없음
        pose = target_pose((place_x, place_y, z + p('approach_height')), current_ee_pose(self.moveit, ee).orientation)
        move_to_pose(self.moveit, self.arm, pose, ee, self.lin, self.commanded.set_trajectory)
        time.sleep(0.5)

        final = self.bodies.pose(name)
        if final is None:
            raise Failed(f'cannot read final {name} pose')
        distance = math.hypot(final.position.x - place_x, final.position.y - place_y)
        success = distance <= p('success_tolerance')
        log.info(f'done: success={success}, {distance:.3f} m from target')
        return PickPlace.Result(success=success, final_distance=distance, final_object_pose=final,
                                message=f'grasp_offset_x={-dx:+.4f}')


def main():
    rclpy.init()
    node = PickPlaceServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.moveit.shutdown()
        node.helper.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()
