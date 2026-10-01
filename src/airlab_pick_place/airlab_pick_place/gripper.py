"""시뮬레이터 그리퍼 (panda_gripper_sim_node의 GripperCommand 액션) 클라이언트."""
import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient

ACTION_NAME = '/panda_gripper_sim_node/gripper_action'


class Gripper:
    def __init__(self, node, timeout=10.0):
        self._node = node
        self._client = ActionClient(node, GripperCommand, ACTION_NAME)
        if not self._client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(f'{ACTION_NAME} not available')

    def move(self, width, effort, timeout=10.0):
        """width = 두 손가락 사이 전체 폭 (m). 액션은 손가락 하나의 위치(= width / 2)를 받음."""
        goal = GripperCommand.Goal()
        goal.command.position = width / 2.0
        goal.command.max_effort = effort
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
        handle = future.result()
        if handle is None or not handle.accepted:
            return False
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result, timeout_sec=timeout)
        return result.result() is not None
