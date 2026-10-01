"""MuJoCo body 위치 조회 (/get_body_state) · 이동 (/set_body_state). 좌표는 MuJoCo world."""
import rclpy
from mujoco_ros_msgs.srv import GetBodyState, SetBodyState


class SimBodies:
    def __init__(self, node, timeout=5.0):
        self._node = node
        self._timeout = timeout
        self._get = node.create_client(GetBodyState, '/get_body_state')
        self._set = node.create_client(SetBodyState, '/set_body_state')
        for client in (self._get, self._set):
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f'{client.srv_name} not available (is the sim running?)')

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=self._timeout)
        result = future.result()
        return result if result is not None and result.success else None

    def pose(self, name):
        """geometry_msgs/Pose 또는 None."""
        request = GetBodyState.Request()
        request.name = name
        result = self._call(self._get, request)
        return None if result is None else result.state.pose.pose

    def position(self, name):
        pose = self.pose(name)
        return None if pose is None else (pose.position.x, pose.position.y, pose.position.z)

    def place(self, name, x, y, z):
        """위치 설정 + 속도 0 (이전 움직임이 남지 않게)."""
        request = SetBodyState.Request()
        request.state.name = name
        request.state.pose.pose.position.x = x
        request.state.pose.pose.position.y = y
        request.state.pose.pose.position.z = z
        request.state.pose.pose.orientation.w = 1.0
        request.set_pose = True
        request.set_twist = True
        return self._call(self._set, request) is not None
