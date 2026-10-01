"""pre-grasp 테스트: 큐브 위치를 읽고, 현재 방향을 유지한 채 큐브 위 approach_height로 한 번 이동."""
import rclpy

from .motion import current_ee_pose, move_to_pose, pilz_params, target_pose
from .moveit_setup import create_moveit
from .sim_bodies import SimBodies

OBJECT, EE_LINK, APPROACH_HEIGHT = 'obj_box_01', 'panda_hand_tcp', 0.15


def main():
    rclpy.init()
    node = rclpy.create_node('pregrasp')
    moveit = create_moveit(node_name='pregrasp_moveit')
    arm = moveit.get_planning_component('panda_arm')
    x, y, z = SimBodies(node).position(OBJECT)
    node.get_logger().info(f'{OBJECT} at ({x:.3f}, {y:.3f}, {z:.3f})')

    pose = target_pose((x, y, z + APPROACH_HEIGHT), current_ee_pose(moveit, EE_LINK).orientation)
    ok = move_to_pose(moveit, arm, pose, EE_LINK, pilz_params(moveit, 'PTP', 0.3))
    node.get_logger().info('pre-grasp ' + ('done' if ok else 'FAILED'))

    moveit.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()
