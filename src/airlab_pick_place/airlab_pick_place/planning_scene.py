"""잡은 큐브를 MoveIt planning scene에 붙이고 떼기.

MoveIt 계획용 충돌 검사에만 영향을 주고 MuJoCo 물리에는 영향이 없음 (실제로 잡는 건 손가락 마찰).
큐브는 잡는 순간에만 추가함: 미리 추가하면 접근·하강 계획에서 장애물로 취급되어 계획이 실패함.
"""
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive

from .motion import PLANNING_FRAME

HAND_LINK = 'panda_hand'
TOUCH_LINKS = ['panda_hand', 'panda_leftfinger', 'panda_rightfinger']


def attach_box(moveit, object_id, pose_in_base, size=0.06):
    box = CollisionObject()
    box.id = object_id
    box.header.frame_id = PLANNING_FRAME
    primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[size, size, size])
    box.primitives.append(primitive)
    box.primitive_poses.append(pose_in_base)
    box.operation = CollisionObject.ADD

    attached = AttachedCollisionObject(link_name=HAND_LINK, touch_links=TOUCH_LINKS)
    attached.object.id = object_id
    attached.object.operation = CollisionObject.ADD

    with moveit.get_planning_scene_monitor().read_write() as scene:
        scene.apply_collision_object(box)
        scene.process_attached_collision_object(attached)
        scene.current_state.update()


def detach_box(moveit, object_id):
    """떼고 scene에서도 제거. 놓은 자리에 남겨 두면 바로 다음 후퇴 동작이 손과의 충돌로 막힘."""
    detached = AttachedCollisionObject(link_name=HAND_LINK)
    detached.object.id = object_id
    detached.object.operation = CollisionObject.REMOVE
    removed = CollisionObject(id=object_id, operation=CollisionObject.REMOVE)
    with moveit.get_planning_scene_monitor().read_write() as scene:
        scene.process_attached_collision_object(detached)
        scene.apply_collision_object(removed)
        scene.current_state.update()
