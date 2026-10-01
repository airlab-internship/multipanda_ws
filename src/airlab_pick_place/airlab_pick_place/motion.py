"""좌표 변환, 목표 pose 생성, Pilz 계획·실행."""
from geometry_msgs.msg import PoseStamped
from moveit.planning import PlanRequestParameters

PLANNING_FRAME = 'panda_link0'


def world_to_base(x, y, z):
    """MuJoCo world -> panda_link0. panda_link0은 world 기준 Z축 180° 회전 (panda.xml quat="0 0 0 1")."""
    return -x, -y, z


def target_pose(world_xyz, orientation):
    """MuJoCo world 좌표의 목표 위치 + 유지할 방향 -> planning frame PoseStamped."""
    pose = PoseStamped()
    pose.header.frame_id = PLANNING_FRAME
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = world_to_base(*world_xyz)
    pose.pose.orientation = orientation
    return pose


def current_ee_pose(moveit, ee_link):
    with moveit.get_planning_scene_monitor().read_only() as scene:
        return scene.current_state.get_pose(ee_link)


def pilz_params(moveit, planner_id, scale):
    """PTP = 관절공간 직선 (큰 자세 변경), LIN = 작업공간 직선 (짧은 접근·들기·놓기)."""
    params = PlanRequestParameters(moveit)
    params.planning_pipeline = 'pilz_industrial_motion_planner'
    params.planner_id = planner_id
    params.planning_time = 5.0
    params.planning_attempts = 1
    params.max_velocity_scaling_factor = scale
    params.max_acceleration_scaling_factor = scale
    return params


def _plan_and_execute(moveit, arm, params, on_planned):
    plan = arm.plan(parameters=params)
    if not plan:
        return False
    if on_planned is not None:
        on_planned(plan.trajectory)
    # moveit.execute()는 실행 내내 GIL을 잡고 있어 다른 스레드(명령 pose 타이머)가 멈춤.
    # TrajectoryExecutionManager의 execute_and_wait()는 GIL을 풀어 줌.
    tem = moveit.get_trajectory_execution_manager()
    tem.push(plan.trajectory.get_robot_trajectory_msg())
    return bool(tem.execute_and_wait())


def move_to_pose(moveit, arm, pose_stamped, ee_link, params, on_planned=None):
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=pose_stamped, pose_link=ee_link)
    return _plan_and_execute(moveit, arm, params, on_planned)


def move_to_named(moveit, arm, name, params, on_planned=None):
    """SRDF group_state (예: "ready")로 이동."""
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name=name)
    return _plan_and_execute(moveit, arm, params, on_planned)
