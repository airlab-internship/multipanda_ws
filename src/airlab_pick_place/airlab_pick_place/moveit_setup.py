"""MoveItPy 생성: franka_moveit_config의 로봇 모델·SRDF·컨트롤러 설정 + 이 패키지의 config/*.yaml."""
import os

from ament_index_python.packages import get_package_share_directory
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder

INITIAL_POSITIONS = '0.0 -0.785 0.0 -2.356 0.0 1.571 0.785'


def _config(name):
    return os.path.join(get_package_share_directory('airlab_pick_place'), 'config', name)


def create_moveit(node_name):
    urdf_xacro = os.path.join(get_package_share_directory('franka_description'),
                              'robots', 'sim', 'panda_arm_sim.urdf.xacro')
    config = (
        MoveItConfigsBuilder(robot_name='panda', package_name='franka_moveit_config')
        .robot_description(file_path=urdf_xacro,
                           mappings={'arm_id': 'panda', 'hand': 'true',
                                     'initial_positions': INITIAL_POSITIONS})
        .robot_description_semantic(file_path='srdf/panda_arm.srdf.xacro', mappings={'hand': 'true'})
        .trajectory_execution(file_path='config/panda_controllers.yaml')
        .planning_pipelines(pipelines=['ompl', 'pilz_industrial_motion_planner'],
                            default_planning_pipeline='ompl')
        .joint_limits(file_path=_config('joint_limits.yaml'))
        .pilz_cartesian_limits(file_path=_config('pilz_cartesian_limits.yaml'))
        .moveit_cpp(file_path=_config('moveit_py.yaml'))
        .to_moveit_configs()
    )

    # franka_moveit_config의 ompl_planning.yaml에는 플러그인 이름과 request adapter가 없음
    # (원래 launch 파일에서 직접 넣던 값). 시간 정보를 채우는 AddTimeOptimalParameterization이
    # 빠지면 컨트롤러가 궤적을 거부함.
    ompl = config.planning_pipelines['ompl']
    ompl['planning_plugin'] = 'ompl_interface/OMPLPlanner'
    ompl['request_adapters'] = (
        'default_planner_request_adapters/AddTimeOptimalParameterization '
        'default_planner_request_adapters/ResolveConstraintFrames '
        'default_planner_request_adapters/FixWorkspaceBounds '
        'default_planner_request_adapters/FixStartStateBounds '
        'default_planner_request_adapters/FixStartStateCollision '
        'default_planner_request_adapters/FixStartStatePathConstraints')
    ompl['start_state_max_bounds_error'] = 0.1

    # panda_controllers.yaml 내용은 moveit_simple_controller_manager 이름공간 아래에 있어야 함
    controllers = dict(config.trajectory_execution)
    config.trajectory_execution = {
        'moveit_manage_controllers': controllers.pop('moveit_manage_controllers', True),
        'moveit_simple_controller_manager': controllers,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        # 실행 허용치 완화: 기본값으로는 시뮬레이터 컨트롤러가 계획 시간을 조금만 넘겨도 실행이 중단됨
        'trajectory_execution.allowed_execution_duration_scaling': 5.0,  # 계획 시간의 몇 배까지 허용
        'trajectory_execution.allowed_goal_duration_margin': 3.0,        # 추가 여유 시간 (s)
        'trajectory_execution.allowed_start_tolerance': 0.05,            # 시작 자세 오차 허용 (rad)
    }
    return MoveItPy(node_name=node_name, config_dict=config.to_dict())
