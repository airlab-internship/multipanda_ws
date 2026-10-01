"""pick & place용 시뮬레이터: MuJoCo(씬·카메라) + 팔 컨트롤러 + 로봇 상태 발행.

pick_place_server, collect_episodes는 시뮬레이터가 완전히 뜬 뒤 ros2 run으로 따로 실행.
"""
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node

INITIAL_POSITIONS = '"0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"'


def generate_launch_description():
    scene = os.path.join(share('franka_description'), 'mujoco', 'franka', 'scene_add_camera.xml')
    controllers = os.path.join(share('airlab_pick_place'), 'config', 'sim_controllers.yaml')
    xacro_file = os.path.join(share('franka_description'), 'robots', 'sim', 'panda_arm_sim.urdf.xacro')
    robot_description = Command([FindExecutable(name='xacro'), ' ', xacro_file,
                                 ' arm_id:=panda hand:=true initial_positions:=', INITIAL_POSITIONS])

    mujoco = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(share('franka_bringup'), 'launch', 'sim', 'launch_mujoco_ros_server.launch')),
        launch_arguments={'use_sim_time': 'true', 'modelfile': scene, 'ns': '',
                          'mujoco_plugin_config': controllers, 'unpause': 'true'}.items())

    robot_state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
                                 parameters=[{'robot_description': robot_description}])

    # 팔(joint_state_broadcaster)과 그리퍼(panda_gripper_sim_node)의 관절 상태를 /joint_states로 합침
    joint_state_publisher = Node(package='joint_state_publisher', executable='joint_state_publisher',
                                 parameters=[{'source_list': ['/joint_states',
                                                              '/panda_gripper_sim_node/joint_states'],
                                              'rate': 15}])

    spawners = [Node(package='controller_manager', executable='spawner', arguments=[name])
                for name in ('joint_state_broadcaster', 'panda_arm_controller')]

    return LaunchDescription([mujoco, robot_state_publisher, joint_state_publisher, *spawners])
