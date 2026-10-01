"""config/pick_place.yaml의 값을 노드 파라미터 기본값으로 선언 (실행 시 -p 이름:=값 으로 덮어쓰기 가능)."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory


def declare_from_yaml(node):
    """yaml에서 node 이름과 같은 블록을 읽어 선언하고, 값을 읽는 함수 p(name)를 돌려줌."""
    path = os.path.join(get_package_share_directory('airlab_pick_place'), 'config', 'pick_place.yaml')
    with open(path) as f:
        defaults = yaml.safe_load(f)[node.get_name()]['ros__parameters']
    for name, value in defaults.items():
        node.declare_parameter(name, value)
    return lambda name: node.get_parameter(name).value
