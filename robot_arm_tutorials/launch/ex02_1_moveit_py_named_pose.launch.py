"""
ex02-1 전용 런치 파일 — moveit_py 노드에 MoveIt 파라미터를 실어 실행.

moveit_py는 move_group 서버에 붙지 않고 자체적으로 RobotModel/PlanningScene을
구성하므로 robot_description / SRDF / kinematics / 플래닝 파이프라인 설정을
노드 파라미터로 직접 넘겨줘야 한다.

사용:
  터미널1: ros2 launch robot_arm_moveit_config demo.launch.xml
  터미널2: ros2 launch robot_arm_tutorials ex02_1_moveit_py_named_pose.launch.py
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _flatten_moveit_config(data):
    """MoveItConfigsBuilder.to_dict() 출력을 평탄화한다.

    이 repo 의 yaml 들(ompl_planning.yaml, joint_limits.yaml, kinematics.yaml,
    moveit_controllers.yaml)은 move_group 의 `<param from>` 에 바로 쓸 수
    있게 `/**: ros__parameters:` 래퍼를 갖고 있다. moveit_configs_utils 는
    래퍼를 그대로 각 그룹 키 안에 머지해 버리므로 구조가 한 단 깊어져
    MoveItPy 가 파라미터를 못 읽는다. 각 그룹 키 값이 `/**: ros__parameters:`
    한 덩어리면 내용물을 상위 레벨로 끌어올린다.
    """
    def _is_wrapper(v):
        return (
            isinstance(v, dict)
            and list(v.keys()) == ['ros__parameters']
            and isinstance(v['ros__parameters'], dict)
        )

    result = {}
    for k, v in data.items():
        if k == '/**' and _is_wrapper(v):
            for ik, iv in v['ros__parameters'].items():
                result.setdefault(ik, iv)
        elif (
            isinstance(v, dict)
            and list(v.keys()) == ['/**']
            and _is_wrapper(v['/**'])
        ):
            for ik, iv in v['/**']['ros__parameters'].items():
                result.setdefault(ik, iv)
        else:
            result[k] = v
    return result


def _tuples_to_lists(obj):
    """dict/tuple/list 를 재귀 순회하며 tuple 을 list 로 바꾼다.

    launch_ros 가 파라미터 dict 를 yaml 로 덤프할 때 tuple 이 남아 있으면
    `!!python/tuple` 태그가 붙어 ROS2 파라미터 파서가 이를 배열로 읽지 못한다.
    """
    if isinstance(obj, dict):
        return {k: _tuples_to_lists(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_tuples_to_lists(v) for v in obj]
    return obj


def generate_launch_description():
    description_share = get_package_share_directory('robot_arm_description')
    urdf_xacro = os.path.join(description_share, 'urdf', 'robot_arm.urdf.xacro')

    moveit_cfg_share = get_package_share_directory('robot_arm_moveit_config')
    ompl_yaml_path = os.path.join(moveit_cfg_share, 'config', 'ompl_planning.yaml')
    with open(ompl_yaml_path, 'r') as f:
        ompl_raw = yaml.safe_load(f)
    # ompl_planning.yaml 의 최상위는 `/**: ros__parameters: {...}` 한 덩어리.
    ompl_flat = ompl_raw.get('/**', {}).get('ros__parameters', {}) or ompl_raw

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='robot_arm',
            package_name='robot_arm_moveit_config',
        )
        .robot_description(file_path=urdf_xacro)
        .robot_description_semantic(file_path='config/robot_arm.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    # MoveItConfigsBuilder.planning_pipelines() 는 ompl_planning.yaml 의
    # `/**: ros__parameters:` 래퍼를 그대로 dict 로 머지하기 때문에, 결과가
    # 'ompl' -> '/**' -> 'ros__parameters' -> 'ompl' 처럼 이중으로 싸여
    # MoveItPy 가 `ompl.planning_plugins` 를 찾지 못한다. 여기서 직접 교정.
    params = _flatten_moveit_config(moveit_config.to_dict())
    params['ompl'] = ompl_flat.get('ompl', {})
    # MoveIt 는 `planning_pipelines.pipeline_names: [...]` 형태를 읽는다
    # (ompl_planning.yaml 의 원본 구조). 평탄한 `planning_pipelines: [...]` 가 아님.
    params['planning_pipelines'] = {'pipeline_names': ['ompl']}
    params['default_planning_pipeline'] = 'ompl'

    # MoveItPy.plan() 이 기본으로 요구하는 plan_request_params 기본값.
    params['plan_request_params'] = {
        'planning_pipeline': 'ompl',
        'planner_id': 'RRTConnect',
        'planning_time': 5.0,
        'planning_attempts': 10,
        'max_velocity_scaling_factor': 1.0,
        'max_acceleration_scaling_factor': 1.0,
    }

    # MoveItPy 는 내부에서 moveit_<pid> 노드를 따로 띄우므로 use_sim_time 을
    # `/**:` 스코프(아래 yaml)에 포함시켜야 내부 노드도 시뮬 시계를 따른다.
    # 이게 빠지면 트라젝토리 타임스탬프가 실시간 기준이 되어 Gazebo 컨트롤러가
    # 조용히 건너뛴다 (액션은 SUCCEEDED 로 뜨지만 로봇은 안 움직임).
    params['use_sim_time'] = True

    # Gazebo 실행 직후 관절이 아주 살짝 drift 하면 기본값 0.01 rad 로는
    # 이전 포즈에서 다음 계획으로 넘어갈 때 'start point deviates from current
    # robot state' 로 거부된다. 실기가 아닌 시뮬 용도이므로 여유있게.
    params['trajectory_execution'] = {
        'allowed_start_tolerance': 0.05,
    }

    params = _tuples_to_lists(params)

    # MoveItPy 는 내부 C++ 노드를 'moveit_<pid>' 같은 이름으로 따로 띄우므로
    # launch Node 의 name 스코프로 파라미터를 주면 내부 노드가 읽지 못한다.
    # 직접 `/**: ros__parameters:` 스코프의 yaml 파일을 써서 모든 노드에
    # 적용되도록 한다. 추가로 launch_ros 가 dict 를 yaml 로 변환할 때 list 를
    # tuple 로 바꿔 `!!python/tuple` 태그가 붙는 이슈도 yaml 직접 작성으로 회피.
    params_yaml = {'/**': {'ros__parameters': params}}
    params_file = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='ex02_1_moveit_py_', delete=False
    )
    yaml.safe_dump(params_yaml, params_file, default_flow_style=False)
    params_file.close()

    ex02_1_node = Node(
        package='robot_arm_tutorials',
        executable='ex02_1_moveit_py_named_pose',
        name='moveit_py_named_pose',
        output='screen',
        parameters=[
            params_file.name,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([ex02_1_node])
