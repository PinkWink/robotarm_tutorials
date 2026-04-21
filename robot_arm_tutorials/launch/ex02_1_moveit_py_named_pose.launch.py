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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_share = get_package_share_directory('robot_arm_description')
    urdf_xacro = os.path.join(description_share, 'urdf', 'robot_arm.urdf.xacro')

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

    ex02_1_node = Node(
        package='robot_arm_tutorials',
        executable='ex02_1_moveit_py_named_pose',
        name='moveit_py_named_pose',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([ex02_1_node])
