"""
MoveIt Servo + 키보드 텔레오퍼레이션 런치 파일
===============================================
demo.launch.xml을 포함하여 Gazebo + MoveIt + RViz를 시작하고,
5초 후 MoveIt Servo 노드를 시작합니다.

키보드 노드는 별도 터미널에서 수동 실행:
  ros2 run robot_arm_tutorials ex11_keyboard_servo --ros-args -p use_sim_time:=true
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    moveit_config_dir = get_package_share_directory('robot_arm_moveit_config')

    # demo.launch.xml 포함 (Gazebo + MoveIt + RViz)
    demo_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(moveit_config_dir, 'launch', 'demo.launch.xml')
        ),
    )

    # Servo 파라미터 로드
    servo_params = {
        'moveit_servo': ParameterBuilder('robot_arm_moveit_config')
        .yaml('config/servo_parameters.yaml')
        .to_dict()
    }

    # URDF / SRDF 로드 (servo_node에 필요)
    urdf_file = os.path.join(
        get_package_share_directory('robot_arm_description'),
        'urdf', 'robot_arm.urdf.xacro',
    )
    srdf_file = os.path.join(moveit_config_dir, 'config', 'robot_arm.srdf')
    kinematics_file = os.path.join(moveit_config_dir, 'config', 'kinematics.yaml')
    joint_limits_file = os.path.join(moveit_config_dir, 'config', 'joint_limits.yaml')

    # xacro로 URDF 처리
    import subprocess
    urdf_content = subprocess.check_output(['xacro', urdf_file]).decode('utf-8')

    with open(srdf_file, 'r') as f:
        srdf_content = f.read()

    # 가속도 제한 필터 설정
    acceleration_filter_update_period = {'update_period': 0.034}
    planning_group_name = {'planning_group_name': 'manipulator'}

    # Servo 노드 (5초 후 시작 - demo 기동 대기)
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        output='screen',
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            {'robot_description': urdf_content},
            {'robot_description_semantic': srdf_content},
            kinematics_file,
            joint_limits_file,
            {'use_sim_time': True},
        ],
    )

    delayed_servo = TimerAction(
        period=5.0,
        actions=[servo_node],
    )

    return LaunchDescription([
        demo_launch,
        delayed_servo,
    ])
