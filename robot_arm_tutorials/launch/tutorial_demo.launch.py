"""
튜토리얼 데모 런치 파일
MoveIt + Gazebo 환경에서 튜토리얼 노드를 실행합니다.

사용 방법:
  ros2 launch robot_arm_tutorials tutorial_demo.launch.py tutorial:=ex01_joint_state_reader
  ros2 launch robot_arm_tutorials tutorial_demo.launch.py tutorial:=ex02_named_pose
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 런치 인자
    tutorial_arg = DeclareLaunchArgument(
        'tutorial',
        default_value='ex01_joint_state_reader',
        description='실행할 튜토리얼 이름 (ex01 ~ ex10)'
    )

    # MoveIt demo 런치 포함
    moveit_config_dir = get_package_share_directory('robot_arm_moveit_config')
    demo_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(moveit_config_dir, 'launch', 'demo.launch.xml')
        )
    )

    # 튜토리얼 노드
    tutorial_node = Node(
        package='robot_arm_tutorials',
        executable=LaunchConfiguration('tutorial'),
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        tutorial_arg,
        demo_launch,
        tutorial_node,
    ])
