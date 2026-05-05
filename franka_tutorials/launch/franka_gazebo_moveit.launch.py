"""Launch FR3 in Gazebo Sim with MoveIt + RViz attached.

This launch is self-contained inside `franka_tutorials`:
- The robot URDF comes from franka_description (unmodified) via a
  thin wrapper xacro under `franka_tutorials/urdf/fr3_gazebo.urdf.xacro`
  that only adds Gazebo gravity overrides.
- MoveIt configs (kinematics, OMPL, controllers, RViz) are loaded as-is
  from `franka_fr3_moveit_config`.
- The controllers YAML referenced by franka URDF when gazebo:=true
  (`franka_gazebo_bringup/config/franka_gazebo_controllers.yaml`)
  is provided by the slim `franka_gazebo_bringup` package in this repo.

The flow:
  1) xacro -> URDF (gazebo + ros2_control + effort + hand)
  2) robot_state_publisher
  3) Gazebo Sim (empty world)
  4) /clock bridge (Gz -> ROS)
  5) Spawn entity from /robot_description into Gz
  6) After spawn: spawn joint_state_broadcaster, fr3_arm_controller
  7) move_group + RViz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import yaml


def _load_yaml(package_name, file_path):
    pkg_share = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_share, file_path)
    try:
        with open(abs_path, 'r') as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def generate_launch_description():
    gz_args = LaunchConfiguration('gz_args')
    rviz = LaunchConfiguration('rviz')
    urdf_file = LaunchConfiguration('urdf_file')

    declare_gz_args = DeclareLaunchArgument(
        'gz_args', default_value='-r empty.sdf',
        description='Args forwarded to ros_gz_sim/gz_sim.launch.py')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Run RViz with the MoveIt config')
    declare_urdf_file = DeclareLaunchArgument(
        'urdf_file', default_value='fr3_gazebo.urdf.xacro',
        description='xacro file under franka_tutorials/urdf to load')

    # --- URDF (from our wrapper that includes franka_description fr3 unmodified) ---
    franka_xacro = PathJoinSubstitution([
        FindPackageShare('franka_tutorials'), 'urdf', urdf_file,
    ])
    robot_description_config = Command([
        FindExecutable(name='xacro'), ' ', franka_xacro,
        ' hand:=true',
        ' ee_id:=franka_hand',
        ' ros2_control:=true',
        ' gazebo:=true',
        ' gazebo_effort:=true',
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_config, value_type=str)
    }

    # --- SRDF (from franka_description, unmodified) ---
    franka_srdf = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.srdf.xacro',
    )
    robot_description_semantic_config = Command([
        FindExecutable(name='xacro'), ' ', franka_srdf,
        ' hand:=true', ' ee_id:=franka_hand',
    ])
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            robot_description_semantic_config, value_type=str)
    }

    # --- MoveIt configs from franka_fr3_moveit_config ---
    kinematics_yaml = _load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')
    kinematics_config = {'robot_description_kinematics': kinematics_yaml}

    joint_limits_yaml = _load_yaml(
        'franka_fr3_moveit_config', 'config/fr3_joint_limits.yaml')
    joint_limits_config = {'robot_description_planning': joint_limits_yaml}

    ompl_yaml = _load_yaml(
        'franka_fr3_moveit_config', 'config/ompl_planning.yaml')
    ompl_config = {
        'move_group': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
            ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath',
            ],
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_config['move_group'].update(ompl_yaml)

    moveit_simple_yaml = _load_yaml(
        'franka_fr3_moveit_config', 'config/fr3_controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_yaml,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    psm_params = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    sim_time = {'use_sim_time': True}

    # --- Nodes ---
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='both', parameters=[robot_description, sim_time],
    )

    move_group_node = Node(
        package='moveit_ros_move_group', executable='move_group',
        output='screen',
        parameters=[
            robot_description, robot_description_semantic, kinematics_config,
            joint_limits_config, ompl_config, trajectory_execution,
            moveit_controllers, psm_params, sim_time,
        ],
    )

    rviz_config = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'),
        'rviz', 'moveit.rviz',
    )
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config], output='log',
        parameters=[
            robot_description, robot_description_semantic,
            ompl_config, kinematics_config, sim_time,
        ],
        condition=IfCondition(rviz),
    )

    # --- Gazebo Sim ---
    # Tell Gazebo where to find franka mesh files.
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(
        get_package_share_directory('franka_description'))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
            ])
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', '/robot_description', '-name', 'fr3'],
        output='screen',
    )

    bridge_clock = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # --- Controller spawners (after Gazebo entity spawn) ---
    spawn_jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager-timeout', '60'],
        output='screen',
    )
    spawn_arm = Node(
        package='controller_manager', executable='spawner',
        arguments=['fr3_arm_controller',
                   '--controller-manager-timeout', '60'],
        output='screen',
    )
    # Holds fr3_finger_joint1 in place so the gripper does not jitter
    # (Gazebo Sim does not enforce URDF <mimic> on fr3_finger_joint2,
    # but locking joint1 is enough to remove the visual wiggle in RViz
    # via robot_state_publisher's mimic resolution).
    spawn_gripper = Node(
        package='controller_manager', executable='spawner',
        arguments=['fr3_gripper_controller',
                   '--controller-manager-timeout', '60'],
        output='screen',
    )

    # spawn_entity exits cleanly once the robot is in Gazebo; controllers come up after.
    after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[spawn_jsb, spawn_arm, spawn_gripper],
        )
    )

    return LaunchDescription([
        declare_gz_args,
        declare_rviz,
        declare_urdf_file,
        rsp,
        gz_sim,
        bridge_clock,
        spawn_entity,
        after_spawn,
        move_group_node,
        rviz_node,
    ])
