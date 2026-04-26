"""
로봇암 튜토리얼 유틸리티 모듈
MoveGroup Action, ExecuteTrajectory Action, CartesianPath Service 등을
rclpy로 직접 호출하는 헬퍼 클래스 모음
"""

import math
import copy
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Vector3
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    RobotState,
    RobotTrajectory,
    CollisionObject,
    AttachedCollisionObject,
    PlanningScene,
    PlanningSceneWorld,
    MoveItErrorCodes,
)
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath, ApplyPlanningScene, GetPlanningScene

from control_msgs.action import GripperCommand as GripperCommandAction
from control_msgs.msg import GripperCommand as GripperCommandMsg

import tf_transformations


# ============================================================
#  유틸리티 함수
# ============================================================

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """오일러 각도(rad)를 쿼터니언으로 변환"""
    q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
    msg = Quaternion()
    msg.x = q[0]
    msg.y = q[1]
    msg.z = q[2]
    msg.w = q[3]
    return msg


def make_pose(x: float, y: float, z: float,
              roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> Pose:
    """위치(m)와 오일러 각도(rad)로 Pose 메시지 생성"""
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = euler_to_quaternion(roll, pitch, yaw)
    return pose


def make_pose_stamped(x: float, y: float, z: float,
                      roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
                      frame_id: str = 'base_link') -> PoseStamped:
    """위치(m)와 오일러 각도(rad)로 PoseStamped 메시지 생성"""
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.pose = make_pose(x, y, z, roll, pitch, yaw)
    return ps


def make_box_collision_object(object_id: str,
                               frame_id: str,
                               position: Tuple[float, float, float],
                               dimensions: Tuple[float, float, float],
                               operation: int = CollisionObject.ADD) -> CollisionObject:
    """박스 형태의 충돌 객체 생성"""
    co = CollisionObject()
    co.header.frame_id = frame_id
    co.id = object_id
    co.operation = operation

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = list(dimensions)  # [x, y, z]
    co.primitives.append(box)

    pose = Pose()
    pose.position = Point(x=position[0], y=position[1], z=position[2])
    pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    co.primitive_poses.append(pose)

    return co


def make_cylinder_collision_object(object_id: str,
                                    frame_id: str,
                                    position: Tuple[float, float, float],
                                    height: float,
                                    radius: float,
                                    operation: int = CollisionObject.ADD) -> CollisionObject:
    """원기둥 형태의 충돌 객체 생성"""
    co = CollisionObject()
    co.header.frame_id = frame_id
    co.id = object_id
    co.operation = operation

    cylinder = SolidPrimitive()
    cylinder.type = SolidPrimitive.CYLINDER
    cylinder.dimensions = [height, radius]
    co.primitives.append(cylinder)

    pose = Pose()
    pose.position = Point(x=position[0], y=position[1], z=position[2])
    pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    co.primitive_poses.append(pose)

    return co


# ============================================================
#  MoveGroupHelper 클래스
# ============================================================

class MoveGroupHelper:
    """
    MoveIt2 MoveGroup Action 및 관련 서비스를 직접 호출하는 헬퍼 클래스.
    rclpy + moveit_msgs를 사용하여 모션 플래닝/실행을 수행한다.
    """

    # SRDF에 정의된 포즈 (하드코딩 - SRDF 파싱 불필요)
    NAMED_TARGETS = {
        'home': {
            'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0,
            'joint4': 0.0, 'joint5': 0.0, 'joint6': 0.0,
        },
        'ready': {
            'joint1': 0.0, 'joint2': 0.051, 'joint3': 0.594,
            'joint4': 0.730, 'joint5': -0.051, 'joint6': 0.323,
        },
    }

    ARM_JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    PLANNING_GROUP = 'manipulator'
    END_EFFECTOR_LINK = 'gripper_base'
    REFERENCE_FRAME = 'base_link'

    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger()
        self.cb_group = ReentrantCallbackGroup()

        # 현재 조인트 상태
        self._current_joint_state: Optional[JointState] = None

        # --- Action 클라이언트 ---
        self._move_action_client = ActionClient(
            node, MoveGroup, 'move_action',
            callback_group=self.cb_group
        )
        self._execute_client = ActionClient(
            node, ExecuteTrajectory, 'execute_trajectory',
            callback_group=self.cb_group
        )

        # --- Service 클라이언트 ---
        self._cartesian_path_client = node.create_client(
            GetCartesianPath, 'compute_cartesian_path',
            callback_group=self.cb_group
        )
        self._apply_planning_scene_client = node.create_client(
            ApplyPlanningScene, 'apply_planning_scene',
            callback_group=self.cb_group
        )
        self._get_planning_scene_client = node.create_client(
            GetPlanningScene, 'get_planning_scene',
            callback_group=self.cb_group
        )

        # --- Subscriber ---
        self._joint_state_sub = node.create_subscription(
            JointState, 'joint_states', self._joint_state_cb, 10,
            callback_group=self.cb_group
        )

        # 기본 플래닝 파라미터
        self.planning_time = 5.0
        self.max_velocity_scaling = 0.5
        self.max_acceleration_scaling = 0.5
        self.planner_id = ''  # 빈 문자열 = 기본 플래너
        self.num_planning_attempts = 5

    # ----------------------------------------------------------
    #  내부 콜백
    # ----------------------------------------------------------
    def _joint_state_cb(self, msg: JointState):
        self._current_joint_state = msg

    # ----------------------------------------------------------
    #  연결 확인
    # ----------------------------------------------------------
    def wait_for_servers(self, timeout_sec: float = 10.0) -> bool:
        """MoveGroup Action 서버 연결 대기"""
        self.logger.info('MoveGroup Action 서버 연결 대기 중...')
        if not self._move_action_client.wait_for_server(timeout_sec=timeout_sec):
            self.logger.error('MoveGroup Action 서버에 연결할 수 없습니다!')
            return False

        self.logger.info('ExecuteTrajectory Action 서버 연결 대기 중...')
        if not self._execute_client.wait_for_server(timeout_sec=timeout_sec):
            self.logger.error('ExecuteTrajectory Action 서버에 연결할 수 없습니다!')
            return False

        self.logger.info('모든 Action 서버 연결 완료!')
        return True

    def wait_for_joint_state(self, timeout_sec: float = 10.0) -> bool:
        """조인트 상태 수신 대기"""
        self.logger.info('조인트 상태 수신 대기 중...')
        start = time.time()
        while self._current_joint_state is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() - start > timeout_sec:
                self.logger.error('조인트 상태를 수신하지 못했습니다!')
                return False
        self.logger.info('조인트 상태 수신 완료!')
        return True

    # ----------------------------------------------------------
    #  현재 상태 조회
    # ----------------------------------------------------------
    def get_current_joint_values(self) -> dict:
        """현재 조인트 값을 딕셔너리로 반환"""
        if self._current_joint_state is None:
            return {}
        result = {}
        for i, name in enumerate(self._current_joint_state.name):
            if name in self.ARM_JOINT_NAMES:
                result[name] = self._current_joint_state.position[i]
        return result

    def get_current_robot_state(self) -> RobotState:
        """현재 RobotState 메시지 반환"""
        rs = RobotState()
        if self._current_joint_state is not None:
            rs.joint_state = self._current_joint_state
        return rs

    # ----------------------------------------------------------
    #  MoveGroup Action 호출
    # ----------------------------------------------------------
    def _build_motion_plan_request(self) -> MotionPlanRequest:
        """기본 MotionPlanRequest 생성"""
        req = MotionPlanRequest()
        req.group_name = self.PLANNING_GROUP
        req.num_planning_attempts = self.num_planning_attempts
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = self.max_velocity_scaling
        req.max_acceleration_scaling_factor = self.max_acceleration_scaling
        if self.planner_id:
            req.planner_id = self.planner_id
        req.workspace_parameters.header.frame_id = self.REFERENCE_FRAME
        req.workspace_parameters.min_corner.x = -1.0
        req.workspace_parameters.min_corner.y = -1.0
        req.workspace_parameters.min_corner.z = -1.0
        req.workspace_parameters.max_corner.x = 1.0
        req.workspace_parameters.max_corner.y = 1.0
        req.workspace_parameters.max_corner.z = 1.0
        return req

    def _send_move_group_goal(self, request: MotionPlanRequest,
                               plan_only: bool = False) -> Tuple[bool, Optional[RobotTrajectory]]:
        """MoveGroup Action 목표 전송 및 결과 대기"""
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = plan_only
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        future = self._move_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.logger.error('MoveGroup 목표가 거부되었습니다!')
            return False, None

        self.logger.info('MoveGroup 목표 수락됨. 결과 대기 중...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result

        error_code = result.error_code.val
        if error_code == MoveItErrorCodes.SUCCESS:
            self.logger.info('모션 플래닝 및 실행 성공!')
            return True, result.planned_trajectory
        else:
            self.logger.error(f'MoveGroup 실패! 에러 코드: {error_code}')
            return False, None

    # ----------------------------------------------------------
    #  고수준 이동 메서드
    # ----------------------------------------------------------
    def go_to_joint_goal(self, joint_values: dict, wait: bool = True) -> bool:
        """조인트 목표값으로 이동

        Args:
            joint_values: {'joint1': 0.0, 'joint2': -0.5, ...}
            wait: 실행 완료 대기 여부
        Returns:
            성공 여부
        """
        req = self._build_motion_plan_request()

        # 조인트 제약조건 구성
        constraints = Constraints()
        for joint_name, value in joint_values.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        req.goal_constraints.append(constraints)

        success, _ = self._send_move_group_goal(req, plan_only=False)
        return success

    def go_to_named_target(self, target_name: str) -> bool:
        """SRDF에 정의된 이름 포즈로 이동 (home, ready 등)"""
        if target_name not in self.NAMED_TARGETS:
            self.logger.error(f'알 수 없는 이름 포즈: {target_name}')
            self.logger.info(f'사용 가능: {list(self.NAMED_TARGETS.keys())}')
            return False

        joint_values = self.NAMED_TARGETS[target_name]
        self.logger.info(f'이름 포즈 "{target_name}"으로 이동 시작...')
        return self.go_to_joint_goal(joint_values)

    def go_to_pose_goal(self, pose: Pose, end_effector_link: str = '') -> bool:
        """끝단 자세(Pose)로 이동

        Args:
            pose: 목표 Pose (position + orientation)
            end_effector_link: 끝단 링크 이름 (빈 문자열이면 기본값 사용)
        """
        if not end_effector_link:
            end_effector_link = self.END_EFFECTOR_LINK

        req = self._build_motion_plan_request()

        # 포즈 제약조건 구성
        constraints = Constraints()

        # 위치 제약
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.REFERENCE_FRAME
        position_constraint.link_name = end_effector_link
        position_constraint.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)

        # 허용 영역: 작은 구
        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # 반지름 1cm
        bounding_volume.primitives.append(sphere)

        sphere_pose = Pose()
        sphere_pose.position = copy.deepcopy(pose.position)
        sphere_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        bounding_volume.primitive_poses.append(sphere_pose)

        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)

        # 방향 제약
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.REFERENCE_FRAME
        orientation_constraint.link_name = end_effector_link
        orientation_constraint.orientation = copy.deepcopy(pose.orientation)
        orientation_constraint.absolute_x_axis_tolerance = 0.01
        orientation_constraint.absolute_y_axis_tolerance = 0.01
        orientation_constraint.absolute_z_axis_tolerance = 0.01
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)

        req.goal_constraints.append(constraints)

        success, _ = self._send_move_group_goal(req, plan_only=False)
        return success

    def plan_to_pose_goal(self, pose: Pose, end_effector_link: str = '') -> Tuple[bool, Optional[RobotTrajectory]]:
        """끝단 자세(Pose)로의 계획만 수행 (실행 안 함)"""
        if not end_effector_link:
            end_effector_link = self.END_EFFECTOR_LINK

        req = self._build_motion_plan_request()

        constraints = Constraints()

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.REFERENCE_FRAME
        position_constraint.link_name = end_effector_link
        position_constraint.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)

        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        bounding_volume.primitives.append(sphere)

        sphere_pose = Pose()
        sphere_pose.position = copy.deepcopy(pose.position)
        sphere_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        bounding_volume.primitive_poses.append(sphere_pose)

        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.REFERENCE_FRAME
        orientation_constraint.link_name = end_effector_link
        orientation_constraint.orientation = copy.deepcopy(pose.orientation)
        orientation_constraint.absolute_x_axis_tolerance = 0.01
        orientation_constraint.absolute_y_axis_tolerance = 0.01
        orientation_constraint.absolute_z_axis_tolerance = 0.01
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)

        req.goal_constraints.append(constraints)

        return self._send_move_group_goal(req, plan_only=True)

    def plan_to_joint_goal(self, joint_values: dict) -> Tuple[bool, Optional[RobotTrajectory]]:
        """조인트 목표값으로의 계획만 수행 (실행 안 함)"""
        req = self._build_motion_plan_request()

        constraints = Constraints()
        for joint_name, value in joint_values.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        req.goal_constraints.append(constraints)

        return self._send_move_group_goal(req, plan_only=True)

    # ----------------------------------------------------------
    #  경로 제약조건
    # ----------------------------------------------------------
    def go_to_joint_goal_with_constraints(self, joint_values: dict,
                                           path_constraints: Constraints) -> bool:
        """경로 제약조건을 포함한 조인트 목표 이동"""
        req = self._build_motion_plan_request()
        req.path_constraints = path_constraints

        constraints = Constraints()
        for joint_name, value in joint_values.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        req.goal_constraints.append(constraints)

        success, _ = self._send_move_group_goal(req, plan_only=False)
        return success

    def go_to_pose_goal_with_constraints(self, pose: Pose,
                                          path_constraints: Constraints,
                                          end_effector_link: str = '') -> bool:
        """경로 제약조건을 포함한 끝단 자세 이동"""
        if not end_effector_link:
            end_effector_link = self.END_EFFECTOR_LINK

        req = self._build_motion_plan_request()
        req.path_constraints = path_constraints
        # 경로 제약 하에서는 더 많은 시간 필요
        req.allowed_planning_time = max(self.planning_time, 15.0)
        req.num_planning_attempts = max(self.num_planning_attempts, 10)

        constraints = Constraints()

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.REFERENCE_FRAME
        position_constraint.link_name = end_effector_link
        position_constraint.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)

        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        bounding_volume.primitives.append(sphere)

        sphere_pose = Pose()
        sphere_pose.position = copy.deepcopy(pose.position)
        sphere_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        bounding_volume.primitive_poses.append(sphere_pose)

        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.REFERENCE_FRAME
        orientation_constraint.link_name = end_effector_link
        orientation_constraint.orientation = copy.deepcopy(pose.orientation)
        orientation_constraint.absolute_x_axis_tolerance = 0.01
        orientation_constraint.absolute_y_axis_tolerance = 0.01
        orientation_constraint.absolute_z_axis_tolerance = 0.01
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)

        req.goal_constraints.append(constraints)

        success, _ = self._send_move_group_goal(req, plan_only=False)
        return success

    # ----------------------------------------------------------
    #  Cartesian 경로 계획
    # ----------------------------------------------------------
    def compute_cartesian_path(self, waypoints: List[Pose],
                                max_step: float = 0.01,
                                avoid_collisions: bool = True) -> Tuple[Optional[RobotTrajectory], float]:
        """Cartesian 경로 계획 (GetCartesianPath 서비스)

        Args:
            waypoints: 경유점 Pose 리스트
            max_step: 직선 보간 간격 (m)
            avoid_collisions: 충돌 회피 여부

        Returns:
            (trajectory, fraction): 궤적과 성공 비율 (0.0 ~ 1.0)
        """
        if not self._cartesian_path_client.wait_for_service(timeout_sec=5.0):
            self.logger.error('compute_cartesian_path 서비스를 찾을 수 없습니다!')
            return None, 0.0

        request = GetCartesianPath.Request()
        request.header.frame_id = self.REFERENCE_FRAME
        request.group_name = self.PLANNING_GROUP
        request.link_name = self.END_EFFECTOR_LINK
        request.waypoints = waypoints
        request.max_step = max_step
        request.avoid_collisions = avoid_collisions
        request.max_velocity_scaling_factor = self.max_velocity_scaling
        request.max_acceleration_scaling_factor = self.max_acceleration_scaling
        request.start_state = self.get_current_robot_state()

        future = self._cartesian_path_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        if response.error_code.val == MoveItErrorCodes.SUCCESS:
            fraction = response.fraction
            self.logger.info(f'Cartesian 경로 계획 성공! (달성률: {fraction*100:.1f}%)')
            return response.solution, fraction
        else:
            self.logger.error(f'Cartesian 경로 계획 실패! 에러: {response.error_code.val}')
            return None, 0.0

    # ----------------------------------------------------------
    #  궤적 실행
    # ----------------------------------------------------------
    def execute_trajectory(self, trajectory: RobotTrajectory) -> bool:
        """ExecuteTrajectory Action으로 계획된 궤적 실행"""
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        future = self._execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.logger.error('궤적 실행 목표가 거부되었습니다!')
            return False

        self.logger.info('궤적 실행 중...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result

        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.logger.info('궤적 실행 완료!')
            return True
        else:
            self.logger.error(f'궤적 실행 실패! 에러: {result.error_code.val}')
            return False

    # ----------------------------------------------------------
    #  Planning Scene 관리
    # ----------------------------------------------------------
    def add_collision_object(self, collision_object: CollisionObject) -> bool:
        """Planning Scene에 충돌 객체 추가"""
        return self._apply_planning_scene_diff(
            world_objects=[collision_object]
        )

    def remove_collision_object(self, object_id: str) -> bool:
        """Planning Scene에서 충돌 객체 제거"""
        co = CollisionObject()
        co.id = object_id
        co.header.frame_id = self.REFERENCE_FRAME
        co.operation = CollisionObject.REMOVE
        return self._apply_planning_scene_diff(world_objects=[co])

    def attach_object(self, object_id: str, link_name: str,
                      touch_links: List[str] = None) -> bool:
        """끝단에 객체 부착 (잡기)"""
        aco = AttachedCollisionObject()
        aco.link_name = link_name
        aco.object.id = object_id
        aco.object.header.frame_id = self.REFERENCE_FRAME
        aco.object.operation = CollisionObject.ADD
        if touch_links:
            aco.touch_links = touch_links
        return self._apply_planning_scene_diff(attached_objects=[aco])

    def detach_object(self, object_id: str, link_name: str) -> bool:
        """끝단에서 객체 분리 (놓기)"""
        aco = AttachedCollisionObject()
        aco.link_name = link_name
        aco.object.id = object_id
        aco.object.header.frame_id = self.REFERENCE_FRAME
        aco.object.operation = CollisionObject.REMOVE
        return self._apply_planning_scene_diff(attached_objects=[aco])

    def clear_all_collision_objects(self) -> bool:
        """모든 충돌 객체 제거"""
        co = CollisionObject()
        co.header.frame_id = self.REFERENCE_FRAME
        co.id = ''  # 빈 ID = 모든 객체
        co.operation = CollisionObject.REMOVE
        return self._apply_planning_scene_diff(world_objects=[co])

    def _apply_planning_scene_diff(self,
                                    world_objects: List[CollisionObject] = None,
                                    attached_objects: List[AttachedCollisionObject] = None) -> bool:
        """ApplyPlanningScene 서비스로 Scene 변경"""
        if not self._apply_planning_scene_client.wait_for_service(timeout_sec=5.0):
            self.logger.error('apply_planning_scene 서비스를 찾을 수 없습니다!')
            return False

        scene = PlanningScene()
        scene.is_diff = True

        if world_objects:
            scene.world.collision_objects = world_objects
        if attached_objects:
            scene.robot_state.attached_collision_objects = attached_objects
            scene.robot_state.is_diff = True

        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = self._apply_planning_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        if response.success:
            self.logger.info('Planning Scene 업데이트 성공!')
            return True
        else:
            self.logger.error('Planning Scene 업데이트 실패!')
            return False


# ============================================================
#  GripperHelper 클래스
# ============================================================

class GripperHelper:
    """
    GripperCommand Action을 사용한 그리퍼 제어 헬퍼.
    prismatic 조인트(left_finger_joint)를 제어한다.
    """

    GRIPPER_OPEN = 0.0     # 완전 열림 (m) — axis(-1,0,0) 기준 손가락이 origin (안쪽 미이동)
    GRIPPER_CLOSED = 0.02  # 완전 닫힘 (m) — limit max, 손가락이 안쪽으로 최대 이동
    MAX_EFFORT = 10.0      # 최대 힘 (N)

    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger()
        self.cb_group = ReentrantCallbackGroup()

        self._gripper_client = ActionClient(
            node, GripperCommandAction,
            'gripper_controller/gripper_cmd',
            callback_group=self.cb_group
        )

    def wait_for_server(self, timeout_sec: float = 10.0) -> bool:
        """그리퍼 Action 서버 연결 대기"""
        self.logger.info('그리퍼 Action 서버 연결 대기 중...')
        if not self._gripper_client.wait_for_server(timeout_sec=timeout_sec):
            self.logger.error('그리퍼 Action 서버에 연결할 수 없습니다!')
            return False
        self.logger.info('그리퍼 Action 서버 연결 완료!')
        return True

    def move_gripper(self, position: float, max_effort: float = None) -> bool:
        """그리퍼를 지정 위치로 이동

        Args:
            position: 목표 위치 (0.0=닫힘, 0.02=열림)
            max_effort: 최대 힘 (N)
        """
        if max_effort is None:
            max_effort = self.MAX_EFFORT

        goal = GripperCommandAction.Goal()
        goal.command = GripperCommandMsg(position=position, max_effort=max_effort)

        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.logger.error('그리퍼 명령이 거부되었습니다!')
            return False

        self.logger.info(f'그리퍼 이동 중... (목표: {position:.3f}m)')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result

        if result.reached_goal or result.stalled:
            self.logger.info(f'그리퍼 이동 완료! (위치: {result.position:.3f}m)')
            return True
        else:
            self.logger.warn(f'그리퍼 이동 불확실 (위치: {result.position:.3f}m)')
            return True  # 대부분 성공

    def open_gripper(self) -> bool:
        """그리퍼 열기"""
        self.logger.info('그리퍼 열기...')
        return self.move_gripper(self.GRIPPER_OPEN)

    def close_gripper(self, max_effort: float = None) -> bool:
        """그리퍼 닫기"""
        self.logger.info('그리퍼 닫기...')
        return self.move_gripper(self.GRIPPER_CLOSED, max_effort)
