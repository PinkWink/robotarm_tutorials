"""
예제 02: Named Pose 이동 (Self-contained)
=========================================
SRDF에 정의된 'home'과 'ready' 포즈로 순차 이동하는 예제.
utils.py에 의존하지 않고 단일 파일로 완결되도록 작성되었다.

학습 내용:
- MoveGroup Action 직접 호출 (rclpy + moveit_msgs)
- SRDF group_state 파싱: 로봇에 정의된 이름 포즈를 코드에서 읽어오기
- 현재 조인트 상태 확인과 모션 플래닝 요청의 흐름

실행 방법:
  터미널1: ros2 launch robot_arm_moveit_config demo.launch.xml
  터미널2: ros2 run robot_arm_tutorials ex02_named_pose --ros-args -p use_sim_time:=true
"""

import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter_client import AsyncParameterClient

from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
)


PLANNING_GROUP = 'manipulator'
REFERENCE_FRAME = 'base_link'
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


class NamedPoseDemo(Node):
    def __init__(self):
        super().__init__('named_pose_demo')
        self.get_logger().info('=== 예제 02: Named Pose 이동 (self-contained) ===')

        self._move_client = ActionClient(self, MoveGroup, 'move_action')
        self._joint_state = None
        self.create_subscription(JointState, 'joint_states',
                                 self._joint_state_cb, 10)

        # SRDF 로드 후 채워짐: {'home': {...}, 'ready': {...}}
        self.named_targets: dict = {}

    # ----------------------------------------------------------
    #  SRDF 파싱
    # ----------------------------------------------------------
    def load_named_targets_from_srdf(self, timeout_sec: float = 10.0) -> bool:
        """move_group 노드의 robot_description_semantic 파라미터에서
        SRDF 문자열을 읽어 group_state(이름 포즈) 딕셔너리를 만든다.
        """
        self.get_logger().info('move_group 파라미터에서 SRDF 로드 중...')
        client = AsyncParameterClient(self, 'move_group')
        if not client.wait_for_services(timeout_sec=timeout_sec):
            self.get_logger().error('move_group 파라미터 서비스에 연결할 수 없습니다!')
            return False

        future = client.get_parameters(['robot_description_semantic'])
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        response = future.result()
        if response is None or not response.values:
            self.get_logger().error('robot_description_semantic 파라미터를 가져오지 못했습니다!')
            return False

        srdf_xml = response.values[0].string_value
        if not srdf_xml:
            self.get_logger().error('SRDF 내용이 비어 있습니다!')
            return False

        try:
            root = ET.fromstring(srdf_xml)
        except ET.ParseError as e:
            self.get_logger().error(f'SRDF XML 파싱 실패: {e}')
            return False

        # <group_state name="..." group="manipulator">의 joint value 수집
        for gs in root.findall('group_state'):
            if gs.attrib.get('group') != PLANNING_GROUP:
                continue
            name = gs.attrib.get('name')
            joints = {}
            for j in gs.findall('joint'):
                jname = j.attrib.get('name')
                try:
                    jval = float(j.attrib.get('value', '0.0'))
                except ValueError:
                    continue
                joints[jname] = jval
            if name and joints:
                self.named_targets[name] = joints

        if not self.named_targets:
            self.get_logger().error(f'SRDF에서 "{PLANNING_GROUP}" 그룹의 group_state를 찾지 못했습니다!')
            return False

        self.get_logger().info(f'SRDF에서 로드된 이름 포즈: {list(self.named_targets.keys())}')
        for pname, jv in self.named_targets.items():
            pretty = ', '.join(f'{k}={v:.3f}' for k, v in jv.items())
            self.get_logger().info(f'  - {pname}: {pretty}')
        return True

    # ----------------------------------------------------------
    #  준비 상태 확인
    # ----------------------------------------------------------
    def _joint_state_cb(self, msg: JointState):
        self._joint_state = msg

    def wait_for_ready(self, timeout_sec: float = 30.0) -> bool:
        self.get_logger().info('MoveGroup Action 서버 연결 대기 중...')
        if not self._move_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('MoveGroup Action 서버에 연결할 수 없습니다!')
            return False

        self.get_logger().info('joint_states 수신 대기 중...')
        start = time.time()
        while self._joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout_sec:
                self.get_logger().error('joint_states 를 수신하지 못했습니다!')
                return False
        return True

    # ----------------------------------------------------------
    #  MoveGroup 호출
    # ----------------------------------------------------------
    def go_to_named_target(self, name: str) -> bool:
        if name not in self.named_targets:
            self.get_logger().error(
                f'알 수 없는 이름 포즈: "{name}" (사용 가능: {list(self.named_targets.keys())})')
            return False

        target_joints = self.named_targets[name]
        self.get_logger().info(f'이름 포즈 "{name}"(으)로 이동 시작...')

        req = MotionPlanRequest()
        req.group_name = PLANNING_GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5
        req.workspace_parameters.header.frame_id = REFERENCE_FRAME
        req.workspace_parameters.min_corner.x = -1.0
        req.workspace_parameters.min_corner.y = -1.0
        req.workspace_parameters.min_corner.z = -1.0
        req.workspace_parameters.max_corner.x = 1.0
        req.workspace_parameters.max_corner.y = 1.0
        req.workspace_parameters.max_corner.z = 1.0

        constraints = Constraints()
        for joint_name, value in target_joints.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        send_future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('MoveGroup 목표가 거부되었습니다!')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f'이름 포즈 "{name}" 도달 완료!')
            return True
        self.get_logger().error(
            f'MoveGroup 실패! 에러 코드: {result.error_code.val}')
        return False

    # ----------------------------------------------------------
    #  시나리오 실행
    # ----------------------------------------------------------
    def run(self):
        if not self.wait_for_ready():
            return
        if not self.load_named_targets_from_srdf():
            return

        sequence = ['home', 'ready', 'home']
        for step, name in enumerate(sequence, start=1):
            self.get_logger().info(f'--- {step}단계: "{name}" 포즈로 이동 ---')
            if not self.go_to_named_target(name):
                self.get_logger().error(f'"{name}" 이동 실패 — 시나리오 중단')
                return
            time.sleep(1.0)

        self.get_logger().info('=== 예제 02 완료! ===')


def main(args=None):
    rclpy.init(args=args)
    node = NamedPoseDemo()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
