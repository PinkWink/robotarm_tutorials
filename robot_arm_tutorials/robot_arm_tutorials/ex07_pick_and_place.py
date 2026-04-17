"""
예제 07: Pick & Place (잡기와 놓기)
====================================
RViz Marker로 Pick/Place 테이블을 시각화하고,
가상 물체를 잡아 다른 위치로 옮기는 시퀀스를 실행.

/compute_ik 서비스로 현재 관절 상태를 시드로 IK를 풀어
각 단계에서 최소 관절 이동을 보장한다.

학습 내용:
- IK 시드를 활용한 연속 이동 (과도한 회전 방지)
- 팔 + 그리퍼 통합 시퀀스
- Pick & Place 워크플로우
- RViz Marker를 이용한 환경 시각화

실행 방법:
  터미널1: ros2 launch robot_arm_moveit_config demo.launch.xml
  터미널2: ros2 run robot_arm_tutorials ex07_pick_and_place --ros-args -p use_sim_time:=true
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState, MoveItErrorCodes
from robot_arm_tutorials.utils import (
    MoveGroupHelper, GripperHelper, make_pose,
)


class PickAndPlaceDemo(Node):
    def __init__(self):
        super().__init__('pick_and_place_demo')
        self.get_logger().info('=== 예제 07: Pick & Place ===')

        latched_qos = QoSProfile(depth=10)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._marker_pub = self.create_publisher(
            MarkerArray, '/pick_place_markers', latched_qos
        )
        self._marker_array = MarkerArray()
        self._marker_timer = self.create_timer(2.0, self._republish_markers)

        self._ik_client = self.create_client(GetPositionIK, 'compute_ik')

    def _republish_markers(self):
        if self._marker_array.markers:
            self._marker_pub.publish(self._marker_array)

    def _spin_sleep(self, duration):
        end_time = time.time() + duration
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _go_smooth(self, arm, pose, label):
        """현재 관절을 시드로 IK를 풀어 최소 관절 이동으로 목표에 도달"""
        if not self._ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('compute_ik 없음, go_to_pose_goal 폴백')
            return arm.go_to_pose_goal(pose)

        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = arm.PLANNING_GROUP
        ik_req.ik_request.robot_state = arm.get_current_robot_state()

        ps = PoseStamped()
        ps.header.frame_id = arm.REFERENCE_FRAME
        ps.pose = pose
        ik_req.ik_request.pose_stamped = ps
        ik_req.ik_request.avoid_collisions = False

        future = self._ik_client.call_async(ik_req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()

        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(f'{label}: IK 실패, go_to_pose_goal 폴백')
            return arm.go_to_pose_goal(pose)

        joint_values = {}
        for i, name in enumerate(resp.solution.joint_state.name):
            if name in arm.ARM_JOINT_NAMES:
                joint_values[name] = resp.solution.joint_state.position[i]

        return arm.go_to_joint_goal(joint_values)

    def _publish_tables(self, pick_pos, place_pos):
        """Pick/Place 테이블을 RViz Marker로 표시"""
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, (pos, label, color) in enumerate([
            (pick_pos, 'Pick', ColorRGBA(r=0.3, g=0.7, b=0.3, a=0.6)),
            (place_pos, 'Place', ColorRGBA(r=0.3, g=0.3, b=0.8, a=0.6)),
        ]):
            table = Marker()
            table.header.frame_id = 'base_link'
            table.header.stamp = stamp
            table.ns = 'tables'
            table.id = i
            table.type = Marker.CUBE
            table.action = Marker.ADD
            table.pose.position.x = pos[0]
            table.pose.position.y = pos[1]
            table.pose.position.z = pos[2] - 0.06
            table.pose.orientation.w = 1.0
            table.scale = Vector3(x=0.10, y=0.10, z=0.10)
            table.color = color
            ma.markers.append(table)

            text = Marker()
            text.header.frame_id = 'base_link'
            text.header.stamp = stamp
            text.ns = 'labels'
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pos[0]
            text.pose.position.y = pos[1]
            text.pose.position.z = pos[2] + 0.08
            text.pose.orientation.w = 1.0
            text.scale.z = 0.03
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = label
            ma.markers.append(text)

        self._marker_array = ma
        self._marker_pub.publish(ma)

    def run(self):
        arm = MoveGroupHelper(self)
        gripper = GripperHelper(self)
        arm.max_velocity_scaling = 0.3
        arm.planning_time = 15.0
        arm.num_planning_attempts = 10

        if not arm.wait_for_servers(timeout_sec=30.0):
            return
        if not gripper.wait_for_server(timeout_sec=30.0):
            return
        if not arm.wait_for_joint_state(timeout_sec=10.0):
            return

        # Pick/Place 위치 정의
        pick_pos = (0.30, 0.0, 0.30)
        place_pos = (0.0, 0.30, 0.30)

        # 테이블 Marker 표시
        self._publish_tables(pick_pos, place_pos)
        self.get_logger().info('--- 환경: Pick/Place 테이블 표시 (RViz Marker) ---')
        self._spin_sleep(1.0)

        # 1단계: ready + 그리퍼 열기
        self.get_logger().info('--- 1단계: ready 자세 + 그리퍼 열기 ---')
        arm.go_to_named_target('ready')
        gripper.open_gripper()
        self._spin_sleep(1.0)

        # 2단계: Pick 위치로 이동 (IK 시드: 현재 관절)
        self.get_logger().info('--- 2단계: Pick 위치로 이동 ---')
        pick_pose = make_pose(*pick_pos, math.pi, 0.0, 0.0)
        success = self._go_smooth(arm, pick_pose, 'Pick')
        if not success:
            self.get_logger().error('Pick 위치 이동 실패!')
            return
        self._spin_sleep(0.5)

        # 3단계: 그리퍼 닫기
        self.get_logger().info('--- 3단계: 그리퍼 닫기 (가상 물체 잡기) ---')
        gripper.close_gripper()
        self._spin_sleep(1.0)

        # 4단계: 들어올리기 (현재 관절 시드 → 최소 이동)
        self.get_logger().info('--- 4단계: 물체 들어올리기 ---')
        lift = make_pose(pick_pos[0], pick_pos[1], 0.40, math.pi, 0.0, 0.0)
        self._go_smooth(arm, lift, 'Lift')
        self._spin_sleep(0.5)

        # 5단계: Place 위치 상공으로 이동
        self.get_logger().info('--- 5단계: Place 위치로 이동 ---')
        place_above = make_pose(place_pos[0], place_pos[1], 0.40,
                                math.pi, 0.0, math.pi / 2)
        self._go_smooth(arm, place_above, 'Transport')
        self._spin_sleep(0.5)

        # 6단계: 하강
        self.get_logger().info('--- 6단계: 하강하여 놓기 ---')
        place_pose = make_pose(*place_pos, math.pi, 0.0, math.pi / 2)
        self._go_smooth(arm, place_pose, 'Place')
        self._spin_sleep(0.5)

        # 7단계: 그리퍼 열기
        self.get_logger().info('--- 7단계: 그리퍼 열기 (물체 놓기) ---')
        gripper.open_gripper()
        self._spin_sleep(1.0)

        # 8단계: 후퇴 및 복귀
        self.get_logger().info('--- 8단계: 후퇴 및 home 복귀 ---')
        retreat = make_pose(place_pos[0], place_pos[1], 0.40,
                            math.pi, 0.0, math.pi / 2)
        self._go_smooth(arm, retreat, 'Retreat')
        self._spin_sleep(0.5)

        arm.go_to_named_target('home')
        self._spin_sleep(0.5)

        self.get_logger().info('=== 예제 07 완료! ===')


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceDemo()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
