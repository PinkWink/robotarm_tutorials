"""
예제 07: Pick & Place (잡기와 놓기)
====================================
RViz Marker로 Pick/Place 테이블을 시각화하고,
가상 물체를 잡아 다른 위치로 옮기는 시퀀스를 실행.

/compute_ik 서비스로 현재 관절 상태를 시드로 IK를 풀어
각 단계에서 최소 관절 이동을 보장한다.

각 단계는 /next_step 토픽 신호로 진행되며,
계획된 끝단 경로를 미리 LINE_STRIP 마커로 RViz에 표시한다.

학습 내용:
- IK 시드를 활용한 연속 이동 (과도한 회전 방지)
- 팔 + 그리퍼 통합 시퀀스
- Pick & Place 워크플로우
- RViz Marker를 이용한 환경 시각화
- 단계별 사용자 트리거 (/next_step)
- FK로 계획된 궤적의 끝단 경로 시각화

실행 방법:
  터미널1: ros2 launch robot_arm_moveit_config demo.launch.xml
  터미널2: ros2 run robot_arm_tutorials ex07_pick_and_place --ros-args -p use_sim_time:=true
  터미널3: ros2 topic pub --once /next_step std_msgs/msg/Empty '{}'
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Vector3, Point
from std_msgs.msg import ColorRGBA, Empty
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import RobotState, MoveItErrorCodes
from robot_arm_tutorials.utils import (
    MoveGroupHelper, GripperHelper, make_pose,
)


class PickAndPlaceDemo(Node):
    PATH_COLOR = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)  # 주황색: 계획된 끝단 경로

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
        self._fk_client = self.create_client(GetPositionFK, 'compute_fk')

        # 사용자 트리거: /next_step 토픽으로 각 단계 진행
        self._trigger_received = False
        self._trigger_sub = self.create_subscription(
            Empty, '/next_step', self._trigger_cb, 10
        )

    def _trigger_cb(self, _msg):
        self._trigger_received = True

    def _wait_for_trigger(self, prompt):
        """/next_step 토픽 메시지가 올 때까지 블로킹 대기"""
        self.get_logger().info(prompt)
        self._trigger_received = False
        while rclpy.ok() and not self._trigger_received:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _republish_markers(self):
        if self._marker_array.markers:
            self._marker_pub.publish(self._marker_array)

    def _spin_sleep(self, duration):
        end_time = time.time() + duration
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _compute_trajectory_ee_path(self, trajectory, timeout_sec=5.0):
        """RobotTrajectory의 각 waypoint에 FK를 적용하여 gripper_base 위치 리스트 반환"""
        if not self._fk_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn('compute_fk 서비스 연결 실패 — 경로 시각화 생략')
            return []

        joint_traj = trajectory.joint_trajectory
        ee_points = []

        for point in joint_traj.points:
            request = GetPositionFK.Request()
            request.header.frame_id = 'base_link'
            request.fk_link_names = ['gripper_base']
            rs = RobotState()
            rs.joint_state.name = list(joint_traj.joint_names)
            rs.joint_state.position = list(point.positions)
            request.robot_state = rs

            future = self._fk_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()

            if (response is not None
                    and response.error_code.val == MoveItErrorCodes.SUCCESS
                    and response.pose_stamped):
                p = response.pose_stamped[0].pose.position
                ee_points.append((p.x, p.y, p.z))

        return ee_points

    def _publish_ee_path_marker(self, ee_points, color=None):
        """끝단 경로를 LINE_STRIP 마커로 발행 (같은 ns/id로 매번 교체)"""
        if not ee_points:
            return

        if color is None:
            color = self.PATH_COLOR

        stamp = self.get_clock().now().to_msg()
        line = Marker()
        line.header.frame_id = 'base_link'
        line.header.stamp = stamp
        line.ns = 'ee_path_line'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.006  # 선 굵기 (m)
        line.color = color
        line.points = [Point(x=p[0], y=p[1], z=p[2]) for p in ee_points]

        self._marker_array.markers = [
            m for m in self._marker_array.markers
            if (m.ns, m.id) != ('ee_path_line', 0)
        ]
        self._marker_array.markers.append(line)
        self._marker_pub.publish(self._marker_array)

    def _plan_and_execute_with_viz(self, arm, plan_result):
        """계획 결과(trajectory)를 받아 끝단 경로를 RViz에 표시한 뒤 실행"""
        success, trajectory = plan_result
        if not success or trajectory is None:
            return False

        ee_points = self._compute_trajectory_ee_path(trajectory)
        if ee_points:
            self.get_logger().info(
                f'  계획된 끝단 경로: {len(ee_points)}개 점 — RViz에 표시'
            )
            self._publish_ee_path_marker(ee_points)

        return arm.execute_trajectory(trajectory)

    def _solve_seeded_ik(self, arm, pose):
        """현재 관절 상태를 시드로 IK 풀이. 실패 시 None"""
        if not self._ik_client.wait_for_service(timeout_sec=5.0):
            return None

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

        if resp is None or resp.error_code.val != MoveItErrorCodes.SUCCESS:
            return None

        joint_values = {}
        for i, name in enumerate(resp.solution.joint_state.name):
            if name in arm.ARM_JOINT_NAMES:
                joint_values[name] = resp.solution.joint_state.position[i]
        return joint_values

    def _go_smooth(self, arm, pose, label):
        """IK 시드 + plan-only + 끝단경로 시각화 + execute

        현재 관절을 시드로 IK를 풀어 최소 관절 이동을 보장하면서,
        실행 전에 계획된 끝단 경로를 RViz에 미리 표시한다.
        """
        joint_values = self._solve_seeded_ik(arm, pose)

        if joint_values is not None:
            plan_result = arm.plan_to_joint_goal(joint_values)
        else:
            self.get_logger().warn(f'{label}: IK 실패, plan_to_pose_goal 폴백')
            plan_result = arm.plan_to_pose_goal(pose)

        success = self._plan_and_execute_with_viz(arm, plan_result)
        if not success:
            self.get_logger().error(f'{label}: 계획/실행 실패')
        return success

    def _go_named(self, arm, name):
        """이름 포즈로 plan-only + 끝단경로 시각화 + execute"""
        if name not in arm.NAMED_TARGETS:
            self.get_logger().error(f'알 수 없는 이름 포즈: {name}')
            return False
        plan_result = arm.plan_to_joint_goal(arm.NAMED_TARGETS[name])
        return self._plan_and_execute_with_viz(arm, plan_result)

    def _publish_tables(self, pick_pos, place_pos):
        """Pick/Place 테이블을 RViz Marker로 표시"""
        stamp = self.get_clock().now().to_msg()
        new_markers = []

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
            new_markers.append(table)

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
            new_markers.append(text)

        new_ns_ids = {(m.ns, m.id) for m in new_markers}
        self._marker_array.markers = [
            m for m in self._marker_array.markers
            if (m.ns, m.id) not in new_ns_ids
        ]
        self._marker_array.markers.extend(new_markers)
        self._marker_pub.publish(self._marker_array)

    def run(self):
        arm = MoveGroupHelper(self)
        gripper = GripperHelper(self)
        arm.max_velocity_scaling = 0.3
        arm.planning_time = 15.0
        arm.num_planning_attempts = 10

        # RViz / 트리거 안내
        self.get_logger().info(
            '[RViz2 안내] 마커를 보려면 MarkerArray Display를 추가하고 '
            "Topic을 '/pick_place_markers' 로 설정하세요."
        )
        self.get_logger().info(
            "[단계 진행] 각 단계는 '/next_step' 토픽 신호로 진행됩니다. "
            "다른 터미널에서 다음 명령을 실행하세요:\n"
            "  ros2 topic pub --once /next_step std_msgs/msg/Empty '{}'"
        )

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
        self._wait_for_trigger(
            '>>> [대기] 1단계 ready 자세 + 그리퍼 열기 신호를 기다리는 중...'
        )
        self.get_logger().info('--- 1단계: ready 자세 + 그리퍼 열기 ---')
        self._go_named(arm, 'ready')
        gripper.open_gripper()
        self._spin_sleep(1.0)

        # 2단계: Pick 위치로 이동 (IK 시드: 현재 관절)
        self._wait_for_trigger('>>> [대기] 2단계 Pick 위치 이동 신호를 기다리는 중...')
        self.get_logger().info('--- 2단계: Pick 위치로 이동 ---')
        pick_pose = make_pose(*pick_pos, math.pi, 0.0, 0.0)
        if not self._go_smooth(arm, pick_pose, 'Pick'):
            self.get_logger().error('Pick 위치 이동 실패!')
            return
        self._spin_sleep(0.5)

        # 3단계: 그리퍼 닫기
        self._wait_for_trigger('>>> [대기] 3단계 그리퍼 닫기 신호를 기다리는 중...')
        self.get_logger().info('--- 3단계: 그리퍼 닫기 (가상 물체 잡기) ---')
        gripper.close_gripper()
        self._spin_sleep(1.0)

        # 4단계: 들어올리기 (현재 관절 시드 → 최소 이동)
        self._wait_for_trigger('>>> [대기] 4단계 들어올리기 신호를 기다리는 중...')
        self.get_logger().info('--- 4단계: 물체 들어올리기 ---')
        lift = make_pose(pick_pos[0], pick_pos[1], 0.40, math.pi, 0.0, 0.0)
        self._go_smooth(arm, lift, 'Lift')
        self._spin_sleep(0.5)

        # 5단계: Place 위치 상공으로 이동
        self._wait_for_trigger('>>> [대기] 5단계 Place 상공 이동 신호를 기다리는 중...')
        self.get_logger().info('--- 5단계: Place 위치로 이동 ---')
        place_above = make_pose(place_pos[0], place_pos[1], 0.40,
                                math.pi, 0.0, math.pi / 2)
        self._go_smooth(arm, place_above, 'Transport')
        self._spin_sleep(0.5)

        # 6단계: 하강
        self._wait_for_trigger('>>> [대기] 6단계 하강 신호를 기다리는 중...')
        self.get_logger().info('--- 6단계: 하강하여 놓기 ---')
        place_pose = make_pose(*place_pos, math.pi, 0.0, math.pi / 2)
        self._go_smooth(arm, place_pose, 'Place')
        self._spin_sleep(0.5)

        # 7단계: 그리퍼 열기
        self._wait_for_trigger('>>> [대기] 7단계 그리퍼 열기 신호를 기다리는 중...')
        self.get_logger().info('--- 7단계: 그리퍼 열기 (물체 놓기) ---')
        gripper.open_gripper()
        self._spin_sleep(1.0)

        # 8단계: 후퇴 및 복귀
        self._wait_for_trigger('>>> [대기] 8단계 후퇴 및 home 복귀 신호를 기다리는 중...')
        self.get_logger().info('--- 8단계: 후퇴 및 home 복귀 ---')
        retreat = make_pose(place_pos[0], place_pos[1], 0.40,
                            math.pi, 0.0, math.pi / 2)
        self._go_smooth(arm, retreat, 'Retreat')
        self._spin_sleep(0.5)

        self._go_named(arm, 'home')
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
