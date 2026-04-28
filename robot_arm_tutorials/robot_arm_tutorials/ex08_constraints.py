"""
예제 08: 경로 제약조건 (Path Constraints)
==========================================
끝단의 방향(수평)을 유지하면서 3개 목표 지점으로 이동하는 예제.

각 단계는 /next_step 토픽 신호로 진행되며,
계획된 끝단 경로를 미리 LINE_STRIP 마커로 RViz에 표시한다.

학습 내용:
- OrientationConstraint: 끝단 방향 유지 (물컵 운반 시나리오)
- path_constraints vs goal_constraints 차이
- 제약 하 플래닝 시간 증가 이해
- 허용 편차(tolerance) 조정의 영향
- RViz Marker로 제약 시각화
- 단계별 사용자 트리거 (/next_step)
- FK로 계획된 궤적의 끝단 경로 시각화

실행 방법:
  터미널1: ros2 launch robot_arm_moveit_config demo.launch.xml
  터미널2: ros2 run robot_arm_tutorials ex08_constraints --ros-args -p use_sim_time:=true
  터미널3: ros2 topic pub --once /next_step std_msgs/msg/Empty '{}'
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Empty
from moveit_msgs.msg import Constraints, OrientationConstraint, RobotState, MoveItErrorCodes
from moveit_msgs.srv import GetPositionFK
from robot_arm_tutorials.utils import MoveGroupHelper, make_pose, euler_to_quaternion


class ConstraintsDemo(Node):
    PATH_COLOR = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)  # 주황색: 계획된 끝단 경로

    def __init__(self):
        super().__init__('constraints_demo')
        self.get_logger().info('=== 예제 08: 경로 제약조건 ===')

        latched_qos = QoSProfile(depth=10)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._marker_pub = self.create_publisher(
            MarkerArray, '/constraint_markers', latched_qos)
        self._markers = MarkerArray()
        self._marker_timer = self.create_timer(2.0, self._republish)

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

    def _republish(self):
        if self._markers.markers:
            self._marker_pub.publish(self._markers)

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
        line.scale.x = 0.006
        line.color = color
        line.points = [Point(x=p[0], y=p[1], z=p[2]) for p in ee_points]

        self._markers.markers = [
            m for m in self._markers.markers
            if (m.ns, m.id) != ('ee_path_line', 0)
        ]
        self._markers.markers.append(line)
        self._marker_pub.publish(self._markers)

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

    def _publish_targets(self, start_pos, targets, tolerance_rad):
        """시작점, 목표, 제약 시각화 마커 퍼블리시 (경로 마커는 보존)"""
        stamp = self.get_clock().now().to_msg()
        new_markers = []

        colors = [
            ColorRGBA(r=0.9, g=0.2, b=0.2, a=1.0),  # A 빨강
            ColorRGBA(r=0.2, g=0.8, b=0.2, a=1.0),  # B 초록
            ColorRGBA(r=0.2, g=0.3, b=0.9, a=1.0),  # C 파랑
        ]

        def add_marker(ns, mid, mtype, px, py, pz, sx, sy, sz, color):
            m = Marker()
            m.header.frame_id = 'base_link'
            m.header.stamp = stamp
            m.ns = ns
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.pose.position.x = px
            m.pose.position.y = py
            m.pose.position.z = pz
            m.pose.orientation.w = 1.0
            m.scale = Vector3(x=sx, y=sy, z=sz)
            m.color = color
            new_markers.append(m)
            return m

        sx, sy, sz = start_pos
        add_marker('pt', 0, Marker.SPHERE, sx, sy, sz,
                   0.025, 0.025, 0.025,
                   ColorRGBA(r=1.0, g=0.8, b=0.0, a=1.0))
        m = add_marker('txt', 0, Marker.TEXT_VIEW_FACING, sx, sy, sz + 0.05,
                       0.01, 0.01, 0.025,
                       ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))
        m.text = 'Start'

        for i, tgt in enumerate(targets):
            tx, ty, tz = tgt['pos']
            c = colors[i]
            label = tgt['label']

            add_marker('pt', i + 1, Marker.SPHERE, tx, ty, tz,
                       0.025, 0.025, 0.025, c)

            m = add_marker('txt', i + 1, Marker.TEXT_VIEW_FACING,
                           tx, ty, tz + 0.05,
                           0.01, 0.01, 0.025,
                           ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))
            m.text = label

            line = Marker()
            line.header.frame_id = 'base_link'
            line.header.stamp = stamp
            line.ns = 'lines'
            line.id = i
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.003
            line.color = ColorRGBA(r=c.r, g=c.g, b=c.b, a=0.4)
            line.points = [
                Point(x=sx, y=sy, z=sz),
                Point(x=tx, y=ty, z=tz),
            ]
            new_markers.append(line)

            ring_r = tolerance_rad * 0.15
            ring = Marker()
            ring.header.frame_id = 'base_link'
            ring.header.stamp = stamp
            ring.ns = 'constraint_ring'
            ring.id = i
            ring.type = Marker.CYLINDER
            ring.action = Marker.ADD
            ring.pose.position.x = tx
            ring.pose.position.y = ty
            ring.pose.position.z = tz
            ring.pose.orientation.w = 1.0
            ring.scale.x = ring_r * 2
            ring.scale.y = ring_r * 2
            ring.scale.z = 0.003
            ring.color = ColorRGBA(r=c.r, g=c.g, b=c.b, a=0.25)
            new_markers.append(ring)

            arrow = Marker()
            arrow.header.frame_id = 'base_link'
            arrow.header.stamp = stamp
            arrow.ns = 'dir_arrow'
            arrow.id = i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [
                Point(x=tx, y=ty, z=tz),
                Point(x=tx, y=ty, z=tz - 0.05),
            ]
            arrow.scale = Vector3(x=0.006, y=0.012, z=0.01)
            arrow.color = ColorRGBA(r=c.r, g=c.g, b=c.b, a=0.7)
            new_markers.append(arrow)

        new_ns_ids = {(m.ns, m.id) for m in new_markers}
        self._markers.markers = [
            m for m in self._markers.markers
            if (m.ns, m.id) not in new_ns_ids
        ]
        self._markers.markers.extend(new_markers)
        self._marker_pub.publish(self._markers)

    def _plan_constrained_with_retry(self, arm, pose, constraints, label, attempts=3):
        """제약 플래닝은 OMPL 랜덤 샘플링 — 재시도로 성공률 향상"""
        for attempt in range(attempts):
            success, trajectory = arm.plan_to_pose_goal_with_constraints(
                pose, constraints
            )
            if success and trajectory is not None:
                return True, trajectory
            self.get_logger().warn(
                f'  {label} 시도 {attempt+1}/{attempts} 계획 실패, 재시도...'
            )
        return False, None

    def run(self):
        arm = MoveGroupHelper(self)
        arm.max_velocity_scaling = 0.2
        arm.max_acceleration_scaling = 0.2
        arm.planning_time = 30.0
        arm.num_planning_attempts = 20

        self.get_logger().info(
            '[RViz2 안내] 마커를 보려면 MarkerArray Display를 추가하고 '
            "Topic을 '/constraint_markers' 로 설정하세요."
        )
        self.get_logger().info(
            "[단계 진행] 각 단계는 '/next_step' 토픽 신호로 진행됩니다. "
            "다른 터미널에서 다음 명령을 실행하세요:\n"
            "  ros2 topic pub --once /next_step std_msgs/msg/Empty '{}'"
        )

        if not arm.wait_for_servers(timeout_sec=30.0):
            return
        if not arm.wait_for_joint_state(timeout_sec=10.0):
            return

        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('  그리퍼 수평 유지 + 3지점 이동')
        self.get_logger().info('=' * 50)

        start_pos = (0.22, 0.00, 0.35)
        start = make_pose(*start_pos, math.pi, 0.0, 0.0)

        # 방향 제약 설정
        tolerance = 0.3  # ±17°
        oc = OrientationConstraint()
        oc.header.frame_id = 'base_link'
        oc.link_name = 'gripper_base'
        oc.orientation = euler_to_quaternion(math.pi, 0.0, 0.0)
        oc.absolute_x_axis_tolerance = tolerance
        oc.absolute_y_axis_tolerance = tolerance
        oc.absolute_z_axis_tolerance = 3.14  # Z축 자유
        oc.weight = 1.0

        constraints = Constraints()
        constraints.orientation_constraints.append(oc)

        self.get_logger().info(
            f'방향 제약: 아래 방향 유지, X/Y 허용 ±{math.degrees(tolerance):.0f}°, Z 자유'
        )

        targets = [
            {'label': 'A', 'pos': (0.22, -0.04, 0.30)},
            {'label': 'B', 'pos': (0.22, 0.04, 0.30)},
            {'label': 'C', 'pos': (0.20, 0.00, 0.38)},
        ]

        # Marker 표시
        self._publish_targets(start_pos, targets, tolerance)
        self._spin_sleep(1.0)

        # 시작 위치로 이동 (제약 없이 — 목표 자세만)
        self._wait_for_trigger('>>> [대기] 시작 위치 이동 신호를 기다리는 중...')
        self.get_logger().info('--- 시작 위치로 이동 (그리퍼 아래 방향) ---')
        success = self._plan_and_execute_with_viz(arm, arm.plan_to_pose_goal(start))
        if not success:
            self.get_logger().error('시작 위치 이동 실패!')
            return
        self._spin_sleep(1.0)
        self.get_logger().info('시작 자세 완료')

        # 3개 목표 순회
        for i, tgt in enumerate(targets):
            label = tgt['label']
            pose = make_pose(*tgt['pos'], math.pi, 0.0, 0.0)

            # 목표 이동 (제약 하 플래닝)
            self._wait_for_trigger(
                f'>>> [대기] [{i+1}/3] {label} 제약 이동 신호를 기다리는 중...'
            )
            self.get_logger().info('')
            self.get_logger().info(
                f'--- [{i+1}/3] 제약 이동: {label} ({tgt["pos"]}) ---'
            )
            success, trajectory = self._plan_constrained_with_retry(
                arm, pose, constraints, label
            )
            if success:
                self._publish_ee_path_marker(
                    self._compute_trajectory_ee_path(trajectory)
                )
                if arm.execute_trajectory(trajectory):
                    self.get_logger().info(f'  {label} 도달 (수평 유지)')
                else:
                    self.get_logger().warn(f'  {label} 실행 실패')
            else:
                self.get_logger().warn(f'  {label} 3회 시도 모두 실패, 건너뜀')

            self._spin_sleep(1.0)

            # 시작 위치로 복귀 (제약 하)
            self._wait_for_trigger(
                f'>>> [대기] {label} → 시작 위치 복귀 신호를 기다리는 중...'
            )
            self.get_logger().info('  시작 위치로 복귀...')
            success, trajectory = self._plan_constrained_with_retry(
                arm, start, constraints, 'Return'
            )
            if success:
                self._publish_ee_path_marker(
                    self._compute_trajectory_ee_path(trajectory)
                )
                if arm.execute_trajectory(trajectory):
                    self.get_logger().info('  복귀 완료')
                else:
                    self.get_logger().warn('  복귀 실행 실패')
            else:
                self.get_logger().warn('  제약 복귀 계획 실패, 제약 없이 복귀')
                self._plan_and_execute_with_viz(
                    arm, arm.plan_to_pose_goal(start)
                )
            self._spin_sleep(1.0)

        # home 복귀
        self._wait_for_trigger('>>> [대기] home 포즈로 복귀 신호를 기다리는 중...')
        self.get_logger().info('')
        self.get_logger().info('--- home 복귀 ---')
        self._plan_and_execute_with_viz(
            arm, arm.plan_to_joint_goal(arm.NAMED_TARGETS['home'])
        )
        self._marker_timer.cancel()
        self.get_logger().info('=== 예제 08 완료! ===')


def main(args=None):
    rclpy.init(args=args)
    node = ConstraintsDemo()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
