"""
예제 08: 경로 제약조건 (Path Constraints)
==========================================
끝단의 방향(수평)을 유지하면서 3개 목표 지점으로 이동하는 예제.

학습 내용:
- OrientationConstraint: 끝단 방향 유지 (물컵 운반 시나리오)
- path_constraints vs goal_constraints 차이
- 제약 하 플래닝 시간 증가 이해
- 허용 편차(tolerance) 조정의 영향
- RViz Marker로 제약 시각화

실행 방법:
  터미널1: ros2 launch robot_arm_moveit_config demo.launch.xml
  터미널2: ros2 run robot_arm_tutorials ex08_constraints --ros-args -p use_sim_time:=true
"""

import math
import time
import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from moveit_msgs.msg import Constraints, OrientationConstraint
from robot_arm_tutorials.utils import MoveGroupHelper, make_pose, euler_to_quaternion


class ConstraintsDemo(Node):
    def __init__(self):
        super().__init__('constraints_demo')
        self.get_logger().info('=== 예제 08: 경로 제약조건 ===')

        latched_qos = QoSProfile(depth=10)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._marker_pub = self.create_publisher(
            MarkerArray, '/constraint_markers', latched_qos)
        self._markers = MarkerArray()
        self._marker_timer = self.create_timer(2.0, self._republish)
        self._mid = 0

    def _nid(self):
        self._mid += 1
        return self._mid

    def _republish(self):
        if self._markers.markers:
            self._marker_pub.publish(self._markers)

    def _spin_sleep(self, duration):
        end_time = time.time() + duration
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _publish_markers(self, start_pos, targets, tolerance_rad):
        """시작점, 목표, 제약 시각화 마커 퍼블리시"""
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

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
            ma.markers.append(m)
            return m

        # 시작점 (노란 구체 + 라벨)
        sx, sy, sz = start_pos
        add_marker('pt', 0, Marker.SPHERE, sx, sy, sz,
                   0.025, 0.025, 0.025,
                   ColorRGBA(r=1.0, g=0.8, b=0.0, a=1.0))
        m = add_marker('txt', 0, Marker.TEXT_VIEW_FACING, sx, sy, sz + 0.05,
                       0.01, 0.01, 0.025,
                       ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))
        m.text = 'Start'

        # 각 목표: 구체 + 라벨 + 연결선 + 제약 링(투명 원뿔)
        for i, tgt in enumerate(targets):
            tx = tgt['pos'][0]
            ty = tgt['pos'][1]
            tz = tgt['pos'][2]
            c = colors[i]
            label = tgt['label']

            # 목표 구체
            add_marker('pt', i + 1, Marker.SPHERE, tx, ty, tz,
                       0.025, 0.025, 0.025, c)

            # 라벨
            m = add_marker('txt', i + 1, Marker.TEXT_VIEW_FACING,
                           tx, ty, tz + 0.05,
                           0.01, 0.01, 0.025,
                           ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))
            m.text = label

            # 시작→목표 연결선
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
            ma.markers.append(line)

            # 제약 시각화: 투명 링 (CYLINDER, 매우 얇음)
            # 반지름 = tolerance_rad * 0.15 (각도를 시각적 크기로 스케일링)
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
            ring.scale.x = ring_r * 2  # 지름
            ring.scale.y = ring_r * 2
            ring.scale.z = 0.003       # 두께 (매우 얇음)
            ring.color = ColorRGBA(r=c.r, g=c.g, b=c.b, a=0.25)
            ma.markers.append(ring)

            # 방향 화살표 (아래 방향)
            arrow = Marker()
            arrow.header.frame_id = 'base_link'
            arrow.header.stamp = stamp
            arrow.ns = 'dir_arrow'
            arrow.id = i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            # 시작점 → 아래로 5cm
            arrow.points = [
                Point(x=tx, y=ty, z=tz),
                Point(x=tx, y=ty, z=tz - 0.05),
            ]
            arrow.scale = Vector3(x=0.006, y=0.012, z=0.01)
            arrow.color = ColorRGBA(r=c.r, g=c.g, b=c.b, a=0.7)
            ma.markers.append(arrow)

        self._markers = ma
        self._marker_pub.publish(ma)

    def run(self):
        arm = MoveGroupHelper(self)
        arm.max_velocity_scaling = 0.2
        arm.max_acceleration_scaling = 0.2
        arm.planning_time = 30.0
        arm.num_planning_attempts = 20

        if not arm.wait_for_servers(timeout_sec=30.0):
            return
        if not arm.wait_for_joint_state(timeout_sec=10.0):
            return

        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('  그리퍼 수평 유지 + 3지점 이동')
        self.get_logger().info('=' * 50)

        # 시작 위치
        start_pos = (0.22, 0.00, 0.35)
        start = make_pose(*start_pos, math.pi, 0.0, 0.0)

        self.get_logger().info('--- 시작 위치로 이동 (그리퍼 아래 방향) ---')
        success = arm.go_to_pose_goal(start)
        if not success:
            self.get_logger().error('시작 위치 이동 실패!')
            return
        self._spin_sleep(1.0)
        self.get_logger().info('시작 자세 완료')

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

        self.get_logger().info(f'방향 제약: 아래 방향 유지, X/Y 허용 ±{math.degrees(tolerance):.0f}°, Z 자유')

        # 목표 정의
        targets = [
            {'label': 'A', 'pos': (0.22, -0.04, 0.30)},
            {'label': 'B', 'pos': (0.22, 0.04, 0.30)},
            {'label': 'C', 'pos': (0.20, 0.00, 0.38)},
        ]

        # Marker 표시
        self._publish_markers(start_pos, targets, tolerance)
        self._spin_sleep(1.0)

        # 3개 목표 순회
        for i, tgt in enumerate(targets):
            self.get_logger().info('')
            self.get_logger().info(
                f'--- [{i+1}/3] 제약 이동: {tgt["label"]} ({tgt["pos"]}) ---')

            pose = make_pose(*tgt['pos'], math.pi, 0.0, 0.0)

            # 제약 플래닝은 OMPL 랜덤 샘플링 — 재시도로 성공률 향상
            success = False
            for attempt in range(3):
                success = arm.go_to_pose_goal_with_constraints(pose, constraints)
                if success:
                    break
                self.get_logger().warn(
                    f'  {tgt["label"]} 시도 {attempt+1}/3 실패, 재시도...')

            if success:
                self.get_logger().info(f'  {tgt["label"]} 도달 (수평 유지)')
            else:
                self.get_logger().warn(f'  {tgt["label"]} 3회 시도 모두 실패, 건너뜀')

            self._spin_sleep(1.5)

            # 복귀
            self.get_logger().info('  시작 위치로 복귀...')
            success = arm.go_to_pose_goal_with_constraints(start, constraints)
            if success:
                self.get_logger().info('  복귀 완료')
            else:
                self.get_logger().warn('  제약 복귀 실패, 제약 없이 복귀')
                arm.go_to_pose_goal(start)
            self._spin_sleep(1.0)

        # home 복귀
        self.get_logger().info('')
        self.get_logger().info('--- home 복귀 ---')
        arm.go_to_named_target('home')
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
