"""
예제 05: Cartesian 경로 계획
=============================
Waypoint 기반으로 직선 경로를 계획하고 실행하는 예제.
정사각형 궤적을 그려본다.

학습 내용:
- GetCartesianPath 서비스 사용
- 직선 보간(Linear Interpolation)
- max_step 파라미터
- fraction (달성률) 개념
- 특이점(Singularity) 문제 이해
- RViz2 Marker를 통한 경로 및 웨이포인트 시각화

실행 방법:
  터미널1: ros2 launch robot_arm_moveit_config demo.launch.xml
  터미널2: ros2 run robot_arm_tutorials ex05_cartesian_path --ros-args -p use_sim_time:=true
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState, MoveItErrorCodes
from robot_arm_tutorials.utils import MoveGroupHelper, make_pose, euler_to_quaternion


class CartesianPathDemo(Node):
    # Marker 색상 상수
    COLOR_SQUARE = ColorRGBA(r=0.2, g=0.8, b=1.0, a=0.9)     # 시안: 정사각형 경로
    COLOR_DESCENT = ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.9)     # 주황: 하강 경로
    COLOR_START = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)        # 노란색: 시작점
    COLOR_SUCCESS = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9)      # 초록: 성공
    COLOR_FAIL = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)         # 빨강: 실패
    COLOR_TEXT = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)          # 흰색: 텍스트

    def __init__(self):
        super().__init__('cartesian_path_demo')
        self.get_logger().info('=== 예제 05: Cartesian 경로 계획 ===')

        self._marker_pub = self.create_publisher(
            MarkerArray, '/cartesian_path_markers', 10
        )
        self._markers = MarkerArray()
        self._marker_id = 0

        # IK 서비스 클라이언트
        self._ik_client = self.create_client(GetPositionIK, 'compute_ik')

    def _next_id(self):
        self._marker_id += 1
        return self._marker_id

    def _spin_sleep(self, duration):
        """콜백을 처리하면서 대기 (조인트 상태 갱신 유지)"""
        end_time = time.time() + duration
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _go_to_pose_via_ik(self, helper, pose):
        """ready 포즈를 명시적 시드로 IK를 풀어 관절이 중앙 범위에 위치하도록 이동"""
        if not self._ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('compute_ik 서비스 없음, go_to_pose_goal 폴백')
            return helper.go_to_pose_goal(pose)

        from geometry_msgs.msg import PoseStamped

        # ready 포즈 관절값을 명시적 시드로 사용
        seed_state = RobotState()
        seed_js = JointState()
        seed_js.name = list(helper.ARM_JOINT_NAMES)
        seed_js.position = [0.0, -0.785, 0.785, 0.0, 0.785, 0.0]  # ready
        seed_state.joint_state = seed_js

        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = helper.PLANNING_GROUP
        ik_req.ik_request.robot_state = seed_state
        ps = PoseStamped()
        ps.header.frame_id = helper.REFERENCE_FRAME
        ps.pose = pose
        ik_req.ik_request.pose_stamped = ps
        ik_req.ik_request.avoid_collisions = False

        future = self._ik_client.call_async(ik_req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()

        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(f'IK 실패 (코드: {resp.error_code.val}), go_to_pose_goal 폴백')
            return helper.go_to_pose_goal(pose)

        joint_values = {}
        for i, name in enumerate(resp.solution.joint_state.name):
            if name in helper.ARM_JOINT_NAMES:
                joint_values[name] = resp.solution.joint_state.position[i]

        self.get_logger().info(
            f'  IK 해: {[f"{v:.3f}" for v in joint_values.values()]}'
        )
        return helper.go_to_joint_goal(joint_values)

    def _publish_path_markers(self, waypoints, start_pose, ns, color, label):
        """경로(LINE_STRIP)와 웨이포인트(SPHERE)를 Marker로 퍼블리시"""
        stamp = self.get_clock().now().to_msg()
        all_points = [start_pose.position] + [wp.position for wp in waypoints]

        # LINE_STRIP: 경로 선
        line = Marker()
        line.header.frame_id = 'base_link'
        line.header.stamp = stamp
        line.ns = ns + '_path'
        line.id = self._next_id()
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.004
        line.color = color
        line.points = [Point(x=p.x, y=p.y, z=p.z) for p in all_points]
        self._markers.markers.append(line)

        # SPHERE_LIST: 각 웨이포인트 구체
        spheres = Marker()
        spheres.header.frame_id = 'base_link'
        spheres.header.stamp = stamp
        spheres.ns = ns + '_waypoints'
        spheres.id = self._next_id()
        spheres.type = Marker.SPHERE_LIST
        spheres.action = Marker.ADD
        spheres.pose.orientation.w = 1.0
        spheres.scale = Vector3(x=0.015, y=0.015, z=0.015)
        spheres.color = color
        spheres.points = [Point(x=p.x, y=p.y, z=p.z) for p in all_points]
        self._markers.markers.append(spheres)

        # 시작점 강조 (큰 노란 구체)
        start_sphere = Marker()
        start_sphere.header.frame_id = 'base_link'
        start_sphere.header.stamp = stamp
        start_sphere.ns = ns + '_start'
        start_sphere.id = self._next_id()
        start_sphere.type = Marker.SPHERE
        start_sphere.action = Marker.ADD
        start_sphere.pose.position = Point(
            x=start_pose.position.x,
            y=start_pose.position.y,
            z=start_pose.position.z
        )
        start_sphere.pose.orientation.w = 1.0
        start_sphere.scale = Vector3(x=0.025, y=0.025, z=0.025)
        start_sphere.color = self.COLOR_START
        self._markers.markers.append(start_sphere)

        # 텍스트 라벨
        for i, pt in enumerate(all_points):
            text = Marker()
            text.header.frame_id = 'base_link'
            text.header.stamp = stamp
            text.ns = ns + '_text'
            text.id = self._next_id()
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(x=pt.x, y=pt.y, z=pt.z + 0.03)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.018
            text.color = self.COLOR_TEXT
            text.text = f'{label} 시작' if i == 0 else f'P{i}'
            self._markers.markers.append(text)

        self._marker_pub.publish(self._markers)

    def _publish_result_marker(self, ns, success, label):
        """실행 결과에 따라 경로 색상 변경 + 결과 텍스트 표시"""
        stamp = self.get_clock().now().to_msg()
        color = self.COLOR_SUCCESS if success else self.COLOR_FAIL

        center_point = None
        for m in self._markers.markers:
            if m.ns == ns + '_path':
                m.color = color
                m.header.stamp = stamp
                if m.points:
                    mid = m.points[len(m.points) // 2]
                    center_point = Point(x=mid.x, y=mid.y, z=mid.z + 0.06)

        if center_point:
            result_text = Marker()
            result_text.header.frame_id = 'base_link'
            result_text.header.stamp = stamp
            result_text.ns = ns + '_result'
            result_text.id = self._next_id()
            result_text.type = Marker.TEXT_VIEW_FACING
            result_text.action = Marker.ADD
            result_text.pose.position = center_point
            result_text.pose.orientation.w = 1.0
            result_text.scale.z = 0.025
            result_text.color = color
            result_text.text = f'{label}: {"성공" if success else "실패"}'
            self._markers.markers.append(result_text)

        self._marker_pub.publish(self._markers)

    def run(self):
        helper = MoveGroupHelper(self)
        helper.max_velocity_scaling = 0.2
        helper.planning_time = 10.0

        if not helper.wait_for_servers(timeout_sec=30.0):
            return
        if not helper.wait_for_joint_state(timeout_sec=10.0):
            return

        # ready 포즈로 이동
        self.get_logger().info('--- 초기화: ready 포즈로 이동 ---')
        helper.go_to_named_target('ready')
        self._spin_sleep(1.0)

        # 그리퍼 전방 방향 (pitch=π/2) — Cartesian 경로에 적합한 자세
        # (ex13에서 검증된 방향: 관절이 중앙 범위에 위치하여 Y/Z 이동 자유도 확보)
        orientation = euler_to_quaternion(0.0, math.pi / 2, 0.0)

        # 정사각형 시작점으로 이동 (IK 시드: ready 포즈 → joint 중앙 범위 유지)
        self.get_logger().info('--- 정사각형 시작점으로 이동 ---')
        sx, sy, sz = 0.35, -0.04, 0.35
        start_pose = make_pose(sx, sy, sz, 0.0, math.pi / 2, 0.0)

        success = self._go_to_pose_via_ik(helper, start_pose)
        if not success:
            self.get_logger().error('시작점 이동 실패!')
            helper.go_to_named_target('home')
            return
        self._spin_sleep(1.0)

        # 정사각형 경로 - Waypoint 생성
        self.get_logger().info('--- 정사각형 Cartesian 경로 계획 ---')
        side_length = 0.08  # 한 변 8cm

        # Y-Z 평면 정사각형
        waypoints = []

        # 오른쪽으로 (+Y)
        wp1 = Pose()
        wp1.position = Point(x=sx, y=sy + side_length, z=sz)
        wp1.orientation = orientation
        waypoints.append(wp1)

        # 위로 (+Z)
        wp2 = Pose()
        wp2.position = Point(x=sx, y=sy + side_length, z=sz + side_length)
        wp2.orientation = orientation
        waypoints.append(wp2)

        # 왼쪽으로 (-Y)
        wp3 = Pose()
        wp3.position = Point(x=sx, y=sy, z=sz + side_length)
        wp3.orientation = orientation
        waypoints.append(wp3)

        # 아래로 (-Z) - 시작점 복귀
        wp4 = Pose()
        wp4.position = Point(x=sx, y=sy, z=sz)
        wp4.orientation = orientation
        waypoints.append(wp4)

        self.get_logger().info(f'  경유점 수: {len(waypoints)}개')
        self.get_logger().info(f'  한 변 길이: {side_length*100:.0f}cm')

        # 현재 조인트 상태 확인
        cur = helper.get_current_joint_values()
        self.get_logger().info(f'  현재 조인트: {[f"{v:.3f}" for v in cur.values()]}')

        # 정사각형 경로를 Marker로 표시
        self._publish_path_markers(
            waypoints, start_pose, 'square', self.COLOR_SQUARE, '정사각형'
        )

        # Cartesian 경로 계획
        trajectory, fraction = helper.compute_cartesian_path(
            waypoints, max_step=0.02
        )

        self.get_logger().info(f'  경로 달성률: {fraction*100:.1f}%')

        if trajectory is not None and fraction > 0.8:
            self.get_logger().info('--- 계획된 경로 실행 ---')
            success = helper.execute_trajectory(trajectory)
            if success:
                self.get_logger().info('정사각형 경로 실행 완료!')
            else:
                self.get_logger().error('경로 실행 실패!')
            self._publish_result_marker('square', success, '정사각형')
        else:
            self.get_logger().warn(
                f'경로 달성률이 낮습니다 ({fraction*100:.1f}%). '
                '경로를 실행하지 않습니다.'
            )
            self._publish_result_marker('square', False, '정사각형')

        self._spin_sleep(1.0)

        # 직선 하강 시연
        self.get_logger().info('--- 직선 하강 시연 ---')
        self.get_logger().info('  끝단을 직선으로 8cm 아래로 이동합니다.')

        descent_start = Pose()
        descent_start.position = Point(x=sx, y=sy, z=sz)
        descent_start.orientation = orientation

        descent_waypoints = []
        for i in range(1, 5):
            wp = Pose()
            wp.position = Point(x=sx, y=sy, z=sz - i * 0.02)
            wp.orientation = orientation
            descent_waypoints.append(wp)

        self._publish_path_markers(
            descent_waypoints, descent_start, 'descent', self.COLOR_DESCENT, '하강'
        )

        traj2, frac2 = helper.compute_cartesian_path(
            descent_waypoints, max_step=0.02
        )
        self.get_logger().info(f'  직선 하강 달성률: {frac2*100:.1f}%')

        if traj2 is not None and frac2 > 0.8:
            success2 = helper.execute_trajectory(traj2)
            self._publish_result_marker('descent', success2, '직선 하강')
        else:
            self._publish_result_marker('descent', False, '직선 하강')

        self._spin_sleep(1.0)

        # 최종 복귀
        self.get_logger().info('--- home 포즈로 복귀 ---')
        helper.go_to_named_target('home')
        self.get_logger().info('=== 예제 05 완료! ===')


def main(args=None):
    rclpy.init(args=args)
    node = CartesianPathDemo()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
