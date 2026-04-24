"""
예제 04: 끝단 자세 지령 (Pose Goal)
====================================
Cartesian 공간에서 끝단(gripper_base)의 위치와 방향을 지정하여
이동하는 예제. Euler → Quaternion 변환을 학습한다.

학습 내용:
- PositionConstraint, OrientationConstraint
- IK (역기구학) 솔버의 역할
- 작업 공간(Workspace) 이해
- Euler ↔ Quaternion 변환
- RViz2 Marker를 통한 목표 자세 시각화

실행 방법:
  터미널1: ros2 launch robot_arm_moveit_config demo.launch.xml
  터미널2: ros2 run robot_arm_tutorials ex04_pose_goal --ros-args -p use_sim_time:=true
"""

import math
import time
import rclpy
import tf2_ros
import tf_transformations
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Empty
from geometry_msgs.msg import Vector3, Point
from moveit_msgs.msg import RobotState, MoveItErrorCodes
from moveit_msgs.srv import GetPositionFK
from robot_arm_tutorials.utils import MoveGroupHelper, make_pose, euler_to_quaternion


class PoseGoalDemo(Node):
    # Marker 색상 상수
    COLOR_PENDING = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)   # 노란색: 대기
    COLOR_ACTIVE = ColorRGBA(r=0.2, g=0.5, b=1.0, a=0.9)    # 파란색: 진행 중
    COLOR_SUCCESS = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)    # 초록색: 성공
    COLOR_FAIL = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)       # 빨간색: 실패
    COLOR_TEXT = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)        # 흰색: 텍스트

    def __init__(self):
        super().__init__('pose_goal_demo')
        self.get_logger().info('=== 예제 04: 끝단 자세 지령 ===')

        # Marker publisher for RViz2 시각화
        self._marker_pub = self.create_publisher(
            MarkerArray, '/pose_goal_markers', 10
        )
        self._markers = MarkerArray()

        # TF 조회용 (ready 포즈 마커를 실제 gripper 위치에 찍기 위해)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # 사용자 트리거: /next_step 토픽으로 각 단계 진행
        self._trigger_received = False
        self._trigger_sub = self.create_subscription(
            Empty, '/next_step', self._trigger_cb, 10
        )

        # FK 서비스: 플래닝된 궤적의 끝단 경로 계산용
        self._fk_client = self.create_client(GetPositionFK, 'compute_fk')

    def _trigger_cb(self, _msg):
        self._trigger_received = True

    def _wait_for_trigger(self, prompt):
        """/next_step 토픽 메시지가 올 때까지 블로킹 대기"""
        self.get_logger().info(prompt)
        self._trigger_received = False
        while rclpy.ok() and not self._trigger_received:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _create_markers_for_target(self, idx, target, color):
        """목표 자세에 대한 Marker 3개(화살표, 구, 텍스트)를 생성하여 반환"""
        stamp = self.get_clock().now().to_msg()
        markers = []

        # 1) 화살표 마커: 끝단 방향 시각화
        arrow = Marker()
        arrow.header.frame_id = 'base_link'
        arrow.header.stamp = stamp
        arrow.ns = 'pose_goal_arrow'
        arrow.id = idx
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = make_pose(
            target['x'], target['y'], target['z'],
            target['roll'], target['pitch'], target['yaw']
        )
        arrow.scale = Vector3(x=0.12, y=0.02, z=0.02)
        arrow.color = color
        markers.append(arrow)

        # 2) 구 마커: 목표 위치 시각화
        sphere = Marker()
        sphere.header.frame_id = 'base_link'
        sphere.header.stamp = stamp
        sphere.ns = 'pose_goal_sphere'
        sphere.id = idx
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = target['x']
        sphere.pose.position.y = target['y']
        sphere.pose.position.z = target['z']
        sphere.pose.orientation.w = 1.0
        sphere.scale = Vector3(x=0.045, y=0.045, z=0.045)
        sphere_color = ColorRGBA(r=color.r, g=color.g, b=color.b, a=0.6)
        sphere.color = sphere_color
        markers.append(sphere)

        # 3) 텍스트 마커: 목표 이름 표시
        text = Marker()
        text.header.frame_id = 'base_link'
        text.header.stamp = stamp
        text.ns = 'pose_goal_text'
        text.id = idx
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = target['x']
        text.pose.position.y = target['y']
        text.pose.position.z = target['z'] + 0.12
        text.pose.orientation.w = 1.0
        text.scale.z = 0.05
        text.color = self.COLOR_TEXT
        text.text = target["label"]
        markers.append(text)

        return markers

    def _publish_current_pose_marker(self, idx, label, color, timeout_sec=2.0):
        """현재 gripper_base의 TF를 조회하여 해당 위치에 마커 발행

        joint 공간 목표(ready, home 등)는 cartesian 위치를 미리 알 수 없으므로,
        이동 완료 후 실제 TF를 읽어 마커를 표시한다.
        """
        start = time.time()
        while time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._tf_buffer.can_transform(
                'base_link', 'gripper_base', rclpy.time.Time()
            ):
                break

        try:
            trans = self._tf_buffer.lookup_transform(
                'base_link', 'gripper_base', rclpy.time.Time()
            )
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'TF 조회 실패: {e}')
            return

        t = trans.transform.translation
        q = trans.transform.rotation
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )

        target = {
            'name': label,
            'label': label,
            'x': t.x, 'y': t.y, 'z': t.z,
            'roll': roll, 'pitch': pitch, 'yaw': yaw,
        }
        self._publish_target_marker(idx, target, color)

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
            color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)  # 주황색

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

        arr = MarkerArray()
        arr.markers.append(line)
        self._marker_pub.publish(arr)

    def _plan_and_execute_with_viz(self, helper, plan_result, path_color=None):
        """계획 결과(trajectory)를 받아 끝단 경로를 RViz에 표시한 뒤 실행"""
        success, trajectory = plan_result
        if not success or trajectory is None:
            return False

        ee_points = self._compute_trajectory_ee_path(trajectory)
        if ee_points:
            self.get_logger().info(
                f'  계획된 끝단 경로: {len(ee_points)}개 점 — RViz에 표시'
            )
            self._publish_ee_path_marker(ee_points, color=path_color)

        return helper.execute_trajectory(trajectory)

    def _publish_target_marker(self, idx, target, color):
        """목표 자세의 Marker를 업데이트하여 퍼블리시"""
        new_markers = self._create_markers_for_target(idx, target, color)
        new_ns_ids = {(m.ns, m.id) for m in new_markers}

        # 기존 마커 중 같은 ns/id 제거 후 새 마커 추가
        self._markers.markers = [
            m for m in self._markers.markers
            if (m.ns, m.id) not in new_ns_ids
        ]
        self._markers.markers.extend(new_markers)
        self._marker_pub.publish(self._markers)

    def run(self):
        helper = MoveGroupHelper(self)
        helper.max_velocity_scaling = 0.3
        helper.planning_time = 10.0

        # RViz2 마커 시각화 안내
        self.get_logger().info(
            '[RViz2 안내] 목표 자세 마커를 보려면 RViz2에 MarkerArray Display를 '
            "추가하고 Topic을 '/pose_goal_markers' 로 설정하세요."
        )
        # 사용자 트리거 안내
        self.get_logger().info(
            "[단계 진행] 각 단계는 '/next_step' 토픽 신호로 진행됩니다. "
            "다른 터미널에서 다음 명령을 실행하세요:\n"
            "  ros2 topic pub --once /next_step std_msgs/msg/Empty '{}'"
        )

        if not helper.wait_for_servers(timeout_sec=30.0):
            return
        if not helper.wait_for_joint_state(timeout_sec=10.0):
            return

        # 초기화: ready 포즈
        self._wait_for_trigger('>>> [대기] ready 포즈로 이동 신호를 기다리는 중...')
        self.get_logger().info('--- 초기화: ready 포즈로 이동 ---')
        self._plan_and_execute_with_viz(
            helper,
            helper.plan_to_joint_goal(helper.NAMED_TARGETS['ready']),
        )
        time.sleep(1.0)
        # ready 포즈도 마커로 표시 (실제 gripper_base TF 기반)
        self._publish_current_pose_marker(0, 'Ready', self.COLOR_SUCCESS)

        # 목표 자세 목록 (x, y, z, roll, pitch, yaw)
        targets = [
            {
                'name': '전방 수평 자세',
                'label': 'Forward',
                'desc': '끝단을 로봇 전방으로 뻗은 자세 (아래 방향)',
                'x': 0.3, 'y': 0.0, 'z': 0.3,
                'roll': math.pi, 'pitch': 0.0, 'yaw': 0.0,
            },
            {
                'name': '좌측 자세',
                'label': 'Left',
                'desc': '끝단을 로봇 좌측으로 이동',
                'x': 0.0, 'y': 0.3, 'z': 0.3,
                'roll': math.pi, 'pitch': 0.0, 'yaw': math.pi / 2,
            },
            {
                'name': '높이 올린 자세',
                'label': 'High',
                'desc': '끝단을 위로 올린 자세',
                'x': 0.20, 'y': 0.0, 'z': 0.40,
                'roll': math.pi, 'pitch': 0.0, 'yaw': 0.0,
            },
        ]

        # 모든 목표 지점을 먼저 노란색(대기)으로 표시
        for idx, target in enumerate(targets, 1):
            self._publish_target_marker(idx, target, self.COLOR_PENDING)
        time.sleep(0.5)

        for idx, target in enumerate(targets, 1):
            self._wait_for_trigger(
                f'>>> [대기] {idx}단계 ({target["name"]}) 이동 신호를 기다리는 중...'
            )
            self.get_logger().info(
                f'--- {idx}단계: {target["name"]} ---'
            )
            self.get_logger().info(f'  설명: {target["desc"]}')
            self.get_logger().info(
                f'  위치: ({target["x"]:.2f}, {target["y"]:.2f}, {target["z"]:.2f}) m'
            )
            self.get_logger().info(
                f'  방향: (R={math.degrees(target["roll"]):.0f}°, '
                f'P={math.degrees(target["pitch"]):.0f}°, '
                f'Y={math.degrees(target["yaw"]):.0f}°)'
            )

            # 현재 목표를 파란색(진행 중)으로 표시
            self._publish_target_marker(idx, target, self.COLOR_ACTIVE)

            pose = make_pose(
                target['x'], target['y'], target['z'],
                target['roll'], target['pitch'], target['yaw']
            )

            success = self._plan_and_execute_with_viz(
                helper,
                helper.plan_to_pose_goal(pose),
            )
            if success:
                self.get_logger().info(f'  → {target["name"]} 도달 성공!')
                self._publish_target_marker(idx, target, self.COLOR_SUCCESS)
            else:
                self.get_logger().warn(f'  → {target["name"]} 도달 실패 (IK 해 없음 가능)')
                self._publish_target_marker(idx, target, self.COLOR_FAIL)

        # 최종 복귀
        self._wait_for_trigger('>>> [대기] home 포즈로 복귀 신호를 기다리는 중...')
        self.get_logger().info('--- home 포즈로 복귀 ---')
        self._plan_and_execute_with_viz(
            helper,
            helper.plan_to_joint_goal(helper.NAMED_TARGETS['home']),
        )
        self.get_logger().info('=== 예제 04 완료! ===')


def main(args=None):
    rclpy.init(args=args)
    node = PoseGoalDemo()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
