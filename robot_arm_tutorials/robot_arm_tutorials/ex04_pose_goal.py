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
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3
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
        arrow.scale = Vector3(x=0.08, y=0.012, z=0.012)
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
        sphere.scale = Vector3(x=0.03, y=0.03, z=0.03)
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
        text.pose.position.z = target['z'] + 0.06
        text.pose.orientation.w = 1.0
        text.scale.z = 0.025
        text.color = self.COLOR_TEXT
        text.text = f'{idx}. {target["name"]}'
        markers.append(text)

        return markers

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

        if not helper.wait_for_servers(timeout_sec=30.0):
            return
        if not helper.wait_for_joint_state(timeout_sec=10.0):
            return

        # 초기화: ready 포즈
        self.get_logger().info('--- 초기화: ready 포즈로 이동 ---')
        helper.go_to_named_target('ready')
        time.sleep(1.0)

        # 목표 자세 목록 (x, y, z, roll, pitch, yaw)
        targets = [
            {
                'name': '전방 수평 자세',
                'desc': '끝단을 로봇 전방으로 뻗은 자세 (아래 방향)',
                'x': 0.3, 'y': 0.0, 'z': 0.3,
                'roll': math.pi, 'pitch': 0.0, 'yaw': 0.0,
            },
            {
                'name': '좌측 자세',
                'desc': '끝단을 로봇 좌측으로 이동',
                'x': 0.0, 'y': 0.3, 'z': 0.3,
                'roll': math.pi, 'pitch': 0.0, 'yaw': math.pi / 2,
            },
            {
                'name': '높이 올린 자세',
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

            success = helper.go_to_pose_goal(pose)
            if success:
                self.get_logger().info(f'  → {target["name"]} 도달 성공!')
                self._publish_target_marker(idx, target, self.COLOR_SUCCESS)
            else:
                self.get_logger().warn(f'  → {target["name"]} 도달 실패 (IK 해 없음 가능)')
                self._publish_target_marker(idx, target, self.COLOR_FAIL)

            time.sleep(2.0)

        # 최종 복귀
        self.get_logger().info('--- home 포즈로 복귀 ---')
        helper.go_to_named_target('home')
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
