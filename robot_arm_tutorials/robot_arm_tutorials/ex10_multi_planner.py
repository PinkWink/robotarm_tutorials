"""
예제 10: 플래너 비교 (Multi-Planner Comparison)
=================================================
동일한 목표에 대해 여러 OMPL 플래너를 적용하여
계획 시간, 성공 여부를 비교하고 경로를 RViz에 시각화하는 예제.

학습 내용:
- OMPL 플래너 종류와 특성
- planner_id 파라미터
- FK(순기구학)를 이용한 궤적 → 끝단 경로 변환
- RViz Marker로 플래너별 경로 시각화

실행 방법:
  터미널1: ros2 launch robot_arm_moveit_config demo.launch.xml
  터미널2: ros2 run robot_arm_tutorials ex10_multi_planner --ros-args -p use_sim_time:=true
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from robot_arm_tutorials.utils import MoveGroupHelper, make_pose


# 플래너별 색상과 선 두께
PLANNER_STYLES = {
    'RRTConnect': {'color': ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9), 'width': 0.004},
    'RRT':        {'color': ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.9), 'width': 0.004},
    'PRM':        {'color': ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.9), 'width': 0.004},
    'EST':        {'color': ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.9), 'width': 0.004},
    'KPIECE':     {'color': ColorRGBA(r=0.7, g=0.2, b=0.9, a=0.9), 'width': 0.004},
}


class MultiPlannerDemo(Node):
    def __init__(self):
        super().__init__('multi_planner_demo')
        self.get_logger().info('=== 예제 10: 플래너 비교 ===')

        latched_qos = QoSProfile(depth=10)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._marker_pub = self.create_publisher(
            MarkerArray, '/planner_paths', latched_qos)
        self._markers = MarkerArray()
        self._marker_timer = self.create_timer(2.0, self._republish)
        self._mid = 0

        self._fk_client = self.create_client(GetPositionFK, 'compute_fk')

    def _republish(self):
        if self._markers.markers:
            self._marker_pub.publish(self._markers)

    def _nid(self):
        self._mid += 1
        return self._mid

    def _spin_sleep(self, duration):
        end_time = time.time() + duration
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _trajectory_to_ee_path(self, trajectory, max_points=20):
        """궤적의 조인트 값을 FK로 끝단 위치 리스트로 변환"""
        if not self._fk_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('compute_fk 서비스 없음')
            return []

        jt = trajectory.joint_trajectory
        total = len(jt.points)
        if total == 0:
            return []

        # 균등 샘플링 (최대 max_points개)
        step = max(1, total // max_points)
        indices = list(range(0, total, step))
        if indices[-1] != total - 1:
            indices.append(total - 1)

        path = []
        for idx in indices:
            pt = jt.points[idx]

            fk_req = GetPositionFK.Request()
            fk_req.header.frame_id = 'base_link'
            fk_req.fk_link_names = ['gripper_base']

            rs = RobotState()
            js = JointState()
            js.name = list(jt.joint_names)
            js.position = list(pt.positions)
            rs.joint_state = js
            fk_req.robot_state = rs

            future = self._fk_client.call_async(fk_req)
            rclpy.spin_until_future_complete(self, future)
            resp = future.result()

            if resp.error_code.val == 1 and resp.pose_stamped:
                p = resp.pose_stamped[0].pose.position
                path.append(Point(x=p.x, y=p.y, z=p.z))

        return path

    def _add_path_marker(self, planner_id, path, planner_idx):
        """플래너 경로를 MarkerArray에 추가"""
        if not path:
            return

        style = PLANNER_STYLES.get(planner_id, {
            'color': ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.8),
            'width': 0.003,
        })
        stamp = self.get_clock().now().to_msg()

        # 경로 선 (LINE_STRIP)
        line = Marker()
        line.header.frame_id = 'base_link'
        line.header.stamp = stamp
        line.ns = 'path_' + planner_id
        line.id = self._nid()
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = style['width']
        line.color = style['color']
        line.points = path
        self._markers.markers.append(line)

        # 시작점 구체
        start_m = Marker()
        start_m.header.frame_id = 'base_link'
        start_m.header.stamp = stamp
        start_m.ns = 'start_' + planner_id
        start_m.id = self._nid()
        start_m.type = Marker.SPHERE
        start_m.action = Marker.ADD
        start_m.pose.position = path[0]
        start_m.pose.orientation.w = 1.0
        start_m.scale = Vector3(x=0.015, y=0.015, z=0.015)
        start_m.color = style['color']
        self._markers.markers.append(start_m)

        # 끝점 구체
        end_m = Marker()
        end_m.header.frame_id = 'base_link'
        end_m.header.stamp = stamp
        end_m.ns = 'end_' + planner_id
        end_m.id = self._nid()
        end_m.type = Marker.SPHERE
        end_m.action = Marker.ADD
        end_m.pose.position = path[-1]
        end_m.pose.orientation.w = 1.0
        end_m.scale = Vector3(x=0.02, y=0.02, z=0.02)
        end_m.color = style['color']
        self._markers.markers.append(end_m)

        # 라벨 (경로 중앙에 플래너 이름)
        mid_idx = len(path) // 2
        label = Marker()
        label.header.frame_id = 'base_link'
        label.header.stamp = stamp
        label.ns = 'label_' + planner_id
        label.id = self._nid()
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        # 살짝 오프셋 (겹침 방지)
        label.pose.position = Point(
            x=path[mid_idx].x,
            y=path[mid_idx].y + 0.02 * (planner_idx - 2),
            z=path[mid_idx].z + 0.03)
        label.pose.orientation.w = 1.0
        label.scale.z = 0.02
        label.color = style['color']
        label.text = planner_id
        self._markers.markers.append(label)

    def run(self):
        arm = MoveGroupHelper(self)

        if not arm.wait_for_servers(timeout_sec=30.0):
            return
        if not arm.wait_for_joint_state(timeout_sec=10.0):
            return

        # 비교할 플래너 목록
        planners = [
            {'id': 'RRTConnect', 'desc': '양방향 RRT - 빠르고 안정적 (기본)'},
            {'id': 'RRT',        'desc': '단방향 RRT - 기본적이지만 느릴 수 있음'},
            {'id': 'PRM',        'desc': 'Probabilistic Roadmap - 다중 쿼리에 효과적'},
            {'id': 'EST',        'desc': 'Expansive Space Trees - 좁은 공간에 유리'},
            {'id': 'KPIECE',     'desc': '투영 기반 탐색 - 고차원 공간에 효과적'},
        ]

        # 동일 목표: 로봇 좌측으로 팔을 크게 뻗어 옆으로 기울인 자세
        # (home에서 최대한 큰 이동 → 플래너별 경로 차이 극대화)
        target_joints = {
            'joint1': math.radians(90),    # 좌측 90° 회전
            'joint2': math.radians(-60),   # 어깨 크게 숙임
            'joint3': math.radians(100),   # 팔꿈치 크게 굽힘
            'joint4': math.radians(-90),   # 손목 아래로
            'joint5': math.radians(60),    # 손목 비틀기
            'joint6': math.radians(-45),   # 끝단 회전
        }

        results = []

        for i, planner in enumerate(planners):
            self.get_logger().info(f'\n--- 플래너: {planner["id"]} ---')
            self.get_logger().info(f'  설명: {planner["desc"]}')

            # home으로 초기화
            arm.planner_id = 'RRTConnect'
            arm.go_to_named_target('home')
            self._spin_sleep(1.0)

            # 플래너 설정
            arm.planner_id = planner['id']
            arm.planning_time = 10.0
            arm.num_planning_attempts = 3

            # 계획만 수행
            start_time = time.time()
            success, trajectory = arm.plan_to_joint_goal(target_joints)
            plan_time = time.time() - start_time

            result = {
                'planner': planner['id'],
                'success': success,
                'plan_time': plan_time,
                'n_points': 0,
            }

            if success and trajectory:
                n_points = len(trajectory.joint_trajectory.points)
                result['n_points'] = n_points
                self.get_logger().info(
                    f'  계획 성공! ({plan_time:.3f}초, {n_points}포인트)')

                # FK로 끝단 경로 계산 → RViz 시각화
                path = self._trajectory_to_ee_path(trajectory)
                if path:
                    self._add_path_marker(planner['id'], path, i)
                    self._marker_pub.publish(self._markers)
                    self.get_logger().info(
                        f'  경로 마커 추가 ({len(path)}개 EE 포인트)')

                # 궤적 실행
                arm.execute_trajectory(trajectory)
            else:
                self.get_logger().warn(f'  계획 실패! ({plan_time:.3f}초)')

            results.append(result)
            self._spin_sleep(1.0)

        # 결과 요약
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('  플래너 비교 결과 요약')
        self.get_logger().info('=' * 60)
        self.get_logger().info(
            f'  {"플래너":<15} {"성공":>4} {"시간(초)":>8} {"포인트":>7}')
        self.get_logger().info('-' * 40)

        for r in results:
            status = 'O' if r['success'] else 'X'
            self.get_logger().info(
                f'  {r["planner"]:<15} {status:>4} '
                f'{r["plan_time"]:>8.3f} {r["n_points"]:>7}')

        self.get_logger().info('=' * 60)
        self.get_logger().info(
            'RViz: /planner_paths MarkerArray로 경로 비교 확인')

        # 최종 복귀
        arm.planner_id = 'RRTConnect'
        arm.go_to_named_target('home')
        self._spin_sleep(1.0)
        self.get_logger().info('=== 예제 10 완료! ===')


def main(args=None):
    rclpy.init(args=args)
    node = MultiPlannerDemo()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
