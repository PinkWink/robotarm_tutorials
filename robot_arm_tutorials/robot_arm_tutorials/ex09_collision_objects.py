"""
예제 09: 충돌 객체 관리 (Collision Objects)
============================================
Planning Scene에 장애물을 추가하고,
장애물을 회피하는 경로를 계획하는 예제.

각 단계는 /next_step 토픽 신호로 진행되며,
계획된 끝단 경로를 미리 LINE_STRIP 마커로 RViz에 표시한다.
장애물에 따라 경로가 어떻게 휘는지 비교할 수 있다.

학습 내용:
- PlanningScene, CollisionObject 메시지
- SolidPrimitive (BOX, CYLINDER)
- 충돌 객체 추가/제거
- 장애물 회피 경로 계획
- 단계별 사용자 트리거 (/next_step)
- FK로 계획된 궤적의 끝단 경로 시각화

실행 방법:
  터미널1: ros2 launch robot_arm_moveit_config demo.launch.xml
  터미널2: ros2 run robot_arm_tutorials ex09_collision_objects --ros-args -p use_sim_time:=true
  터미널3: ros2 topic pub --once /next_step std_msgs/msg/Empty '{}'
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, Empty
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import RobotState, MoveItErrorCodes
from robot_arm_tutorials.utils import (
    MoveGroupHelper, make_pose,
    make_box_collision_object, make_cylinder_collision_object,
)


class CollisionObjectsDemo(Node):
    PATH_COLOR = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)  # 주황색: 계획된 끝단 경로

    def __init__(self):
        super().__init__('collision_objects_demo')
        self.get_logger().info('=== 예제 09: 충돌 객체 관리 ===')

        latched_qos = QoSProfile(depth=10)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._marker_pub = self.create_publisher(
            MarkerArray, '/collision_demo_markers', latched_qos
        )
        self._markers = MarkerArray()
        self._marker_timer = self.create_timer(2.0, self._republish)

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

    def _clear_path_marker(self):
        """끝단 경로 마커만 제거 (DELETE action)"""
        stamp = self.get_clock().now().to_msg()
        delete = Marker()
        delete.header.frame_id = 'base_link'
        delete.header.stamp = stamp
        delete.ns = 'ee_path_line'
        delete.id = 0
        delete.action = Marker.DELETE
        ma = MarkerArray()
        ma.markers.append(delete)
        self._markers.markers = [
            m for m in self._markers.markers
            if (m.ns, m.id) != ('ee_path_line', 0)
        ]
        self._marker_pub.publish(ma)

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
        """현재 관절 상태를 시드로 IK 풀이 (충돌 회피 포함). 실패 시 None"""
        if not self._ik_client.wait_for_service(timeout_sec=5.0):
            return None

        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = arm.PLANNING_GROUP
        ik_req.ik_request.robot_state = arm.get_current_robot_state()
        ps = PoseStamped()
        ps.header.frame_id = arm.REFERENCE_FRAME
        ps.pose = pose
        ik_req.ik_request.pose_stamped = ps
        ik_req.ik_request.avoid_collisions = True

        future = self._ik_client.call_async(ik_req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()

        if resp is None or resp.error_code.val != MoveItErrorCodes.SUCCESS:
            return None

        jv = {}
        for i, name in enumerate(resp.solution.joint_state.name):
            if name in arm.ARM_JOINT_NAMES:
                jv[name] = resp.solution.joint_state.position[i]
        return jv

    def _go_smooth(self, arm, pose, label):
        """IK 시드 + plan-only + 끝단경로 시각화 + execute"""
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

    def run(self):
        arm = MoveGroupHelper(self)
        arm.max_velocity_scaling = 0.3
        arm.planning_time = 15.0
        arm.num_planning_attempts = 10

        self.get_logger().info(
            '[RViz2 안내] 마커를 보려면 MarkerArray Display를 추가하고 '
            "Topic을 '/collision_demo_markers' 로 설정하세요. "
            "장애물(CollisionObject)은 RViz의 Planning Scene Display에서 보입니다."
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

        # 초기 home
        self._wait_for_trigger('>>> [대기] home 자세 이동 신호를 기다리는 중...')
        self.get_logger().info('--- 초기: home 자세로 이동 ---')
        self._go_named(arm, 'home')
        self._spin_sleep(1.0)

        # ===================================
        # 시나리오 1: 낮은 벽 넘어 이동
        # ===================================
        self.get_logger().info('')
        self.get_logger().info('=== 시나리오 1: 낮은 벽 넘어 이동 ===')

        wall = make_box_collision_object(
            'wall', 'base_link',
            position=(0.18, 0.0, 0.06),
            dimensions=(0.02, 0.20, 0.12)
        )
        arm.add_collision_object(wall)
        self.get_logger().info('  벽 추가 (x=0.18, 높이 12cm, 폭 20cm)')
        self._spin_sleep(1.0)

        self._wait_for_trigger(
            '>>> [대기] 벽 뒤 목표로 이동(벽 위 회피) 신호를 기다리는 중...'
        )
        self.get_logger().info('--- 벽 뒤 목표로 이동 (벽 위로 회피) ---')
        target1 = make_pose(0.25, 0.0, 0.30, math.pi, 0.0, 0.0)
        success = self._go_smooth(arm, target1, '벽 뒤')
        self.get_logger().info(
            f'  결과: {"성공 (벽 위 회피)" if success else "실패"}'
        )
        self._spin_sleep(1.0)

        self._wait_for_trigger('>>> [대기] home 복귀 + 벽 제거 신호를 기다리는 중...')
        self._go_named(arm, 'home')
        self._spin_sleep(1.0)
        arm.remove_collision_object('wall')
        self._clear_path_marker()
        self.get_logger().info('  벽 제거됨')
        self._spin_sleep(0.5)

        # ===================================
        # 시나리오 2: 옆 장애물 우회
        # ===================================
        self.get_logger().info('')
        self.get_logger().info('=== 시나리오 2: 옆 장애물 우회 ===')

        pillar = make_box_collision_object(
            'pillar', 'base_link',
            position=(0.20, -0.04, 0.20),
            dimensions=(0.04, 0.04, 0.40)
        )
        arm.add_collision_object(pillar)
        self.get_logger().info('  기둥 추가 (x=0.20, y=-0.04, 높이 40cm)')
        self._spin_sleep(1.0)

        self._wait_for_trigger(
            '>>> [대기] 기둥 우회 목표 이동 신호를 기다리는 중...'
        )
        self.get_logger().info('--- 기둥 우회하여 목표 이동 ---')
        target2 = make_pose(0.22, 0.04, 0.30, math.pi, 0.0, 0.0)
        success = self._go_smooth(arm, target2, '기둥 우회')
        self.get_logger().info(
            f'  결과: {"성공 (우회 경로)" if success else "실패"}'
        )
        self._spin_sleep(1.0)

        self._wait_for_trigger('>>> [대기] home 복귀 + 기둥 제거 신호를 기다리는 중...')
        self._go_named(arm, 'home')
        self._spin_sleep(1.0)
        arm.remove_collision_object('pillar')
        self._clear_path_marker()
        self.get_logger().info('  기둥 제거됨')
        self._spin_sleep(0.5)

        # ===================================
        # 시나리오 3: 원기둥 장애물 회피
        # ===================================
        self.get_logger().info('')
        self.get_logger().info('=== 시나리오 3: 원기둥 장애물 회피 ===')

        cylinder = make_cylinder_collision_object(
            'cylinder', 'base_link',
            position=(0.15, 0.0, 0.20),
            height=0.30, radius=0.03
        )
        arm.add_collision_object(cylinder)
        self.get_logger().info('  원기둥 추가 (x=0.15, 반지름 3cm, 높이 30cm)')
        self._spin_sleep(1.0)

        self._wait_for_trigger(
            '>>> [대기] 원기둥 회피 목표 이동 신호를 기다리는 중...'
        )
        self.get_logger().info('--- 원기둥 회피하여 목표 이동 ---')
        target3 = make_pose(0.22, -0.04, 0.30, math.pi, 0.0, 0.0)
        success = self._go_smooth(arm, target3, '원기둥 회피')
        self.get_logger().info(
            f'  결과: {"성공 (회피 경로)" if success else "실패"}'
        )
        self._spin_sleep(1.0)

        # 정리
        self._wait_for_trigger('>>> [대기] home 복귀 + 모두 정리 신호를 기다리는 중...')
        self._go_named(arm, 'home')
        self._spin_sleep(1.0)
        arm.clear_all_collision_objects()
        self._clear_path_marker()
        self.get_logger().info('--- 모든 충돌 객체 정리 완료 ---')
        self.get_logger().info('=== 예제 09 완료! ===')


def main(args=None):
    rclpy.init(args=args)
    node = CollisionObjectsDemo()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
