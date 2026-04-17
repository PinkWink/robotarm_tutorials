"""
예제 09: 충돌 객체 관리 (Collision Objects)
============================================
Planning Scene에 장애물을 추가하고,
장애물을 회피하는 경로를 계획하는 예제.

학습 내용:
- PlanningScene, CollisionObject 메시지
- SolidPrimitive (BOX, CYLINDER)
- 충돌 객체 추가/제거
- 장애물 회피 경로 계획

실행 방법:
  터미널1: ros2 launch robot_arm_moveit_config demo.launch.xml
  터미널2: ros2 run robot_arm_tutorials ex09_collision_objects --ros-args -p use_sim_time:=true
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState, MoveItErrorCodes
from robot_arm_tutorials.utils import (
    MoveGroupHelper, make_pose,
    make_box_collision_object, make_cylinder_collision_object,
)


class CollisionObjectsDemo(Node):
    def __init__(self):
        super().__init__('collision_objects_demo')
        self.get_logger().info('=== 예제 09: 충돌 객체 관리 ===')
        self._ik_client = self.create_client(GetPositionIK, 'compute_ik')

    def _spin_sleep(self, duration):
        end_time = time.time() + duration
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _go_smooth(self, arm, pose, label=''):
        """현재 관절 시드로 IK → go_to_joint_goal (최소 회전)"""
        if not self._ik_client.wait_for_service(timeout_sec=5.0):
            return arm.go_to_pose_goal(pose)

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

        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(f'{label}: IK 실패, go_to_pose_goal 폴백')
            return arm.go_to_pose_goal(pose)

        jv = {}
        for i, name in enumerate(resp.solution.joint_state.name):
            if name in arm.ARM_JOINT_NAMES:
                jv[name] = resp.solution.joint_state.position[i]
        return arm.go_to_joint_goal(jv)

    def run(self):
        arm = MoveGroupHelper(self)
        arm.max_velocity_scaling = 0.3
        arm.planning_time = 15.0
        arm.num_planning_attempts = 10

        if not arm.wait_for_servers(timeout_sec=30.0):
            return
        if not arm.wait_for_joint_state(timeout_sec=10.0):
            return

        arm.go_to_named_target('home')
        self._spin_sleep(1.0)

        # ===================================
        # 시나리오 1: 낮은 벽 넘어 이동
        # ===================================
        self.get_logger().info('=== 시나리오 1: 낮은 벽 넘어 이동 ===')

        wall = make_box_collision_object(
            'wall', 'base_link',
            position=(0.18, 0.0, 0.06),
            dimensions=(0.02, 0.20, 0.12)  # 얇고 낮은 벽 (12cm 높이)
        )
        arm.add_collision_object(wall)
        self.get_logger().info('  벽 추가 (x=0.18, 높이 12cm, 폭 20cm)')
        self._spin_sleep(1.0)

        # 벽 뒤쪽 목표 (벽 위로 넘어서 도달)
        self.get_logger().info('--- 벽 뒤 목표로 이동 (벽 위로 회피) ---')
        target1 = make_pose(0.25, 0.0, 0.30, math.pi, 0.0, 0.0)
        success = self._go_smooth(arm, target1, '벽 뒤')
        self.get_logger().info(
            f'  결과: {"성공 (벽 위 회피)" if success else "실패"}')
        self._spin_sleep(2.0)

        arm.go_to_named_target('home')
        self._spin_sleep(1.0)
        arm.remove_collision_object('wall')
        self.get_logger().info('  벽 제거됨')
        self._spin_sleep(0.5)

        # ===================================
        # 시나리오 2: 옆 장애물 우회
        # ===================================
        self.get_logger().info('=== 시나리오 2: 옆 장애물 우회 ===')

        # 로봇 전방 오른쪽에 기둥
        pillar = make_box_collision_object(
            'pillar', 'base_link',
            position=(0.20, -0.04, 0.20),
            dimensions=(0.04, 0.04, 0.40)  # 가는 기둥
        )
        arm.add_collision_object(pillar)
        self.get_logger().info('  기둥 추가 (x=0.20, y=-0.04, 높이 40cm)')
        self._spin_sleep(1.0)

        # 기둥 왼쪽으로 우회하여 도달
        self.get_logger().info('--- 기둥 우회하여 목표 이동 ---')
        target2 = make_pose(0.22, 0.04, 0.30, math.pi, 0.0, 0.0)
        success = self._go_smooth(arm, target2, '기둥 우회')
        self.get_logger().info(
            f'  결과: {"성공 (우회 경로)" if success else "실패"}')
        self._spin_sleep(2.0)

        arm.go_to_named_target('home')
        self._spin_sleep(1.0)
        arm.remove_collision_object('pillar')
        self.get_logger().info('  기둥 제거됨')
        self._spin_sleep(0.5)

        # ===================================
        # 시나리오 3: 원기둥 장애물 회피
        # ===================================
        self.get_logger().info('=== 시나리오 3: 원기둥 장애물 회피 ===')

        cylinder = make_cylinder_collision_object(
            'cylinder', 'base_link',
            position=(0.15, 0.0, 0.20),
            height=0.30, radius=0.03
        )
        arm.add_collision_object(cylinder)
        self.get_logger().info('  원기둥 추가 (x=0.15, 반지름 3cm, 높이 30cm)')
        self._spin_sleep(1.0)

        # 원기둥 옆으로 이동
        self.get_logger().info('--- 원기둥 회피하여 목표 이동 ---')
        target3 = make_pose(0.22, -0.04, 0.30, math.pi, 0.0, 0.0)
        success = self._go_smooth(arm, target3, '원기둥 회피')
        self.get_logger().info(
            f'  결과: {"성공 (회피 경로)" if success else "실패"}')
        self._spin_sleep(2.0)

        # 정리
        arm.go_to_named_target('home')
        self._spin_sleep(1.0)
        arm.clear_all_collision_objects()
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
