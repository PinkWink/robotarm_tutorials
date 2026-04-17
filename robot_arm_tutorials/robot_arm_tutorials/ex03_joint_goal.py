"""
예제 03: 개별 조인트 제어
=========================
각 조인트를 하나씩 움직여서 어떤 조인트가 어떤 동작을 하는지
시각적으로 확인하는 예제.

joint1(베이스 회전) → joint2(어깨) → joint3(팔꿈치) →
joint4(손목 회전) → joint5(손목 굽힘) → joint6(손목 비틀기)

학습 내용:
- JointConstraint 메시지 구성
- velocity/acceleration scaling factor
- 각 조인트의 역할 이해

실행 방법:
  터미널1: ros2 launch robot_arm_moveit_config demo.launch.xml
  터미널2: ros2 run robot_arm_tutorials ex03_joint_goal --ros-args -p use_sim_time:=true
"""

import math
import time
import rclpy
from rclpy.node import Node
from robot_arm_tutorials.utils import MoveGroupHelper


class JointGoalDemo(Node):
    def __init__(self):
        super().__init__('joint_goal_demo')
        self.get_logger().info('=== 예제 03: 개별 조인트 제어 ===')

    def run(self):
        helper = MoveGroupHelper(self)
        helper.max_velocity_scaling = 0.3  # 느리게 움직여서 관찰

        if not helper.wait_for_servers(timeout_sec=30.0):
            return
        if not helper.wait_for_joint_state(timeout_sec=10.0):
            return

        # 먼저 home 위치로
        self.get_logger().info('--- 초기화: home 포즈로 이동 ---')
        helper.go_to_named_target('home')
        time.sleep(1.0)

        # 각 조인트 설명
        joint_descriptions = {
            'joint1': ('베이스 회전', math.radians(45)),
            'joint2': ('어깨 (앞뒤 기울기)', math.radians(-30)),
            'joint3': ('팔꿈치 (굽힘)', math.radians(45)),
            'joint4': ('손목 회전 (Roll)', math.radians(45)),
            'joint5': ('손목 굽힘 (Pitch)', math.radians(30)),
            'joint6': ('손목 비틀기 (Yaw)', math.radians(45)),
        }

        # 각 조인트를 하나씩 움직이고 복귀
        for idx, (joint_name, (desc, angle)) in enumerate(joint_descriptions.items(), 1):
            self.get_logger().info(
                f'--- {idx}단계: {joint_name} ({desc}) ---'
            )
            self.get_logger().info(
                f'  {joint_name}을 {math.degrees(angle):.0f}° 로 이동합니다.'
            )

            # 해당 조인트만 움직임 (나머지는 0)
            joint_values = {j: 0.0 for j in helper.ARM_JOINT_NAMES}
            joint_values[joint_name] = angle

            success = helper.go_to_joint_goal(joint_values)
            if not success:
                self.get_logger().error(f'{joint_name} 이동 실패!')
                continue

            time.sleep(1.5)

            # home으로 복귀
            self.get_logger().info(f'  → home으로 복귀')
            helper.go_to_named_target('home')
            time.sleep(1.0)

        # 마지막: 여러 조인트 동시 이동
        self.get_logger().info('--- 보너스: 여러 조인트 동시 이동 ---')
        multi_joint = {
            'joint1': math.radians(30),
            'joint2': math.radians(-45),
            'joint3': math.radians(60),
            'joint4': math.radians(0),
            'joint5': math.radians(45),
            'joint6': math.radians(-30),
        }
        self.get_logger().info('  여러 조인트를 동시에 이동합니다.')
        helper.go_to_joint_goal(multi_joint)
        time.sleep(2.0)

        # 최종 복귀
        helper.go_to_named_target('home')
        self.get_logger().info('=== 예제 03 완료! ===')


def main(args=None):
    rclpy.init(args=args)
    node = JointGoalDemo()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
