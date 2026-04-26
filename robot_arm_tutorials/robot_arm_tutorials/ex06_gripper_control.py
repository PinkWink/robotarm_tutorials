"""
예제 06: 그리퍼 제어
====================
GripperCommand Action을 사용하여 그리퍼를 열고 닫는 예제.

학습 내용:
- GripperCommand 액션 인터페이스
- prismatic 조인트 (직선 이동 조인트)
- 그리퍼 컨트롤러와 팔 컨트롤러의 차이
- max_effort 파라미터

실행 방법:
  터미널1: ros2 launch robot_arm_moveit_config demo.launch.xml
  터미널2: ros2 run robot_arm_tutorials ex06_gripper_control --ros-args -p use_sim_time:=true
"""

import time
import rclpy
from rclpy.node import Node
from robot_arm_tutorials.utils import GripperHelper


class GripperControlDemo(Node):
    def __init__(self):
        super().__init__('gripper_control_demo')
        self.get_logger().info('=== 예제 06: 그리퍼 제어 ===')

    def run(self):
        gripper = GripperHelper(self)

        if not gripper.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('그리퍼 서버 연결 실패!')
            return

        # 1단계: 그리퍼 열기
        self.get_logger().info('--- 1단계: 그리퍼 열기 (0.0m) ---')
        self.get_logger().info('  left_finger_joint = 0.0m (최대 열림)')
        gripper.open_gripper()
        time.sleep(2.0)

        # 2단계: 그리퍼 닫기
        self.get_logger().info('--- 2단계: 그리퍼 닫기 (0.02m) ---')
        self.get_logger().info('  left_finger_joint = 0.02m (완전 닫힘)')
        gripper.close_gripper()
        time.sleep(2.0)

        # 3단계: 반쯤 열기
        self.get_logger().info('--- 3단계: 그리퍼 반쯤 열기 (0.01m) ---')
        self.get_logger().info('  left_finger_joint = 0.01m (절반 열림)')
        gripper.move_gripper(0.01)
        time.sleep(2.0)

        # 4단계: 다시 열기
        self.get_logger().info('--- 4단계: 그리퍼 다시 열기 ---')
        gripper.open_gripper()
        time.sleep(1.0)

        self.get_logger().info('=== 예제 06 완료! ===')


def main(args=None):
    rclpy.init(args=args)
    node = GripperControlDemo()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
