"""
예제 02: Named Pose 이동
========================
SRDF에 정의된 'home'(전체 0도)과 'ready'(팔꿈치 들기) 포즈로
순차적으로 이동하는 예제.

학습 내용:
- MoveGroup Action 기본 사용법
- SRDF group_state (이름 포즈)
- 모션 플래닝 파이프라인 이해

실행 방법:
  터미널1: ros2 launch robot_arm_moveit_config demo.launch.xml
  터미널2: ros2 run robot_arm_tutorials ex02_named_pose --ros-args -p use_sim_time:=true
"""

import rclpy
from rclpy.node import Node
from robot_arm_tutorials.utils import MoveGroupHelper


class NamedPoseDemo(Node):
    def __init__(self):
        super().__init__('named_pose_demo')
        self.get_logger().info('=== 예제 02: Named Pose 이동 ===')

    def run(self):
        helper = MoveGroupHelper(self)

        # 서버 연결 대기
        if not helper.wait_for_servers(timeout_sec=30.0):
            return
        if not helper.wait_for_joint_state(timeout_sec=10.0):
            return

        # 1단계: home 포즈로 이동
        self.get_logger().info('--- 1단계: home 포즈로 이동 ---')
        self.get_logger().info('home = 모든 조인트 0도 (직립 자세)')
        success = helper.go_to_named_target('home')
        if not success:
            self.get_logger().error('home 포즈 이동 실패!')
            return

        import time
        time.sleep(1.0)

        # 2단계: ready 포즈로 이동
        self.get_logger().info('--- 2단계: ready 포즈로 이동 ---')
        self.get_logger().info('ready = 팔꿈치를 들어올린 작업 준비 자세')
        success = helper.go_to_named_target('ready')
        if not success:
            self.get_logger().error('ready 포즈 이동 실패!')
            return

        time.sleep(1.0)

        # 3단계: 다시 home으로 복귀
        self.get_logger().info('--- 3단계: home 포즈로 복귀 ---')
        success = helper.go_to_named_target('home')
        if not success:
            self.get_logger().error('home 복귀 실패!')
            return

        self.get_logger().info('=== 예제 02 완료! ===')


def main(args=None):
    rclpy.init(args=args)
    node = NamedPoseDemo()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
