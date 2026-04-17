"""
예제 01: 조인트 상태 읽기 (Joint State Reader)
==============================================
/joint_states 토픽을 구독하여 7개 조인트(arm 6개 + gripper 1개)의
실시간 상태를 출력하는 기초 예제.

학습 내용:
- ROS2 Subscription 패턴
- sensor_msgs/JointState 메시지 구조
- use_sim_time 파라미터

실행 방법:
  터미널1: ros2 launch robot_arm_moveit_config demo.launch.xml
  터미널2: ros2 run robot_arm_tutorials ex01_joint_state_reader --ros-args -p use_sim_time:=true
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateReader(Node):
    """조인트 상태 구독 노드"""

    def __init__(self):
        super().__init__('joint_state_reader')
        self.get_logger().info('=== 예제 01: 조인트 상태 읽기 ===')
        self.get_logger().info('/joint_states 토픽 구독을 시작합니다...')

        # 조인트 이름 정의 (arm 6개 + gripper 1개)
        self.arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.gripper_joints = ['left_finger_joint']
        self.all_joints = self.arm_joints + self.gripper_joints

        # 수신 카운트
        self.msg_count = 0

        # 구독자 생성
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

    def joint_state_callback(self, msg: JointState):
        """조인트 상태 콜백 - 매 10번째 메시지마다 출력"""
        self.msg_count += 1

        # 너무 빠른 출력 방지: 10번에 한 번만 출력
        if self.msg_count % 10 != 0:
            return

        self.get_logger().info(f'--- 조인트 상태 [#{self.msg_count}] ---')

        # 조인트 이름과 값을 매칭하여 출력
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                pos_deg = math.degrees(msg.position[i])
                pos_rad = msg.position[i]

                vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
                eff = msg.effort[i] if i < len(msg.effort) else 0.0

                if name in self.arm_joints:
                    self.get_logger().info(
                        f'  [팔] {name:>20s}: '
                        f'{pos_rad:+7.3f} rad ({pos_deg:+7.1f}°) | '
                        f'속도: {vel:+6.3f} | 힘: {eff:+6.3f}'
                    )
                elif name in self.gripper_joints:
                    self.get_logger().info(
                        f'  [그리퍼] {name:>17s}: '
                        f'{pos_rad:+7.4f} m | '
                        f'속도: {vel:+6.3f} | 힘: {eff:+6.3f}'
                    )


def main(args=None):
    rclpy.init(args=args)
    node = JointStateReader()

    try:
        node.get_logger().info('Ctrl+C로 종료할 수 있습니다.')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
