"""
예제 02-1: moveit_py를 이용한 Named Pose 이동
==============================================
ex02와 같은 시나리오(home → ready → home)를 MoveItPy 파이썬 래퍼로 구현.

ex02와의 차이:
- SRDF를 직접 파싱할 필요 없이 `set_goal_state(configuration_name="ready")` 한 줄로
  group_state 이름을 그대로 전달하면 내부에서 SRDF를 조회해 조인트 값으로 변환함.
- MoveGroup Action을 직접 조립하지 않고 `planning_component.plan()` +
  `robot.execute(trajectory)` 의 고수준 API 사용.

선행 조건:
  sudo apt install ros-jazzy-moveit-py
  sudo apt install ros-jazzy-moveit-configs-utils

실행 방법:
  터미널1: ros2 launch robot_arm_moveit_config demo.launch.xml
  터미널2: ros2 launch robot_arm_tutorials ex02_1_moveit_py_named_pose.launch.py
"""

import time

import rclpy
from rclpy.logging import get_logger

from moveit.planning import MoveItPy


PLANNING_GROUP = 'manipulator'
POSE_SEQUENCE = ['home', 'ready', 'home']


def plan_and_execute(robot, planning_component, logger, sleep_time: float = 0.0):
    """플래닝 → 실행의 공통 루틴."""
    logger.info('플래닝 중...')
    plan_result = planning_component.plan()
    if not plan_result:
        logger.error('플래닝 실패')
        return False

    logger.info('플래닝 성공, 실행 중...')
    robot_trajectory = plan_result.trajectory
    robot.execute(robot_trajectory, controllers=[])
    if sleep_time > 0.0:
        time.sleep(sleep_time)
    return True


def main():
    rclpy.init()
    logger = get_logger('moveit_py.named_pose')
    logger.info('=== 예제 02-1: moveit_py Named Pose 이동 ===')

    # MoveItPy 초기화 — 내부적으로 move_group 파라미터를 참조해
    # RobotModel / PlanningScene / PlanningComponent 를 구성한다.
    robot = MoveItPy(node_name='moveit_py_named_pose')
    arm = robot.get_planning_component(PLANNING_GROUP)
    # MoveItPy 초기화 직후 arm_controller/follow_joint_trajectory action 연결과
    # joint_states 타임스탬프 수집이 아직 덜 돼 있어, 바로 plan/execute 를 호출하면
    # 1단계는 'Action client not connected', 2단계는 'allowed_start_tolerance
    # 초과' 로 실패한다. 두 가지 모두 warm-up 지연으로 해소된다.
    logger.info('MoveItPy 초기화 완료 — 컨트롤러/상태 monitor warm-up 대기')
    time.sleep(3.0)

    for idx, pose_name in enumerate(POSE_SEQUENCE, start=1):
        logger.info(f'--- {idx}단계: "{pose_name}" 포즈로 이동 ---')
        arm.set_start_state_to_current_state()
        # configuration_name 으로 SRDF group_state 이름을 그대로 전달
        arm.set_goal_state(configuration_name=pose_name)
        if not plan_and_execute(robot, arm, logger, sleep_time=1.0):
            logger.error(f'"{pose_name}" 이동 실패 — 시나리오 중단')
            break

    logger.info('=== 예제 02-1 완료 ===')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
