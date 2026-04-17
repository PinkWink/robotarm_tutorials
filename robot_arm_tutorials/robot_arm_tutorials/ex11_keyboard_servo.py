"""
예제 11: MoveIt Servo 키보드 텔레오퍼레이션
=============================================
curses 기반 키보드 입력으로 MoveIt Servo를 통한 실시간 로봇 제어.

키 매핑:
  W/S : 전진/후진 (X 선형)
  A/D : 좌/우 (Y 선형)
  Q/E : 상승/하강 (Z 선형)
  Shift+W/S : Roll ± (X 회전)
  Shift+A/D : Pitch ± (Y 회전)
  Shift+Q/E : Yaw ± (Z 회전)
  Space : 비상 정지 (속도 0)
  ESC : 종료

아키텍처:
  [키보드 노드 (Python/curses)] ─TwistStamped→ [MoveIt Servo] ─JointTrajectory→ [arm_controller]

실행 방법:
  터미널1: ros2 launch robot_arm_tutorials servo_keyboard.launch.py
  터미널2: ros2 run robot_arm_tutorials ex11_keyboard_servo --ros-args -p use_sim_time:=true
"""

import os
import curses
import time
import threading
import rclpy

# terminfo 데이터베이스 경로 설정 (curses 초기화에 필요)
if 'TERMINFO' not in os.environ:
    candidate_paths = [
        '/usr/share/terminfo',
        '/usr/lib/terminfo',
        '/lib/terminfo',
    ]
    for path in candidate_paths:
        if os.path.isdir(path):
            os.environ['TERMINFO'] = path
            break

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from moveit_msgs.msg import ServoStatus
from robot_arm_tutorials.utils import MoveGroupHelper


# CommandType 상수 (moveit_servo/utils/datatypes.hpp)
COMMAND_TYPE_TWIST = 1


class KeyboardServoNode(Node):
    """MoveIt Servo 키보드 텔레오퍼레이션 노드"""

    # 키 매핑 상수 (소문자 = 선형, 대문자 = 회전)
    KEY_MAP = {
        # 선형 이동
        ord('w'): ('linear', 'x', 1.0),
        ord('s'): ('linear', 'x', -1.0),
        ord('a'): ('linear', 'y', 1.0),
        ord('d'): ('linear', 'y', -1.0),
        ord('q'): ('linear', 'z', 1.0),
        ord('e'): ('linear', 'z', -1.0),
        # 회전 (Shift 키 = 대문자)
        ord('W'): ('angular', 'x', 1.0),
        ord('S'): ('angular', 'x', -1.0),
        ord('A'): ('angular', 'y', 1.0),
        ord('D'): ('angular', 'y', -1.0),
        ord('Q'): ('angular', 'z', 1.0),
        ord('E'): ('angular', 'z', -1.0),
    }

    def __init__(self):
        super().__init__('keyboard_servo_node')
        self.get_logger().info('=== 예제 11: MoveIt Servo 키보드 텔레오퍼레이션 ===')

        self._cb_group = ReentrantCallbackGroup()

        # TwistStamped 퍼블리셔
        self._twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10,
        )

        # 현재 twist 값
        # base_link 기준: W=전진(+X), A=좌(+Y), Q=상승(+Z) — 월드 좌표 기준으로 직관적
        self._twist = TwistStamped()
        self._twist.header.frame_id = 'base_link'

        # ServoCommandType 서비스 클라이언트
        self._switch_cmd_type_client = self.create_client(
            ServoCommandType,
            '/servo_node/switch_command_type',
            callback_group=self._cb_group,
        )

        # Servo 상태 구독 (ServoStatus 메시지)
        self._servo_status = -1
        self._status_sub = self.create_subscription(
            ServoStatus, '/servo_node/status', self._status_cb, 10,
        )

        # 종료 플래그
        self._running = True
        self._key_pressed = False
        self._servo_ready = False

    def _status_cb(self, msg):
        self._servo_status = msg.code

    def _switch_to_twist_mode(self):
        """Servo를 TWIST 명령 모드로 전환"""
        self.get_logger().info('Servo command type을 TWIST로 전환 중...')
        if not self._switch_cmd_type_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('switch_command_type 서비스를 찾을 수 없습니다!')
            return False

        request = ServoCommandType.Request()
        request.command_type = COMMAND_TYPE_TWIST
        future = self._switch_cmd_type_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None and future.result().success:
            self.get_logger().info('TWIST 모드 전환 성공!')
            return True
        else:
            self.get_logger().error('TWIST 모드 전환 실패!')
            return False

    # 특이점을 확실히 피하는 서보용 초기 자세
    # - joint2/3: 팔꿈치 충분히 구부림
    # - joint4: 손목1 오프셋 (joint4=0은 wrist singularity 유발)
    # - joint5: 손목2 충분히 구부림 (joint5=0은 wrist singularity)
    SERVO_READY_JOINTS = {
        'joint1': 0.0,
        'joint2': -1.0,
        'joint3': 1.2,
        'joint4': -0.5,
        'joint5': 1.0,
        'joint6': 0.3,
    }

    def _move_to_ready_pose(self):
        """특이점을 확실히 피한 서보용 초기 자세로 이동"""
        self.get_logger().info('서보용 초기 자세로 이동 (모든 특이점 회피)...')
        helper = MoveGroupHelper(self)
        helper.max_velocity_scaling = 0.3
        if not helper.wait_for_servers(timeout_sec=30.0):
            self.get_logger().warn('MoveGroup 서버 연결 실패, 현재 자세에서 시작합니다.')
            return
        if not helper.wait_for_joint_state(timeout_sec=10.0):
            return
        helper.go_to_joint_goal(self.SERVO_READY_JOINTS)
        time.sleep(1.0)
        self.get_logger().info('서보용 초기 자세 이동 완료!')

    def _publish_twist(self):
        """30Hz로 TwistStamped 발행"""
        if not self._running:
            return
        self._twist.header.stamp = self.get_clock().now().to_msg()
        self._twist_pub.publish(self._twist)

    def _reset_twist(self):
        """모든 속도를 0으로 리셋"""
        self._twist.twist.linear.x = 0.0
        self._twist.twist.linear.y = 0.0
        self._twist.twist.linear.z = 0.0
        self._twist.twist.angular.x = 0.0
        self._twist.twist.angular.y = 0.0
        self._twist.twist.angular.z = 0.0

    def _apply_key(self, key):
        """키 입력에 따라 twist 값 설정"""
        self._reset_twist()

        if key == ord(' '):
            # 비상 정지
            return True

        if key in self.KEY_MAP:
            component, axis, value = self.KEY_MAP[key]
            if component == 'linear':
                setattr(self._twist.twist.linear, axis, value)
            else:
                setattr(self._twist.twist.angular, axis, value)
            self._key_pressed = True
            return True

        return False

    # UI에 필요한 최소 터미널 크기
    MIN_ROWS = 22
    MIN_COLS = 50

    def run_curses(self, stdscr):
        """curses 메인 루프"""
        # curses 설정
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(50)  # 50ms

        def safe_addstr(row, col, text, attr=curses.A_NORMAL):
            try:
                stdscr.addstr(row, col, text, attr)
            except curses.error:
                pass

        # 화면 그리기
        def draw_ui():
            stdscr.erase()
            h, w = stdscr.getmaxyx()

            # 터미널이 너무 작으면 경고만 표시
            if h < self.MIN_ROWS or w < self.MIN_COLS:
                msg = f'Terminal too small: {w}x{h}'
                req = f'Need at least {self.MIN_COLS}x{self.MIN_ROWS}'
                safe_addstr(0, 0, msg, curses.A_BOLD)
                safe_addstr(1, 0, req)
                stdscr.refresh()
                return

            title = '=== MoveIt Servo 키보드 텔레오퍼레이션 ==='
            safe_addstr(0, max(0, (w - len(title)) // 2), title, curses.A_BOLD)

            safe_addstr(2, 2, '선형 이동 (Linear):', curses.A_UNDERLINE)
            safe_addstr(3, 4, 'W/S : 전진/후진 (X)')
            safe_addstr(4, 4, 'A/D : 좌/우     (Y)')
            safe_addstr(5, 4, 'Q/E : 상승/하강 (Z)')

            safe_addstr(7, 2, '회전 (Angular):', curses.A_UNDERLINE)
            safe_addstr(8, 4, 'Shift+W/S : Roll  (X)')
            safe_addstr(9, 4, 'Shift+A/D : Pitch (Y)')
            safe_addstr(10, 4, 'Shift+Q/E : Yaw   (Z)')

            safe_addstr(12, 2, '제어:', curses.A_UNDERLINE)
            safe_addstr(13, 4, 'Space : 비상 정지')
            safe_addstr(14, 4, 'ESC   : 종료')

            # 현재 상태
            safe_addstr(16, 2, '현재 명령:', curses.A_UNDERLINE)
            lx = self._twist.twist.linear.x
            ly = self._twist.twist.linear.y
            lz = self._twist.twist.linear.z
            ax = self._twist.twist.angular.x
            ay = self._twist.twist.angular.y
            az = self._twist.twist.angular.z

            safe_addstr(17, 4, f'Linear  : x={lx:+.1f}  y={ly:+.1f}  z={lz:+.1f}')
            safe_addstr(18, 4, f'Angular : x={ax:+.1f}  y={ay:+.1f}  z={az:+.1f}')

            # Servo 상태
            status_map = {
                -1: '연결 대기...',
                0: '정상 (No Warning)',
                1: '감속 (특이점 접근)',
                2: '정지 (특이점)',
                3: '감속 (특이점 이탈)',
                4: '감속 (충돌 근접)',
                5: '정지 (충돌)',
                6: '조인트 한계',
            }
            status_text = status_map.get(self._servo_status, f'코드: {self._servo_status}')
            ready_text = 'READY' if self._servo_ready else 'NOT READY'
            safe_addstr(20, 2, f'Servo: {ready_text} | 상태: {status_text}')

            stdscr.refresh()

        # 메인 루프
        while self._running:
            key = stdscr.getch()

            if key == 27:  # ESC
                self._running = False
                self._reset_twist()
                break
            elif key == -1:
                # 키를 떼면 속도 0
                if self._key_pressed:
                    self._reset_twist()
                    self._key_pressed = False
            else:
                self._apply_key(key)

            draw_ui()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardServoNode()

    # 1) ready 포즈로 이동 (특이점 회피)
    node._move_to_ready_pose()

    # 2) Servo를 TWIST 모드로 전환
    if node._switch_to_twist_mode():
        node._servo_ready = True
    else:
        node.get_logger().warn('Servo TWIST 전환 실패. 키보드 UI는 시작하지만 동작하지 않을 수 있습니다.')

    # 3) 30Hz 타이머 시작
    node._timer = node.create_timer(1.0 / 30.0, node._publish_twist)

    # 4) rclpy.spin을 별도 스레드에서 실행
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        # curses 래퍼로 실행
        curses.wrapper(node.run_curses)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node._reset_twist()
        node.get_logger().info('키보드 서보 종료')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
