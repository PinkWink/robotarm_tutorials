"""
예제 11: MoveIt Servo 키보드 텔레오퍼레이션
=============================================
curses 기반 키보드 입력으로 MoveIt Servo를 통한 실시간 로봇 제어.

실행하면 약 1초 후 서보용 ready 자세로 자동 이동하고,
키 입력 시 끝단(gripper_base)에 명령 방향을 화살표/회전 인디케이터로 표시.
이동한 끝단 궤적은 LINE_STRIP 마커로 누적 기록한다.

키 매핑:
  W/S : 전진/후진 (X 선형)
  A/D : 좌/우 (Y 선형)
  Q/E : 상승/하강 (Z 선형)
  Shift+W/S : Roll ± (X 회전)
  Shift+A/D : Pitch ± (Y 회전)
  Shift+Q/E : Yaw ± (Z 회전)
  Space : 비상 정지 (속도 0)
  ESC : 종료

시각화 (RViz MarkerArray Topic: /servo_viz_markers):
  - 시안 화살표: 선형 명령 방향 (끝단에서 명령 방향으로)
  - 마젠타 화살표 + 'ROT' 라벨: 회전 명령 축
  - 초록 LINE_STRIP: 끝단의 과거 이동 경로 (최근 2000점)

아키텍처:
  [키보드 노드 (Python/curses)] ─TwistStamped→ [MoveIt Servo] ─JointTrajectory→ [arm_controller]

실행 방법:
  터미널1: ros2 launch robot_arm_tutorials servo_keyboard.launch.py
  터미널2: ros2 run robot_arm_tutorials ex11_keyboard_servo --ros-args -p use_sim_time:=true
"""

import math
import os
import curses
import time
import threading
import rclpy
import tf2_ros

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
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TwistStamped, Point, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
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

        # ready 자세 복귀 진행 플래그 (R 키 처리 중복 방지)
        self._returning_to_ready = False

        # 시각화: 끝단 명령 방향 + 이동 경로
        latched_qos = QoSProfile(depth=10)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._marker_pub = self.create_publisher(
            MarkerArray, '/servo_viz_markers', latched_qos
        )
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._path_points = []        # 끝단 누적 경로 (base_link 기준, 튜플 리스트)
        self._last_path_point = None  # 마지막 추가 포인트 (중복 방지)
        self._viz_timer = None        # ready 자세 도달 후 시작

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

    # SRDF 'ready'는 joint2≈0, joint5≈0이라 shoulder/wrist 양쪽 특이점 근방.
    # 또 본 로봇은 reach ~0.78m로 작아 야코비안 단위 mismatch 때문에 좋은 자세에서도
    # 조건수가 자연스럽게 100~수백까지 뜸. SRDF값 일부만 override 해서는 부족하므로,
    # Servo 전용으로 모든 특이점 family에서 충분히 떨어진 자세를 별도 정의.
    SERVO_READY_JOINTS = {
        'joint1': 0.0,
        'joint2': 0.7,   # 어깨 전방 기울기 → wrist를 joint1(Z)축에서 멀리 (shoulder 비특이점)
        'joint3': 1.0,   # 팔꿈치 굽힘 (elbow 비특이점)
        'joint4': 0.0,
        'joint5': 1.0,   # wrist YZY 중간각 (wrist 비특이점)
        'joint6': 0.0,
    }

    def _servo_safe_ready_joints(self, helper):
        """Servo 시작용 비특이점 ready 자세 dict 반환 (SRDF 무관)"""
        return dict(self.SERVO_READY_JOINTS)

    def _move_to_ready_pose(self):
        """Servo-safe ready 자세로 이동 (특이점 회피용 별도 자세).

        SRDF 'ready'는 joint2≈0(shoulder), joint5≈0(wrist)이라 Servo에서 사용 불가.
        Servo 전용으로 j2=0.7, j3=1.0, j5=1.0 등 비특이점 자세 사용.
        """
        self.get_logger().info("Servo-safe ready 자세로 이동...")
        helper = MoveGroupHelper(self)
        helper.max_velocity_scaling = 0.3
        if not helper.wait_for_servers(timeout_sec=30.0):
            self.get_logger().warn('MoveGroup 서버 연결 실패, 현재 자세에서 시작합니다.')
            return
        if not helper.wait_for_joint_state(timeout_sec=10.0):
            return
        helper.go_to_joint_goal(self._servo_safe_ready_joints(helper))
        time.sleep(1.0)
        self.get_logger().info("ready 자세 이동 완료!")

    def _trigger_go_to_ready(self):
        """R 키 처리: ready 자세 복귀를 별도 스레드에서 비동기로 실행"""
        if self._returning_to_ready:
            return
        self._returning_to_ready = True
        self._reset_twist()
        self._key_pressed = False
        threading.Thread(target=self._do_go_to_ready, daemon=True).start()

    def _do_go_to_ready(self):
        """별도 노드로 ready 이동 (메인 spin 스레드와 충돌 회피).

        MoveGroupHelper는 내부에서 rclpy.spin_until_future_complete(helper_node, ...)
        를 직접 호출하므로, helper_node는 외부 executor에 절대 붙이면 안 된다.
        (붙이면 "Executor is already spinning" 에러)
        메인 self 노드는 spin_thread가 잡고 있으니, 임시 노드만 새로 만들어 helper에
        넘기면 helper가 자기 spin을 돌려 안전하게 처리한다.
        """
        helper_node = None
        try:
            helper_node = rclpy.create_node('keyboard_servo_ready_helper')

            helper = MoveGroupHelper(helper_node)
            helper.max_velocity_scaling = 0.3
            if (helper.wait_for_servers(timeout_sec=10.0)
                    and helper.wait_for_joint_state(timeout_sec=5.0)):
                helper.go_to_joint_goal(self._servo_safe_ready_joints(helper))
                self.get_logger().info('ready 복귀 완료')
            else:
                self.get_logger().warn('ready 복귀: MoveGroup 연결 실패')
        except Exception as e:
            self.get_logger().error(f'ready 이동 예외: {e}')
        finally:
            if helper_node is not None:
                helper_node.destroy_node()
            self._returning_to_ready = False

    def _publish_twist(self):
        """30Hz로 TwistStamped 발행 (ready 복귀 중에는 발행 중단)"""
        if not self._running or self._returning_to_ready:
            return
        self._twist.header.stamp = self.get_clock().now().to_msg()
        self._twist_pub.publish(self._twist)

    # ----------------------------------------------------------
    #  시각화 (끝단 명령 화살표 + 이동 경로)
    # ----------------------------------------------------------
    PATH_DIST_THRESHOLD = 0.005  # 5mm 이상 이동 시 새 path point 추가
    MAX_PATH_POINTS = 2000       # path 슬라이딩 윈도우 길이
    LINEAR_ARROW_LEN = 0.15      # 선형 명령 화살표 길이 (m)
    ANGULAR_ARROW_LEN = 0.12     # 회전 명령 화살표 길이 (m)

    def _start_viz_timer(self):
        """ready 자세 도달 후 시각화 타이머 시작 (20Hz)"""
        self._viz_timer = self.create_timer(1.0 / 20.0, self._viz_callback)

    def _lookup_ee(self):
        """gripper_base의 base_link 기준 위치 조회. 실패 시 None"""
        try:
            tr = self._tf_buffer.lookup_transform(
                'base_link', 'gripper_base', rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return None
        t = tr.transform.translation
        return (t.x, t.y, t.z)

    def _viz_callback(self):
        """20Hz: 현재 EE 위치 추적 → 경로 누적 + 명령 방향 마커 갱신"""
        ee = self._lookup_ee()
        if ee is None:
            return

        # 경로 누적 (이전 포인트와 충분히 멀 때만)
        if (self._last_path_point is None or
                self._dist(ee, self._last_path_point) > self.PATH_DIST_THRESHOLD):
            self._path_points.append(ee)
            self._last_path_point = ee
            if len(self._path_points) > self.MAX_PATH_POINTS:
                # 슬라이딩 윈도우: 오래된 포인트 제거
                self._path_points = self._path_points[-self.MAX_PATH_POINTS:]

        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # 1) 경로 LINE_STRIP
        if len(self._path_points) >= 2:
            ma.markers.append(self._make_path_marker(stamp))

        # 2) 명령 방향 마커 (선형/회전)
        lin = self._twist.twist.linear
        ang = self._twist.twist.angular
        has_lin = abs(lin.x) > 0.01 or abs(lin.y) > 0.01 or abs(lin.z) > 0.01
        has_ang = abs(ang.x) > 0.01 or abs(ang.y) > 0.01 or abs(ang.z) > 0.01

        if has_lin:
            ma.markers.append(self._make_arrow_marker(
                stamp, 'cmd_linear', ee,
                (lin.x, lin.y, lin.z), self.LINEAR_ARROW_LEN,
                ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0),  # cyan
            ))
        else:
            ma.markers.append(self._make_delete_marker(stamp, 'cmd_linear'))

        if has_ang:
            # 회전축을 따라 양방향(중앙 기준)으로 그어 회전축임을 표시
            ma.markers.append(self._make_axis_marker(
                stamp, 'cmd_angular', ee,
                (ang.x, ang.y, ang.z), self.ANGULAR_ARROW_LEN,
                ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0),  # magenta
            ))
            ma.markers.append(self._make_text_marker(
                stamp, 'cmd_angular_label', ee, 'ROT',
                ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0),
            ))
        else:
            ma.markers.append(self._make_delete_marker(stamp, 'cmd_angular'))
            ma.markers.append(self._make_delete_marker(stamp, 'cmd_angular_label'))

        self._marker_pub.publish(ma)

    @staticmethod
    def _dist(a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

    def _make_path_marker(self, stamp):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = stamp
        m.ns = 'ee_path'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.004
        m.color = ColorRGBA(r=0.2, g=1.0, b=0.4, a=0.85)
        m.points = [Point(x=p[0], y=p[1], z=p[2]) for p in self._path_points]
        return m

    def _make_arrow_marker(self, stamp, ns, origin, direction, length, color):
        """origin에서 direction 방향으로 length만큼 뻗는 화살표"""
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = stamp
        m.ns = ns
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.points = [
            Point(x=origin[0], y=origin[1], z=origin[2]),
            Point(x=origin[0] + direction[0] * length,
                  y=origin[1] + direction[1] * length,
                  z=origin[2] + direction[2] * length),
        ]
        m.scale = Vector3(x=0.008, y=0.020, z=0.025)  # shaft, head_d, head_len
        m.color = color
        return m

    def _make_axis_marker(self, stamp, ns, origin, axis, length, color):
        """회전축을 origin 중심으로 양방향 표시 (회전 명령 시각화)"""
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = stamp
        m.ns = ns
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        half = length * 0.5
        m.points = [
            Point(x=origin[0] - axis[0] * half,
                  y=origin[1] - axis[1] * half,
                  z=origin[2] - axis[2] * half),
            Point(x=origin[0] + axis[0] * half,
                  y=origin[1] + axis[1] * half,
                  z=origin[2] + axis[2] * half),
        ]
        m.scale = Vector3(x=0.006, y=0.020, z=0.020)
        m.color = color
        return m

    def _make_text_marker(self, stamp, ns, origin, text, color):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = stamp
        m.ns = ns
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = origin[0]
        m.pose.position.y = origin[1]
        m.pose.position.z = origin[2] + 0.08
        m.pose.orientation.w = 1.0
        m.scale.z = 0.025
        m.color = color
        m.text = text
        return m

    def _make_delete_marker(self, stamp, ns):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = stamp
        m.ns = ns
        m.id = 0
        m.action = Marker.DELETE
        return m

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

        if key in (ord('r'), ord('R')):
            # ready 자세 복귀 (별도 스레드에서 MoveGroup 액션 호출)
            self._trigger_go_to_ready()
            return True

        # ready 복귀 중에는 twist 입력 무시 (controller에 트래픽 충돌 방지)
        if self._returning_to_ready:
            return False

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
    MIN_ROWS = 23
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
            safe_addstr(14, 4, 'R     : ready 자세 복귀')
            safe_addstr(15, 4, 'ESC   : 종료')

            # 현재 상태
            safe_addstr(17, 2, '현재 명령:', curses.A_UNDERLINE)
            lx = self._twist.twist.linear.x
            ly = self._twist.twist.linear.y
            lz = self._twist.twist.linear.z
            ax = self._twist.twist.angular.x
            ay = self._twist.twist.angular.y
            az = self._twist.twist.angular.z

            safe_addstr(18, 4, f'Linear  : x={lx:+.1f}  y={ly:+.1f}  z={lz:+.1f}')
            safe_addstr(19, 4, f'Angular : x={ax:+.1f}  y={ay:+.1f}  z={az:+.1f}')

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
            safe_addstr(21, 2, f'Servo: {ready_text} | 상태: {status_text}')

            if self._returning_to_ready:
                safe_addstr(22, 2, '*** ready 자세 복귀 중... (twist 입력 무시) ***',
                            curses.A_BOLD)

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

    # 1) 시작 후 1초 대기 (사용자가 RViz 화면을 확인할 시간)
    node.get_logger().info('1초 후 서보용 ready 자세로 이동합니다...')
    time.sleep(1.0)

    # 2) ready 포즈로 이동 (특이점 회피)
    node._move_to_ready_pose()

    # 3) Servo를 TWIST 모드로 전환
    if node._switch_to_twist_mode():
        node._servo_ready = True
    else:
        node.get_logger().warn('Servo TWIST 전환 실패. 키보드 UI는 시작하지만 동작하지 않을 수 있습니다.')

    # 4) 30Hz twist 타이머 시작
    node._timer = node.create_timer(1.0 / 30.0, node._publish_twist)

    # 5) 시각화 타이머 시작 (20Hz: 끝단 명령 화살표 + 이동 경로)
    node._start_viz_timer()
    node.get_logger().info(
        '[RViz2 안내] /servo_viz_markers 토픽으로 명령 방향(화살표) + '
        '이동 경로(LINE_STRIP)를 표시합니다.'
    )

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
