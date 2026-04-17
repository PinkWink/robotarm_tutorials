"""
예제 15: IK 기반 키보드 텔레오퍼레이션
=======================================
curses 기반 키보드 입력으로 실시간 IK를 통한 로봇 제어.
MoveIt Servo 없이 IK 서비스 + 직접 컨트롤러 스트리밍 방식.

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
  [키보드 노드] ─목표포즈→ [MoveIt IK] ─관절각→ [arm_controller]
  (ex11과 달리 Servo 노드 불필요, 특이점 제한 없음)

예제 11 vs 예제 15 비교:
  ex11: 키보드 → TwistStamped → MoveIt Servo → arm_controller
        (Servo 노드 필요, 특이점에서 정지)
  ex15: 키보드 → 목표 포즈 계산 → IK 서비스 → arm_controller 직접
        (Servo 노드 불필요, IK 해만 있으면 동작)

실행 방법:
  터미널1: ros2 launch robot_arm_moveit_config demo.launch.xml
  터미널2: ros2 run robot_arm_tutorials ex15_keyboard_ik --ros-args -p use_sim_time:=true
"""

import os
import curses
import math
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
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState, MoveItErrorCodes

import tf2_ros
import tf_transformations

from robot_arm_tutorials.utils import MoveGroupHelper

JOINT_NAMES = ['joint1', 'joint2', 'joint3',
               'joint4', 'joint5', 'joint6']

# 이동 속도 (m/step, rad/step)
LINEAR_STEP = 0.003     # 3mm per keypress cycle
ANGULAR_STEP = 0.03     # ~1.7° per keypress cycle


class KeyboardIKNode(Node):
    """IK 기반 키보드 텔레오퍼레이션 노드"""

    # 키 매핑 (소문자 = 선형, 대문자 = 회전)
    KEY_MAP = {
        # 선형 이동
        ord('w'): ('linear', 'x', 1.0),
        ord('s'): ('linear', 'x', -1.0),
        ord('a'): ('linear', 'y', 1.0),
        ord('d'): ('linear', 'y', -1.0),
        ord('q'): ('linear', 'z', 1.0),
        ord('e'): ('linear', 'z', -1.0),
        # 회전 (Shift = 대문자)
        ord('W'): ('angular', 'x', 1.0),
        ord('S'): ('angular', 'x', -1.0),
        ord('A'): ('angular', 'y', 1.0),
        ord('D'): ('angular', 'y', -1.0),
        ord('Q'): ('angular', 'z', 1.0),
        ord('E'): ('angular', 'z', -1.0),
    }

    def __init__(self):
        super().__init__('keyboard_ik_node')
        self.get_logger().info('=== 예제 15: IK 기반 키보드 텔레오퍼레이션 ===')

        self._cb_group = ReentrantCallbackGroup()

        # 직접 컨트롤러 발행
        self._traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # IK 서비스 클라이언트
        self._ik_client = self.create_client(
            GetPositionIK, '/compute_ik',
            callback_group=self._cb_group)

        # 현재 관절 상태
        self._current_joints = None
        self._joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # 현재 EE 포즈 (절대)
        self._current_pose = None

        # 키 입력 상태
        self._cmd = {'lx': 0.0, 'ly': 0.0, 'lz': 0.0,
                     'ax': 0.0, 'ay': 0.0, 'az': 0.0}
        self._running = True
        self._key_pressed = False
        self._ik_ok = True
        self._ik_status = 'READY'

    def _joint_state_cb(self, msg):
        joints = {}
        for name, pos in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                joints[name] = pos
        if len(joints) == len(JOINT_NAMES):
            self._current_joints = joints

    def _update_current_pose(self):
        """TF에서 현재 EE 포즈 읽기"""
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                'base_link', 'gripper_base', rclpy.time.Time())
            t = tf_msg.transform.translation
            r = tf_msg.transform.rotation
            pose = Pose()
            pose.position = Point(x=t.x, y=t.y, z=t.z)
            pose.orientation = Quaternion(x=r.x, y=r.y, z=r.z, w=r.w)
            self._current_pose = pose
            return True
        except Exception:
            return False

    def _compute_ik(self, target_pose):
        """IK 서비스 호출"""
        if not self._ik_client.service_is_ready():
            return None

        request = GetPositionIK.Request()
        request.ik_request.group_name = 'manipulator'
        request.ik_request.pose_stamped.header.frame_id = 'base_link'
        request.ik_request.pose_stamped.header.stamp = (
            self.get_clock().now().to_msg())
        request.ik_request.pose_stamped.pose = target_pose
        request.ik_request.timeout.sec = 0
        request.ik_request.timeout.nanosec = int(0.05 * 1e9)

        if self._current_joints:
            rs = RobotState()
            js = JointState()
            js.name = list(self._current_joints.keys())
            js.position = list(self._current_joints.values())
            rs.joint_state = js
            request.ik_request.robot_state = rs

        future = self._ik_client.call_async(request)
        deadline = time.time() + 0.1
        while not future.done() and time.time() < deadline:
            time.sleep(0.001)

        if future.done() and future.result() is not None:
            result = future.result()
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return dict(zip(
                    result.solution.joint_state.name,
                    result.solution.joint_state.position))
        return None

    def _publish_joint_cmd(self, target_joints):
        """관절 위치를 arm_controller에 직접 발행"""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [
            target_joints.get(name, 0.0) for name in JOINT_NAMES]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(0.05 * 1e9)
        msg.points = [point]

        self._traj_pub.publish(msg)

    def _apply_command(self):
        """현재 키 입력을 반영하여 IK 계산 + 관절 명령 발행"""
        if self._current_pose is None:
            return

        # 키 입력 없으면 건너뜀
        lx = self._cmd['lx']
        ly = self._cmd['ly']
        lz = self._cmd['lz']
        ax = self._cmd['ax']
        ay = self._cmd['ay']
        az = self._cmd['az']

        if abs(lx) + abs(ly) + abs(lz) + abs(ax) + abs(ay) + abs(az) < 0.01:
            return

        # 현재 포즈에서 상대적 변위 적용 → 새 목표 포즈
        target = Pose()

        # 위치: 현재 + 선형 변위
        target.position = Point(
            x=self._current_pose.position.x + lx * LINEAR_STEP,
            y=self._current_pose.position.y + ly * LINEAR_STEP,
            z=self._current_pose.position.z + lz * LINEAR_STEP)

        # 방향: 현재 쿼터니언에 회전 변위 적용
        cur_q = self._current_pose.orientation
        q_cur = [cur_q.x, cur_q.y, cur_q.z, cur_q.w]

        # 미소 회전을 쿼터니언으로 변환
        droll = ax * ANGULAR_STEP
        dpitch = ay * ANGULAR_STEP
        dyaw = az * ANGULAR_STEP
        q_delta = tf_transformations.quaternion_from_euler(
            droll, dpitch, dyaw)
        q_new = tf_transformations.quaternion_multiply(q_cur, q_delta)

        target.orientation = Quaternion(
            x=q_new[0], y=q_new[1], z=q_new[2], w=q_new[3])

        # IK 계산 + 발행
        ik_result = self._compute_ik(target)
        if ik_result:
            self._publish_joint_cmd(ik_result)
            self._current_pose = target
            self._ik_ok = True
            self._ik_status = 'OK'
        else:
            self._ik_ok = False
            self._ik_status = 'IK FAIL (한계)'

    def _control_loop(self):
        """30Hz 제어 루프"""
        while self._running:
            self._update_current_pose()
            self._apply_command()
            time.sleep(1.0 / 30.0)

    def _apply_key(self, key):
        """키 입력 처리"""
        self._cmd = {'lx': 0.0, 'ly': 0.0, 'lz': 0.0,
                     'ax': 0.0, 'ay': 0.0, 'az': 0.0}

        if key == ord(' '):
            return True

        if key in self.KEY_MAP:
            component, axis, value = self.KEY_MAP[key]
            if component == 'linear':
                self._cmd['l' + axis] = value
            else:
                self._cmd['a' + axis] = value
            self._key_pressed = True
            return True

        return False

    def run_curses(self, stdscr):
        """curses 메인 루프"""
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(50)

        def safe_addstr(row, col, text, attr=curses.A_NORMAL):
            try:
                stdscr.addstr(row, col, text, attr)
            except curses.error:
                pass

        MIN_ROWS = 18
        MIN_COLS = 45

        def draw_ui():
            stdscr.erase()
            h, w = stdscr.getmaxyx()

            # 터미널이 너무 작으면 경고만 표시
            if h < MIN_ROWS or w < MIN_COLS:
                safe_addstr(0, 0, f'Terminal too small: {w}x{h}',
                            curses.A_BOLD)
                safe_addstr(1, 0, f'Need at least {MIN_COLS}x{MIN_ROWS}')
                stdscr.refresh()
                return

            safe_addstr(0, max(0, (w - 30) // 2),
                        '=== IK Keyboard Teleop ===', curses.A_BOLD)

            safe_addstr(2, 2, 'Linear:', curses.A_UNDERLINE)
            safe_addstr(3, 4, 'W/S:X  A/D:Y  Q/E:Z')

            safe_addstr(5, 2, 'Angular (Shift):', curses.A_UNDERLINE)
            safe_addstr(6, 4, 'W/S:Roll  A/D:Pitch  Q/E:Yaw')

            safe_addstr(8, 2, 'Space:Stop  ESC:Quit')

            c = self._cmd
            safe_addstr(10, 2, 'Cmd:', curses.A_UNDERLINE)
            safe_addstr(11, 4,
                        f'L: x={c["lx"]:+.1f} y={c["ly"]:+.1f} '
                        f'z={c["lz"]:+.1f}')
            safe_addstr(12, 4,
                        f'A: x={c["ax"]:+.1f} y={c["ay"]:+.1f} '
                        f'z={c["az"]:+.1f}')

            if self._current_pose:
                p = self._current_pose.position
                safe_addstr(14, 2,
                            f'EE: x={p.x:.3f} y={p.y:.3f} '
                            f'z={p.z:.3f}')

            ik_attr = curses.A_BOLD if not self._ik_ok else curses.A_NORMAL
            safe_addstr(16, 2, f'IK: {self._ik_status}', ik_attr)

            stdscr.refresh()

        while self._running:
            key = stdscr.getch()

            if key == 27:  # ESC
                self._running = False
                break
            elif key == -1:
                if self._key_pressed:
                    self._cmd = {'lx': 0.0, 'ly': 0.0, 'lz': 0.0,
                                 'ax': 0.0, 'ay': 0.0, 'az': 0.0}
                    self._key_pressed = False
            else:
                self._apply_key(key)

            draw_ui()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardIKNode()

    # 1) ready 포즈로 이동
    node.get_logger().info('초기 자세로 이동 중...')
    helper = MoveGroupHelper(node)
    helper.max_velocity_scaling = 0.3
    if helper.wait_for_servers(timeout_sec=30.0):
        if helper.wait_for_joint_state(timeout_sec=10.0):
            helper.go_to_named_target('ready')
            time.sleep(1.0)

    # 2) IK 서비스 대기
    node.get_logger().info('IK 서비스 대기 중...')
    if not node._ik_client.wait_for_service(timeout_sec=15.0):
        node.get_logger().error('/compute_ik 서비스 연결 실패!')
        return
    node.get_logger().info('IK 서비스 연결 완료!')

    # 3) 초기 EE 포즈 읽기
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.2)
        if node._update_current_pose():
            break

    # 4) rclpy.spin 별도 스레드
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # 5) 제어 루프 별도 스레드
    control_thread = threading.Thread(
        target=node._control_loop, daemon=True)
    control_thread.start()

    try:
        curses.wrapper(node.run_curses)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.get_logger().info('키보드 IK 텔레오퍼레이션 종료')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
