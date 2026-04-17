"""
예제 14: YZ 평면 원형 경로 - 실시간 IK 제어
=============================================
예제 13과 동일한 원형 경로를 실시간 폐루프 제어로 실행하는 예제.

동작 방식: 실시간 IK + 직접 컨트롤러 스트리밍
----------------------------------------------
매 제어 주기(30Hz)마다:
  1. 원 위의 목표점 계산 (절대 위치, θ를 일정 속도로 전진)
  2. MoveIt IK 서비스로 목표 관절각 계산 (현재 관절을 시드로 사용)
  3. 현재 EE 위치 읽기 (TF, 절대 위치)
  4. 관절 명령을 arm_controller에 직접 발행 (상대적 변위)
  → MoveIt Servo를 사용하지 않음 (Servo의 특이점 감지가 과도하게 보수적)
  → IK 해가 존재하면 동작 가능 (ex13에서 검증 완료)

에러 계산:
  - 타겟: 원 위의 절대 위치 (cx, cy + R*sin(θ), cz + R*cos(θ))
  - 현재: TF에서 읽은 EE 절대 위치
  - 에러: |target - current| (유클리드 거리)

예제 13 vs 예제 14 비교:
  ex13 (Cartesian Path):
    - 오프라인 계획: 73개 웨이포인트로 전체 궤적을 미리 계산
    - 오픈루프 실행: 계획된 궤적을 컨트롤러가 한번에 재생
    - 실행 중 보정 없음

  ex14 (실시간 IK 스트리밍):
    - 실시간 계획: 매 주기(33ms)마다 IK로 목표 관절각 계산
    - 폐루프 실행: 현재 위치를 읽어 목표와의 차이를 모니터링
    - 관절 명령을 직접 스트리밍 (Servo 미사용, 특이점 제한 없음)

실행 방법:
  터미널1: ros2 launch robot_arm_moveit_config demo.launch.xml
  터미널2: ros2 run robot_arm_tutorials ex14_circular_servo --ros-args -p use_sim_time:=true
"""

import math
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState, MoveItErrorCodes

import tf2_ros

from robot_arm_tutorials.utils import MoveGroupHelper, euler_to_quaternion

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

plt.rcParams['font.family'] = 'NanumGothic'
plt.rcParams['axes.unicode_minus'] = False

JOINT_NAMES = ['joint1', 'joint2', 'joint3',
               'joint4', 'joint5', 'joint6']


class CircularServoDemo(Node):

    RADIUS = 0.10           # 10cm
    NUM_LOOPS = 3
    CONTROL_RATE = 30.0     # Hz
    ANGULAR_SPEED = 0.5     # rad/s (원 위 기준점 이동 속도)

    def __init__(self):
        super().__init__('circular_servo_demo')
        self.get_logger().info('=== 예제 14: YZ 원형 경로 (실시간 IK) ===')

        self._cb_group = ReentrantCallbackGroup()

        # 직접 컨트롤러 발행 (Servo 미사용)
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

        # RViz 마커
        self._marker_pub = self.create_publisher(Marker, 'path_markers', 10)
        self._stored_markers = []
        self._marker_timer = self.create_timer(2.0, self._republish_markers)

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # 추적 데이터
        self._track_data = {
            't': [], 'x': [], 'y': [], 'z': [],
            'tgt_y': [], 'tgt_z': [], 'err': [],
        }

        # matplotlib
        self._fig = None
        self._axes = None

        # 백그라운드 스핀
        self._spinning = False
        self._spin_thread = None

        # 그리퍼 방향 (ex13과 동일)
        self._gripper_orientation = euler_to_quaternion(
            0.0, math.pi / 2, 0.0)

    def _joint_state_cb(self, msg):
        joints = {}
        for name, pos in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                joints[name] = pos
        if len(joints) == len(JOINT_NAMES):
            self._current_joints = joints

    # ==============================================================
    #  백그라운드 스핀
    # ==============================================================
    def _start_spin_thread(self):
        self._spinning = True
        self._spin_thread = threading.Thread(
            target=self._spin_loop, daemon=True)
        self._spin_thread.start()

    def _stop_spin_thread(self):
        self._spinning = False
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
            self._spin_thread = None

    def _spin_loop(self):
        while self._spinning and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    # ==============================================================
    #  IK + 직접 컨트롤러 발행
    # ==============================================================
    def _compute_ik(self, target_pose):
        """MoveIt IK 서비스로 목표 관절각 계산 (현재 관절을 시드로)"""
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'manipulator'
        request.ik_request.pose_stamped.header.frame_id = 'base_link'
        request.ik_request.pose_stamped.header.stamp = (
            self.get_clock().now().to_msg())
        request.ik_request.pose_stamped.pose = target_pose
        request.ik_request.timeout.sec = 0
        request.ik_request.timeout.nanosec = int(0.05 * 1e9)

        # 현재 관절을 시드로 사용 (연속적 IK 해 보장)
        if self._current_joints:
            rs = RobotState()
            js = JointState()
            js.name = list(self._current_joints.keys())
            js.position = list(self._current_joints.values())
            rs.joint_state = js
            request.ik_request.robot_state = rs

        future = self._ik_client.call_async(request)

        # 스핀 스레드가 콜백 처리 → future 완료 대기
        deadline = time.time() + 0.1
        while not future.done() and time.time() < deadline:
            time.sleep(0.001)

        if future.done() and future.result() is not None:
            result = future.result()
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                sol = result.solution.joint_state
                return dict(zip(sol.name, sol.position))
        return None

    def _publish_joint_cmd(self, target_joints, duration_s=0.05):
        """관절 위치를 직접 arm_controller에 발행"""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [
            target_joints.get(name, 0.0) for name in JOINT_NAMES]
        sec = int(duration_s)
        nsec = int((duration_s - sec) * 1e9)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nsec
        msg.points = [point]

        self._traj_pub.publish(msg)

    # ==============================================================
    #  RViz Marker
    # ==============================================================
    def _republish_markers(self):
        for m in self._stored_markers:
            m.header.stamp = self.get_clock().now().to_msg()
            self._marker_pub.publish(m)

    def _publish_circle_marker(self, cx, cy, cz, radius):
        stamp = self.get_clock().now().to_msg()

        line = Marker()
        line.header.frame_id = 'base_link'
        line.header.stamp = stamp
        line.ns = 'target_circle'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.004
        line.color = ColorRGBA(r=0.1, g=0.8, b=0.1, a=1.0)
        line.pose.orientation.w = 1.0
        for i in range(49):
            theta = 2.0 * math.pi * i / 48
            line.points.append(Point(
                x=cx,
                y=cy + radius * math.sin(theta),
                z=cz + radius * math.cos(theta)))

        start = Marker()
        start.header.frame_id = 'base_link'
        start.header.stamp = stamp
        start.ns = 'start_point'
        start.id = 0
        start.type = Marker.SPHERE
        start.action = Marker.ADD
        start.scale.x = start.scale.y = start.scale.z = 0.015
        start.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        start.pose.position = Point(x=cx, y=cy, z=cz + radius)
        start.pose.orientation.w = 1.0

        self._marker_pub.publish(line)
        self._marker_pub.publish(start)
        self._stored_markers.extend([line, start])

    def _publish_actual_marker(self):
        d = self._track_data
        if len(d['x']) < 2:
            return

        line = Marker()
        line.header.frame_id = 'base_link'
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = 'actual_line'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.003
        line.color = ColorRGBA(r=1.0, g=0.4, b=0.0, a=0.8)
        line.pose.orientation.w = 1.0
        for x, y, z in zip(d['x'], d['y'], d['z']):
            line.points.append(Point(x=x, y=y, z=z))

        self._marker_pub.publish(line)
        self._stored_markers.append(line)

    # ==============================================================
    #  실시간 matplotlib
    # ==============================================================
    def _create_realtime_figure(self):
        fig, axes = plt.subplots(1, 3, figsize=(16, 5))
        fig.suptitle('YZ Circle IK-Streaming Real-time Monitor',
                     fontsize=13, fontweight='bold')

        ax0 = axes[0]
        ax0.set_title('Y Position')
        ax0.set_xlabel('Time (s)')
        ax0.set_ylabel('Y (cm)')
        ax0.grid(True, alpha=0.3)
        self._line_y_tgt, = ax0.plot(
            [], [], 'g--', lw=1.5, label='Target Y')
        self._line_y_act, = ax0.plot(
            [], [], 'r-', lw=1.2, label='Actual Y')
        ax0.legend(fontsize=8, loc='upper right')

        ax1 = axes[1]
        ax1.set_title('Z Position')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Z (cm)')
        ax1.grid(True, alpha=0.3)
        self._line_z_tgt, = ax1.plot(
            [], [], 'b--', lw=1.5, label='Target Z')
        self._line_z_act, = ax1.plot(
            [], [], 'm-', lw=1.2, label='Actual Z')
        ax1.legend(fontsize=8, loc='upper right')

        ax2 = axes[2]
        ax2.set_title('Tracking Error')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Error (cm)')
        ax2.grid(True, alpha=0.3)
        self._line_err, = ax2.plot(
            [], [], 'k-', lw=1.5, label='|Error|')
        self._text_err = ax2.text(
            0.98, 0.98, '', transform=ax2.transAxes, fontsize=9,
            va='top', ha='right',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        ax2.legend(fontsize=8, loc='upper left')

        plt.tight_layout(rect=[0, 0, 1, 0.92])
        fig.canvas.draw()
        plt.show(block=False)
        plt.pause(0.01)

        self._fig = fig
        self._axes = axes

    def _update_realtime_plots(self):
        d = self._track_data
        n = min(len(d['t']), len(d['tgt_y']),
                len(d['tgt_z']), len(d['err']))
        if n < 2 or self._fig is None:
            return

        t = np.array(d['t'][:n])
        ay = np.array(d['y'][:n]) * 100
        az = np.array(d['z'][:n]) * 100
        tgt_y = np.array(d['tgt_y'][:n]) * 100
        tgt_z = np.array(d['tgt_z'][:n]) * 100
        err = np.array(d['err'][:n]) * 100

        t_max = max(t[-1], 1.0)

        self._line_y_tgt.set_data(t, tgt_y)
        self._line_y_act.set_data(t, ay)
        ax0 = self._axes[0]
        ax0.set_xlim(0, t_max)
        all_y = np.concatenate([tgt_y, ay])
        margin = max(1.0, (all_y.max() - all_y.min()) * 0.15)
        ax0.set_ylim(all_y.min() - margin, all_y.max() + margin)

        self._line_z_tgt.set_data(t, tgt_z)
        self._line_z_act.set_data(t, az)
        ax1 = self._axes[1]
        ax1.set_xlim(0, t_max)
        all_z = np.concatenate([tgt_z, az])
        margin = max(1.0, (all_z.max() - all_z.min()) * 0.15)
        ax1.set_ylim(all_z.min() - margin, all_z.max() + margin)

        self._line_err.set_data(t, err)
        ax2 = self._axes[2]
        ax2.set_xlim(0, t_max)
        if len(err) > 0:
            ax2.set_ylim(0, max(err.max() * 1.3, 0.5))
            self._text_err.set_text(
                f'Max: {err.max():.2f} cm\n'
                f'RMS: {np.sqrt(np.mean(err**2)):.2f} cm')

        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()

    # ==============================================================
    #  메인 실행
    # ==============================================================
    def run(self):
        helper = MoveGroupHelper(self)
        helper.max_velocity_scaling = 0.3
        helper.planning_time = 15.0

        if not helper.wait_for_servers(timeout_sec=30.0):
            return
        if not helper.wait_for_joint_state(timeout_sec=10.0):
            return

        # IK 서비스 대기
        self.get_logger().info('IK 서비스 대기 중...')
        if not self._ik_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error('/compute_ik 서비스를 찾을 수 없습니다!')
            return
        self.get_logger().info('IK 서비스 연결 완료!')

        # TF 대기
        self.get_logger().info('TF 데이터 대기 중...')
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.2)
            try:
                self._tf_buffer.lookup_transform(
                    'base_link', 'gripper_base', rclpy.time.Time())
                self.get_logger().info('TF 연결 완료!')
                break
            except Exception:
                pass

        # ---- matplotlib 실시간 모니터링 창 열기 ----
        plt.ion()
        self._create_realtime_figure()

        # ---- Step 1: 원의 중심 위치로 이동 (ex13과 동일) ----
        cx, cy, cz = 0.35, 0.10, 0.40
        center_pose = Pose()
        center_pose.position = Point(x=cx, y=cy, z=cz)
        center_pose.orientation = self._gripper_orientation

        self.get_logger().info('--- Step 1: 원의 중심 위치로 이동 ---')
        self.get_logger().info(
            f'목표 위치: ({cx:.3f}, {cy:.3f}, {cz:.3f})')
        success = helper.go_to_pose_goal(center_pose)
        if not success:
            self.get_logger().error('원의 중심 위치 이동 실패!')
            return
        time.sleep(1.0)

        # 실제 도달 위치를 원 중심으로 사용
        rclpy.spin_once(self, timeout_sec=0.5)
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                'base_link', 'gripper_base', rclpy.time.Time())
            cx = tf_msg.transform.translation.x
            cy = tf_msg.transform.translation.y
            cz = tf_msg.transform.translation.z
            self.get_logger().info(
                f'실제 EE 위치 (원의 중심): '
                f'({cx:.3f}, {cy:.3f}, {cz:.3f})')
        except Exception:
            pass

        # ---- Step 2: 시작점으로 이동 (MoveGroup) ----
        start_pose = Pose()
        start_pose.position = Point(
            x=cx, y=cy, z=cz + self.RADIUS)
        start_pose.orientation = self._gripper_orientation

        self.get_logger().info('--- Step 2: 원의 시작점으로 이동 ---')
        success = helper.go_to_pose_goal(start_pose)
        if not success:
            self.get_logger().error('시작점 이동 실패!')
            return
        time.sleep(1.0)
        self.get_logger().info('시작점 도달 완료!')

        # 원 중심 재확인
        rclpy.spin_once(self, timeout_sec=0.5)
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                'base_link', 'gripper_base', rclpy.time.Time())
            cy = tf_msg.transform.translation.y
            cz = tf_msg.transform.translation.z - self.RADIUS
            self.get_logger().info(
                f'최종 원 중심: ({cx:.3f}, {cy:.3f}, {cz:.3f})')
        except Exception:
            pass

        # RViz에 목표 원 표시
        self._publish_circle_marker(cx, cy, cz, self.RADIUS)
        self.get_logger().info(
            'RViz: Add > By topic > /path_markers > Marker')

        # 카운트다운
        self.get_logger().info('')
        for i in [3, 2, 1]:
            self.get_logger().info(f'  >>> {i} ...')
            time.sleep(1.0)
        self.get_logger().info('  >>> START!')

        # ---- Step 3: 실시간 IK 스트리밍 제어 루프 ----
        # 백그라운드 스핀 시작 (TF/콜백/IK 응답 처리)
        self._start_spin_thread()

        dt = 1.0 / self.CONTROL_RATE
        theta = 0.0
        total_angle = 2.0 * math.pi * self.NUM_LOOPS
        start_time = time.time()
        plot_counter = 0
        ik_fail_count = 0

        self.get_logger().info(
            f'실시간 IK 제어 시작 '
            f'(ω={self.ANGULAR_SPEED:.1f} rad/s, '
            f'{self.NUM_LOOPS}회)')

        while theta < total_angle:
            loop_start = time.time()

            # 기준점 전진
            theta += self.ANGULAR_SPEED * dt

            # ---- 목표 위치 (절대, 원 위의 점) ----
            tgt_x = cx
            tgt_y = cy + self.RADIUS * math.sin(theta)
            tgt_z = cz + self.RADIUS * math.cos(theta)

            # 목표 포즈 생성
            target_pose = Pose()
            target_pose.position = Point(
                x=tgt_x, y=tgt_y, z=tgt_z)
            target_pose.orientation = self._gripper_orientation

            # ---- IK 계산 + 관절 명령 발행 ----
            ik_result = self._compute_ik(target_pose)
            if ik_result:
                self._publish_joint_cmd(ik_result, dt * 1.5)
                ik_fail_count = 0
            else:
                ik_fail_count += 1
                if ik_fail_count % 30 == 1:
                    self.get_logger().warn(
                        f'IK 실패 (θ={math.degrees(theta):.0f}°)')

            # ---- 현재 위치 (절대, TF) ----
            try:
                tf_msg = self._tf_buffer.lookup_transform(
                    'base_link', 'gripper_base', rclpy.time.Time())
                cur_x = tf_msg.transform.translation.x
                cur_y = tf_msg.transform.translation.y
                cur_z = tf_msg.transform.translation.z
            except Exception:
                time.sleep(dt)
                continue

            # ---- 추적 데이터 기록 ----
            elapsed = time.time() - start_time
            delta_x = tgt_x - cur_x
            delta_y = tgt_y - cur_y
            delta_z = tgt_z - cur_z
            err = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

            self._track_data['t'].append(elapsed)
            self._track_data['x'].append(cur_x)
            self._track_data['y'].append(cur_y)
            self._track_data['z'].append(cur_z)
            self._track_data['tgt_y'].append(tgt_y)
            self._track_data['tgt_z'].append(tgt_z)
            self._track_data['err'].append(err)

            # matplotlib 갱신 (~300ms 주기)
            plot_counter += 1
            if plot_counter % 9 == 0:
                self._update_realtime_plots()
                plt.pause(0.001)

            # 주기 유지
            elapsed_loop = time.time() - loop_start
            sleep_time = dt - elapsed_loop
            if sleep_time > 0:
                time.sleep(sleep_time)

        # ---- 정지 ----
        self._stop_spin_thread()

        # 최종 플롯 갱신
        self._update_realtime_plots()

        self.get_logger().info('원형 경로 실시간 IK 실행 완료!')
        self._publish_actual_marker()

        # ---- 결과 표시 ----
        d = self._track_data
        err = np.array(d['err']) * 100
        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info(
            f'  추적 샘플: {len(d["t"])}개')
        if len(err) > 0:
            self.get_logger().info(
                f'  Max Error: {err.max():.2f} cm  |  '
                f'RMS Error: {np.sqrt(np.mean(err**2)):.2f} cm')
        self.get_logger().info('=' * 50)

        # home 복귀 (MoveGroup)
        self.get_logger().info('--- home 포즈로 복귀 ---')
        helper.go_to_named_target('home')

        self._marker_timer.cancel()

        # 제목 최종 업데이트
        self._fig.suptitle(
            f'YZ Circle IK-Streaming  r={self.RADIUS*100:.0f}cm, '
            f'{self.NUM_LOOPS} loops  '
            f'(ω={self.ANGULAR_SPEED})',
            fontsize=13, fontweight='bold')
        self._fig.canvas.draw()

        self.get_logger().info('=== 예제 14 완료! ===')
        self.get_logger().info(
            'matplotlib 창을 닫으면 프로그램이 종료됩니다.')
        plt.ioff()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = CircularServoDemo()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 종료됨')
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
