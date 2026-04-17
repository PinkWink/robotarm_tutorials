"""
예제 13: YZ 평면 원형 경로 (실시간 시각화)
========================================
로봇 끝단이 YZ 평면에서 원을 3회 반복 그리는 예제.

동작 방식: Cartesian Path (웨이포인트 기반, 오프라인 계획)
-------------------------------------------------------
1. 웨이포인트 생성 (73개)
   - 원 위의 균등 분할점 (24점/회 × 3회 + 1)

2. compute_cartesian_path(waypoints, max_step=0.02)
   - MoveIt 서비스가 웨이포인트 사이를 2cm 간격으로 직선 보간
   - 각 보간점에서 IK(역기구학)를 풀어 관절각 산출
   - 연속 관절 궤적(JointTrajectory) 생성
   - 속도/가속도 프로파일 적용 (time parameterization)

3. execute_trajectory(traj)
   - 완성된 관절 궤적을 컨트롤러에 한 번에 전달
   - 컨트롤러가 시간표대로 관절을 구동

특징:
  - 계획: 실행 전에 전체 경로를 미리 계산 (오프라인)
  - 실행: 계산된 궤적을 오픈루프로 재생 (실시간 보정 없음)
  - 보간: 웨이포인트 사이를 직선으로 잇고, IK로 관절각 변환
  - 속도: MoveIt이 관절 한계에 맞춰 사다리꼴/S-curve 가감속 자동 적용

요약:
  "원 위의 73개 점을 찍어놓고 → MoveIt이 그 점들을 잇는 관절 궤적을
   한번에 계산 → 로봇이 그대로 재생" 하는 방식.
  실시간으로 위치를 추적/보정하지 않으므로, 미리 만든 궤적을 그대로
  따라가며 Target-Actual 간 시간 지연이 발생할 수 있음.

과정:
  1. 원의 중심 위치로 이동 (go_to_pose_goal)
  2. 원의 시작점으로 이동 (go_to_pose_goal)
  3. 카운트다운 3, 2, 1 → 시작
  4. YZ 평면 원 3회 반복 실행 (실시간 모니터링)

시각화:
  - RViz2: /path_markers 토픽 (Add > By topic > Marker)
  - matplotlib: Y/Z 지령·현재값, 추적 에러 (실시간)

실행 방법:
  터미널1: ros2 launch robot_arm_moveit_config demo.launch.xml
  터미널2: ros2 run robot_arm_tutorials ex13_circular_path --ros-args -p use_sim_time:=true
"""

import math
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

import tf2_ros
import tf_transformations

from robot_arm_tutorials.utils import MoveGroupHelper, euler_to_quaternion

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

plt.rcParams['font.family'] = 'NanumGothic'
plt.rcParams['axes.unicode_minus'] = False


class CircularPathDemo(Node):

    RADIUS = 0.10        # 10cm
    NUM_POINTS = 24      # waypoints per loop
    NUM_LOOPS = 3

    def __init__(self):
        super().__init__('circular_path_demo')
        self.get_logger().info('=== 예제 13: YZ 평면 원형 경로 ===')

        self._marker_pub = self.create_publisher(Marker, 'path_markers', 10)
        self._stored_markers = []
        self._marker_timer = self.create_timer(2.0, self._republish_markers)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._tracking = False
        self._track_start_time = 0.0
        self._track_data = {
            't': [], 'x': [], 'y': [], 'z': [],
            'tgt_y': [], 'tgt_z': [], 'err': [],
        }
        self._tracking_thread = None
        self._target_info = None

        # matplotlib 실시간 플롯
        self._fig = None
        self._axes = None

    # ==============================================================
    #  RViz Marker
    # ==============================================================
    def _republish_markers(self):
        for m in self._stored_markers:
            m.header.stamp = self.get_clock().now().to_msg()
            self._marker_pub.publish(m)

    def _publish_target_marker(self, waypoints):
        stamp = self.get_clock().now().to_msg()

        line = Marker()
        line.header.frame_id = 'base_link'
        line.header.stamp = stamp
        line.ns = 'target_line'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.004
        line.color = ColorRGBA(r=0.1, g=0.8, b=0.1, a=1.0)
        line.pose.orientation.w = 1.0
        for wp in waypoints:
            line.points.append(Point(
                x=wp.position.x, y=wp.position.y, z=wp.position.z))

        dots = Marker()
        dots.header.frame_id = 'base_link'
        dots.header.stamp = stamp
        dots.ns = 'target_dots'
        dots.id = 0
        dots.type = Marker.SPHERE_LIST
        dots.action = Marker.ADD
        dots.scale.x = dots.scale.y = dots.scale.z = 0.006
        dots.color = ColorRGBA(r=0.1, g=0.8, b=0.1, a=0.6)
        dots.pose.orientation.w = 1.0
        dots.points = list(line.points)

        start = Marker()
        start.header.frame_id = 'base_link'
        start.header.stamp = stamp
        start.ns = 'start_point'
        start.id = 0
        start.type = Marker.SPHERE
        start.action = Marker.ADD
        start.scale.x = start.scale.y = start.scale.z = 0.015
        start.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        start.pose = waypoints[0]

        self._marker_pub.publish(line)
        self._marker_pub.publish(dots)
        self._marker_pub.publish(start)
        self._stored_markers.extend([line, dots, start])

    def _publish_actual_marker(self):
        xs = self._track_data['x']
        ys = self._track_data['y']
        zs = self._track_data['z']
        if len(xs) < 2:
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
        for x, y, z in zip(xs, ys, zs):
            line.points.append(Point(x=x, y=y, z=z))

        self._marker_pub.publish(line)
        self._stored_markers.append(line)

    # ==============================================================
    #  EE 추적 (별도 스레드, 20Hz)
    # ==============================================================
    def _start_tracking(self):
        self._track_data = {
            't': [], 'x': [], 'y': [], 'z': [],
            'tgt_y': [], 'tgt_z': [], 'err': [],
        }
        self._tracking = True
        self._track_start_time = time.time()
        self._tracking_thread = threading.Thread(
            target=self._tracking_loop, daemon=True)
        self._tracking_thread.start()

    def _stop_tracking(self):
        self._tracking = False
        if self._tracking_thread is not None:
            self._tracking_thread.join(timeout=2.0)
            self._tracking_thread = None

    def _tracking_loop(self):
        while self._tracking:
            try:
                tf = self._tf_buffer.lookup_transform(
                    'base_link', 'gripper_base', rclpy.time.Time())
                elapsed = time.time() - self._track_start_time
                ay = tf.transform.translation.y
                az = tf.transform.translation.z

                self._track_data['t'].append(elapsed)
                self._track_data['x'].append(tf.transform.translation.x)
                self._track_data['y'].append(ay)
                self._track_data['z'].append(az)

                # 지령값 계산 (기하학적: 원 위의 최근접점 기준)
                if self._target_info:
                    info = self._target_info
                    dy = ay - info['cy']
                    dz = az - info['cz']
                    dist = math.sqrt(dy**2 + dz**2)
                    # 원 위의 최근접점
                    if dist > 1e-6:
                        tgt_y = info['cy'] + info['radius'] * dy / dist
                        tgt_z = info['cz'] + info['radius'] * dz / dist
                    else:
                        tgt_y = info['cy'] + info['radius']
                        tgt_z = info['cz']
                    # 반경 에러 = |실제 거리 - 목표 반경|
                    err = abs(dist - info['radius'])
                else:
                    tgt_y = ay
                    tgt_z = az
                    err = 0.0

                self._track_data['tgt_y'].append(tgt_y)
                self._track_data['tgt_z'].append(tgt_z)
                self._track_data['err'].append(err)

            except Exception:
                pass
            time.sleep(0.05)

    # ==============================================================
    #  YZ 원 웨이포인트 생성
    # ==============================================================
    def _generate_circle_yz(self, cx, cy, cz, radius,
                            pts_per_loop, num_loops):
        """YZ 평면 원형 웨이포인트 (끝단이 +X 방향 고정)

        각 theta에서:
          위치: (cx, cy + r*sin(theta), cz + r*cos(theta))
          방향: 그리퍼가 항상 +X 방향(원의 회전축)을 향함 (고정)
        """
        waypoints = []
        total = pts_per_loop * num_loops
        # 고정 자세: pitch=+pi/2 → 그리퍼가 +X 방향
        orientation = euler_to_quaternion(0.0, math.pi / 2, 0.0)
        for i in range(total + 1):
            theta = 2.0 * math.pi * i / pts_per_loop
            wp = Pose()
            wp.position = Point(
                x=cx,
                y=cy + radius * math.sin(theta),
                z=cz + radius * math.cos(theta))
            wp.orientation = orientation
            waypoints.append(wp)
        return waypoints

    # ==============================================================
    #  실시간 matplotlib
    # ==============================================================
    def _create_realtime_figure(self):
        """실시간 모니터링 figure 생성"""
        fig, axes = plt.subplots(1, 3, figsize=(16, 5))
        fig.suptitle('YZ Circle Real-time Monitor',
                     fontsize=13, fontweight='bold')

        # Y 지령 vs 현재
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

        # Z 지령 vs 현재
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

        # 에러
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
        """추적 데이터로 실시간 플롯 갱신"""
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

        # Y plot
        self._line_y_tgt.set_data(t, tgt_y)
        self._line_y_act.set_data(t, ay)
        ax0 = self._axes[0]
        ax0.set_xlim(0, t_max)
        all_y = np.concatenate([tgt_y, ay])
        margin = max(1.0, (all_y.max() - all_y.min()) * 0.15)
        ax0.set_ylim(all_y.min() - margin, all_y.max() + margin)

        # Z plot
        self._line_z_tgt.set_data(t, tgt_z)
        self._line_z_act.set_data(t, az)
        ax1 = self._axes[1]
        ax1.set_xlim(0, t_max)
        all_z = np.concatenate([tgt_z, az])
        margin = max(1.0, (all_z.max() - all_z.min()) * 0.15)
        ax1.set_ylim(all_z.min() - margin, all_z.max() + margin)

        # Error plot
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
        helper.max_velocity_scaling = 0.15
        helper.max_acceleration_scaling = 0.15
        helper.planning_time = 15.0

        if not helper.wait_for_servers(timeout_sec=30.0):
            return
        if not helper.wait_for_joint_state(timeout_sec=10.0):
            return

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

        # ---- Step 1: 원의 중심 위치로 이동 ----
        cx, cy, cz = 0.35, 0.10, 0.40
        center_pose = Pose()
        center_pose.position = Point(x=cx, y=cy, z=cz)
        center_pose.orientation = euler_to_quaternion(
            0.0, math.pi / 2, 0.0)

        self.get_logger().info('--- Step 1: 원의 중심 위치로 이동 ---')
        self.get_logger().info(
            f'목표 위치: ({cx:.3f}, {cy:.3f}, {cz:.3f})')
        success = helper.go_to_pose_goal(center_pose)
        if not success:
            self.get_logger().error('원의 중심 위치 이동 실패!')
            return
        time.sleep(1.0)

        # 실제 도달 위치 확인
        rclpy.spin_once(self, timeout_sec=0.5)
        try:
            tf = self._tf_buffer.lookup_transform(
                'base_link', 'gripper_base', rclpy.time.Time())
            cx = tf.transform.translation.x
            cy = tf.transform.translation.y
            cz = tf.transform.translation.z
            self.get_logger().info(
                f'실제 EE 위치 (원의 중심): '
                f'({cx:.3f}, {cy:.3f}, {cz:.3f})')
        except Exception:
            pass

        # ---- 웨이포인트 생성 ----
        waypoints = self._generate_circle_yz(
            cx, cy, cz, self.RADIUS,
            self.NUM_POINTS, self.NUM_LOOPS)
        self.get_logger().info(
            f'YZ 원 웨이포인트: {len(waypoints)}개 '
            f'(r={self.RADIUS*100:.0f}cm, {self.NUM_LOOPS}회)')

        # RViz에 목표 경로 표시
        self._publish_target_marker(waypoints)
        self.get_logger().info(
            'RViz: Add > By topic > /path_markers > Marker')

        # ---- Step 2: 시작점으로 이동 ----
        self.get_logger().info('')
        self.get_logger().info('--- Step 2: 원의 시작점으로 이동 ---')
        success = helper.go_to_pose_goal(waypoints[0])
        if not success:
            self.get_logger().error('시작점 이동 실패!')
            return
        time.sleep(1.0)
        self.get_logger().info('시작점 도달 완료!')

        # ---- Step 3: 카운트다운 ----
        self.get_logger().info('')
        for i in [3, 2, 1]:
            self.get_logger().info(f'  >>> {i} ...')
            time.sleep(1.0)
        self.get_logger().info('  >>> START!')

        # ---- Step 4: Cartesian 경로 계획 및 실행 ----
        traj, fraction = helper.compute_cartesian_path(
            waypoints, max_step=0.02)
        self.get_logger().info(
            f'경로 달성률: {fraction * 100:.1f}%')

        if traj is None or fraction < 0.1:
            self.get_logger().error(
                f'경로 계획 실패 (달성률 {fraction*100:.1f}%)')
            return

        # 궤적 총 시간 추출
        pts = traj.joint_trajectory.points
        total_traj_time = (pts[-1].time_from_start.sec +
                           pts[-1].time_from_start.nanosec * 1e-9)
        self.get_logger().info(f'궤적 시간: {total_traj_time:.1f}초')

        # 지령값 계산을 위한 정보 저장
        self._target_info = {
            'cx': cx, 'cy': cy, 'cz': cz,
            'radius': self.RADIUS,
            'num_loops': self.NUM_LOOPS,
            'total_time': total_traj_time,
        }

        # 추적 시작 → 실행 (별도 스레드) → 실시간 플롯
        self._start_tracking()
        exec_thread = threading.Thread(
            target=helper.execute_trajectory,
            args=(traj,), daemon=True)
        exec_thread.start()

        while exec_thread.is_alive():
            self._update_realtime_plots()
            plt.pause(0.1)

        exec_thread.join()
        self._stop_tracking()

        # 최종 갱신
        self._update_realtime_plots()

        self.get_logger().info('원형 경로 실행 완료!')
        self._publish_actual_marker()

        # ---- 결과 표시 ----
        d = self._track_data
        err = np.array(d['err']) * 100
        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info(
            f'  달성률: {fraction*100:.1f}%  |  '
            f'추적 샘플: {len(d["t"])}개')
        if len(err) > 0:
            self.get_logger().info(
                f'  Max Error: {err.max():.2f} cm  |  '
                f'RMS Error: {np.sqrt(np.mean(err**2)):.2f} cm')
        self.get_logger().info('=' * 50)

        # home 복귀
        self.get_logger().info('--- home 포즈로 복귀 ---')
        helper.go_to_named_target('home')

        self._marker_timer.cancel()

        # 제목 최종 업데이트
        self._fig.suptitle(
            f'YZ Circle  r={self.RADIUS*100:.0f}cm, '
            f'{self.NUM_LOOPS} loops  '
            f'(달성률: {fraction*100:.1f}%)',
            fontsize=13, fontweight='bold')
        self._fig.canvas.draw()

        self.get_logger().info('=== 예제 13 완료! ===')
        self.get_logger().info(
            'matplotlib 창을 닫으면 프로그램이 종료됩니다.')
        plt.ioff()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = CircularPathDemo()

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
