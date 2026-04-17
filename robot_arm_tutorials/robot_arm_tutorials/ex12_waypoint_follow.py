"""
예제 12: YAML 웨이포인트 순차 추적
===================================
YAML 파일에서 웨이포인트를 로드하여 순차적으로 방문하는 예제.
Smooth segment로 지정된 구간은 compute_cartesian_path()로 부드럽게 이동.

학습 내용:
- YAML 파일 파싱 (yaml.safe_load)
- ROS2 파라미터로 파일 경로 지정
- go_to_pose_goal() 순차 실행
- compute_cartesian_path() 부드러운 이동
- 경유점 간 대기 시간 적용

실행 방법:
  터미널1: ros2 launch robot_arm_moveit_config demo.launch.xml
  터미널2: ros2 run robot_arm_tutorials ex12_waypoint_follow --ros-args -p use_sim_time:=true

  # 커스텀 웨이포인트 파일 사용:
  ros2 run robot_arm_tutorials ex12_waypoint_follow --ros-args \\
    -p use_sim_time:=true -p waypoint_file:=/path/to/your_waypoints.yaml
"""

import math
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState, MoveItErrorCodes
from ament_index_python.packages import get_package_share_directory
from robot_arm_tutorials.utils import MoveGroupHelper, make_pose

JOINT_NAMES = ['joint1', 'joint2', 'joint3',
               'joint4', 'joint5', 'joint6']


class WaypointFollowDemo(Node):
    def __init__(self):
        super().__init__('waypoint_follow_demo')
        self.get_logger().info('=== 예제 12: YAML 웨이포인트 순차 추적 ===')

        # 기본 웨이포인트 파일 경로
        default_path = ''
        try:
            pkg_dir = get_package_share_directory('robot_arm_tutorials')
            default_path = pkg_dir + '/config/sample_waypoints.yaml'
        except Exception:
            pass

        self.declare_parameter('waypoint_file', default_path)

        self._cb_group = ReentrantCallbackGroup()

        # IK 서비스
        self._ik_client = self.create_client(
            GetPositionIK, '/compute_ik',
            callback_group=self._cb_group)

        # RViz 마커
        self._marker_pub = self.create_publisher(
            MarkerArray, 'waypoint_markers', 10)
        self._marker_timer = None

    def _load_waypoints(self, filepath):
        """YAML 파일에서 웨이포인트 로드"""
        self.get_logger().info(f'웨이포인트 파일 로드: {filepath}')
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)

        waypoints = []
        for wp_data in data.get('waypoints', []):
            pos = wp_data['position']
            ori = wp_data.get('orientation', {'roll': math.pi, 'pitch': 0.0, 'yaw': 0.0})
            pause = wp_data.get('pause_sec', 0.5)
            name = wp_data.get('name', f'waypoint_{len(waypoints)}')

            pose = make_pose(
                pos['x'], pos['y'], pos['z'],
                ori.get('roll', 0.0), ori.get('pitch', 0.0), ori.get('yaw', 0.0),
            )
            waypoints.append({
                'name': name,
                'pose': pose,
                'pause_sec': pause,
            })

        smooth_segments = []
        for seg in data.get('smooth_segments', []):
            smooth_segments.append({
                'name': seg.get('name', ''),
                'start_index': seg['start_index'],
                'end_index': seg['end_index'],
            })

        return waypoints, smooth_segments

    def _publish_waypoint_markers(self, waypoints, current_idx=-1):
        """웨이포인트를 RViz에 마커로 표시"""
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, wp in enumerate(waypoints):
            # 구체 마커
            m = Marker()
            m.header.frame_id = 'base_link'
            m.header.stamp = stamp
            m.ns = 'waypoints'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose = wp['pose']
            m.scale.x = m.scale.y = m.scale.z = 0.025
            if i == current_idx:
                m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
                m.scale.x = m.scale.y = m.scale.z = 0.035
            elif i < current_idx:
                m.color = ColorRGBA(r=0.3, g=0.3, b=0.3, a=0.5)
            else:
                m.color = ColorRGBA(r=0.1, g=0.8, b=0.1, a=1.0)
            ma.markers.append(m)

            # 텍스트 라벨
            t = Marker()
            t.header.frame_id = 'base_link'
            t.header.stamp = stamp
            t.ns = 'waypoint_labels'
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position = Point(
                x=wp['pose'].position.x,
                y=wp['pose'].position.y,
                z=wp['pose'].position.z + 0.04)
            t.pose.orientation.w = 1.0
            t.scale.z = 0.025
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            t.text = str(i + 1)
            ma.markers.append(t)

        # 웨이포인트 연결 라인
        line = Marker()
        line.header.frame_id = 'base_link'
        line.header.stamp = stamp
        line.ns = 'waypoint_path'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.003
        line.color = ColorRGBA(r=0.5, g=0.5, b=1.0, a=0.6)
        line.pose.orientation.w = 1.0
        for wp in waypoints:
            line.points.append(wp['pose'].position)
        ma.markers.append(line)

        self._marker_pub.publish(ma)

    def _go_to_pose_via_ik(self, target_pose, helper):
        """IK로 현재 관절 근처의 충돌 없는 해를 구한 뒤 joint goal로 이동.

        go_to_pose_goal 대비 장점:
        - MoveIt IK 서비스를 직접 호출하여 현재 관절을 시드로 전달
          → 현재 자세에서 가장 가까운 IK 해를 보장
        - avoid_collisions=True로 자기충돌 없는 해만 반환
        - joint goal로 계획 → 관절 공간에서 경로 탐색이 단순해짐
        - IK 실패 시 pose goal로 자동 폴백
        """
        joints = helper.get_current_joint_values()
        if not joints:
            self.get_logger().warn('현재 관절 상태를 읽을 수 없어 pose goal로 대체')
            return helper.go_to_pose_goal(target_pose)

        request = GetPositionIK.Request()
        request.ik_request.group_name = 'manipulator'
        request.ik_request.pose_stamped.header.frame_id = 'base_link'
        request.ik_request.pose_stamped.header.stamp = (
            self.get_clock().now().to_msg())
        request.ik_request.pose_stamped.pose = target_pose
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 0
        request.ik_request.timeout.nanosec = int(0.5 * 1e9)

        # 현재 관절을 시드로 사용 → IK 해가 현재 자세와 가까움
        rs = RobotState()
        js = JointState()
        js.name = list(joints.keys())
        js.position = list(joints.values())
        rs.joint_state = js
        request.ik_request.robot_state = rs

        future = self._ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if (future.result() is not None and
                future.result().error_code.val == MoveItErrorCodes.SUCCESS):
            sol = future.result().solution.joint_state
            goal = {}
            for name, pos in zip(sol.name, sol.position):
                if name in JOINT_NAMES:
                    goal[name] = pos
            self.get_logger().info('  IK 해 계산 완료 (충돌 없음) → joint goal로 이동')
            return helper.go_to_joint_goal(goal)
        else:
            self.get_logger().warn('  IK 실패 (충돌 또는 도달 불가) → pose goal로 대체')
            return helper.go_to_pose_goal(target_pose)

    def _is_in_smooth_segment(self, index, smooth_segments):
        """주어진 인덱스가 smooth segment 범위 안에 있는지 확인"""
        for seg in smooth_segments:
            if seg['start_index'] <= index <= seg['end_index']:
                return seg
            # start_index 직전 인덱스인 경우 (시작점 진입)
        return None

    def run(self):
        helper = MoveGroupHelper(self)
        helper.max_velocity_scaling = 0.3
        helper.max_acceleration_scaling = 0.3
        helper.planning_time = 15.0
        helper.num_planning_attempts = 10

        if not helper.wait_for_servers(timeout_sec=30.0):
            return
        if not helper.wait_for_joint_state(timeout_sec=10.0):
            return

        # IK 서비스 대기
        self.get_logger().info('IK 서비스 대기 중...')
        if not self._ik_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().warn('IK 서비스 없음, pose goal만 사용합니다.')
        else:
            self.get_logger().info('IK 서비스 연결 완료!')

        # 웨이포인트 파일 로드
        filepath = self.get_parameter('waypoint_file').get_parameter_value().string_value
        if not filepath:
            self.get_logger().error('waypoint_file 파라미터가 지정되지 않았습니다!')
            return

        try:
            waypoints, smooth_segments = self._load_waypoints(filepath)
        except Exception as e:
            self.get_logger().error(f'웨이포인트 파일 로드 실패: {e}')
            return

        self.get_logger().info(f'총 {len(waypoints)}개 웨이포인트 로드 완료')
        self.get_logger().info(f'Smooth segment: {len(smooth_segments)}개')

        # RViz에 모든 웨이포인트 표시
        self._publish_waypoint_markers(waypoints)
        self._marker_timer = self.create_timer(
            2.0, lambda: self._publish_waypoint_markers(
                waypoints, self._current_wp_idx))
        self._current_wp_idx = -1
        self.get_logger().info(
            'RViz: Add > By topic > /waypoint_markers > MarkerArray')

        # 초기화
        self.get_logger().info('--- 초기화: ready 포즈로 이동 ---')
        helper.go_to_named_target('ready')
        time.sleep(1.0)

        # =============================================
        # 1부: 개별 웨이포인트 순차 이동 (go_to_pose_goal)
        # =============================================
        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('1부: 개별 웨이포인트 순차 이동')
        self.get_logger().info('=' * 50)

        for i, wp in enumerate(waypoints):
            self._current_wp_idx = i
            self.get_logger().info(
                f'[{i + 1}/{len(waypoints)}] 이동: "{wp["name"]}" '
                f'(x={wp["pose"].position.x:.2f}, '
                f'y={wp["pose"].position.y:.2f}, '
                f'z={wp["pose"].position.z:.2f})'
            )
            success = self._go_to_pose_via_ik(wp['pose'], helper)
            if success:
                self.get_logger().info(f'  -> "{wp["name"]}" 도달 완료')
            else:
                self.get_logger().warn(f'  -> "{wp["name"]}" 이동 실패, 다음으로 진행')

            if wp['pause_sec'] > 0:
                self.get_logger().info(f'  -> {wp["pause_sec"]}초 대기')
                time.sleep(wp['pause_sec'])

        time.sleep(1.0)

        # =============================================
        # 2부: Smooth segment (Cartesian 경로)
        # =============================================
        if smooth_segments:
            self.get_logger().info('')
            self.get_logger().info('=' * 50)
            self.get_logger().info('2부: Smooth Segment (Cartesian 직선 보간)')
            self.get_logger().info('=' * 50)

            for seg in smooth_segments:
                start_idx = seg['start_index']
                end_idx = seg['end_index']
                seg_name = seg['name']

                if start_idx >= len(waypoints) or end_idx >= len(waypoints):
                    self.get_logger().warn(
                        f'Segment "{seg_name}" 인덱스 범위 초과, 건너뜁니다.'
                    )
                    continue

                self.get_logger().info(
                    f'Segment "{seg_name}": '
                    f'웨이포인트 [{start_idx}] "{waypoints[start_idx]["name"]}" → '
                    f'[{end_idx}] "{waypoints[end_idx]["name"]}"'
                )

                # 시작 웨이포인트로 이동
                self.get_logger().info(f'  시작점으로 이동...')
                success = self._go_to_pose_via_ik(
                    waypoints[start_idx]['pose'], helper)
                if not success:
                    self.get_logger().warn('  시작점 이동 실패, 건너뜁니다.')
                    continue
                time.sleep(0.5)

                # Cartesian 경로 웨이포인트 수집
                cart_waypoints = []
                for j in range(start_idx + 1, end_idx + 1):
                    cart_waypoints.append(waypoints[j]['pose'])

                if not cart_waypoints:
                    self.get_logger().warn('  Cartesian 웨이포인트 없음, 건너뜁니다.')
                    continue

                self.get_logger().info(f'  Cartesian 경유점 수: {len(cart_waypoints)}개')

                traj, fraction = helper.compute_cartesian_path(
                    cart_waypoints, max_step=0.02,
                )
                self.get_logger().info(f'  달성률: {fraction * 100:.1f}%')

                if traj is not None and fraction > 0.3:
                    success = helper.execute_trajectory(traj)
                    if success:
                        self.get_logger().info(f'  Segment "{seg_name}" 실행 완료!')
                    else:
                        self.get_logger().warn(f'  Segment "{seg_name}" 실행 실패')
                else:
                    self.get_logger().warn(
                        f'  달성률 부족 ({fraction * 100:.1f}%), 실행하지 않습니다.'
                    )
                time.sleep(1.0)

        # 최종 복귀
        self.get_logger().info('')
        self.get_logger().info('--- home 포즈로 복귀 ---')
        helper.go_to_named_target('home')
        if self._marker_timer:
            self._marker_timer.cancel()
        self.get_logger().info('=== 예제 12 완료! ===')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowDemo()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
