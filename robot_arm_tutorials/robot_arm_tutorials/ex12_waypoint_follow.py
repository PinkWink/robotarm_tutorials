"""
예제 12: YAML 웨이포인트 순차 추적
===================================
YAML 파일에서 웨이포인트를 로드하여 순차적으로 방문하는 예제.
ex04/ex05처럼 사용자 트리거(/next_step)로 단계별 진행하며,
계획된 끝단 경로를 FK로 계산해 RViz2에 시각화한다.

학습 내용:
- YAML 파일 파싱 (yaml.safe_load)
- /next_step 토픽으로 단계별 진행 제어
- compute_fk로 계획 궤적의 끝단 경로 시각화
- 1부: 개별 웨이포인트 이동 (IK 시드 + plan_to_joint_goal, 점마다 트리거)
- 2부: 전체 일주 (plan_via_ik 자동 진행, 트리거 1회 + 점마다 0.5s 정지)
       각 segment의 trajectory를 FK로 변환해 끝단 경로를 누적 표시
- 웨이포인트 상태(대기/진행/성공/실패/지남) 색상 표현

실행 방법:
  터미널1: ros2 launch robot_arm_moveit_config demo.launch.xml
  터미널2: ros2 run robot_arm_tutorials ex12_waypoint_follow --ros-args -p use_sim_time:=true
  터미널3: ros2 topic pub --once /next_step std_msgs/msg/Empty '{}'

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
from geometry_msgs.msg import Pose, Point, Vector3
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Empty
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import RobotState, MoveItErrorCodes
from ament_index_python.packages import get_package_share_directory
from robot_arm_tutorials.utils import MoveGroupHelper, make_pose

JOINT_NAMES = ['joint1', 'joint2', 'joint3',
               'joint4', 'joint5', 'joint6']


class WaypointFollowDemo(Node):
    # 웨이포인트 상태 색상
    COLOR_PENDING = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.85)   # 노랑: 대기
    COLOR_ACTIVE = ColorRGBA(r=0.2, g=0.5, b=1.0, a=0.95)    # 파랑: 진행 중
    COLOR_SUCCESS = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.85)   # 초록: 성공
    COLOR_FAIL = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.85)      # 빨강: 실패
    COLOR_PASSED = ColorRGBA(r=0.4, g=0.4, b=0.4, a=0.5)     # 회색: 지나감
    COLOR_TEXT = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
    COLOR_LINK = ColorRGBA(r=0.5, g=0.5, b=1.0, a=0.5)       # 웨이포인트 연결선
    COLOR_EE_PATH = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.95)   # 주황: 끝단 실제 경로
    COLOR_SMOOTH_PATH = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.95)  # 시안: smooth 구간

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

        # IK / FK 서비스
        self._ik_client = self.create_client(
            GetPositionIK, '/compute_ik',
            callback_group=self._cb_group)
        self._fk_client = self.create_client(
            GetPositionFK, '/compute_fk',
            callback_group=self._cb_group)

        # RViz 마커
        self._marker_pub = self.create_publisher(
            MarkerArray, 'waypoint_markers', 10)

        # 사용자 트리거
        self._trigger_received = False
        self._trigger_sub = self.create_subscription(
            Empty, '/next_step', self._trigger_cb, 10)

        # 상태 관리
        self._wp_status = {}  # idx -> 'pending'/'active'/'success'/'fail'/'passed'
        self._waypoints = []

    # ----------------------------------------------------------
    #  트리거 / 대기
    # ----------------------------------------------------------
    def _trigger_cb(self, _msg):
        self._trigger_received = True

    def _wait_for_trigger(self, prompt):
        """/next_step 토픽 메시지가 올 때까지 블로킹 대기"""
        self.get_logger().info(prompt)
        self._trigger_received = False
        while rclpy.ok() and not self._trigger_received:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _spin_sleep(self, duration):
        """콜백 처리하면서 대기 (조인트 상태 갱신 유지)"""
        end_time = time.time() + duration
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

    # ----------------------------------------------------------
    #  YAML 로드
    # ----------------------------------------------------------
    def _load_waypoints(self, filepath):
        """YAML 파일에서 웨이포인트 로드 (smooth_segments 필드는 무시)"""
        self.get_logger().info(f'웨이포인트 파일 로드: {filepath}')
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)

        waypoints = []
        for wp_data in data.get('waypoints', []):
            pos = wp_data['position']
            ori = wp_data.get('orientation',
                              {'roll': math.pi, 'pitch': 0.0, 'yaw': 0.0})
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

        return waypoints

    # ----------------------------------------------------------
    #  FK 기반 끝단 경로 계산 / 시각화
    # ----------------------------------------------------------
    def _compute_trajectory_ee_path(self, trajectory, timeout_sec=5.0):
        """RobotTrajectory의 각 waypoint에 FK 적용해 gripper_base 위치 리스트 반환"""
        if not self._fk_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn('compute_fk 서비스 연결 실패 — 경로 시각화 생략')
            return []

        joint_traj = trajectory.joint_trajectory
        ee_points = []

        for point in joint_traj.points:
            request = GetPositionFK.Request()
            request.header.frame_id = 'base_link'
            request.fk_link_names = ['gripper_base']
            rs = RobotState()
            rs.joint_state.name = list(joint_traj.joint_names)
            rs.joint_state.position = list(point.positions)
            request.robot_state = rs

            future = self._fk_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()

            if (response is not None
                    and response.error_code.val == MoveItErrorCodes.SUCCESS
                    and response.pose_stamped):
                p = response.pose_stamped[0].pose.position
                ee_points.append((p.x, p.y, p.z))

        return ee_points

    def _publish_ee_path_marker(self, ee_points, color=None, ns='ee_path_line'):
        """끝단 경로를 LINE_STRIP 마커로 발행 (같은 ns/id로 매번 교체)"""
        if not ee_points:
            return

        if color is None:
            color = self.COLOR_EE_PATH

        stamp = self.get_clock().now().to_msg()
        line = Marker()
        line.header.frame_id = 'base_link'
        line.header.stamp = stamp
        line.ns = ns
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.006
        line.color = color
        line.points = [Point(x=p[0], y=p[1], z=p[2]) for p in ee_points]

        arr = MarkerArray()
        arr.markers.append(line)
        self._marker_pub.publish(arr)

    # ----------------------------------------------------------
    #  Plan / Execute 헬퍼
    # ----------------------------------------------------------
    def _plan_via_ik(self, helper, target_pose):
        """IK로 현재 관절 시드 기반 해를 구해 plan_to_joint_goal로 (성공, 궤적) 반환.

        IK 실패 시 plan_to_pose_goal로 폴백.
        """
        joints = helper.get_current_joint_values()
        if not joints or not self._ik_client.wait_for_service(timeout_sec=2.0):
            return helper.plan_to_pose_goal(target_pose)

        request = GetPositionIK.Request()
        request.ik_request.group_name = 'manipulator'
        request.ik_request.pose_stamped.header.frame_id = 'base_link'
        request.ik_request.pose_stamped.header.stamp = (
            self.get_clock().now().to_msg())
        request.ik_request.pose_stamped.pose = target_pose
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 0
        request.ik_request.timeout.nanosec = int(0.5 * 1e9)

        rs = RobotState()
        js = JointState()
        js.name = list(joints.keys())
        js.position = list(joints.values())
        rs.joint_state = js
        request.ik_request.robot_state = rs

        future = self._ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        result = future.result()
        if (result is None or
                result.error_code.val != MoveItErrorCodes.SUCCESS):
            self.get_logger().warn('  IK 실패 → plan_to_pose_goal 폴백')
            return helper.plan_to_pose_goal(target_pose)

        sol = result.solution.joint_state
        goal = {}
        for name, pos in zip(sol.name, sol.position):
            if name in JOINT_NAMES:
                goal[name] = pos
        self.get_logger().info('  IK 해 획득 → joint goal로 계획')
        return helper.plan_to_joint_goal(goal)

    def _plan_and_execute_with_viz(self, helper, plan_result,
                                    path_color=None, path_ns='ee_path_line'):
        """(성공, 궤적) → FK로 EE 경로 표시 후 실행"""
        success, trajectory = plan_result
        if not success or trajectory is None:
            return False

        ee_points = self._compute_trajectory_ee_path(trajectory)
        if ee_points:
            self.get_logger().info(
                f'  계획된 끝단 경로: {len(ee_points)}개 점 — RViz 표시'
            )
            self._publish_ee_path_marker(
                ee_points, color=path_color, ns=path_ns)

        return helper.execute_trajectory(trajectory)

    # ----------------------------------------------------------
    #  웨이포인트 마커 (상태별 색상)
    # ----------------------------------------------------------
    def _color_for_status(self, status):
        return {
            'pending': self.COLOR_PENDING,
            'active': self.COLOR_ACTIVE,
            'success': self.COLOR_SUCCESS,
            'fail': self.COLOR_FAIL,
            'passed': self.COLOR_PASSED,
        }.get(status, self.COLOR_PENDING)

    def _set_status(self, idx, status):
        """idx번 웨이포인트 상태 갱신 후 마커 다시 발행"""
        self._wp_status[idx] = status
        self._publish_waypoint_markers()

    def _publish_waypoint_markers(self):
        """모든 웨이포인트를 상태에 맞는 색으로 표시 (구체+라벨+연결선)"""
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, wp in enumerate(self._waypoints):
            status = self._wp_status.get(i, 'pending')
            color = self._color_for_status(status)
            scale = 0.035 if status == 'active' else 0.025

            sphere = Marker()
            sphere.header.frame_id = 'base_link'
            sphere.header.stamp = stamp
            sphere.ns = 'waypoints'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = wp['pose']
            sphere.scale = Vector3(x=scale, y=scale, z=scale)
            sphere.color = color
            ma.markers.append(sphere)

            label = Marker()
            label.header.frame_id = 'base_link'
            label.header.stamp = stamp
            label.ns = 'waypoint_labels'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(
                x=wp['pose'].position.x,
                y=wp['pose'].position.y,
                z=wp['pose'].position.z + 0.05)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.028
            label.color = self.COLOR_TEXT
            label.text = f'{i + 1}'
            ma.markers.append(label)

        # 웨이포인트 연결선 (참고용 — 실제 끝단 경로와는 다름)
        if len(self._waypoints) >= 2:
            line = Marker()
            line.header.frame_id = 'base_link'
            line.header.stamp = stamp
            line.ns = 'waypoint_link'
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.003
            line.color = self.COLOR_LINK
            line.points = [wp['pose'].position for wp in self._waypoints]
            ma.markers.append(line)

        self._marker_pub.publish(ma)

    # ----------------------------------------------------------
    #  메인 시나리오
    # ----------------------------------------------------------
    def run(self):
        helper = MoveGroupHelper(self)
        helper.max_velocity_scaling = 0.3
        helper.max_acceleration_scaling = 0.3
        helper.planning_time = 15.0
        helper.num_planning_attempts = 10

        # 안내 메시지
        self.get_logger().info(
            '[RViz2 안내] 웨이포인트/끝단 경로를 보려면 RViz2에 MarkerArray '
            "Display를 추가하고 Topic을 '/waypoint_markers' 로 설정하세요."
        )
        self.get_logger().info(
            "[단계 진행] 각 단계는 '/next_step' 토픽 신호로 진행됩니다. "
            "다른 터미널에서 다음 명령을 반복 실행하세요:\n"
            "  ros2 topic pub --once /next_step std_msgs/msg/Empty '{}'"
        )

        if not helper.wait_for_servers(timeout_sec=30.0):
            return
        if not helper.wait_for_joint_state(timeout_sec=10.0):
            return

        if not self._ik_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().warn('IK 서비스 없음, plan_to_pose_goal만 사용합니다.')
        else:
            self.get_logger().info('IK 서비스 연결 완료!')

        # 웨이포인트 로드
        filepath = self.get_parameter(
            'waypoint_file').get_parameter_value().string_value
        if not filepath:
            self.get_logger().error('waypoint_file 파라미터가 지정되지 않았습니다!')
            return

        try:
            self._waypoints = self._load_waypoints(filepath)
        except Exception as e:
            self.get_logger().error(f'웨이포인트 파일 로드 실패: {e}')
            return

        self.get_logger().info(f'총 {len(self._waypoints)}개 웨이포인트 로드 완료')

        # 모든 웨이포인트를 노란색(대기)으로 미리 표시
        for i in range(len(self._waypoints)):
            self._wp_status[i] = 'pending'
        self._publish_waypoint_markers()
        self._spin_sleep(0.5)

        # ---------- 초기화: ready 포즈 ----------
        self._wait_for_trigger('>>> [대기] ready 포즈로 이동 신호를 기다리는 중...')
        self.get_logger().info('--- 초기화: ready 포즈로 이동 ---')
        self._plan_and_execute_with_viz(
            helper,
            helper.plan_to_joint_goal(helper.NAMED_TARGETS['ready']),
        )
        self._spin_sleep(1.0)

        # ---------- 1부: 개별 웨이포인트 순차 이동 ----------
        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('1부: 개별 웨이포인트 순차 이동')
        self.get_logger().info('=' * 50)

        for i, wp in enumerate(self._waypoints):
            self._wait_for_trigger(
                f">>> [대기] {i + 1}/{len(self._waypoints)} '{wp['name']}' "
                "이동 신호를 기다리는 중..."
            )
            self.get_logger().info(
                f'[{i + 1}/{len(self._waypoints)}] 이동: "{wp["name"]}" '
                f'(x={wp["pose"].position.x:.2f}, '
                f'y={wp["pose"].position.y:.2f}, '
                f'z={wp["pose"].position.z:.2f})'
            )

            # 현재 웨이포인트를 active 상태로
            self._set_status(i, 'active')

            success = self._plan_and_execute_with_viz(
                helper, self._plan_via_ik(helper, wp['pose']),
                path_color=self.COLOR_EE_PATH,
            )

            if success:
                self.get_logger().info(f'  → "{wp["name"]}" 도달 완료')
                self._set_status(i, 'success')
            else:
                self.get_logger().warn(f'  → "{wp["name"]}" 이동 실패, 계속 진행')
                self._set_status(i, 'fail')

            if wp['pause_sec'] > 0:
                self._spin_sleep(wp['pause_sec'])

            # 다음 웨이포인트로 넘어갈 때 현재를 passed로
            if i < len(self._waypoints) - 1 and success:
                self._set_status(i, 'passed')

        # ---------- 2부: 전체 웨이포인트 자동 일주 ----------
        # 트리거 1회로 시작, plan_via_ik(joint-space)로 모든 점을 자동 일주.
        # 각 segment의 trajectory에 FK 적용 → 누적해서 시안색 LINE_STRIP로 표시.
        # (compute_cartesian_path 직선 보간은 본 로봇 + gripper-down 자세에서 IK
        #  실패로 fraction=0이 자주 나와 신뢰성 떨어짐 → joint-space 방식 사용)
        self._wait_for_trigger(
            '>>> [대기] 2부 전체 일주 (점마다 0.5s 정지) 신호 대기...'
        )
        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('2부: 전체 웨이포인트 자동 일주')
        self.get_logger().info('=' * 50)
        self.get_logger().info(
            f'  {len(self._waypoints)}개 웨이포인트를 자동으로 순차 일주, '
            '각 점에서 0.5초 정지'
        )

        # 1부 결과를 시각적으로 리셋 (모두 pending)
        for i in range(len(self._waypoints)):
            self._wp_status[i] = 'pending'
        self._publish_waypoint_markers()
        self._spin_sleep(0.3)

        # 누적 끝단 경로 (시안색 LINE_STRIP — 각 segment FK 결과를 이어붙임)
        running_ee_points = []

        for i, wp in enumerate(self._waypoints):
            self._set_status(i, 'active')
            self.get_logger().info(
                f'  [{i + 1}/{len(self._waypoints)}] '
                f'"{wp["name"]}"로 이동...'
            )

            plan_success, traj = self._plan_via_ik(helper, wp['pose'])

            if plan_success and traj is not None:
                # FK로 segment 끝단 경로 추출 → 누적 후 표시
                ee_seg = self._compute_trajectory_ee_path(traj)
                if ee_seg:
                    running_ee_points.extend(ee_seg)
                    self._publish_ee_path_marker(
                        running_ee_points, color=self.COLOR_SMOOTH_PATH,
                        ns='smooth_segment_path',
                    )

                seg_success = helper.execute_trajectory(traj)
                self._set_status(i, 'success' if seg_success else 'fail')
            else:
                self.get_logger().warn(f'    계획 실패 → 건너뜀')
                self._set_status(i, 'fail')

            self._spin_sleep(0.5)

        self.get_logger().info(
            f'  2부 일주 완료! (누적 끝단 경로 {len(running_ee_points)}개 점)'
        )

        # ---------- 최종 복귀 ----------
        self._wait_for_trigger('>>> [대기] home 포즈로 복귀 신호를 기다리는 중...')
        self.get_logger().info('')
        self.get_logger().info('--- home 포즈로 복귀 ---')
        self._plan_and_execute_with_viz(
            helper,
            helper.plan_to_joint_goal(helper.NAMED_TARGETS['home']),
        )
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
