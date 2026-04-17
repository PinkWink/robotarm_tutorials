# Robot Arm MoveIt2 Tutorials (ROS2 Jazzy)

6-DOF 로봇암을 MoveIt2 Python API로 제어하는 15개 단계별 예제 모음입니다.

## 리포지토리 구성

| 패키지 | 설명 |
|--------|------|
| `robot_arm_description` | URDF/Xacro 로봇 모델, RViz 설정 |
| `robot_arm_moveit_config` | MoveIt2 설정 (SRDF, 컨트롤러, 플래너, Servo 등) |
| `robot_arm_tutorials` | 15개 단계별 Python 예제 코드 |

## 사전 요구사항

- Ubuntu 24.04
- ROS2 Jazzy (apt 설치, [공식 설치 가이드](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html))
- Gazebo Harmonic (ROS2 Jazzy의 기본 시뮬레이터)
- Python 3 가상환경 (권장)
- colcon 빌드 도구

### ROS2 Jazzy 기본 설정 확인

```bash
# ROS2가 올바르게 설치되었는지 확인
source /opt/ros/jazzy/setup.bash
ros2 --version

# colcon 빌드 도구 설치 (없는 경우)
sudo apt install python3-colcon-common-extensions

# rosdep 초기화 (처음 한 번만)
sudo rosdep init 2>/dev/null || true
rosdep update
```

## 의존성 설치

### 방법 1: rosdep으로 자동 설치 (권장)

클론 후 워크스페이스 루트에서 실행하면 `package.xml`에 명시된 모든 의존성을 자동 설치합니다.

```bash
cd ~/ros_ws
rosdep install --from-paths src --ignore-src -r -y
```

> **참고**: `rosdep`으로 설치되지 않는 Python 패키지는 별도로 설치해야 합니다 (아래 Python 의존성 참조).

### 방법 2: apt 수동 설치

`rosdep`이 동작하지 않거나 개별 패키지를 직접 관리하고 싶은 경우 아래 명령어로 설치합니다.

```bash
# ROS2 Jazzy 환경 소싱
source /opt/ros/jazzy/setup.bash
```

#### 1) MoveIt2 (메타 패키지 + 개별 패키지)

`ros-jazzy-moveit` 메타 패키지를 설치하면 아래 서브 패키지가 모두 포함됩니다:
- `moveit-core`, `moveit-ros-move-group`, `moveit-ros-visualization` (RViz 인터랙티브 마커)
- `moveit-kinematics` (IK 솔버), `moveit-planners-ompl`, `moveit-simple-controller-manager`
- `moveit-servo` (실시간 텔레오퍼레이션)

```bash
sudo apt install ros-jazzy-moveit ros-jazzy-moveit-servo
```

#### 2) Gazebo Harmonic 시뮬레이션

Gazebo Harmonic과 ROS2 연동 패키지입니다. **반드시 모두 설치해야 합니다.**

```bash
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
  ros-jazzy-gz-ros2-control
```

> **주의**: `ros-jazzy-gazebo-ros2-control`이 아닌 `ros-jazzy-gz-ros2-control`입니다 (Gazebo Harmonic용).

#### 3) ros2_control 및 컨트롤러

```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-gripper-controllers
```

#### 4) 로봇 모델 관련

```bash
sudo apt install ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2
```

#### 5) ROS2 메시지 패키지

```bash
sudo apt install ros-jazzy-control-msgs ros-jazzy-trajectory-msgs \
  ros-jazzy-shape-msgs ros-jazzy-visualization-msgs \
  ros-jazzy-moveit-msgs
```

#### 6) tf 변환

```bash
sudo apt install ros-jazzy-tf-transformations ros-jazzy-tf2-ros
```

### Python 의존성

```bash
pip install transforms3d pyyaml
```

### 설치 확인

모든 의존성이 올바르게 설치되었는지 확인합니다.

```bash
# MoveIt 설치 확인
ros2 pkg list | grep moveit

# Gazebo 설치 확인
gz sim --version

# ros2_control 설치 확인
ros2 pkg list | grep controller

# 누락된 의존성 확인 (워크스페이스에서)
cd ~/ros_ws
rosdep check --from-paths src --ignore-src
```

## 빌드

```bash
# 워크스페이스 생성
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src

# 이 리포지토리 클론
git clone <repository_url>

# 3개 패키지를 src/ 아래에 배치
# (클론 후 구조: src/robot_arm_description, src/robot_arm_moveit_config, src/robot_arm_tutorials)

# 빌드
cd ~/ros_ws
colcon build
source install/setup.bash
```

## 실행 방법

모든 예제는 MoveIt + RViz 환경이 먼저 실행되어야 합니다.

```bash
# 터미널 1: MoveIt + RViz 실행
ros2 launch robot_arm_moveit_config demo.launch.xml
```

```bash
# 터미널 2: 예제 실행 (ex01 ~ ex15)
ros2 run robot_arm_tutorials ex01_joint_state_reader --ros-args -p use_sim_time:=true
```

또는 런치 파일로 한 번에 실행할 수 있습니다:

```bash
ros2 launch robot_arm_tutorials tutorial_demo.launch.py tutorial:=ex02_named_pose
```

## 예제 목록

| # | 파일 | 주제 | 핵심 API |
|---|------|------|----------|
| 01 | `ex01_joint_state_reader.py` | 관절 상태 읽기 | `Subscription`, `JointState` |
| 02 | `ex02_named_pose.py` | SRDF Named Pose 이동 | `MoveGroup Action`, SRDF |
| 03 | `ex03_joint_goal.py` | 개별 관절 각도 제어 | `JointConstraint` |
| 04 | `ex04_pose_goal.py` | 끝단 위치+방향 지정 이동 | `PositionConstraint`, `OrientationConstraint`, IK |
| 05 | `ex05_cartesian_path.py` | 직교 공간 직선 경로 | `GetCartesianPath` 서비스, Waypoint |
| 06 | `ex06_gripper_control.py` | 그리퍼 열기/닫기 | `GripperCommand Action` |
| 07 | `ex07_pick_and_place.py` | Pick & Place 시퀀스 | IK 시드, 팔+그리퍼 통합 |
| 08 | `ex08_constraints.py` | 경로 제약조건 (수평 유지) | `OrientationConstraint`, `path_constraints` |
| 09 | `ex09_collision_objects.py` | 충돌 객체 추가/회피 | `CollisionObject`, `PlanningScene` |
| 10 | `ex10_multi_planner.py` | OMPL 플래너 비교 | `planner_id`, FK 시각화 |
| 11 | `ex11_keyboard_servo.py` | 키보드 실시간 텔레오퍼레이션 | `MoveIt Servo`, `TwistStamped` |
| 12 | `ex12_waypoint_follow.py` | YAML 웨이포인트 순차 추적 | YAML 파싱, Cartesian 보간 |
| 13 | `ex13_circular_path.py` | 원형 경로 (Cartesian Path) | 원형 Waypoint 생성 |
| 14 | `ex14_circular_servo.py` | 원형 경로 (실시간 IK 스트리밍) | IK + `JointTrajectory` 퍼블리시 |
| 15 | `ex15_keyboard_ik.py` | IK 기반 키보드 텔레오퍼레이션 | IK 직접 호출, curses UI |

## 예제별 실행 명령어

### 기본 예제 (ex01 ~ ex10, ex12 ~ ex13)

```bash
# 터미널 1
ros2 launch robot_arm_moveit_config demo.launch.xml

# 터미널 2
ros2 run robot_arm_tutorials ex01_joint_state_reader --ros-args -p use_sim_time:=true
ros2 run robot_arm_tutorials ex02_named_pose --ros-args -p use_sim_time:=true
ros2 run robot_arm_tutorials ex03_joint_goal --ros-args -p use_sim_time:=true
ros2 run robot_arm_tutorials ex04_pose_goal --ros-args -p use_sim_time:=true
ros2 run robot_arm_tutorials ex05_cartesian_path --ros-args -p use_sim_time:=true
ros2 run robot_arm_tutorials ex06_gripper_control --ros-args -p use_sim_time:=true
ros2 run robot_arm_tutorials ex07_pick_and_place --ros-args -p use_sim_time:=true
ros2 run robot_arm_tutorials ex08_constraints --ros-args -p use_sim_time:=true
ros2 run robot_arm_tutorials ex09_collision_objects --ros-args -p use_sim_time:=true
ros2 run robot_arm_tutorials ex10_multi_planner --ros-args -p use_sim_time:=true
ros2 run robot_arm_tutorials ex12_waypoint_follow --ros-args -p use_sim_time:=true
ros2 run robot_arm_tutorials ex13_circular_path --ros-args -p use_sim_time:=true
```

### MoveIt Servo 예제 (ex11)

Servo 노드가 별도로 필요합니다:

```bash
# 터미널 1: MoveIt + Servo 노드 동시 실행
ros2 launch robot_arm_tutorials servo_keyboard.launch.py

# 터미널 2: 키보드 제어 노드
ros2 run robot_arm_tutorials ex11_keyboard_servo --ros-args -p use_sim_time:=true
```

### 실시간 IK 예제 (ex14, ex15)

```bash
# 터미널 1
ros2 launch robot_arm_moveit_config demo.launch.xml

# 터미널 2
ros2 run robot_arm_tutorials ex14_circular_servo --ros-args -p use_sim_time:=true
ros2 run robot_arm_tutorials ex15_keyboard_ik --ros-args -p use_sim_time:=true
```

### 커스텀 웨이포인트 파일 사용 (ex12)

```bash
ros2 run robot_arm_tutorials ex12_waypoint_follow --ros-args \
  -p use_sim_time:=true \
  -p waypoint_file:=/absolute/path/to/your_waypoints.yaml
```

## 패키지 구조

```
ros_ws/src/
├── robot_arm_description/          # 로봇 모델
│   ├── urdf/
│   │   └── robot_arm.urdf.xacro    # 6-DOF + 그리퍼 URDF
│   ├── rviz/
│   │   └── robot_arm.rviz
│   ├── launch/
│   │   └── display.launch.xml      # URDF 시각화용
│   ├── CMakeLists.txt
│   └── package.xml
│
├── robot_arm_moveit_config/        # MoveIt2 설정
│   ├── config/
│   │   ├── robot_arm.srdf          # SRDF (Named Pose, 그룹 정의)
│   │   ├── kinematics.yaml         # IK 솔버 설정
│   │   ├── joint_limits.yaml       # 관절 제한
│   │   ├── ompl_planning.yaml      # OMPL 플래너 설정
│   │   ├── servo_parameters.yaml   # MoveIt Servo 설정
│   │   ├── ros2_controllers.yaml   # ros2_control 컨트롤러
│   │   ├── moveit_controllers.yaml # MoveIt 컨트롤러
│   │   └── moveit.rviz             # MoveIt용 RViz 설정
│   ├── launch/
│   │   ├── demo.launch.xml         # MoveIt + RViz 통합 실행
│   │   ├── move_group.launch.xml
│   │   ├── moveit_rviz.launch.xml
│   │   └── robot_arm_gazebo.launch.xml
│   ├── CMakeLists.txt
│   └── package.xml
│
└── robot_arm_tutorials/            # 15개 예제
    ├── config/
    │   └── sample_waypoints.yaml   # ex12용 샘플 웨이포인트
    ├── launch/
    │   ├── tutorial_demo.launch.py # MoveIt + 예제 동시 실행
    │   └── servo_keyboard.launch.py# MoveIt + Servo 노드 실행
    ├── robot_arm_tutorials/
    │   ├── __init__.py
    │   ├── utils.py                # MoveGroupHelper, GripperHelper
    │   ├── ex01_joint_state_reader.py
    │   ├── ...
    │   └── ex15_keyboard_ik.py
    ├── resource/
    ├── package.xml
    ├── setup.cfg
    └── setup.py
```

## 핵심 유틸리티 (utils.py)

| 클래스/함수 | 설명 |
|-------------|------|
| `MoveGroupHelper` | MoveGroup Action, Cartesian 경로, Planning Scene 관리 |
| `GripperHelper` | GripperCommand Action 제어 |
| `make_pose()` | 위치 + 오일러 각도로 Pose 메시지 생성 |
| `euler_to_quaternion()` | Euler → Quaternion 변환 |
| `make_box_collision_object()` | 박스 충돌 객체 생성 |
| `make_cylinder_collision_object()` | 원기둥 충돌 객체 생성 |

## 트러블슈팅

### RViz 인터랙티브 마커(액티브 휠) 조작 시 에러

RViz에서 로봇 끝단의 인터랙티브 마커(회전/이동 휠)를 잡고 드래그할 때 에러가 발생하는 경우:

**원인**: MoveIt 시각화 또는 IK 관련 패키지 누락

```bash
# 아래 패키지가 모두 설치되어 있는지 확인
sudo apt install ros-jazzy-moveit-ros-visualization \
  ros-jazzy-moveit-kinematics \
  ros-jazzy-moveit-ros-robot-interaction \
  ros-jazzy-moveit-ros-planning-interface

# 또는 메타 패키지로 한 번에 설치 (가장 확실)
sudo apt install ros-jazzy-moveit
```

### Gazebo 실행 시 로봇이 로드되지 않는 경우

```bash
# Gazebo 플러그인 경로 확인
echo $GZ_SIM_SYSTEM_PLUGIN_PATH
# 비어있으면 설정:
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib

# gz_ros2_control 플러그인 설치 확인
sudo apt install ros-jazzy-gz-ros2-control
```

### Controller Manager 타임아웃 에러

`demo.launch.xml` 실행 시 컨트롤러 스포너가 타임아웃되는 경우:

```bash
# 컨트롤러 관련 패키지 재설치
sudo apt install ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-gripper-controllers \
  ros-jazzy-gz-ros2-control
```

### move_group 노드가 플래너를 찾지 못하는 경우

```bash
sudo apt install ros-jazzy-moveit-planners-ompl
```

### Python 모듈 에러 (transforms3d 등)

```bash
# 가상환경 사용 시
source ~/venv/ros_jazzy/bin/activate
pip install transforms3d pyyaml

# 시스템 Python 사용 시
pip3 install transforms3d pyyaml
```

### 의존성 한 번에 전체 재설치

위 방법으로 해결이 안 되는 경우, 모든 패키지를 한 번에 설치합니다:

```bash
sudo apt install \
  ros-jazzy-moveit \
  ros-jazzy-moveit-servo \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-gripper-controllers \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2 \
  ros-jazzy-control-msgs \
  ros-jazzy-trajectory-msgs \
  ros-jazzy-shape-msgs \
  ros-jazzy-visualization-msgs \
  ros-jazzy-moveit-msgs \
  ros-jazzy-tf-transformations \
  ros-jazzy-tf2-ros
```

## License

Apache-2.0
