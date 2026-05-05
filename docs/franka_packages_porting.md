# Franka 패키지 포팅 노트

이 저장소(`robotarm_tutorials/`)에 포함된 Franka 관련 세 패키지(`franka_description`, `franka_fr3_moveit_config`, `franka_gazebo_bringup`)는 모두 [frankaemika/franka_ros2](https://github.com/frankaemika/franka_ros2)에서 가져온 것입니다. 이 문서는 무엇을, 왜, 어떻게 잘라냈는지 정리합니다.

원본 라이선스(Apache-2.0)와 NOTICE는 각 패키지 디렉토리 안에 그대로 보존했습니다.

## 출처

| 항목 | 값 |
|---|---|
| Repo | <https://github.com/frankaemika/franka_ros2> |
| Branch / Tag | `jazzy` (= `v3.3.0` 시점) |
| 추가로 vcs import한 의존 | `franka_description 2.7.0`, `libfranka 0.20.5`, `ros2_control jazzy`, `gz_ros2_control jazzy` 등 (`dependency.repos`) |
| 검증 워크스페이스 | `~/franka_ros2_ws/` (풀 빌드 후 `franka_fr3_moveit_config moveit.launch.py`로 정상 동작 확인 — 이 단계가 끝난 뒤에야 슬림 작업 시작) |

## 포팅 목표와 제약

- **목표**: 별도 워크스페이스를 만들지 않고도 `colcon build` 한 번으로 FR3 (Franka Research 3, 3 kg payload) MoveIt2 + Gazebo 데모를 굴릴 수 있게 함.
- **제약**:
  - GitHub 공개 배포 가정 → 저장소 크기 최소화 (Git LFS 미사용).
  - `libfranka`/`franka_hardware`는 apt에 없음 → 실로봇 모드는 배포 범위 밖, fake hardware + Gazebo만 다룸.
  - 저장하는 파일은 가능한 한 원본 그대로(LICENSE/NOTICE 보존). 코드 변경은 launch와 일부 yaml에 국한.

## 가져온 패키지 요약

| 패키지 | 원본 | 본 저장소 크기 | 핵심 변경 |
|---|---|---|---|
| `franka_description` | `frankarobotics/franka_description` 2.7.0 | 27 MB | 미니멀 — FR3 외 모델/액세서리/부속 mesh 제거 |
| `franka_fr3_moveit_config` | `frankaemika/franka_ros2` 하위 패키지 | 116 KB | launch에서 `franka_gripper`/`franka_robot_state_broadcaster` 제거 |
| `franka_gazebo_bringup` | 상동 | < 10 KB | 슬림 — config yaml 1개만 호스팅, launch/urdf 전부 제거 |

## `franka_description` 슬림 (190 MB → 27 MB)

원본은 FER, FP3, FR3, FR3v2, FR3v2_1, FR3_duo, mobile_FR3_duo, TMRv0_2 등 8종 모델과 다양한 액세서리·말단 mesh를 모두 포함합니다. 우리는 FR3 단일 변형 + 표준 그리퍼만 쓰므로 다음을 삭제했습니다.

```
robots/{fer, fp3, fr3v2, fr3v2_1, fr3_duo, mobile_fr3_duo_v0_2, tmrv0_2}/
meshes/robots/{fer, fp3, fr3v2, fr3v2_1, tmrv0_2}/
accessories/, meshes/accessories/         # franka_head, franka_spine, fr3_duo_mount
end_effectors/cobot_pump, meshes/robot_ee/cobot_pump/
meshes/robot_ee/franka_hand_black/         # 기본 ee_color=white만 유지
Jenkinsfile, scripts/visualize_franka.sh, scripts/create_urdf.sh, test/
```

`CMakeLists.txt`에서 `BUILD_TESTING` 블록과 `accessories` install 항목을 제거했습니다. 그 외 xacro/yaml은 원본 그대로입니다.

남은 구성:

```
franka_description/
├── robots/{common, fr3}/
├── meshes/robots/fr3/
├── meshes/robot_ee/franka_hand_white/
├── end_effectors/{common, franka_hand}/
├── launch/visualize_franka.launch.py
├── rviz/visualize_franka.rviz
└── scripts/create_urdf.py
```

## `franka_fr3_moveit_config` — launch 슬림화

원본 `moveit.launch.py`는 다음 두 가지 외부 의존을 가집니다.

1. `franka_gripper` 패키지 (`gripper.launch.py` include) — 그리퍼 액션 서버
2. `franka_robot_state_broadcaster` 컨트롤러 — 실로봇용 state 브로드캐스터

둘 다 apt에 없고 `franka_ros2` 풀 워크스페이스 빌드가 필요합니다. 시뮬 데모만 필요한 이 저장소에서는 launch에서 두 부분을 제거했습니다.

`launch/moveit.launch.py` 변경:
- `IncludeLaunchDescription(... franka_gripper/launch/gripper.launch.py ...)` 블록 삭제
- `franka_robot_state_broadcaster` Node spawner 삭제
- 관련 import 정리 (`UnlessCondition`, `IncludeLaunchDescription`, `PythonLaunchDescriptionSource`, `FindPackageShare`)

`config/fr3_ros_controllers.yaml`:
- `controller_manager:` 아래 `franka_robot_state_broadcaster` 컨트롤러 항목 삭제

`package.xml`:
- `<exec_depend>franka_hardware</exec_depend>` 삭제
- `<exec_depend>franka_gripper</exec_depend>` 삭제

원본 SRDF, kinematics, OMPL planner 설정, RViz 설정 등은 일체 손대지 않았습니다.

`use_fake_hardware:=true`로 호출 시 URDF 안에서 `mock_components/GenericSystem`(ros2_control 표준)으로 분기되어 `libfranka`/`franka_hardware` 없이도 MoveIt이 정상 동작합니다.

## `franka_gazebo_bringup` — 슬림 재구성

원본 `franka_gazebo_bringup`은 `franka_example_controllers`, `franka_mobile`, `franka_mobile_sensors` 등 추가 패키지에 의존하고, FR3 단일 팔 외에도 듀얼 FR3, 모바일 베이스, TMR 등의 launch와 controller를 포함합니다.

문제: `franka_arm.ros2_control.xacro`(franka_description 안)가 `gazebo:=true`일 때 `<plugin filename="gz_ros2_control-system">` 태그에 `$(find franka_gazebo_bringup)/config/franka_gazebo_controllers.yaml` 경로를 *xacro 처리 시점*에 박아 넣습니다. 즉 이 패키지가 워크스페이스에 존재하고 그 경로의 yaml이 있어야 URDF 생성이 성공합니다.

해법: 원본 `franka_gazebo_bringup`을 통째로 가져오는 대신, 동일한 이름·경로 규약을 만족하는 **슬림 버전**을 직접 작성했습니다.

```
franka_gazebo_bringup/
├── package.xml         # exec_depend 3개 (gz_ros2_control, jtc, jsb)
├── CMakeLists.txt      # config 디렉토리만 install
├── LICENSE / NOTICE    # 원본 보존
└── config/
    └── franka_gazebo_controllers.yaml
```

`franka_gazebo_controllers.yaml`은 본 데모에서 쓰는 컨트롤러 두 개만 정의:
- `fr3_arm_controller` (`joint_trajectory_controller`, effort + PID, 7-DOF)
- `fr3_gripper_controller` (`joint_trajectory_controller`, effort + PID, `fr3_finger_joint1`)
- `joint_state_broadcaster`

원본의 모바일/듀오/임피던스/카르테시안 예제 컨트롤러 등은 모두 제거했고, gain 값은 `franka_fr3_moveit_config/config/fr3_ros_controllers.yaml`(원본)과 동일하게 맞춰서 거동이 일관되게 했습니다.

`launch/`, `urdf/`, `worlds/`, `test/`는 모두 삭제. 본 저장소의 Gazebo 시나리오용 launch와 URDF 래퍼는 다음 절의 `franka_tutorials`에 따로 두었습니다.

## `franka_tutorials` — 신규 작성 (본 저장소 고유)

위 세 패키지를 그대로 두고 데모용 launch만 모은 패키지. 외부 pkg는 손대지 않은 채 *위에 얹기* 위해 만들었습니다.

```
franka_tutorials/
├── package.xml
├── CMakeLists.txt        # launch + urdf + worlds 설치
├── launch/
│   ├── franka_gazebo_moveit.launch.py    # FR3 + Gazebo + MoveIt + RViz
│   └── franka_with_d435.launch.py        # 위 + D435 카메라 부착 + ros_gz_bridge
├── urdf/
│   ├── fr3_gazebo.urdf.xacro             # 원본 fr3.urdf.xacro include + 중력 오버라이드
│   └── fr3_gazebo_with_d435.urdf.xacro   # 위 wrapper + D435 link/optical/sensor
└── worlds/
    └── empty_with_sensors.sdf            # gz-sim Sensors 시스템 플러그인 추가
```

**`fr3_gazebo.urdf.xacro`**: `franka_description/robots/fr3/fr3.urdf.xacro`를 `<xacro:include>`로 끌어오고, FR3 팔/손/손가락 12개 링크에 `<gazebo reference="..."><gravity>false</gravity></gazebo>`를 추가. `joint_trajectory_controller`(effort+PID)가 명시적 중력보상을 하지 않기 때문에, 이걸 안 넣으면 정지 자세에서 후반 관절이 처집니다 (원본 `franka_gazebo_bringup`과 같은 처리).

**`fr3_gazebo_with_d435.urdf.xacro`**: 위 래퍼를 또 한번 `<xacro:include>`해서 D435 본체 링크, 광학 프레임 두 개, 그리고 Gazebo `<sensor type="camera">`를 추가합니다. 자세한 설명은 [`franka_d435_tf_demo.md`](franka_d435_tf_demo.md)에 따로 정리.

**`empty_with_sensors.sdf`**: `gz-sim8`의 기본 `empty.sdf`에는 `gz::sim::systems::Sensors` 플러그인이 빠져 있어 카메라가 실제로 publish되지 않습니다. 본 저장소 사본에 Sensors 플러그인 한 줄과 카메라 시야에 들어올 빨간 박스 한 개를 추가해 두었습니다.

## 최종 의존성 (apt 기준)

다음 패키지가 시스템에 깔려 있으면 `colcon build` → 실행이 됩니다 (`rosdep install --from-paths src --ignore-src -y`로 일괄 설치 가능).

- ROS 2 Jazzy 코어 + `ros-jazzy-moveit`
- `ros-jazzy-ros2-control`, `ros-jazzy-ros2-controllers`, `ros-jazzy-controller-manager`
- `ros-jazzy-joint-state-broadcaster`, `ros-jazzy-joint-trajectory-controller`
- `ros-jazzy-ros-gz`, `ros-jazzy-gz-ros2-control`, `ros-jazzy-sdformat-urdf`
- `ros-jazzy-xacro`, `ros-jazzy-robot-state-publisher`, `ros-jazzy-rviz2`

`libfranka`, `franka_hardware`, `franka_msgs`, `franka_gripper`, `franka_robot_state_broadcaster` 등은 본 배포 범위 밖이며, 실로봇 제어를 원하면 [원본 franka_ros2](https://github.com/frankaemika/franka_ros2)를 따로 빌드해야 합니다.

## 빌드 / 실행

```bash
cd ~/robot_arm
colcon build --symlink-install
source install/setup.bash

# (1) MoveIt + RViz (mock_components, 가벼움)
ros2 launch franka_fr3_moveit_config moveit.launch.py \
    robot_ip:=dont-care use_fake_hardware:=true

# (2) Gazebo + MoveIt + RViz (gz_ros2_control)
ros2 launch franka_tutorials franka_gazebo_moveit.launch.py

# (3) 위 + D435 RGB 카메라
ros2 launch franka_tutorials franka_with_d435.launch.py
```

세 launch 모두 본 저장소 안의 패키지만으로 동작하며, 외부 `franka_ros2` 워크스페이스는 필요 없습니다.
