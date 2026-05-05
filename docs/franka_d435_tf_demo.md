# 다른 로봇 패키지에 센서를 *위에 얹기* — D435 부착 데모

## 문제 정의

남이 만든 로봇 패키지(여기서는 `franka_description`, `franka_fr3_moveit_config`)를 가져왔을 때, 그 패키지를 직접 수정하지 않고 어떻게 새 센서를 추가해 TF 트리에 합류시키고, 시뮬레이터의 영상까지 ROS 토픽으로 끌어올 수 있을까?

이 데모는 그 답을 보여줍니다. 결과물은 `franka_tutorials/launch/franka_with_d435.launch.py` 단 하나의 launch이고, 이를 실행하면

- 원본 그대로의 FR3가 Gazebo Sim 위에 떠 있고,
- MoveIt + RViz로 모션 플래닝까지 가능하며,
- 그 위에 새로 부착한 D435가 TF 트리에 합류해서 `fr3_hand → d435_link → d435_color_optical_frame` 체인을 publish하고,
- Gazebo가 시뮬레이션한 RGB 영상이 `/d435/color/image_raw` 토픽으로 흘러나옵니다.

전 과정에서 `franka_description`과 `franka_fr3_moveit_config` 디렉토리 내부 파일은 단 한 줄도 손대지 않습니다.

## 핵심 원칙

1. **Wrapper xacro로 감싼다.** 원본 URDF(`franka_description/robots/fr3/fr3.urdf.xacro`)를 수정하는 대신, 우리 패키지 안의 새 xacro에서 `<xacro:include>`로 끌어오고 *위에 추가*만 합니다. xacro는 nested `<robot>` 태그를 자식 요소로 평탄화해 주므로 결과 URDF는 한 덩어리가 됩니다.
2. **TF 트리에 합류는 fixed joint로.** 새 링크(`d435_link`)를 부모 링크(`fr3_hand`)에 `<joint type="fixed">`로 묶기만 하면 `robot_state_publisher`가 알아서 `/tf_static`으로 publish합니다. 별도의 `static_transform_publisher`도 필요 없습니다 (그 방식도 가능하지만 URDF에 두면 RViz가 모델까지 같이 그려줍니다).
3. **Gazebo 센서는 `<gazebo>` 태그로 *추가*만.** `<gazebo reference="d435_link">` 안에 카메라 sensor를 정의하면 원본 URDF는 손대지 않은 채 시뮬레이션 센서가 붙습니다.
4. **시뮬레이터 ↔ ROS는 ros_gz_bridge로.** Gazebo 토픽을 ROS 토픽으로 변환하는 표준 도구. URDF에 손대지 않고 launch에서만 처리합니다.

## 파일 구성

```
franka_tutorials/
├── launch/
│   ├── franka_gazebo_moveit.launch.py     # 베이스: FR3 + Gazebo + MoveIt + RViz
│   └── franka_with_d435.launch.py         # 위 + D435 + 영상 브리지 (이 데모의 진입점)
├── urdf/
│   ├── fr3_gazebo.urdf.xacro              # franka_description/fr3 + 중력 오버라이드만 추가
│   └── fr3_gazebo_with_d435.urdf.xacro    # 위 + D435 링크/광학 프레임/카메라 센서
└── worlds/
    └── empty_with_sensors.sdf             # 카메라가 publish하려면 Sensors 시스템 필요
```

원본은 일체 건드리지 않습니다. 추가된 모든 파일은 본 저장소(`franka_tutorials/`) 안에만 있습니다.

## 어떻게 동작하는가

### 1) URDF 래퍼 — `fr3_gazebo_with_d435.urdf.xacro`

```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="fr3_with_d435">

  <!-- 원본 FR3 (gravity off가 더해진 1차 래퍼) -->
  <xacro:include filename="$(find franka_tutorials)/urdf/fr3_gazebo.urdf.xacro"/>

  <!-- D435 본체 마운트 -->
  <joint name="d435_mount_joint" type="fixed">
    <parent link="fr3_hand"/>
    <child link="d435_link"/>
    <origin xyz="0 0 0.05" rpy="0 -1.5707963 0"/>
  </joint>
  <link name="d435_link">
    <visual>... 작은 검정 박스 ...</visual>
    <inertial>...</inertial>
  </link>

  <!-- ROS REP-105 광학 프레임 (z 전방, x 우측, y 하단) -->
  <joint name="d435_color_optical_joint" type="fixed">
    <parent link="d435_link"/>
    <child link="d435_color_optical_frame"/>
    <origin xyz="0 -0.015 0" rpy="-1.5707963 0 -1.5707963"/>
  </joint>
  <link name="d435_color_optical_frame"/>
  <!-- depth_optical_frame도 동일 패턴 -->

  <!-- Gazebo 카메라 센서 -->
  <gazebo reference="d435_link">
    <sensor name="d435_color" type="camera">
      <camera>
        <horizontal_fov>1.21126</horizontal_fov>
        <image><width>640</width><height>480</height><format>R8G8B8</format></image>
        <clip><near>0.1</near><far>10.0</far></clip>
      </camera>
      <update_rate>30</update_rate>
      <topic>d435/color/image_raw</topic>
      <gz_frame_id>d435_color_optical_frame</gz_frame_id>
    </sensor>
  </gazebo>

</robot>
```

### 2) 마운트 회전이 왜 `rpy="0 -1.5707963 0"` 인가

`fr3_hand` 프레임은 표준 Franka 컨벤션을 따라 **+Z**가 그리퍼 방향(전방)입니다. 그런데 Gazebo 카메라 센서의 시야 축은 부착된 링크의 **+X**입니다. 따라서 `d435_link`를 부착할 때 그 +X가 부모의 +Z와 일치하도록 회전해야 카메라가 그리퍼 너머를 바라봅니다. URDF rpy 표기에서 이 회전은 **−π/2 around Y** = `(0, -π/2, 0)`이고, 부호를 반대로 적으면 카메라가 로봇 몸통을 향해 뒤를 보게 됩니다 (실제로 처음엔 그렇게 잘못 적었다가 잡았어요).

### 3) Sensors 시스템 — `empty_with_sensors.sdf`

`gz-sim8`의 기본 `empty.sdf`는 Physics, UserCommands, SceneBroadcaster, Contact 시스템만 로드합니다. 우리는 카메라가 실제로 이미지 데이터를 publish하길 바라므로, 다음 한 줄을 더 적은 사본 월드를 두었습니다.

```xml
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

이게 빠져 있으면 sensor 태그는 URDF에 들어가도 Gazebo는 publish하지 않으므로 ros_gz_bridge가 받을 게 없습니다. (실제로 처음 시도했을 때 정확히 이 증상이었습니다.)

추가로 카메라 시야에 무언가가 보이도록 빨간색 reference box를 월드에 같이 넣어 두었습니다.

### 4) Launch — `franka_with_d435.launch.py`

```python
franka_gazebo_moveit = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare('franka_tutorials'),
                              'launch', 'franka_gazebo_moveit.launch.py'])
    ),
    launch_arguments={
        'urdf_file': 'fr3_gazebo_with_d435.urdf.xacro',     # ← 베이스 launch에 다른 URDF 주입
        'gz_args':  f'-r {world_path}',                      # ← Sensors 들어있는 월드
    }.items(),
)

d435_bridge = Node(
    package='ros_gz_bridge', executable='parameter_bridge',
    arguments=[
        '/d435/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        '/d435/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
    ],
)

return LaunchDescription([franka_gazebo_moveit, d435_bridge])
```

베이스 launch(`franka_gazebo_moveit.launch.py`)는 그대로 재사용하면서, 두 개의 launch argument(`urdf_file`, `gz_args`)만 갈아 끼워 robot_description과 Gazebo 월드를 교체합니다. 이 패턴은 베이스 launch가 그 두 인자를 외부 주입 가능하게 만들어 두었기에 가능합니다.

## 실행과 확인

```bash
cd ~/robot_arm && source install/setup.bash
ros2 launch franka_tutorials franka_with_d435.launch.py
```

올라오면:

```bash
# TF 체인 확인
ros2 run tf2_ros tf2_echo fr3_link0 d435_color_optical_frame

# 토픽 확인
ros2 topic hz /d435/color/image_raw          # 약 8~10 Hz
ros2 topic echo /d435/color/image_raw --field header   # frame_id: d435_color_optical_frame

# RViz에서 영상 보기
#   Add → By topic → /d435/color/image_raw → Image
# 또는
ros2 run rqt_image_view rqt_image_view /d435/color/image_raw
```

RViz의 MotionPlanning 패널에서 인터랙티브 마커로 팔을 움직이면 d435 프레임 세 개도 같이 따라오고, 카메라 영상은 그리퍼가 향하는 방향의 월드를 비춥니다.

## 체크리스트 — 무엇을 해야 했고 무엇은 *안 해도* 됐나

| 한 일 | 손댄 곳 | 손대지 않은 곳 |
|---|---|---|
| 부모 링크에 fixed joint로 새 링크 추가 | `franka_tutorials/urdf/*` | `franka_description/**` |
| Gazebo 카메라 센서 추가 | `franka_tutorials/urdf/*` | `franka_description/**` |
| Sensors 시스템 들어간 월드 마련 | `franka_tutorials/worlds/*` | gz-sim 표준 worlds |
| 영상 ROS 변환 | `franka_tutorials/launch/*` (ros_gz_bridge Node) | URDF 변경 없음 |
| MoveIt 설정 | (변경 없음) | `franka_fr3_moveit_config/**` |
| SRDF | (변경 없음) | `franka_description/robots/fr3/fr3.srdf.xacro` |

원본 패키지 안의 파일을 단 한 줄도 수정하지 않고도, 새 센서를 URDF/TF/Gazebo 모두에서 1급 시민으로 등록한 셈입니다.

## 응용 — 다른 센서로 확장할 때

같은 패턴을 LiDAR, IMU, depth-only 카메라, force-torque 센서 등에 그대로 적용할 수 있습니다. 일반화된 절차:

1. 새 wrapper xacro 만들기 → 원본 robot xacro를 `<xacro:include>` (수정 X)
2. 부모 링크 위에 `<joint type="fixed">`와 새 `<link>` 추가
3. 광학/측정 프레임이 따로 필요하면 REP-103/REP-105 회전과 함께 child link로 추가
4. `<gazebo reference="...">`로 시뮬 sensor 정의
5. 필요시 월드에 `gz::sim::systems::Sensors` 플러그인 보장
6. launch에서 `ros_gz_bridge`로 토픽 변환

이때 원본 패키지에 들어가는 변경은 **0**이어야 합니다. 그게 이 패턴의 핵심이고, upstream을 그대로 추적할 수 있게 해주는 이유입니다.
