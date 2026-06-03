# MAC에서 ROS2 설치 및 MoveIt 설정 (Robostack)

macOS(Apple Silicon · Intel)에서 `apt` 없이 **conda + Robostack** 으로 ROS2 Jazzy 환경을 세우고,
이 저장소(`robotarm_tutorials`)의 **FR3 + MoveIt2 + Gazebo** 예제를 끝까지 돌리기 위한 설치 가이드입니다.

> 모든 코드블록은 위에서 아래로 그대로 따라 하면 됩니다. macOS 특유의 함정은 각 단계 끝의 **⚠️ 노트**와 맨 아래 **트러블슈팅** 표에 정리했습니다.

---

## 0. 사전 준비

- **conda (Miniforge 권장)** — Apple Silicon 네이티브, 기본 채널이 conda-forge라 Robostack과 충돌이 없음
  ```bash
  brew install --cask miniforge      # 또는 공식 인스톨러
  conda init zsh && exec $SHELL
  ```
- **Homebrew** — Gazebo 본체 설치에 사용

---

## 1. ROS2 설치 (Robostack)

```bash
# ros-jazzy desktop 환경 생성
conda create -n ros_jazzy -c conda-forge -c robostack-jazzy ros-jazzy-desktop

# 환경 활성화
conda activate ros_jazzy

# 이 환경에만 robostack 채널 추가 (이후 conda install ros-jazzy-* 자동 검색)
conda config --env --add channels robostack-jazzy
```

동작 확인:

```bash
ros2 --help
rviz2            # 창이 뜨면 정상
```

🎥 관련 영상: <https://www.youtube.com/watch?v=YHfMkKN8sWM> — PinkLAB · 맥에 ROS2 설치

---

## 2. `ros_activate` 함수 등록 (`~/.zshrc`)

매번 환경/자동완성/플러그인 경로를 잡기 번거로우므로, 아래 함수를 `~/.zshrc`에 넣고 터미널마다 `ros_activate` 한 번으로 끝냅니다.

```zsh
ros_activate() {
  conda activate ros_jazzy
  source "$CONDA_PREFIX/setup.zsh"                                   # zsh 전용 ament 설정 (경고 제거 + 환경 훅 정상화)
  autoload -Uz bashcompinit && bashcompinit
  eval "$(register-python-argcomplete ros2)"
  eval "$(register-python-argcomplete colcon)"
  export PATH=/usr/local/opt/ruby/bin:$PATH                         # gz sim 이 Homebrew ruby 로 실행됨
  export GZ_SIM_SYSTEM_PLUGIN_PATH=$CONDA_PREFIX/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}
  echo "ros2 jazzy activated (tab completion on)!!"
}
```

> ⚠️ **`source "$CONDA_PREFIX/setup.zsh"` 가 중요합니다.** zsh에서 `setup.sh`가 source되면
> `command not found: ament_zsh_to_array` 경고와 함께 일부 환경변수(특히 `GZ_SIM_SYSTEM_PLUGIN_PATH`)
> append가 누락됩니다. `.zsh` 변형을 명시적으로 다시 source하면 해결됩니다.
>
> ⚠️ **`GZ_SIM_SYSTEM_PLUGIN_PATH=$CONDA_PREFIX/lib`** 가 없으면 Gazebo가 `gz_ros2_control-system`
> 플러그인을 못 찾습니다(라이브러리는 있는데 검색 경로에 없음).

---

## 3. Gazebo 설치 (Harmonic) + ROS2 브릿지

### 3-1. Gazebo 본체 (Homebrew)

공식 가이드: <https://gazebosim.org/docs/harmonic/install_osx/>

```bash
brew tap osrf/simulation
brew install gz-harmonic
```

확인 — macOS는 서버/GUI를 **따로** 실행합니다:

```bash
gz sim -s     # 터미널 1: 서버(physics)
gz sim -g     # 터미널 2: GUI
```

### 3-2. ROS2 ↔ Gazebo 브릿지 (ros-gz)

```bash
conda install -c robostack-jazzy -c conda-forge \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim
```

🎥 관련 영상: <https://www.youtube.com/watch?v=VUJiN3vkW50> — PinkLAB · 맥에서 Gazebo

---

## 4. MoveIt2 · ros2_control 설치

`ros-jazzy-desktop`에는 MoveIt과 ros2_control이 포함되지 않으므로 따로 설치합니다.

```bash
# MoveIt2 (move_group · OMPL · moveit_servo 포함)
conda install -c robostack-jazzy -c conda-forge ros-jazzy-moveit

# ros2_control 스택 (controller_manager · 컨트롤러 · Gazebo 연동 플러그인)
conda install -c robostack-jazzy -c conda-forge \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control
```

---

## 5. 워크스페이스 clone & 빌드

### 5-1. colcon 설치

```bash
conda install -c conda-forge colcon-common-extensions
# (대안) pip install -U colcon-common-extensions
```

### 5-2. clone

```bash
mkdir -p ~/ws/robot_arm/src && cd ~/ws/robot_arm/src
git clone https://github.com/PinkWink/robotarm_tutorials.git
```

### 5-3. 빌드 전 — setuptools 다운그레이드 (필수)

최신 setuptools(≥80)는 `colcon --symlink-install`이 쓰는 `setup.py develop --editable`를 제거해
ament_python 패키지 빌드가 `error: option --editable not recognized`로 깨집니다.

```bash
pip install "setuptools<80"
```

### 5-4. 빌드

```bash
cd ~/ws/robot_arm
rm -rf build install log          # 이전 실패 캐시 정리
colcon build --symlink-install
source install/setup.zsh          # zsh (bash면 setup.bash)
```

> ⚠️ 빌드 중 `CMake Deprecation Warning`, `Unknown distribution option: 'tests_require'` 는 **무해**합니다.

---

## 6. 누락 파이썬 의존성 보강 (robostack 빠뜨림)

런타임에서 드러나는 두 모듈을 미리 깔아 둡니다.

```bash
# 컨트롤러 spawner 가 import 하는 모듈 (없으면 컨트롤러가 하나도 안 올라옴)
pip install filelock

# 노트북 예제에서 쓰는 tf 변환 모듈 (+ 의존 모듈 transforms3d)
conda install -c robostack-jazzy -c conda-forge ros-jazzy-tf-transformations
pip install transforms3d
```

---

## 7. 실행 — 두 터미널 (서버 / GUI 분리)

macOS는 `gz sim` 서버+GUI 동시 실행이 불가하므로, launch는 **서버 전용(`-s`)** 으로 띄우고 GUI는 따로 엽니다.

**터미널 A — launch (Gazebo 서버 + MoveIt + RViz):**
```bash
ros_activate
source ~/ws/robot_arm/install/setup.zsh
ros2 launch franka_tutorials franka_gazebo_moveit.launch.py gz_args:='-s -r empty.sdf'
```

**터미널 B — Gazebo GUI:**
```bash
ros_activate
gz sim -g
```

확인:
```bash
ros2 control list_controllers     # joint_state_broadcaster / fr3_arm_controller / fr3_gripper_controller = active
ros2 topic info /joint_states -v  # Publisher count: 1
```

세 컨트롤러가 `active`이고 `/joint_states` 발행자가 1이면, RViz에서 **Plan → Execute** 가 동작합니다.

> ⚠️ `gz sim -g` 창이 안 보이면 Mission Control(F3)에서 숨은 창 확인, 또는 `gz sim -g --render-engine ogre`.

---

## 8. Jupyter Notebook

```bash
conda activate ros_jazzy
conda install -c conda-forge notebook ipykernel
```

**반드시 ROS 환경을 잡은 터미널에서** 실행해야 노트북에서 `import rclpy`가 됩니다(커널은 띄운 셸 환경을 상속):

```bash
ros_activate
source ~/ws/robot_arm/install/setup.zsh
jupyter notebook
```

> 패키지를 추가 설치한 뒤에는 **Kernel → Restart** 해야 새 모듈이 잡힙니다.

---

## 트러블슈팅 한눈에 보기

| 증상 | 원인 | 해결 |
|------|------|------|
| `error: option --editable not recognized` (colcon build) | setuptools ≥80이 `setup.py develop --editable` 제거 | `pip install "setuptools<80"` 후 `rm -rf build install log` → 재빌드 |
| `package 'moveit_ros_move_group' not found` | MoveIt 미설치 | `conda install ... ros-jazzy-moveit` |
| `package 'controller_manager' not found` | ros2_control 미설치 | `conda install ... ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control` |
| `gz_ros2_control-system : Could not find shared library` | 플러그인 검색 경로 누락 | `export GZ_SIM_SYSTEM_PLUGIN_PATH=$CONDA_PREFIX/lib` (ros_activate에 포함) |
| 컨트롤러 0개 · `/joint_states` Publisher 0 · execute ABORTED | spawner가 `ModuleNotFoundError: filelock`로 즉사 | `pip install filelock` |
| 노트북 `ModuleNotFoundError: tf_transformations` | robostack 미포함 | `conda install ... ros-jazzy-tf-transformations` + `pip install transforms3d` |
| `command not found: ament_zsh_to_array` (활성화 시) | zsh에서 `setup.sh` source됨 | `ros_activate`에서 `source "$CONDA_PREFIX/setup.zsh"` |
| Gazebo 창이 안 뜸 (launch에 에러 없음) | `-s`는 headless 서버라 정상 | 터미널 B에서 `gz sim -g` |
| `Protocol family not supported` (affinity) 등 도배 | macOS FastDDS 잡음 | 무해 — 무시 |
| `Virtual joint ... 'base' is not known` | SRDF 베이스링크 경고 | 무해(fixed joint 폴백) |

---

## 빠른 시작 요약 (설치 한 번에)

```bash
# 1) ROS2 환경
conda create -n ros_jazzy -c conda-forge -c robostack-jazzy ros-jazzy-desktop
conda activate ros_jazzy
conda config --env --add channels robostack-jazzy

# 2) 핵심 패키지
conda install -c robostack-jazzy -c conda-forge \
    ros-jazzy-moveit \
    ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim \
    ros-jazzy-tf-transformations
conda install -c conda-forge colcon-common-extensions
pip install "setuptools<80" filelock transforms3d

# 3) Gazebo 본체
brew tap osrf/simulation && brew install gz-harmonic

# 4) 빌드
mkdir -p ~/ws/robot_arm/src && cd ~/ws/robot_arm/src
git clone https://github.com/PinkWink/robotarm_tutorials.git
cd ~/ws/robot_arm && colcon build --symlink-install && source install/setup.zsh
```

이후 `~/.zshrc`에 **2번의 `ros_activate` 함수**를 등록하면 끝입니다.
