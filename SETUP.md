# 설치 가이드 (Ubuntu 22.04 x86_64 데스크탑)

A.R.G.U.S. UGV 워크스페이스를 데스크탑에서 처음부터 세팅하는 절차입니다.
원본은 Jetson Orin(aarch64)에서 개발됐고, 아래는 x86_64 데스크탑 기준입니다.

Step 1부터 순서대로 실행하면 됩니다. 프로젝트 개요는 [README](README.md),
실행 방법은 [HOW_TO_RUN.md](HOW_TO_RUN.md),
구조·알고리즘 상세는 [ARCHITECTURE.md](ARCHITECTURE.md) 를 참고한다.

---

## 목차

**[워크스페이스 구조](#워크스페이스-구조)** ·
**[Jetson → 데스크탑 차이점](#jetson--데스크탑-주요-차이점)** ·
**[Step 1. ROS 2 확인](#step-1-ros2-humble-기본-확인)** ·
**[2. 시스템 의존성](#step-2-시스템-의존성-설치)** ·
**[3. Gazebo](#step-3-gazebo-설치-harmonic-권장)** ·
**[4. Nav2 + SLAM](#step-4-nav2--slam-설치)** ·
**[5. Python 패키지](#step-5-python-패키지-설치)** ·
**[6. rosdep](#step-6-rosdep-초기화-및-의존성-설치)** ·
**[7. 빌드](#step-7-워크스페이스-빌드)** ·
**[8. bashrc](#step-8-bashrc-설정)** ·
**[9. 실행](#step-9-gazebo-시뮬레이션-실행)** ·
**[Gazebo 버전별 주의](#gazebo-버전별-주의사항)** ·
**[트러블슈팅](#트러블슈팅)**

---

## 워크스페이스 구조

```
ugv_ws/
├── src/
│   ├── ugv_bringup       # launch 파일 (gazebo, robot, slam_nav 등)
│   ├── ugv_description   # URDF/xacro 로봇 모델, RViz config, world SDF
│   ├── ugv_msgs          # 커스텀 ROS2 메시지
│   ├── ugv_navigation    # Nav2, EKF launch 파일
│   ├── ugv_teleop        # 조이스틱/키보드 텔레op
│   ├── ugv_vision        # YOLO 기반 비전 노드
│   ├── sllidar_ros2      # LiDAR 드라이버 (시뮬에서는 미사용)
│   ├── micro_ros_setup   # MCU 통신 (시뮬에서는 미사용)
│   └── uros              # micro-ROS 에이전트 (시뮬에서는 미사용)
└── README.md
```

---

## Jetson → 데스크탑 주요 차이점

| 항목 | Jetson (원본) | 데스크탑 (목표) |
|------|--------------|----------------|
| 아키텍처 | aarch64 (ARM) | x86_64 |
| OS | Ubuntu 22.04.5 | Ubuntu 22.04 |
| ROS2 | Humble | Humble |
| Python | 3.10.12 | 3.10.12 |
| OpenCV | 4.13.0 | 4.13.0 |
| Gazebo | Gz Garden (gz-sim7) | **Gz Harmonic (gz-sim8)** 설치 |
| CUDA | 12.6 (Orin 통합 GPU) | 데스크탑 NVIDIA GPU (없으면 CPU 추론) |
| PyTorch | 2.8.0 (CUDA) | GPU면 CUDA 빌드, 아니면 CPU 빌드 |

---

## Step 1. ROS2 Humble 기본 확인

```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

---

## Step 2. 시스템 의존성 설치

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-pip
```

---

## Step 3. Gazebo 설치 (Harmonic 권장)

> **이 저장소는 Gazebo Harmonic(gz-sim8) 기준으로 동작 검증돼 있다.**
> Harmonic 이면 아래 블록 대신 이렇게 설치한다:
>
> ```bash
> sudo apt install -y gz-harmonic
> sudo apt install -y \
>   ros-humble-ros-gzharmonic-sim \
>   ros-humble-ros-gzharmonic-bridge \
>   ros-humble-ros-gzharmonic-image \
>   ros-humble-ros-gzharmonic-interfaces
> ```
>
> Garden(gz-sim7)으로 되돌릴 경우 URDF 수정이 필요하다 —
> [Gazebo 버전별 주의사항](#gazebo-버전별-주의사항) 참조.

```bash
# (참고) Gazebo Garden 저장소 추가
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update

# Gazebo Garden
sudo apt install -y gz-garden

# ROS2-Gazebo 브리지
sudo apt install -y \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-image \
  ros-humble-ros-gz-interfaces
```

---

## Step 4. Nav2 + SLAM 설치

```bash
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-joy \
  ros-humble-robot-localization
```

---

## Step 5. Python 패키지 설치

**PyTorch 는 반드시 따로 설치한다.** `--index-url` 을 한 명령에 같이 걸면 그 인덱스에
없는 `ultralytics` 등을 못 찾아 설치가 실패한다.

```bash
# ① PyTorch — NVIDIA GPU가 있으면 CUDA 빌드로 (권장)
#    nvidia-smi 의 "CUDA Version" 에 맞는 인덱스를 고를 것 (cu126 / cu128 / cu130)
pip3 install --user --index-url https://download.pytorch.org/whl/cu130 \
  torch==2.12.1+cu130 torchvision==0.27.1+cu130

#    GPU가 없을 때만 CPU 빌드
# pip3 install --user --index-url https://download.pytorch.org/whl/cpu torch torchvision

# ② 나머지 (기본 PyPI 사용)
pip3 install --user \
  ultralytics==8.4.39 \
  numpy==1.26.4 \
  scipy \
  scikit-learn \
  PyYAML
```

> **CPU 빌드를 깔면 YOLO가 GPU를 못 씁니다.** `yolo_pose_node`는
> `torch.cuda.is_available()`로 device를 자동 선택하므로 코드 수정은 필요 없고,
> **설치된 휠이 CPU 전용인지가 전부입니다.** 확인:
> ```bash
> python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
> # 2.12.1+cu130 True   ← 정상
> # 2.12.1+cpu   False  ← CPU 빌드. 위 ①로 재설치할 것
> ```
> 노드 기동 시 로그의 `YOLO 추론 device = 0` (GPU) / `= cpu` 로도 구분됩니다.
>
> 실측 차이 (yolov8n-pose, 640×480, RTX 3060):
> | | 추론 시간 | 처리량 |
> |---|---|---|
> | CPU 빌드 | 104.5 ms/frame | 9.6 FPS — 카메라 15Hz를 못 따라감 |
> | CUDA 빌드 | 9.2 ms/frame | 108.9 FPS |
>
> `scikit-learn` 은 트리아지 모델(`triage_model_rf_robust.pkl`) 로드에 필요하다.
> 없으면 `ModuleNotFoundError: sklearn` 으로 노드가 즉시 죽는다.

---

## Step 6. rosdep 초기화 및 의존성 설치

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
cd ~/ugv_ws
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "micro_ros_setup micro_ros_agent micro_ros_msgs realsense2_camera sllidar_ros2"
```

> `--skip-keys`에 포함된 패키지는 실제 하드웨어(MCU, RealSense, LiDAR) 전용이므로
> 시뮬레이션 환경에서는 설치하지 않아도 됩니다.

---

## Step 7. 워크스페이스 빌드

```bash
cd ~/ugv_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-skip micro_ros_setup micro_ros_agent uros sllidar_ros2
```

> `micro_ros_setup`, `uros`, `sllidar_ros2`는 실제 하드웨어 전용 패키지라
> 데스크탑 시뮬에서는 빌드하지 않는다.

---

## Step 8. bashrc 설정

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ugv_ws/install/setup.bash" >> ~/.bashrc
echo "export GZ_SIM_RESOURCE_PATH=~/ugv_ws/install/ugv_description/share:$GZ_SIM_RESOURCE_PATH" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 9. Gazebo 시뮬레이션 실행

```bash
source ~/ugv_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=~/ugv_ws/install/ugv_description/share:$GZ_SIM_RESOURCE_PATH

# ① Gazebo + 로봇 스폰 + 브리지만 (동작 확인용)
ros2 launch ugv_bringup gazebo.launch.py

# ② SLAM + Nav2 + 비전 풀스택
ros2 launch ugv_bringup slam_nav_sim.launch.py

# ③ 경비순찰 + 열화상 화재감지 (②에 순찰/화재 노드 추가)
ros2 launch ugv_bringup patrol_sim.launch.py
```

노드들이 0~12초에 걸쳐 순차 기동(SLAM 4s / Nav2 8s / 비전 10s / 순찰·화재 12s)하므로
초반에는 덜 뜬 것처럼 보이는 게 정상입니다.

---

## Gazebo 버전별 주의사항

Garden(gz-sim7)과 Harmonic(gz-sim8)은 **URDF 안의 플러그인/센서 표기법이 다릅니다.**
현재 `ugv.urdf.xacro`는 Harmonic 표기를 씁니다.

| 항목 | Garden (gz-sim7) | Harmonic (gz-sim8) |
|------|------------------|--------------------|
| 플러그인 `filename` | `ignition-gazebo-*-system` | `gz-sim-*-system` |
| 플러그인 `name` | `ignition::gazebo::systems::*` | `gz::sim::systems::*` |
| 센서 frame_id | `<ignition_frame_id>` | `<gz_frame_id>` |

**증상별 원인 정리**

- **로봇이 안 움직이고 `/odom`·`/joint_states`가 안 나옴**
  → 플러그인 이름이 그 Gazebo 버전에 없는 것. 해당 `.so`가 없으면 **에러 없이 조용히
  로드 실패**하므로 로그만 봐서는 모릅니다. 확인:
  ```bash
  ls /usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/ | grep diff-drive
  ```

- **SLAM/Nav2에서 TF 조회 실패, `/scan`의 frame_id가 `ugv/base_footprint/lidar` 같은 스코프명**
  → frame_id 엘리먼트명이 버전과 안 맞는 것. Harmonic이 읽는 이름은 `gz_frame_id`이며
  다음으로 확인한다:
  ```bash
  strings /usr/lib/x86_64-linux-gnu/libgz-sensors8.so.8 | grep frame_id
  ```

- **로봇 스폰 직후 시뮬이 멈춘 것처럼 보이고 RTF가 0.001까지 떨어짐**
  → **정상입니다.** ogre2 셰이더 최초 컴파일 구간이며 잠시 후 RTF 1.0으로 회복됩니다.
  이 구간에는 `gz service .../control` 호출도 타임아웃 나서 데드락으로 오진하기 쉽다.

- **URDF를 고쳤는데 반영이 안 된 것처럼 보임**
  → gz 서버가 두 개 이상 떠 있으면 ROS 브리지가 옛 서버 토픽을 뭅니다.
  검증 전에 인스턴스가 하나인지 확인한다:
  ```bash
  pgrep -af "gz sim"
  ```
  정리할 때 `pkill -f "ros2 launch ugv_bringup"` 류는 **자기 셸의 cmdline까지 매칭해
  스스로 죽으므로** PID 로 kill 한다.

---

## 트러블슈팅

### Gazebo가 실행되지 않을 때
```bash
# 환경 변수 확인
echo $GZ_SIM_RESOURCE_PATH
# Gazebo 단독 실행 테스트
gz sim --version
```

### colcon build 오류 (missing package)
```bash
# 개별 패키지만 빌드
colcon build --packages-select ugv_msgs
colcon build --packages-select ugv_description ugv_bringup ugv_navigation ugv_teleop ugv_vision
```

### YOLO 모델 파일 없음
`yolov8n.pt` · `yolov8n-pose.pt` 파일이 필요하다:
```bash
pip3 install ultralytics
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"  # 자동 다운로드
```

---

## 원본 환경 정보 (Jetson Orin)
- JetPack R36.5 / CUDA 12.6
- Gazebo Garden (gz-sim 7.0.0)
- ros-humble-ros-gz-bridge 0.244.24
- torch 2.8.0 / ultralytics 8.4.39
- OpenCV 4.13.0

---

## 동작 검증 환경 (데스크탑)
- Ubuntu 22.04 x86_64 / ROS2 Humble / RTX 3060
- **Gazebo Harmonic (gz-sim 8.14.0)** + ros-humble-ros-gzharmonic-* 0.244.12
- Python 3.10, torch CPU 빌드 (YOLO는 CPU 추론)

검증된 동작: `/scan` 9.8Hz(frame_id=`laser_frame`), `/odom` 50Hz, RGB 15Hz, 열화상 10Hz,
cmd_vel 주행 및 포탑 구동, SLAM 맵 생성, Nav2 lifecycle 전체 active,
열화상 화재감지(raw 59881 = 598.8K) → 순찰 정지·포탑 조준 → 화재 구역 우회, RTF 1.0.
