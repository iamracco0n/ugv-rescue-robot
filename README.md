# ugv_ws — Desktop Setup Guide (for Claude Code)

이 README는 Jetson Orin(aarch64)에서 개발된 ROS2 워크스페이스를
**Ubuntu 22.04 x86_64 데스크탑**으로 이식하기 위한 설정 가이드입니다.
Claude Code가 이 파일을 읽고 아래 단계를 순서대로 실행해주세요.

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
| Gazebo | Gz Garden (gz-sim7) | Gz Garden 설치 필요 |
| CUDA | 12.6 (Orin 통합 GPU) | 없음 (CPU 추론) |
| PyTorch | 2.8.0 (CUDA) | CPU 버전 재설치 필요 |

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

## Step 3. Gazebo Garden + ros_gz 설치

```bash
# Gazebo Garden 저장소 추가
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

```bash
pip3 install \
  ultralytics==8.4.39 \
  torch torchvision --index-url https://download.pytorch.org/whl/cpu \
  numpy==1.26.4 \
  scipy \
  PyYAML
```

> **주의:** 데스크탑에 NVIDIA GPU + CUDA가 없으면 CPU 버전 PyTorch를 사용합니다.
> GPU가 있다면 CUDA 버전으로 설치하세요.

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
> 데스크탑 시뮬에서는 빌드하지 않습니다.

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
ros2 launch ugv_bringup gazebo.launch.py
```

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
`yolov8n.pt`, `yolov8n-pose.pt` 파일이 필요합니다:
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
