#!/usr/bin/env bash
# ugv_ws 데스크탑 이식 — sudo(apt)가 필요한 설치 단계 (README Step 2,3,4,6-init)
# 실행:  ! bash ~/ugv_ws/setup_apt.sh
set -e

echo "== Step 2. 시스템 의존성 =="
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-pip

echo "== Step 3. Gazebo Garden 저장소 (이미 등록돼 있으면 건너뜀) =="
if [ ! -f /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg ]; then
  sudo curl https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update

echo "== Step 3. Gazebo Garden + ros_gz 브리지 =="
sudo apt install -y gz-garden
sudo apt install -y \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-image \
  ros-humble-ros-gz-interfaces

echo "== Step 4. Nav2 + SLAM + 로봇 상태 =="
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

echo "== Step 6. rosdep 초기화 =="
sudo rosdep init 2>/dev/null || true

echo ""
echo "=== apt 설치 단계 완료. 이제 Claude에게 '됐어'라고 알려주면 pip 설치 + 빌드 이어서 진행. ==="
