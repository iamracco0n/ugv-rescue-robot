# 프로젝트 전체 개요 — AI 인수인계 문서

> 작성일: 2026-05-21  
> 이 문서는 다른 AI(또는 새 세션)가 프로젝트 전체를 즉시 파악할 수 있도록 작성된 인수인계 문서입니다.

---

## 0. 한 줄 요약

이 저장소는 **두 개의 독립적인 로봇 프로젝트**가 같은 개발 머신에서 동시에 개발되고 있습니다.

| 프로젝트 | 경로 | 목적 |
|----------|------|------|
| **VTOL 자율비행** | `~/vtol_ws` | 제24회 한국로봇항공기경연대회 참가용 VTOL 드론 미션 |
| **UGV 구조 수색** | `~/ugv_ws` | 6륜 무장 UGV로 건물 내부 수색·구조 (시뮬레이션 개발 중) |

두 프로젝트는 서로 코드를 공유하지 않습니다. 개발 환경만 공유합니다.

---

## 1. 개발 환경 (공통)

- **OS**: Ubuntu 22.04.x x86_64
- **ROS2**: Humble
- **Gazebo**: Harmonic (gz-sim 8.11.0) — `gz sim` 명령 사용
- **Python**: 3.10.12
- **위치**: 광나루 비행장 (한강드론공원), 위도 37.547234 / 경도 127.119596
- **QGC 설정**: `~/.config/QGroundControl/QGroundControl.ini` — 초기 지도 위치 위 좌표로 설정

---

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# PROJECT A: VTOL 자율비행 시스템 (`~/vtol_ws`)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

## A-1. 대회 개요

**제24회 한국로봇항공기경연대회** 정규부문 참가.  
수직이착륙 고정익(VTOL/AAM)으로 조난자 구조 임무를 자율 수행.

### 임무 시나리오 (14단계)
```
버티포트 이륙(MC) → WP1 호버 → MC→FW 천이 → WP2~WP5 고정익 비행 →
FW→MC 천이 → REP 구조 호버 → MC→FW 천이 → WP5~WP2 역방향 →
FW→MC 천이 → 버티포트 ArUco 정밀 착륙
```

### 채점 (총 1200점)
| 항목 | 배점 |
|------|------|
| WP 통과 5개 × 20점 | 200점 |
| MC↔FW 천이 4회 (고도침하 + 가속도) | 250점 |
| 조난자 인식(100) + 이송 성공(250) | 350점 |
| 착륙 정확도 (100 − 20×d[m]) | 100점 |
| 기체 디자인 | 150점 |
| 임무 수행 시간 | 50점 |
| 자체 소프트웨어 가산점 | 100점 |

**WP 통과 오차**: 수평<2m & 수직<4m → 만점(20점)  
**임무 시간**: 준비 3분 + 임무 25분. 초과 시 1초당 1점 감점.

### 버티포트 마커
- 전체 직경 3.0m 원형, 안전구역 직경 2.0m
- **중심: ArUco 마커 50cm × 50cm** (공식 마커만 허용)

---

## A-2. 기술 스택

| 항목 | 내용 |
|------|------|
| FC | Pixhawk (PX4 v1.18.0-alpha1) |
| CC → FC 통신 | uXRCE-DDS (`MicroXRCEAgent udp4 -p 8888`) |
| ROS2 토픽 네임스페이스 | `/fmu/in/` (publish to FC), `/fmu/out/` (receive from FC) |
| MAVROS | **사용 안 함** |
| 시뮬레이터 | PX4 SITL + Gazebo Harmonic |
| ArUco | OpenCV 4.13.0 (cv_bridge 미사용, numpy 직접 변환) |

---

## A-3. 워크스페이스 구조

```
~/vtol_ws/src/vtol_mission/
├── config/
│   └── mission_params.yaml          # 전체 파라미터 (고도, 게인, 임계값 등)
├── launch/
│   ├── simulation.launch.py         # MicroXRCE + 노드들 일괄 실행
│   └── simulation_custom.launch.py  # 커스텀 모델(lift_cruise_vtol_px4) 전용
├── scripts/
│   ├── run_sitl.sh                  # 표준 모델 SITL 전체 실행 (터미널 4개)
│   ├── run_sitl_custom.sh           # 커스텀 모델 SITL 실행 (터미널 3개)
│   ├── generate_qgc_plan.py         # QGC .plan 파일 생성
│   └── test_nodes.py                # 유닛 테스트 17개 (PX4 없이 실행)
├── vtol_mission/
│   ├── mission_node.py              # 메인 미션 로직
│   ├── aruco_landing_node.py        # ArUco 탐지 + 정밀 착륙
│   ├── gnss_logger_node.py          # GNSS 10Hz 로거 (대회 제출용)
│   ├── gz_cam_bridge.py             # Gazebo→ROS 카메라 브릿지 (gz.transport13)
│   └── sim_aruco_node.py            # 시뮬 ArUco 마커 생성기
├── worlds/
│   └── vtol_mission.sdf             # Gazebo 월드 (GPS 기준점 포함)
├── lift_cruise_vtol_px4/            # 커스텀 VTOL 모델 (SDF)
└── mono_cam/                        # 카메라 모델 960×720 오버라이드
```

---

## A-4. 핵심 노드: mission_node.py

### 미션 상태 머신
```
IDLE → TAKEOFF → HOVER_WP1 → FW_WAYPOINTS (WP1~WP5 순회) →
RESCUE_HOVER → ARUCO_SEARCH → ARUCO_APPROACH → PRECISION_LAND →
RETURN → LAND → DISARM
```

### 웨이포인트 (NED 좌표, 시뮬용 — 대회 당일 교체)
```
WP1: (10,   0,  -30)  — MC 호버, 버티포트 북쪽 (0,0이면 heading NaN 버그)
WP2: (100, 100, -30)  — FW 슬라럼 진입
WP3: (200,  50, -30)  — 슬라럼 변곡
WP4: (300, 150, -30)  — 슬라럼 변곡
WP5: (400,  50, -30)  — 종점/구조구간
REP: (450,   0, -15)  — 구조지점 (WP5 너머 감속·하강)
귀환: WP5→WP4→WP3→WP2 역방향
```

### PX4 비행 모드 구분
- **수동 개입** (미션 중단): MANUAL=0, ALTCTL=1, POSCTL=2, ACRO=10, STAB=15
- **Failsafe** (경고만): AUTO_RTL=5, DESCEND=12, TERMINATION=13, AUTO_LAND=18
- **⚠️ nav=5(AUTO_RTL)는 수동이 아님** — SIM_BAT_DRAIN=0.0으로 배터리 failsafe 비활성화

---

## A-5. 핵심 노드: aruco_landing_node.py

### Phase 기반 활성화 (CPU 과부하 방지)
```python
_ACTIVE_PHASES = {
    'RESCUE_HOVER', 'ARUCO_APPROACH', 'ARUCO_SEARCH',
    'PRECISION_LAND', 'LAND_RETRY', 'EMERGENCY_LAND'
}
```
- 비활성 단계에서 image_cb 즉시 return (OpenCV 실행 없음)
- **⚠️ 이 phase 체크를 절대 제거하지 말 것** — 제거 시 10Hz OpenCV 상시 실행으로 RAM 폭증 → Lockstep 붕괴

### cv_bridge 금지
- numpy 2.x와 충돌 (segfault) — 직접 `np.frombuffer()` 사용

---

## A-6. SITL 실행 방법

### 표준 모델 (standard_vtol_cam)
```bash
pkill -9 -f px4; pkill -9 -f "gz sim"; pkill -9 -f MicroXRCEAgent
cd ~/PX4-Autopilot
GZ_SIM_RESOURCE_PATH=$HOME/vtol_ws/src/vtol_mission \
PX4_SYS_AUTOSTART=4004 \
PX4_GZ_MODEL=standard_vtol_cam \
PX4_GZ_MODEL_POSE="0,0,0.1,0,0,0" \
PX4_GZ_WORLD=vtol_mission \
PX4_HOME_LAT=37.547234 \
PX4_HOME_LON=127.119596 \
PX4_HOME_ALT=26 \
./build/px4_sitl_default/bin/px4 -d
```

### 커스텀 모델 (lift_cruise_vtol_px4)
```bash
bash ~/vtol_ws/src/vtol_mission/scripts/run_sitl_custom.sh
```

### 주의사항
- `PX4_HOME_LAT/LON/ALT` 설정 필수 — 없으면 QGC가 취리히(기본값)로 이동
- `vtol_mission.sdf`의 `<spherical_coordinates>`도 동일 좌표로 설정됨
- Gazebo 엔티티명: `standard_vtol_0` (standard_vtol_**cam**_0 아님)
- `real_time_update_rate=250` 유지 필수 — 낮추면 Lockstep 프리징

---

## A-7. 알려진 버그 및 해결책

| 버그 | 원인 | 해결 |
|------|------|------|
| Lockstep 프리징 | `real_time_update_rate` 부족 | worlds/vtol_mission.sdf에서 250 유지 |
| cv_bridge segfault | numpy 2.x 비호환 | np.frombuffer() 직접 변환 |
| RAM 폭증 | phase 체크 없이 상시 OpenCV | _camera_active 플래그 + _ACTIVE_PHASES |
| WP1 NaN heading | WP가 origin(0,0)이면 atan2 NaN | WP1을 (10,0,...) 등 오프셋 부여 |
| QGC 취리히 이동 | SITL 홈 위치 미설정 | PX4_HOME_LAT/LON/ALT 환경변수 설정 |

---

## A-8. 대회 당일 교체 항목

1. `mission_node.py` — WP1~WP5, REP 좌표 (NED)
2. `worlds/vtol_mission.sdf` — `<spherical_coordinates>` GPS 기준점
3. `config/mission_params.yaml` — `land_lateral_gain` 등 착륙 게인
4. `scripts/run_sitl_custom.sh` / `run_sitl.sh` — `PX4_HOME_LAT/LON/ALT`

---

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# PROJECT B: UGV 구조 수색 시스템 (`~/ugv_ws`)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

## B-1. 프로젝트 개요

건물 내부 구조 수색을 위한 6륜 무장 UGV 자율주행 시스템.  
**Jetson Orin(aarch64)에서 개발 → Ubuntu 22.04 x86_64 데스크탑으로 이식** 중.

### 하드웨어 (실 로봇)
- **차체**: 6륜 스키드 스티어링, 바퀴 반지름 42mm, 트랙 폭 162.5mm (slip factor 1.45)
- **MCU**: Teensy (wheel_encoders, turret_fb 토픽으로 ROS2와 통신)
- **포탑**: 2-DOF (yaw + pitch), 모터 110rpm / 10기어 = 1.15 rad/s
- **카메라**: RealSense (depth + RGB), 수평 FOV ≈ 62° (1.089 rad), 640×480
- **LiDAR**: SLAMTEC (슬라이다) — 시뮬에서는 Gazebo 플러그인으로 대체
- **IMU**: RealSense 내장 IMU (`camera/imu`)

---

## B-2. 워크스페이스 구조

```
~/ugv_ws/src/
├── ugv_bringup/      # 런치 파일들 (시뮬/실로봇 전체 스택 기동)
├── ugv_description/  # URDF/xacro 로봇 모델, RViz 설정, 월드 SDF
├── ugv_msgs/         # 커스텀 ROS2 메시지 (TargetDetection 등)
├── ugv_navigation/   # Nav2, EKF, SLAM 설정 및 오도메트리 노드
├── ugv_teleop/       # 조이스틱/키보드 텔레op
├── ugv_vision/       # YOLO 비전 + 전술 커버리지 네비게이터
├── sllidar_ros2/     # LiDAR 드라이버 (시뮬 빌드 제외)
├── micro_ros_setup/  # MCU 통신 (시뮬 빌드 제외)
└── uros/             # micro-ROS 에이전트 (시뮬 빌드 제외)
```

---

## B-3. 커스텀 메시지 (`ugv_msgs`)

```
TargetDetection.msg
  int32   target_id
  float64 x, y          # 화면 픽셀 좌표
  float64 distance      # 추정 거리 (m)
  float64 global_x, y   # 월드 좌표 (lock-on 후 계산)
  string  status        # "TRACKING" | "LOCKED"
  int32   triage_level  # 1=Critical, 2=Urgent, 3=Normal
  string  triage_label
  bool    is_in_center_fov

ChassisCommand.msg
  float64 drive_speed_fl/ml/rl/fr/mr/rr  # 6륜 각각 속도 (m/s)
  bool    center_drop_active             # 실로봇 드롭 장치 트리거

TurretCommand.msg
  float64 x, y, z       # 픽셀 오차 기반 명령 (레거시 실로봇용)
  bool    is_locked_on
  uint8   scan_mode     # 0=MANUAL, 1=SECTOR, 2=TRACKING
```

---

## B-4. 로봇 모델 (URDF)

```
base_footprint
  └── base_link (50cm × 34cm × 14cm, 5kg)
        ├── front/mid/rear _left/right _wheel_link (×6)
        ├── laser_frame (LiDAR, base_link 전방 22cm)
        ├── camera_link (RealSense, base_link 전방 21cm, 12cm 높이)
        ├── turret_link (포탑 베이스, turret_yaw_joint: revolute)
        └── gun_link    (총열, turret_pitch_joint: revolute)
```

**Gazebo 플러그인**:
- `JointController` (velocity mode) — `/turret_yaw_cmd`, `/turret_pitch_cmd` (Float64, rad/s)
- `DifferentialDrive` — `/cmd_vel` (Twist), `/odom` (Odometry)
- `GpuLidar` — `/scan` (LaserScan)
- `RgbdCamera` — `/camera/image`, `/camera/depth_image`

---

## B-5. 노드 구조 및 역할

### ugv_navigation/odometry_node.py (실로봇 전용)
- 바퀴 엔코더(`wheel_encoders`) + IMU(`camera/imu`) → 오도메트리 계산
- `wheel/odom` (Odometry) + `measured_joint_states` (JointState) 발행
- 스키드 스티어 slip factor = 1.45 보정 포함

### ugv_vision/yolo_pose_node.py
- YOLOv8n-pose (`yolov8n-pose.pt`) 로 사람 골격 탐지
- 탐지 → 트리아지 분류 (ML모델: `triage_model_rf_robust.pkl`, 없으면 규칙 기반)
- 바운딩박스 대각선 기반 거리 추정 (focal=535px, 몸통 대각선≈1.75m)
- `/camera/camera/color/image_raw` 구독 → `/target_detection` 발행

### ugv_vision/target_manager_node.py
- **포탑 제어 담당** (vision_coverage_navigator와 역할 분리)
- 상태: `SEARCH` (사인파 스윕 ±50°) / `TRACK` (픽셀 오차 P제어)
- SEARCH 시 `/apex_aim_point` 수신 중이면 sine sweep 대신 해당 방향 고정 조준
- Lock-on 후 환자 위치 기록 (`patient_locations.txt`)
- 기발견 환자 재감지 시 추적 즉시 중단, SEARCH 복귀

### ugv_vision/vision_coverage_navigator.py ⬅️ 핵심, 최근 대폭 수정
아래 B-6 참조.

### ugv_vision/visual_coverage_map.py
- 라이다 SLAM 맵과 별개로, 카메라 FOV가 실제로 훑은 격자 관리
- 격자값: 0=Unseen, 1=Seen, -1=Unknown
- `update_fov()`: 로봇 위치 + 포탑 yaw + FOV + range → 부채꼴 영역 Seen 처리

---

## B-6. VisionCoverageNavigator 상세 설계

### 설계 철학 (중요)
> **Coverage는 Navigation layer가 아니라 Perception assistance layer**

- 사용자(Nav2 목표) 이동이 최우선
- Coverage는 그 이동을 방해하지 않는 선에서 보조
- 사각지대 제거는 현재 이동 방향과 관련된 경우에만 수행

### 상태 머신

```
IDLE
  ↓ (map 수신 + coverage_enabled=True)
FIND_APEX ──→ apex relevance 평가
                ├── relevance ≥ 0.5 (±60°) → SLICE    (Tier 2: 차체 이동)
                ├── relevance ≥ 0.0 (±90°) → TURRET_INSPECT (Tier 1: 포탑만)
                └── relevance < 0.0 (후방)  → DIRECT_EXPLORE (무시)

SLICE: 파이 쪼개기 측방 기동 (arc waypoints)
  ↓ (arc 완료 or 탐색 충분)
FIND_APEX

TURRET_INSPECT: 포탑 apex 조준, 차체 정지, 2.5초 드웰
  ↓ (드웰 완료 or unseen < 2.0)
FIND_APEX

DIRECT_EXPLORE: 가장 가까운 미탐색 지점으로 직접 이동
  ↓
FIND_APEX

MANUAL: RViz 수동 목표 추종 중, coverage goal 발행 중단
  ↓ (목표 도달 WP_REACH_DIST=0.4m)
FIND_APEX

PATIENT_INTERRUPT: 환자 감지 → 탐색 일시 정지 (5초 타임아웃)
  ↓
RESUME → SLICE or FIND_APEX
```

### Apex Relevance (핵심 개념)

```python
relevance = dot(goal_direction, apex_direction)  # [-1.0, 1.0]
```

- `goal_direction`: `nav_goal` → 로봇 방향 벡터 (없으면 현재 헤딩)
- `nav_goal`: RViz에서 수동 목표 수신 시 갱신됨
- `RELEVANCE_TIER2 = 0.5` (≈±60°): Tier 2 차체 이동 허용
- `RELEVANCE_TIER1 = 0.0` (±90°): Tier 1 포탑만
- 목표 도달 후 `nav_goal` 클리어 → 헤딩 대리 사용

### 수동 제어 우선순위 (MANUAL 상태)

- `/goal_pose` 에코 구독으로 외부(RViz) 목표 감지
- `_just_published` 플래그: 우리가 발행한 것이면 무시
- 외부 목표 감지 시 즉시 MANUAL 전환, coverage goal 발행 중단
- `nav_goal_x/y` 갱신 (이후 relevance 계산에 사용)

### 포탑 조준 (SLICE/TURRET_INSPECT)

- `/apex_aim_point` (geometry_msgs/Point) 발행
- 조준점 = apex + 0.5m (로봇→apex 방향 연장)
- **카메라 중심이 코너 벽 바로 너머를 응시**하도록 의도적 오프셋
- `target_manager_node`가 구독 → SEARCH 중 sine sweep 대신 이 방향 고정

### 커버리지 활성화

```python
# launch parameter (기본 false):
self.declare_parameter('coverage_enabled_on_boot', False)

# 런타임 override:
ros2 topic pub /coverage_enable std_msgs/msg/Bool "data: true" --once
```

### 주요 파라미터 상수

```python
APEX_MIN_DIST   = 0.8   # Apex 최소 탐지 거리 (m)
APEX_MAX_DIST   = 6.0   # Apex 최대 탐지 거리 (m)
SLICE_RADIUS    = 1.5   # 파이쪼개기 원호 반경 (m)
SLICE_STEP_M    = 0.4   # 원호 이동 보폭 (m)
ASSIST_ANGLE_THRESH = 75°  # SLICE 웨이포인트 허용 헤딩 편차
TURRET_DWELL_SEC = 2.5  # TURRET_INSPECT 체류 시간 (초)
APEX_AIM_OFFSET = 0.5   # 포탑 조준 오프셋 (m, apex 너머)
FOV_RAD         = 1.089 # 카메라 수평 FOV (≈62°)
CAM_RANGE_M     = 4.0   # 유효 가시거리 (m)
```

---

## B-7. Nav2 설정 요점

```yaml
min_vel_x: 0.0          # 후진 금지 (전진 우선 정책)
max_vel_x: 0.65         # 110rpm × 2π × 0.042 / 60 = 0.69, 안전 마진
max_vel_theta: 1.15     # 포탑 최대 각속도와 동일
inflation_radius: 0.55  # 장애물 팽창 반경
robot_base_frame: base_footprint
```

---

## B-8. 시뮬레이션 실행

### 기본 (Gazebo만)
```bash
source ~/.bashrc
ros2 launch ugv_bringup gazebo.launch.py
```

### 전체 스택 (SLAM + Nav2 + Vision)
```bash
ros2 launch ugv_bringup slam_nav_sim.launch.py
# coverage 즉시 시작:
ros2 launch ugv_bringup slam_nav_sim.launch.py coverage_enabled_on_boot:=true
```

### 실행 순서 (slam_nav_sim.launch.py)
```
0초:  Gazebo + 로봇 스폰
4초:  SLAM Toolbox (맵 생성 시작)
8초:  Nav2 (자율주행 스택)
10초: YoloPoseNode + TargetManagerNode
12초: VisionCoverageNavigator
```

---

## B-9. 실로봇 vs 시뮬 차이점

| 항목 | 실로봇 | 시뮬 |
|------|--------|------|
| 포탑 명령 | `/turret_cmd` (Vector3, Teensy 픽셀 오차) | `/turret_yaw_cmd`, `/turret_pitch_cmd` (Float64, rad/s) |
| 오도메트리 | `wheel_encoders` + IMU → odometry_node | Gazebo DiffDrive 플러그인 직접 |
| Joint 상태 | `/measured_joint_states` | `/joint_states` |
| LiDAR | sllidar_ros2 (하드웨어) | Gazebo GpuLidar 플러그인 |
| 카메라 | RealSense SDK | Gazebo RgbdCamera 플러그인 |

`target_manager_node`는 두 토픽 모두 발행 (`_cmd_turret()` 내부에서 병렬 발행).

---

## B-10. 빌드 방법

```bash
cd ~/ugv_ws
source /opt/ros/humble/setup.bash
# 시뮬용 빌드 (하드웨어 패키지 제외)
colcon build --symlink-install \
  --packages-skip micro_ros_setup micro_ros_agent uros sllidar_ros2
```

---

## B-11. 알려진 이슈 및 주의사항

| 이슈 | 내용 |
|------|------|
| `laser_frame` TF drop | gazebo.launch.py 단독 실행 시 SLAM 없어 map→odom 체인 없음. 정상 현상. |
| ign gazebo 경고 | ros_gz_sim이 ign gazebo 6 (Fortress) 내부 호출. 시스템에는 Harmonic(8)도 공존. 동작에 문제 없음. |
| numpy 버전 충돌 | opencv-python 4.13은 numpy≥2 요구. ugv_ws는 numpy 1.26.4 사용. 경고는 나오지만 동작함. |
| turret_yaw_cmd 충돌 가능성 | vision_coverage_navigator와 target_manager 모두 `/apex_aim_point` 통해 조율. navigator가 직접 turret cmd 발행 금지. |

---

## 부록: 토픽 맵 요약 (ugv_ws)

```
/cmd_vel          ← Nav2 DWB planner → Gazebo DiffDrive
/odom             ← Gazebo DiffDrive → Nav2, EKF
/scan             ← Gazebo GpuLidar → SLAM Toolbox, Nav2 costmap
/map              ← SLAM Toolbox → VisionCoverageNavigator
/joint_states     ← Gazebo → TargetManager, VisionCoverageNavigator
/goal_pose        ← RViz(수동) or VisionCoverageNavigator → Nav2
/target_detection ← YoloPoseNode → TargetManager, VisionCoverageNavigator
/apex_aim_point   ← VisionCoverageNavigator → TargetManager (포탑 조준 조율)
/coverage_enable  ← 외부 명령 → VisionCoverageNavigator
/visual_coverage  → RViz (커버리지 맵 시각화)
/slice_debug      → RViz (Apex, arc 경로 시각화)
/patient_markers  → RViz (발견 환자 마커)
/turret_yaw_cmd   ← TargetManager → Gazebo JointController
/turret_pitch_cmd ← TargetManager → Gazebo JointController
```
