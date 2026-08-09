# A.R.G.U.S. UGV 프로젝트 전체 문서

> **A.R.G.U.S.** (Autonomous Rescue Ground Unit System)  
> 구조 수색 자율 지상 로봇 시스템  
> ROS2 Humble · Gazebo Harmonic (gz-sim8) · Nav2 · SLAM Toolbox · YOLOv8n-pose

---

## 목차

1. [프로젝트 개요](#1-프로젝트-개요)
2. [하드웨어 사양](#2-하드웨어-사양)
3. [워크스페이스 구조](#3-워크스페이스-구조)
4. [패키지별 상세 설명](#4-패키지별-상세-설명)
5. [핵심 알고리즘: Slicing the Pie + Perception Assistance Layer](#5-핵심-알고리즘)
6. [노드별 상세 스펙](#6-노드별-상세-스펙)
7. [커스텀 메시지 정의](#7-커스텀-메시지-정의)
8. [Nav2 파라미터 상세](#8-nav2-파라미터-상세)
9. [전체 토픽 플로우 맵](#9-전체-토픽-플로우-맵)
10. [시뮬레이션 vs 실로봇 차이점](#10-시뮬레이션-vs-실로봇-차이점)
11. [빌드 및 실행](#11-빌드-및-실행)
12. [런타임 조작 명령](#12-런타임-조작-명령)
13. [알려진 이슈 및 해결 이력](#13-알려진-이슈-및-해결-이력)

---

## 1. 프로젝트 개요

A.R.G.U.S.는 재난 구조 상황(건물 내부 수색)에서 자율적으로 환자를 탐색·위치 확인·트리아지 분류하는 UGV(Unmanned Ground Vehicle) 시스템이다.

### 핵심 임무 흐름

```
건물 내부 스폰 → [대기] → /coverage_enable true 수신
  → Slicing the Pie 전술 기동으로 방 구석구석 탐색
  → YOLOv8n-pose로 인체 골격 감지
  → ML 기반 트리아지 분류 (L1 Critical / L2 Urgent / L3 Normal)
  → RViz 3D 맵에 환자 위치·중증도 마킹
  → 운용자가 RViz에서 /goal_pose 발행 시 즉시 수동 모드 전환
  → 목적지 도착 후 자율 탐색 재개
```

### 개발 환경

| 항목 | 내용 |
|------|------|
| 플랫폼 | Ubuntu 22.04 (데스크탑 개발), Jetson Orin aarch64 (실로봇 배포) |
| ROS 버전 | ROS2 Humble |
| 시뮬레이터 | Gazebo Harmonic (gz-sim8) |
| 자율항법 | Nav2 (DWB 로컬 플래너) |
| SLAM | SLAM Toolbox (online_async) |
| 컴퓨터 비전 | YOLOv8n-pose (Ultralytics), RandomForest 트리아지 분류기 |

---

## 2. 하드웨어 사양

### 2.1 차체 (Chassis)

| 항목 | 수치 |
|------|------|
| 형태 | 6륜 스키드 스티어링 (Skid-Steer) |
| 차체 크기 | 50 × 34 × 14 cm (L × W × H) |
| 차체 질량 | 5.0 kg |
| 구동 방식 | 좌측 3륜 / 우측 3륜 독립 속도 제어 |
| 최대 속도 | 0.65 m/s (안전 마진 적용, 이론치 0.69 m/s) |
| 모터 | 110 RPM × 2π × 0.06 m / 60 = 0.69 m/s |
| 바퀴 반경 | 0.042 m |
| 바퀴 트랙 폭 (물리) | 0.1625 m |
| 슬립 보정 계수 | 1.45 (실로봇 오도메트리 튜닝값) |
| 엔코더 분해능 | 3960 ticks/rev |

**6개 휠 조인트 이름** (URDF 기준):
- `front_left_wheel_joint`, `front_right_wheel_joint`
- `mid_left_wheel_joint`, `mid_right_wheel_joint`
- `rear_left_wheel_joint`, `rear_right_wheel_joint`

### 2.2 포탑 (Turret) — 2-DOF

| 항목 | 수치 |
|------|------|
| 자유도 | 2-DOF: Yaw(수평) + Pitch(수직) |
| Yaw 범위 | ±90° (-1.57 ~ +1.57 rad) |
| Pitch 범위 | ±30° (-0.52 ~ +0.52 rad) |
| 최대 각속도 | 1.15 rad/s (110 RPM / 10기어) |
| 포탑 마운트 위치 | 차체 상단 (z=0.245 m from base_link) |
| 포탑 크기 | 6 × 12 × 30 cm |
| 포탑 질량 | 1.0 kg |

**포탑 조인트 체인**:
```
base_link
  └── turret_yaw_joint (revolute, z축 회전)
        └── turret_link
              └── turret_pitch_joint (revolute, y축 회전)
                    └── gun_link
                          └── camera_joint (fixed)
                                └── camera_link  ← 카메라 장착 위치
```

### 2.3 센서

#### LiDAR
- 위치: `laser_frame` (차체 앞쪽 x=0.245 m, z=0.08 m from base_link)
- 시뮬: Gazebo GPU LiDAR — 360° 1080 샘플, 범위 0.12~25 m, 10 Hz
- 실로봇: RPLIDAR A1/A3 (sllidar_ros2 패키지 — 데스크탑 빌드 제외)
- 토픽: `/scan` (sensor_msgs/LaserScan)

#### RGB-D 카메라
- 위치: `camera_link` (gun_link에 고정, 포탑 끝단)
- 시뮬: Gazebo RGBD Camera
  - 해상도: 640 × 480, 15 Hz
  - 수평 FOV: 1.089 rad (≈ 62°)
  - 유효 거리: 0.1 ~ 8.0 m
  - focal_px: 535 px (= 320 / tan(1.089/2))
- 실로봇: Intel RealSense D435i (realsense2_camera 패키지 — 데스크탑 빌드 제외)
- RGB 토픽: `/camera/camera/color/image_raw`
- Depth 토픽: `/camera/camera/aligned_depth_to_color/image_raw`

#### IMU (실로봇 전용)
- RealSense D435i 내장 IMU
- 토픽: `/camera/imu`
- 용도: 포탑 Yaw 상보 필터 (complementary filter, α=0.98)

---

## 3. 워크스페이스 구조

```
~/ugv_ws/
├── src/
│   ├── ugv_bringup/          # 마스터 런치 + 월드 파일
│   │   ├── launch/
│   │   │   ├── gazebo.launch.py          # Gazebo + RSP + 브리지 + RViz
│   │   │   ├── slam_nav_sim.launch.py    # 전체 시뮬 스택 (메인 런치)
│   │   │   ├── slam_nav_bringup.launch.py# 실로봇 전체 스택
│   │   │   └── robot.launch.py           # 실로봇 기본 런치
│   │   └── worlds/
│   │       └── rescue_building.sdf       # 구조 훈련 건물 시뮬 월드
│   │
│   ├── ugv_description/      # URDF/Xacro 로봇 모델
│   │   └── urdf/
│   │       └── ugv.urdf.xacro
│   │
│   ├── ugv_msgs/             # 커스텀 ROS2 메시지 정의
│   │   └── msg/
│   │       ├── TargetDetection.msg
│   │       ├── ChassisCommand.msg
│   │       └── TurretCommand.msg
│   │
│   ├── ugv_navigation/       # 오도메트리 + Nav2 설정
│   │   ├── ugv_navigation/
│   │   │   └── odometry_node.py          # 바퀴 엔코더 오도메트리 (실로봇)
│   │   ├── config/
│   │   │   ├── nav2_params.yaml          # 시뮬 Nav2 파라미터
│   │   │   ├── nav2_params_robot.yaml    # 실로봇 Nav2 파라미터
│   │   │   ├── mapper_params_online_async.yaml  # SLAM Toolbox 설정
│   │   │   ├── ekf.yaml                  # EKF 오도메트리 퓨전 (실로봇)
│   │   │   └── box_filter.yaml           # 포인트클라우드 박스 필터
│   │   └── launch/
│   │       ├── ekf.launch.py
│   │       └── nav2.launch.py
│   │
│   ├── ugv_teleop/           # 조이스틱/키보드 원격제어
│   │   └── ugv_teleop/
│   │       ├── teleop_joy_node.py
│   │       └── teleop_keyboard_node.py
│   │
│   ├── ugv_vision/           # 핵심 비전 + 자율탐색 노드
│   │   └── ugv_vision/
│   │       ├── yolo_pose_node.py          # YOLOv8n-pose + 트리아지 분류
│   │       ├── target_manager_node.py     # 포탑 제어 + 환자 등록
│   │       ├── vision_coverage_navigator.py # Slicing the Pie 탐색 항법
│   │       ├── visual_coverage_map.py     # 시야 커버리지 격자 맵
│   │       ├── yolov8n-pose.pt            # YOLOv8 모델 가중치
│   │       ├── triage_model_rf_robust.pkl # RandomForest 트리아지 분류기
│   │       └── triage_scaler_robust.pkl   # 피처 스케일러
│   │
│   ├── sllidar_ros2/         # RPLIDAR 드라이버 (실로봇 전용, 저장소 미포함)
│   └── uros/                 # micro-ROS 에이전트 (실로봇 전용, 저장소 미포함)
│
├── README.md                 # 프로젝트 소개
└── docs/
    ├── SETUP.md              # 설치 가이드
    └── ARCHITECTURE.md       # 이 문서
```

---

## 4. 패키지별 상세 설명

### ugv_bringup

마스터 런치 패키지. 시뮬과 실로봇 양쪽의 시작점.

**주요 런치 파일:**

#### `gazebo.launch.py`
1. Gazebo Harmonic 실행 (`rescue_building.sdf` 월드 로드)
2. `robot_state_publisher` (URDF → TF 트리)
3. 3초 후 UGV 스폰 (`ros_gz_sim create`)
4. `ros_gz_bridge` — ROS ↔ Gazebo 토픽 브릿지:
   - `/cmd_vel`, `/odom`, `/tf`, `/clock`, `/joint_states`
   - `/scan`, `/turret_yaw_cmd`, `/turret_pitch_cmd`
   - 카메라: `/camera/image` → `/camera/camera/color/image_raw`
   - Depth: `/camera/depth_image` → `/camera/camera/aligned_depth_to_color/image_raw`
5. RViz2 (`ugv.rviz` 설정 파일)

#### `slam_nav_sim.launch.py` (시뮬 메인 런치)
순차 지연 실행 구조:

| 지연 | 구성요소 |
|------|----------|
| 0 s | `gazebo.launch.py` (Gazebo + 스폰 + 브릿지 + RViz) |
| 4 s | SLAM Toolbox `online_async_launch.py` |
| 8 s | Nav2 `navigation_launch.py` (slam=True) |
| 10 s | `yolo_pose_node`, `target_manager_node` |
| 12 s | `vision_coverage_navigator` |

**런치 인자:**
```
coverage_enabled_on_boot: false (기본값)
  - false: 스폰 후 대기, /coverage_enable true 신호 대기
  - true:  스폰 즉시 커버리지 탐색 시작
```

---

## 5. 핵심 알고리즘

### 5.1 Slicing the Pie (파이 쪼개기)

구조대원이 문 밖에서 코너를 축으로 돌며 방 내부 사각지대를 한 조각씩 열어가는 전술적 시야 확보 기동을 로봇에 적용한 알고리즘.

```
전통적 8방향 스캔 (폐기):
  로봇 제자리 → 고정 방향 8섹터 회전 → 기계적, 맥락 없음

Slicing the Pie:
  라이다 맵 분석 → 벽 모서리(Apex) 식별
    → Apex를 시선 고정점으로 포탑 조준
    → 로봇은 Apex 중심으로 측방 호 이동
    → 호를 따라 이동할수록 코너 너머 사각지대가 열림
    → FOV가 채워지면 다음 Apex로 이동
```

**Apex 정의 (URDF 맵 기준):**
- OccupancyGrid에서 `cell > 50` (점유된 벽 셀)
- 상하좌우 4방향 중 자유공간(`cell == 0`) 이웃이 2개 이상인 셀
- = 벽이 두 방향으로 자유공간과 맞닿는 모서리/끝점
- 탐색 범위: 로봇으로부터 0.8 ~ 6.0 m

**Apex 우선순위 점수:**
```python
score = unseen_beyond(apex) / distance(robot, apex)
```
Apex 너머 미탐색 셀이 많을수록 + 가까울수록 높은 점수.

### 5.2 Perception Assistance Layer (인식 보조 레이어)

커버리지 탐색은 **주행의 주도권을 갖지 않는다**. 운용자의 이동 의도(nav_goal) 방향에 얼마나 일치하는지에 따라 행동을 결정한다.

**핵심 척도: Apex Relevance (관련성)**
```python
relevance = dot(goal_direction, apex_direction)
          = cos(goal 방향과 apex 방향 사이 각도)
```

- `nav_goal`이 있으면: `goal_direction = normalize(nav_goal - robot_pos)`
- `nav_goal`이 없으면: `goal_direction = (cos(θ), sin(θ))` (현재 헤딩)

**Tier 분기 기준:**

| relevance 값 | 해석 | 동작 |
|---|---|---|
| ≥ 0.5 (±60° 이내) | Tier 2: 진행 방향에 apex 있음 | 차체 이동 + 파이 쪼개기 |
| ≥ 0.0 (±90° 이내) | Tier 1: 측방 apex | 포탑만 응시, 차체 정지 |
| < 0.0 (후방 apex) | 무관 | 무시 → DIRECT_EXPLORE |

### 5.3 VisualCoverageGrid (이중 맵 구조)

```
SLAM OccupancyGrid (/map)      VisualCoverageGrid (/visual_coverage)
  - 라이다로 벽/장애물 감지       - 카메라 FOV가 실제로 본 영역 추적
  - 이동 경로 계획에 사용          - 탐색 완료 여부 판단에 사용
  
교집합: SLAM에서 자유공간 && VisualCoverage에서 Unseen
       = 이동 가능하지만 아직 못 본 구역 → 탐색 대상
```

**격자 값:**
```
 0 = Unseen  (자유공간이지만 카메라로 못 봄)
 1 = Seen    (카메라 FOV가 통과한 셀)
-1 = Unknown (SLAM도 모르는 구역)
```

**FOV 업데이트 방식:**
- 카메라 각도 = `robot_theta + turret_yaw`
- 수평 FOV 1.089 rad 내에서 30개의 레이를 팬아웃
- 각 레이를 0.3 m ~ 4.0 m 범위까지 격자 해상도(0.2 m)의 0.8배 간격으로 샘플링
- 벽(-1) 셀에 도달하면 해당 레이 종료

**격자 사양:**
- 해상도: 0.2 m/cell
- 크기: 40 m × 40 m (200 × 200 셀)
- 원점: 월드 원점 중심 (origin = -20.0 m)

---

## 6. 노드별 상세 스펙

### 6.1 VisionCoverageNavigator (`vision_coverage_navigator.py`)

**역할:** 자율 탐색 주 노드. 슬라이싱 더 파이 기반 시야 확보 기동.

**파라미터:**
```
coverage_enabled_on_boot: bool (기본값: false)
  - 런치 시 즉시 커버리지 시작 여부
  - 런타임에는 /coverage_enable (Bool) 토픽으로 override 가능
```

**상태 머신 (NavState):**

```
IDLE
  조건: SLAM 맵 수신 완료 AND coverage_enabled == True
  전이 → FIND_APEX

FIND_APEX
  동작: _find_best_apex() 호출
        → apex 발견: _goal_relevance() 계산 후 Tier 분기
          · relevance >= 0.5 → arc_wps 생성 → SLICE
          · relevance >= 0.0 → _inspect_start 기록 → TURRET_INSPECT
          · relevance <  0.0 → DIRECT_EXPLORE
        → apex 없음: → DIRECT_EXPLORE

SLICE (Tier 2 — 차체 이동 + 파이 쪼개기)
  동작:
    1. _publish_apex_aim() — 포탑에 조준점 전달
    2. arc_wps[arc_idx] 까지 이동
    3. 웨이포인트 방향과 현재 헤딩의 각도 차이 확인:
       · |diff| ≤ 75° → _send_goal() 발행
       · |diff| > 75° → 이번 틱 스킵 (강제 이동 금지)
    4. _unseen_score_beyond(apex) < 2.0 이면 조기 종료
    완료 → FIND_APEX

TURRET_INSPECT (Tier 1 — 포탑만, 차체 정지)
  동작:
    1. _publish_apex_aim() — 포탑에 조준점 전달
    2. elapsed > 2.5초 OR unseen < 2.0 이면 종료
    완료 → FIND_APEX

DIRECT_EXPLORE (Apex 없을 때)
  동작: VisualCoverageGrid.get_unseen_frontiers()에서
        가장 가까운 미탐색 지점으로 _send_goal()
        → FIND_APEX (매 tick Apex 재탐색)

MANUAL (운용자 nav 우선)
  진입 조건: /goal_pose 에코 수신 시 (_just_published == False)
  동작: 로봇이 수동 목표에 도달할 때까지 대기
        (Nav2가 이미 목표를 받았으므로 추가 발행 없음)
  완료 조건: dist(robot, _manual_goal) < 0.4 m
  완료 → nav_goal_x/y = None, FIND_APEX

PATIENT_INTERRUPT (환자 감지)
  진입 조건: /target_detection 수신 시 (status == "TRACKING")
  동작: 5초 타이머 후 RESUME
  완료 → RESUME

RESUME
  동작: arc_wps 남아있으면 SLICE, 없으면 FIND_APEX
```

**구독 토픽:**

| 토픽 | 타입 | 용도 |
|------|------|------|
| `/odom` | nav_msgs/Odometry | 로봇 위치·헤딩 갱신 |
| `/joint_states` | sensor_msgs/JointState | turret_yaw_joint 위치 읽기 |
| `/map` | nav_msgs/OccupancyGrid | SLAM 맵 수신 (Apex 탐지용) |
| `/target_detection` | ugv_msgs/TargetDetection | 환자 감지 인터럽트 |
| `/goal_pose` | geometry_msgs/PoseStamped | RViz 수동 목표 에코 감지 |
| `/coverage_enable` | std_msgs/Bool | 커버리지 런타임 활성화/비활성화 |

**발행 토픽:**

| 토픽 | 타입 | 용도 |
|------|------|------|
| `/goal_pose` | geometry_msgs/PoseStamped | Nav2에 이동 목표 전달 |
| `/visual_coverage` | nav_msgs/OccupancyGrid | RViz 커버리지 맵 시각화 |
| `/slice_debug` | visualization_msgs/MarkerArray | Apex 구, 호 경로 시각화 |
| `/apex_aim_point` | geometry_msgs/Point | 포탑 조준점 (apex 너머 0.5 m) |

**타이머:**
- `_state_machine()`: 0.5초 (2 Hz)
- `_cov_update()`: 0.5초 (2 Hz) — FOV 커버리지 갱신
- `_publish_coverage()`: 0.5초 (2 Hz) — RViz 시각화

**에코 감지 메커니즘 (`_just_published` 플래그):**
```
노드가 /goal_pose를 발행할 때:
  1. _just_published = True 설정
  2. goal_pub.publish(goal) 호출
  3. /goal_pose 에코 수신 시 _goal_echo_cb 호출됨
     → _just_published == True: 플래그 False로 초기화, 무시 (자기 에코)
     → _just_published == False: 외부(RViz) 목표 → nav_goal 갱신 + MANUAL 전환
```

**전진 우선 정책 (`_send_goal`):**
- 목표 방향이 현재 헤딩에서 100° 이상 뒤쪽이면:
  1. 제자리 회전 목표를 먼저 발행 (현재 위치, 목표 방향으로 헤딩)
  2. 다음 tick에 실제 이동 목표 발행
- Nav2 `min_vel_x=0.0`으로 후진 방지

**핵심 상수:**

```python
APEX_MIN_DIST     = 0.8    # m  — Apex 최소 거리 (너무 가까우면 제외)
APEX_MAX_DIST     = 6.0    # m  — Apex 최대 탐색 거리
SLICE_RADIUS      = 1.5    # m  — 파이 쪼개기 원호 반경
SLICE_STEP_M      = 0.4    # m  — 원호 이동 보폭
WP_REACH_DIST     = 0.4    # m  — 웨이포인트 도달 판정 거리
FOV_RAD           = 1.089  # rad — 카메라 수평 FOV (~62°)
CAM_RANGE_M       = 4.0    # m  — 카메라 유효 가시거리
APEX_AIM_OFFSET   = 0.5    # m  — 포탑 조준점: apex 너머 이 거리만큼 더
RELEVANCE_TIER2   = 0.5    # dot product ≥ 0.5 → Tier 2
RELEVANCE_TIER1   = 0.0    # dot product ≥ 0.0 → Tier 1
TURRET_DWELL_SEC  = 2.5    # s  — Tier 1 포탑 응시 유지 시간
ASSIST_ANGLE_THRESH = 75°  # SLICE 웨이포인트 허용 각도 (헤딩 기준)
REVERSE_AVOID_RAD   = 100° # 제자리 회전 임계 (이보다 뒤쪽이면 먼저 회전)
```

---

### 6.2 TargetManager (`target_manager_node.py`)

**역할:** 포탑 서보 제어 + 환자 Lock-on + 환자 등록부 관리.  
시뮬과 실로봇 양쪽 인터페이스 동시 지원.

**상태 머신:**

```
SEARCH
  1. apex_aim_point 수신 중이면 (1초 이내):
       → 해당 방향으로 포탑 조준 (커버리지 모드 포탑 정렬)
     아니면:
       → 사인파 스윕 ±50°, 주기 ≈ 4.2초
  2. Pitch는 항상 0(정면 수평)으로 복귀
  3. /target_detection 수신 → TRACK

TRACK
  1. 픽셀 오차 P-controller:
       yaw_vel   = -KP_TRACK × (cx - 320)     [KP_TRACK ≈ 0.0136 rad/s/px]
       pitch_vel = -KP_TRACK × (cy - 240)     [10px 데드밴드]
  2. Lock-on 판정: |err_x| < 25px AND |err_y| < 25px AND dist > 0.1m
     → 기발견 환자 (MERGE_M=1.5m 이내) → 추적 중단, SEARCH 복귀
     → 신규 환자:
         세계 좌표 계산: gx = robot_x + dist × cos(robot_theta + turret_yaw)
         환자 등록부(confirmed{}) 추가
         patient_locations.txt 파일에 기록
         SEARCH 복귀
```

**apex_aim 연동 (VisionCoverageNavigator ↔ TargetManager):**
```
VisionCoverageNavigator:
  SLICE 또는 TURRET_INSPECT 상태에서
  → /apex_aim_point 발행 (apex + 0.5m 방향의 월드 좌표)

TargetManager:
  → apex_aim 수신 후 1초 이내이면:
      aim_angle = atan2(aim.y - robot.y, aim.x - robot.x)
      target_yaw = aim_angle - robot_theta  (로봇 상대 각도)
      포탑을 그 방향으로 정렬
  → 1초 이상 수신 없으면: apex_aim = None → 사인파 스윕 복귀
```

**포탑 제어 이중 인터페이스 (`_cmd_turret`):**

| 인터페이스 | 토픽 | 타입 | 용도 |
|---|---|---|---|
| 시뮬 | `/turret_yaw_cmd` | std_msgs/Float64 | Gazebo JointController (rad/s 속도) |
| 시뮬 | `/turret_pitch_cmd` | std_msgs/Float64 | Gazebo JointController (rad/s 속도) |
| 실로봇 | `/turret_cmd` | geometry_msgs/Vector3 | Teensy MCU 레거시 (픽셀 오차 기반) |

```python
def _cmd_turret(self, yaw_vel, pitch_vel, z_flag=0.0):
    self.yaw_pub.publish(Float64(data=yaw_vel))     # 시뮬
    self.pitch_pub.publish(Float64(data=pitch_vel)) # 시뮬
    cmd = Vector3()
    cmd.x = -yaw_vel * 100     # 실로봇 스케일 (부호 반전 주의)
    cmd.y = -pitch_vel * 100
    cmd.z = z_flag             # -1.0=탐색중, 0.0=추적중, 1.0=락온
    self.turret_pub.publish(cmd)
```

**환자 시각화 마커 (RViz):**

| 마커 | 형태 | 색상 코드 |
|------|------|---------|
| Sphere (환자 위치) | 구(0.5m) | L1:빨강, L2:주황, L3:초록 |
| Ring (기발견 구분) | 납작 실린더(0.9m) | 동일 색상, 투명도 35% |
| Text (라벨) | 텍스트 | triage_label + 방 이름 + "(CONFIRMED)" |

**방 이름 자동 판별 (`get_room_name`):**
```
y > 4.0  → Room A (x < -1.0) / Room B (x >= -1.0)
y < -4.0 → Room C (x < -1.0) / Room D (x >= -1.0)
그 외    → Main Hall
```

**튜닝 파라미터:**

```python
SEARCH_AMP   = 50°       # 사인파 탐색 진폭 (카메라 FOV 62°의 적정 범위)
SEARCH_OMEGA = 1.5 rad/s # 사인파 각속도 → 주기 ≈ 4.2초
KP_SRCH      = 2.0       # SEARCH 포지션 P게인 (rad/s per rad error)
KP_TRACK     = 0.0136    # TRACK 픽셀 P게인 (rad/s per pixel)
LOCK_PX      = 25        # Lock-on 픽셀 임계값 (±25px ≈ ±2.7°)
MERGE_M      = 1.5       # 기발견 환자 중복 판정 반경 (m)
```

**구독 토픽:**

| 토픽 | 타입 | 용도 |
|------|------|------|
| `/target_detection` | ugv_msgs/TargetDetection | YOLO 감지 결과 |
| `/odom` | nav_msgs/Odometry | 로봇 위치·헤딩 |
| `/joy` | sensor_msgs/Joy | 조이스틱 수동 입력 (2초간 제어권 양보) |
| `/joint_states` | sensor_msgs/JointState | 시뮬 포탑 각도 |
| `/measured_joint_states` | sensor_msgs/JointState | 실로봇 포탑 각도 |
| `/apex_aim_point` | geometry_msgs/Point | 커버리지 네비게이터 조준점 |

**발행 토픽:**

| 토픽 | 타입 | 용도 |
|------|------|------|
| `/turret_yaw_cmd` | std_msgs/Float64 | 시뮬 포탑 Yaw 속도 |
| `/turret_pitch_cmd` | std_msgs/Float64 | 시뮬 포탑 Pitch 속도 |
| `/turret_cmd` | geometry_msgs/Vector3 | 실로봇 Teensy 명령 |
| `/patient_markers` | visualization_msgs/MarkerArray | 환자 위치 마커 |
| `/turret_heading` | visualization_msgs/Marker | 포탑 방향 화살표 |

**타이머:**
- `control_loop()`: 0.05초 (20 Hz)
- `_publish_turret_arrow()`: 0.1초 (10 Hz)
- `republish_markers()`: 1.0초 (1 Hz) — 환자 마커 재발행

---

### 6.3 YoloPoseNode (`yolo_pose_node.py`)

**역할:** 카메라 영상에서 인체를 감지하고, 거리 추정 후 트리아지 분류.

#### 감지 파이프라인

```
RGB + Depth 동기화 수신 (ApproximateTimeSynchronizer, slop=0.1s)
  → YOLO v8n-pose 추론 (conf=0.25, GPU)
  → 각 바운딩박스에 대해:
      1. 깊이 거리 측정 (8×8 윈도우 중앙값, 유효범위 0.1~8.0m)
      2. 깊이 = 0이면 대각선 폴백: dist = focal_px × body_diag / pixel_diag
      3. 스켈레톤 정규화 → 34차원 피처 추출
      4. RandomForest 분류기 → 트리아지 레벨 (L1/L2/L3)
  → 화면 중앙에 가장 가까운 탐지체를 "best" 선택
  → 최근 10프레임 다수결 필터 적용 → 최종 레벨 결정
  → /target_detection 발행
```

#### 트리아지 분류 시스템

**34차원 피처 추출 (`extract_skeleton_features`):**
```
입력: YOLOv8 17개 키포인트 (x, y) × 2 = 34차원
정규화:
  1. 엉덩이 중심 (l_hip + r_hip 평균)을 원점으로 이동 (translation)
  2. 최대 거리로 나누어 스케일 정규화 (scale invariant)
출력: [34] float 배열
```

**분류:**
- ML 모델: RandomForest (triage_model_rf_robust.pkl)
- 스케일러: StandardScaler (triage_scaler_robust.pkl)
- 폴백 (모델 없음): 항상 L3:Normal 반환

**트리아지 레벨:**
```
L1: Critical (빨강)  — 즉각 처치 필요
L2: Urgent   (주황)  — 빠른 처치 필요
L3: Normal   (초록)  — 경상
```

#### 거리 추정 폴백

깊이 카메라 값이 0인 경우(누운 자세, 반사 등):
```python
dist = focal_px × body_diag / pixel_diag
     = 535 × 1.75 / sqrt(bbox_w² + bbox_h²)
```
누운 사람·서 있는 사람 모두 바운딩박스 대각선 길이가  
몸통 대각선(≈ 1.75 m)에 비례한다는 가정을 활용.

**카메라 파라미터:**
```
focal_px = 535.0 px   (= 320 / tan(1.089/2))
body_diag = 1.75 m    (= sqrt(1.7² + 0.4²))
해상도: 640×480
수평 FOV: 1.089 rad
```

**발행 토픽:**

| 토픽 | 타입 | 내용 |
|------|------|------|
| `/target_detection` | ugv_msgs/TargetDetection | 감지 결과 (픽셀 좌표, 거리, 트리아지) |

**구독 토픽:**

| 토픽 | 타입 | 내용 |
|------|------|------|
| `/camera/camera/color/image_raw` | sensor_msgs/Image | RGB 영상 |
| `/camera/camera/aligned_depth_to_color/image_raw` | sensor_msgs/Image | Depth 영상 |

---

### 6.4 OdometryNode (`odometry_node.py`) — 실로봇 전용

**역할:** 6개 바퀴 엔코더 + IMU 상보 필터로 오도메트리 계산.  
시뮬에서는 Gazebo OdometryPublisher 플러그인이 대신함.

**오도메트리 계산 방식:**
```
각 엔코더 틱 → 좌/우 3개 평균 → 차동구동 기구학:
  d_center = (d_left + d_right) / 2
  d_theta  = (d_right - d_left) / effective_track_width

  x += d_center × cos(θ + d_theta/2)
  y += d_center × sin(θ + d_theta/2)
  θ += d_theta
```

**포탑 Yaw 상보 필터:**
```
relative_gyro_z = imu_gyro_z - chassis_angular_vel_z
fused_yaw = 0.98 × (fused_yaw + relative_gyro_z × dt)
          + 0.02 × encoder_turret_yaw
```

**하드웨어 상수:**
```python
WHEEL_RADIUS      = 0.042 m
PHYSICAL_TRACK_WIDTH = 0.1625 m
SLIP_FACTOR       = 1.45
TRACK_WIDTH       = 0.1625 × 1.45 = 0.235625 m (유효값)
TICKS_PER_REV     = 3960
M_PER_TICK        = 2π × 0.042 / 3960 ≈ 0.0000666 m/tick
```

**구독:**
- `wheel_encoders` (Int32MultiArray) — 6개 바퀴 엔코더 틱
- `turret_fb` (Int32MultiArray) — 포탑 엔코더 [yaw_mrad, pitch_mrad]
- `camera/imu` (sensor_msgs/Imu) — RealSense IMU

**발행:**
- `wheel/odom` (nav_msgs/Odometry) — 50 Hz
- `measured_joint_states` (sensor_msgs/JointState) — 10 Hz

---

### 6.5 TeleopJoyNode (`teleop_joy_node.py`)

**역할:** 조이스틱으로 차체 및 포탑 수동 제어.

**매핑:**

| 조이스틱 입력 | 동작 | 스케일 |
|---|---|---|
| axes[1] (좌스틱 상하) | 전진/후진 (linear.x) | × 0.5 |
| axes[0] (좌스틱 좌우) | 회전 (angular.z) | × 1.0 |
| axes[3] (우스틱 좌우) | 포탑 Yaw | × 200.0 |
| axes[4] (우스틱 상하) | 포탑 Pitch | × 150.0 |
| buttons[5] | Lock-on 신호 (z=1.0) | — |

**데드존:** ±0.05

---

## 7. 커스텀 메시지 정의

### ugv_msgs/TargetDetection.msg

```
int32  target_id         # 탐지 ID (현재 미사용, 예약)
float64 x                # 바운딩박스 중심 픽셀 X
float64 y                # 바운딩박스 중심 픽셀 Y
float64 distance         # 추정 거리 (m) — depth 또는 대각선 폴백
float64 global_x         # 세계 좌표 X (Lock-on 후 설정)
float64 global_y         # 세계 좌표 Y (Lock-on 후 설정)
string  status           # "TRACKING" | "LOCKED"
int32   triage_level     # 1=Critical, 2=Urgent, 3=Normal
string  triage_label     # "L1:Critical" | "L2:Urgent" | "L3:Normal"
bool    is_in_center_fov # 화면 중앙 FOV 내 여부 (현재 미사용)
```

### ugv_msgs/ChassisCommand.msg

```
float64 drive_speed_fl   # 전좌 바퀴 속도 (m/s)
float64 drive_speed_ml   # 중좌 바퀴 속도 (m/s)
float64 drive_speed_rl   # 후좌 바퀴 속도 (m/s)
float64 drive_speed_fr   # 전우 바퀴 속도 (m/s)
float64 drive_speed_mr   # 중우 바퀴 속도 (m/s)
float64 drive_speed_rr   # 후우 바퀴 속도 (m/s)
bool    center_drop_active  # 중앙 도어(드롭) 액추에이터 활성화
```

### ugv_msgs/TurretCommand.msg

```
float64 x               # Yaw 명령 (픽셀 오차 또는 rad/s, 인터페이스에 따라 다름)
float64 y               # Pitch 명령
float64 z               # 상태 플래그: -1.0=탐색, 0.0=추적, 1.0=락온
bool    is_locked_on    # Lock-on 상태
uint8   scan_mode       # 0=MANUAL, 1=SECTOR, 2=TRACKING
uint8   SCAN_MODE_MANUAL = 0
uint8   SCAN_MODE_SECTOR = 1
uint8   SCAN_MODE_TRACKING = 2
```

---

## 8. Nav2 파라미터 상세

파일: `ugv_navigation/config/nav2_params.yaml`

### 로컬 플래너: DWB (Dynamic Window Approach B*)

```yaml
FollowPath:
  plugin: "dwb_core::DWBLocalPlanner"

  # 속도 제한
  min_vel_x:    0.0    # 후진 금지 — 전진 우선 정책
  max_vel_x:    0.65   # 최대 전진 속도 (m/s)
  max_vel_theta: 1.15  # 최대 회전 속도 (rad/s)
  max_speed_xy:  0.65
  max_speed_theta: 1.15

  # 가속도 제한
  acc_lim_x:      3.0
  acc_lim_theta:  4.0
  decel_lim_x:   -3.0
  decel_lim_theta: -4.0

  # 제어 주파수
  controller_frequency: 10.0 Hz

  # Critics (가중치)
  BaseObstacle.scale: 50.0     # 장애물 회피 (높은 가중치)
  PathAlign.scale:    32.0     # 경로 방향 정렬
  PathDist.scale:     32.0     # 경로 거리 유지
  RotateToGoal.scale: 32.0     # 목표 방향 회전
  GoalAlign.scale:    24.0     # 목표 방향 정렬
  GoalDist.scale:     24.0     # 목표 거리
```

### 코스트맵

```yaml
footprint: "[[-0.25, -0.2], [0.25, -0.2], [0.25, 0.2], [-0.25, 0.2]]"
# 실제 차체 크기 50×40cm를 정확히 반영

inflation_radius: 0.55   # 장애물 팽창 반경 (차체 폭 20cm + 안전 마진)

로컬 코스트맵:
  크기: 3m × 3m (rolling window)
  해상도: 0.05 m/cell
  LiDAR 최대 감지: 24.5 m (marking), 25.0 m (raytrace)

글로벌 코스트맵:
  SLAM 맵 기반 (static_layer)
  allow_unknown: True (미탐색 구역 통과 허용)
```

### 글로벌 플래너

```yaml
plugin: "nav2_navfn_planner/NavfnPlanner"
use_astar: True        # A* 알고리즘
allow_unknown: True    # 미탐색 구역으로도 경로 계획 허용
tolerance: 0.5         # 목표 도달 허용 오차 (m)
```

### 목표 도달 허용 오차

```yaml
xy_goal_tolerance:  0.25 m
yaw_goal_tolerance: 0.25 rad
```

---

## 9. 전체 토픽 플로우 맵

```
[Gazebo / 실로봇 하드웨어]
        │
        ├─/odom ──────────────────────────────┬──> VisionCoverageNavigator
        │                                     ├──> TargetManager
        │                                     └──> Nav2 (EKF 퓨전, 실로봇)
        │
        ├─/joint_states ──────────────────────┬──> VisionCoverageNavigator (turret_yaw)
        │                                     └──> TargetManager (turret_yaw/pitch)
        │
        ├─/scan ──────────────────────────────> Nav2 코스트맵
        │                                    └─> SLAM Toolbox → /map
        │
        ├─/camera/camera/color/image_raw ────> YoloPoseNode ─┐
        └─/camera/camera/aligned_depth_to_color/image_raw ──┘
                                                              │
                                              TargetDetection │
                                              /target_detection
                                                    │     │
                              VisionCoverageNavigator     TargetManager
                              (환자 인터럽트 신호)         (추적·등록)

[VisionCoverageNavigator]
        │
        ├─/goal_pose ────────────────────────> Nav2 BT Navigator → /cmd_vel → 차체
        │  (발행 + 에코 수신)
        ├─/visual_coverage ──────────────────> RViz2
        ├─/slice_debug ──────────────────────> RViz2 (Apex 구, 호 경로)
        └─/apex_aim_point ───────────────────> TargetManager

[TargetManager]
        │
        ├─/turret_yaw_cmd ───────────────────> Gazebo JointController (시뮬)
        ├─/turret_pitch_cmd ─────────────────> Gazebo JointController (시뮬)
        ├─/turret_cmd ───────────────────────> Teensy MCU (실로봇)
        ├─/patient_markers ──────────────────> RViz2 (환자 위치 3D 마커)
        └─/turret_heading ───────────────────> RViz2 (포탑 방향 화살표)

[운용자]
        │
        ├─RViz2 2D Nav Goal → /goal_pose ────> VisionCoverageNavigator (에코 감지)
        │                                    └─> Nav2 BT Navigator
        └─/coverage_enable (Bool) ───────────> VisionCoverageNavigator
```

---

## 10. 시뮬레이션 vs 실로봇 차이점

| 항목 | 시뮬 (Gazebo Harmonic) | 실로봇 (Jetson Orin) |
|------|----------------------|---------------------|
| 오도메트리 | Gazebo OdometryPublisher 플러그인 → `/odom` | `odometry_node.py` (바퀴 엔코더 + IMU 상보 필터) |
| 포탑 각도 | `/joint_states` (Gazebo JointStatePublisher) | `/measured_joint_states` (OdometryNode 상보 필터 출력) |
| LiDAR | Gazebo GPU LiDAR → `ros_gz_bridge` → `/scan` | RPLIDAR A1/A3 (sllidar_ros2) |
| 카메라 | Gazebo RGBD Camera → `ros_gz_bridge` | RealSense D435i (realsense2_camera) |
| 포탑 제어 | `/turret_yaw_cmd`, `/turret_pitch_cmd` (Float64 rad/s) | `/turret_cmd` (Vector3, Teensy 픽셀 오차) |
| 빌드 | sllidar_ros2, realsense2_camera 제외 | 전체 포함 |
| 런치 | `slam_nav_sim.launch.py` | `slam_nav_bringup.launch.py` |
| 시간 소스 | `use_sim_time: True` | `use_sim_time: False` |
| EKF | 없음 (Gazebo 오도메트리 직접 사용) | `ekf.launch.py` (robot_localization) |
| 스폰 | `ros_gz_sim create` | 실물 하드웨어 전원 인가 |

**TargetManager 양쪽 지원 방식:**
```python
# 시뮬과 실로봇 포탑 각도 동시 구독 (두 토픽 중 수신되는 것을 사용)
self.create_subscription(JointState, '/joint_states',          self.joint_cb, 10)
self.create_subscription(JointState, '/measured_joint_states', self.joint_cb, 10)
```

---

## 11. 빌드 및 실행

### 빌드

```bash
cd ~/ugv_ws

# 의존성 설치
rosdep install --from-paths src --ignore-src -r -y

# 빌드 (sllidar, realsense 제외 — 데스크탑 환경)
colcon build --packages-skip sllidar_ros2 realsense2_camera \
             --symlink-install

source install/setup.bash
```

> **주의:** `ugv_bringup/package.xml`에서 `sllidar_ros2`와 `realsense2_camera`의  
> `<exec_depend>` 항목은 이미 제거되어 있음.  
> 실로봇 Jetson 환경에서는 해당 패키지를 포함하여 빌드.

### 시뮬 실행

```bash
# 전체 스택 (Gazebo + SLAM + Nav2 + Vision + Coverage)
ros2 launch ugv_bringup slam_nav_sim.launch.py

# 스폰 즉시 커버리지 시작
ros2 launch ugv_bringup slam_nav_sim.launch.py coverage_enabled_on_boot:=true
```

---

## 12. 런타임 조작 명령

### 커버리지 활성화/비활성화

```bash
# 커버리지 탐색 시작
ros2 topic pub /coverage_enable std_msgs/msg/Bool "data: true" --once

# 커버리지 탐색 중지
ros2 topic pub /coverage_enable std_msgs/msg/Bool "data: false" --once
```

### 수동 이동 목표 (RViz 대신 CLI)

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}' \
  --once
```

> 위 명령은 VisionCoverageNavigator의 에코 감지를 트리거하여 MANUAL 상태로 전환됨.

### 상태 모니터링

```bash
# 현재 NavState 확인 (로그 출력 모니터링)
ros2 topic echo /rosout | grep VisionCoverageNavigator

# 커버리지 맵 확인
ros2 topic echo /visual_coverage --no-arr  # 메타데이터만

# 환자 등록 현황
cat patient_locations.txt

# Apex 조준점 실시간 확인
ros2 topic echo /apex_aim_point
```

### 포탑 수동 명령 (시뮬)

```bash
# Yaw 정방향 1 rad/s
ros2 topic pub /turret_yaw_cmd std_msgs/msg/Float64 "data: 1.0" --once

# Pitch 정방향 0.5 rad/s
ros2 topic pub /turret_pitch_cmd std_msgs/msg/Float64 "data: 0.5" --once
```

---

## 13. 알려진 이슈 및 해결 이력

### [해결] ugv_bringup 빌드 실패

**원인:** `package.xml`에 `sllidar_ros2`, `realsense2_camera`가 `exec_depend`로 선언되어 있으나 데스크탑 환경에 없음.  
**해결:** 두 패키지를 `package.xml`에서 제거 → 데스크탑 빌드 성공.

---

### [해결] 커버리지가 수동 Goal보다 우선 동작

**원인:** RViz에서 /goal_pose를 발행해도 커버리지 navigator가 바로 덮어씀.  
**해결:**  
- `/goal_pose` 에코 구독 추가
- `_just_published` 플래그로 자기 에코 필터링
- 외부 goal 감지 시 MANUAL 상태로 전환
- 수동 목표 도달 후 자동으로 FIND_APEX 복귀

---

### [해결] 포탑이 Apex 자체를 조준 (코너 점 바라봄)

**원인:** apex 좌표를 그대로 조준점으로 사용 → 코너 점 자체를 응시.  
**해결:** `APEX_AIM_OFFSET = 0.5 m` 적용.  
```python
aim = apex + 0.5m × normalize(apex - robot)  # apex 너머 0.5m 지점
```

---

### [해결] 스폰 즉시 자율 탐색 시작

**원인:** `coverage_enabled` 기본값이 True였음.  
**해결:** 기본값 False로 변경. `/coverage_enable true` 또는 `coverage_enabled_on_boot:=true` 로 명시적 활성화.

---

### [해결] 커버리지가 장애물 방향으로 돌진

**원인:** 옆/뒤쪽 Apex에도 Tier 2 차체 이동이 발동 → 진행 방향과 무관한 방향으로 이동.  
**해결:**  
- Relevance 티어 시스템 도입 (dot product 기반)
- `ASSIST_ANGLE_THRESH = 75°`: SLICE 웨이포인트 방향이 현재 헤딩과 75° 이상 벗어나면 이번 틱 스킵
- Tier 2는 nav_goal 방향 ±60° 이내 Apex에만 발동

---

### [해결] SLAM 맵 미수신

**원인:** slam_nav_sim.launch.py에서 SLAM 노드가 너무 일찍 시작되어 Gazebo와 브릿지 초기화 전에 실행됨.  
**해결:** SLAM을 4초 후 TimerAction으로 지연 시작.

---

*이 문서는 ugv_ws 프로젝트의 전체 아키텍처, 알고리즘, 인터페이스를 다른 AI 또는 개발자가 완전히 이해할 수 있도록 작성되었습니다.*
