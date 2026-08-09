# ugv_ws 실행 가이드

시뮬레이션을 켜고 RViz에서 로봇을 움직이며 자율 탐색·조난자 탐지·화재 감지를 돌리는 방법.
최초 1회 환경 설치는 **[SETUP.md](SETUP.md)** 를 먼저 따르고, 설치가 끝났다면 이 문서만 보면 된다.
전체 노드·토픽 설계는 **[ARCHITECTURE.md](ARCHITECTURE.md)** 참조.

> ROS는 노드마다 터미널 하나를 점유한다(로그가 계속 흐르는 창엔 다른 명령을 칠 수 없다).
> 보통 **터미널 2개**면 된다 — [터미널 1]로 시뮬을 켜두고, [터미널 2]로 명령을 보낸다.
> 각 블록은 새 터미널에 통째로 붙여넣으면 된다(맨 앞 소싱 줄 포함).

---

## 0. 이 프로젝트가 하는 일

6륜 UGV가 재난 건물 내부를 **SLAM으로 지도화 + Nav2로 자율주행**하면서,
카메라 **YOLOv8n-pose로 조난자를 탐지**하고 중증도를 분류해 기록한다.
LiDAR 점유격자와 별개로 **카메라가 실제로 관측한 영역**을 커버리지 격자로 관리하며
미관측 구역을 우선 훑고, **열화상으로 화재를 감지**해 그 구역을 피해 다닌다.

조난자를 발견해도 그 자리에서 마킹하지 않는다 — **접근 → 정지 → 포탑 조준 → 표본 중앙값**으로
좌표를 확정한다. 움직이면서 마킹하면 위치가 크게 튀기 때문이다.

핵심 패키지: `ugv_bringup`(launch) · `ugv_description`(URDF/월드/RViz) · `ugv_navigation`(Nav2/SLAM) ·
`ugv_vision`(탐지 + 탐사 + 화재) · `ugv_teleop`(수동 조종).

---

## 1. 사전 준비 — 빌드 & 소싱

전체 미션 launch는 Nav2 + slam_toolbox를 쓴다. 설치가 안 됐으면 [SETUP.md](SETUP.md)를 먼저 따를 것.

빌드가 안 돼 있으면:

```bash
cd ~/ugv_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-skip micro_ros_setup micro_ros_agent uros sllidar_ros2
source install/setup.bash
```

`.bashrc`에 소싱을 넣어두지 않았다면 새 터미널마다 다음을 실행한다:

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=~/ugv_ws/install/ugv_description/share:$GZ_SIM_RESOURCE_PATH
```

---

## 2. 기본 사용 — 터미널 2개

### [터미널 1] 구조수색 시뮬 켜기 (켜두고 그대로 둔다)

```bash
cd ~/ugv_ws
source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=~/ugv_ws/install/ugv_description/share:$GZ_SIM_RESOURCE_PATH
ros2 launch ugv_bringup patrol_sim.launch.py
```

- Gazebo + 로봇 + RViz + SLAM + Nav2 + YOLO + 순찰 + 화재 감지까지 이 한 창에서 전부 실행된다.
- 기동 순서(순차 지연): Gazebo·RViz(0s) → SLAM(14s) → Nav2(22s) → 비전(48s) → 순찰·화재(58s).
- 로그가 계속 흐르는 게 정상. 이 창은 그대로 두고, 종료할 땐 `Ctrl+C`.

> **기동 간격이 넓은 이유** — 비전 노드가 torch/CUDA를 로딩할 때 CPU를 크게 먹는다.
> 이게 Nav2 lifecycle 전환과 겹치면 `controller_server change_state` 가 타임아웃나고
> `planner_server`·`bt_navigator` 가 `unconfigured` 로 남는다. 그러면 목표를 보내도
> 로봇이 못 움직이는데, 로그에는 "도달 실패"로만 보여 원인을 놓치기 쉽다.
> 확인: `ros2 lifecycle get /planner_server` → `active` 여야 한다.

`patrol_navigator`는 SLAM 맵을 받는 즉시 자동으로 탐사를 시작한다(별도 on/off 토글 없음).

### 큰 월드 + 실종자 수 지정

```bash
ros2 launch ugv_bringup patrol_sim.launch.py \
    world:=rescue_building_large expected_victims:=7
```

- `world` — `rescue_building`(기본, 20×16 m) 또는 `rescue_building_large`(56×40 m, 조난자 7명·화재 4건)
- `expected_victims` — 구조본부가 아는 실종자 수. **다 찾기 전에는 수색을 끝내지 않는다.**
  전 구역을 훑었는데 인원이 모자라면 시야 기록을 지우고 재수색한다. `0`이면 면적 기준만 사용.

### [터미널 2] 명령 보내기 (새 창을 하나 더 연다)

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/setup.bash
```

탐사는 자동으로 시작된다. 진행 상태는 [터미널 1] 로그에 60초마다 찍힌다:

```
수색 진행 — 미탐사 경계 545셀(완료 기준 40), 미관측 286.1m²(기준 42.1), 조난자 5/7명
```

조난자를 다 찾으면 즉시 보고가 뜨고, 이후는 `보충 수색`으로 표시된다:

```
🏁 전원 발견! 조난자 7/7명 확인 — 구조 대기
  L1:Critical 2명 / L2:NeedHelp 1명 / L3:Normal 4명
  화재 4건
   · #0 L3:Normal (-0.9, 15.5)
   ...
```

**수동으로 몰고 싶으면 키보드 텔레op을 쓴다(아래 3절).**
RViz `2D Goal Pose`로 목표를 보내면 순찰이 잠시 멈추고 그쪽으로 간다(도착하거나 90초가 지나면 순찰 복귀).
`2D Pose Estimate`는 보통 불필요하다(slam_toolbox가 위치추정을 자동 처리).

---

## 3. RViz에서 볼 것

| 레이어 | 토픽 | 의미 |
|---|---|---|
| **Fog_of_War** | `/coverage_map` | **검정** = 가본 적 없음 · **회색** = 지나갔지만 눈으로 미확인 · **투명** = 확인 완료 |
| Patient_Markers | `/patient_markers` | 조난자 위치·등급(빨강 L1 / 주황 L2 / 초록 L3) |
| Fire_Markers · Fire_Heatmap | `/fire_heatmap` | 화재 위치 |
| SLAM_Map | `/map` | LiDAR 점유격자 |
| Nav2_Plan | `/plan` | Nav2 계획 경로 |
| Detection_Image · Fire_Image | `/detection/image_annotated` · `/fire/image_annotated` | 사람 / 화재 오버레이 |

> **안개 레이어가 핵심이다.** SLAM 맵은 "라이다가 지나갔나"만 보여준다.
> 방을 통과만 해도 지도상으로는 다 아는 것처럼 보이지만, 구석을 눈으로 안 봤으면 조난자를 놓친 것이다.
> 회색으로 남은 구역이 아직 할 일이다.

---

## 4. 그 외 (필요할 때만)

### Gazebo + RViz만 (SLAM/Nav 없이 순수 시뮬 확인)

```bash
cd ~/ugv_ws
source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=~/ugv_ws/install/ugv_description/share:$GZ_SIM_RESOURCE_PATH
ros2 launch ugv_bringup gazebo.launch.py
ros2 launch ugv_bringup gazebo.launch.py world:=rescue_building_large   # 큰 월드
```

### SLAM + Nav2 + 비전만 (순찰·화재 없이)

```bash
ros2 launch ugv_bringup slam_nav_sim.launch.py
```

### 키보드로 직접 조종 (또 다른 새 터미널)

키를 받으려면 이 창이 활성 상태(포커스)여야 한다.

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/setup.bash
ros2 run ugv_teleop teleop_keyboard_node
```

### 비전 노드만 따로 실행 (디버깅)

```bash
ros2 run ugv_vision yolo_pose_node               # YOLO 조난자 탐지 + 자세 판정
ros2 run ugv_vision target_manager_node          # 포탑 조준 + 정지·조준 확인 등록
ros2 run ugv_vision patrol_navigator             # 프론티어+커버리지 탐사
ros2 run ugv_vision fire_detection_node          # 열화상 화재 감지
ros2 run ugv_vision visibility_overlay_node      # RViz 커버리지 오버레이
```

### 조난자 탐지 결과 보기

```bash
cat ~/ugv_ws/patient_locations.txt
```

### 큰 월드 다시 만들기

월드 생성기는 **표준출력으로만 뱉는다**. 고쳤으면 반드시 파일로 저장해야 반영된다.

```bash
cd ~/ugv_ws/src/ugv_bringup/worlds
python3 gen_rescue_large.py > rescue_building_large.sdf
```

---

## 5. 상태 점검 / 진단

```bash
ros2 topic list                                  # 토픽 확인
ros2 topic echo /scan --once                     # LiDAR 수신 확인
ros2 topic echo /odom --once                     # 오도메트리
ros2 topic hz /camera/camera/color/image_raw     # 카메라 프레임레이트
ros2 topic hz /coverage_map                      # 안개(커버리지) 발행 주기 (0.5 Hz)
ros2 lifecycle get /planner_server               # Nav2 살아있는지 — active 여야 함
ros2 node list                                   # 노드 기동 확인
ros2 topic echo /clock --once                    # sim time 흐르는지
ros2 run tf2_tools view_frames                   # TF 체인 확인 → frames.pdf 생성
```

주요 브리지 토픽:

| ROS 토픽 | 방향 | 내용 |
|---|---|---|
| `/cmd_vel` | ROS→Gz | 차체 속도 명령 |
| `/odom` | Gz→ROS | 오도메트리 |
| `/scan` | Gz→ROS | 2D LiDAR |
| `/camera/camera/color/image_raw` | Gz→ROS | RGB 영상 |
| `/camera/camera/aligned_depth_to_color/image_raw` | Gz→ROS | 깊이 영상 |
| `/thermal/image_raw` | Gz→ROS | 열화상(mono16, 픽셀값 = 온도[K] / 0.01) |
| `/turret_yaw_cmd`, `/turret_pitch_cmd` | ROS→Gz | 포탑(카메라) 각도 |
| `/goal_pose` | patrol_navigator·RViz→Nav2 | 자율주행 목표 |
| `/coverage_map` | patrol_navigator→RViz | 전장의 안개(미관측 구역) |
| `/fire_cloud` | fire_detection→Nav2 | 화재 마킹(코스트맵 장애물) |
| `/sweep_complete` | patrol_navigator→ | 전원 발견 / 수색 완료 신호 |

---

## 6. 코드 수정 후 재빌드

비전 노드를 수정했으면 [터미널 1]을 `Ctrl+C`로 끄고:

```bash
cd ~/ugv_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ugv_vision
source install/setup.bash
```

그다음 [터미널 1] 블록으로 시뮬을 다시 켠다.
`ugv_msgs`를 고쳤다면 먼저 `colcon build --packages-select ugv_msgs` 후 나머지를 빌드한다.

---

## 7. 종료

각 터미널에서 `Ctrl+C`. Gazebo가 남으면 **PID로** 정리한다.

```bash
ps -ef | grep -E 'gz sim|ros_gz'      # PID 확인 후
kill -9 <PID>
```

> `pkill -f 'gz sim'` 같은 패턴 kill은 **자기 셸의 cmdline까지 매칭해 스스로 죽는다**.
> 명령 안에 그 문자열이 들어 있기 때문이다. PID로 지울 것.

---

## 8. 트러블슈팅

- **Gazebo 창이 안 뜸 / 모델 안 보임:** `echo $GZ_SIM_RESOURCE_PATH` 확인(1번 export), `gz sim --version`으로 설치 확인.
- **로봇이 목표를 못 감 / "도달 실패"만 반복:** Nav2가 안 떴을 수 있다. `ros2 lifecycle get /planner_server`가 `active`가 아니면 CPU 과부하로 lifecycle 전환이 실패한 것이다. 무거운 프로세스(안드로이드 에뮬레이터·화면보호기 등)를 끄고 다시 띄울 것.
- **스폰 직후 RTF가 0.001까지 떨어짐:** ogre2 셰이더 최초 컴파일 구간이다. 잠시 후 1.0으로 회복된다. 데드락으로 오진하기 쉽다.
- **수정이 반영 안 된 것 같음:** `gz sim` 서버가 두 개 떠 있으면 브리지가 옛 서버를 문다. 검증 전 단일 인스턴스인지 확인할 것.
- **TF `use_sim_time` 경고:** launch에 `use_sim_time:=true`가 고정돼 있다. 노드를 수동 실행할 땐 파라미터를 직접 붙일 것.
- **YOLO 모델 없음:** `~/ugv_ws/yolov8n-pose.pt` 존재 확인. 없으면 `python3 -c "from ultralytics import YOLO; YOLO('yolov8n-pose.pt')"`.
- **`_ARRAY_API not found` / 비전 노드 세그폴트:** numpy 2.x가 설치돼 cv_bridge와 ABI가 어긋난 것이다. `pip install 'numpy==1.26.4'`.

---

## 요약

```bash
source ~/ugv_ws/install/setup.bash                        # 1) 환경 소싱
ros2 launch ugv_bringup patrol_sim.launch.py \
    world:=rescue_building_large expected_victims:=7      # 2) 구조수색 시뮬
# 3) ~58초 뒤 자동 탐사 시작 → 로그의 '수색 진행' 과 RViz 안개로 확인
```

| 터미널 | 역할 | 명령 |
|---|---|---|
| **1** | 시뮬 전체 켜기 | `ros2 launch ugv_bringup patrol_sim.launch.py` |
| **2** | Nav2 생존 확인 | `ros2 lifecycle get /planner_server` |
| **2** | 수동 조종 | `ros2 run ugv_teleop teleop_keyboard_node` |
| RViz | 남은 수색 구역 확인 | `Fog_of_War` 레이어 (`/coverage_map`) |
