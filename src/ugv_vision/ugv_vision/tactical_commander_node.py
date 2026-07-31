"""
tactical_commander_node.py
==========================
CQB '파이 쪼개기(Slicing the Pie)' 전술 자율주행 상위 노드.

────────────────────────────────────────────────────────────
아키텍처 개요
────────────────────────────────────────────────────────────
                  ┌─────────────────────────────────┐
                  │      TacticalCommanderNode       │
                  │                                  │
  /map ──────────►│  Phase 1: Apex 식별              │
  /visual_coverage►│  Phase 2: Arc 웨이포인트 생성   │──► /goal_pose (Nav2)
  /odom ─────────►│  Phase 3: 포탑 시선 동기화      │──► /turret_yaw_cmd
  /target_detect ─►│  Phase 4: 환자 발견 인터럽트   │──► /turret_pitch_cmd
                  └─────────────────────────────────┘

상태 머신:
  IDLE ──► SECTOR_EXPLORE ──► SLICE_PIE ──► CLEAR
                  ▲                │
                  │    PATIENT_INTERRUPT (환자 발견)
                  └────────────────┘

────────────────────────────────────────────────────────────
Phase 1: Apex 식별 (문틀 코너 탐지)
────────────────────────────────────────────────────────────
SLAM OccupancyGrid에서 '벽이 끝나는 지점'을 Apex로 정의한다.
  - 현재 로봇 위치에서 8방향(45° 간격)으로 레이캐스팅
  - 각 레이가 벽(>50)에 닿기 직전 자유공간(0)의 마지막 셀 = Apex 후보
  - 후보 중 미수색 VisualCoverageMap 셀이 가장 많은 방향 선택

────────────────────────────────────────────────────────────
Phase 2: Arc 웨이포인트 생성 (파이 쪼개기 경로)
────────────────────────────────────────────────────────────
직진 대신 Apex를 중심으로 반경 R=1.5m 원호를 따라 이동한다.
  - 웨이포인트 = [Apex + R*cos(θ), Apex + R*sin(θ)] (θ를 점진적으로 증가)
  - Nav2 ActionClient로 순차 전송 (WaypointFollower 활용 가능)

────────────────────────────────────────────────────────────
Phase 3: 포탑 시선 동기화
────────────────────────────────────────────────────────────
차체가 원호를 따라 이동하는 동안 카메라는 새로 열리는 사각지대를 추적:
  θ_turret_cmd = atan2(Y_target - Y_robot, X_target - X_robot) - θ_robot

────────────────────────────────────────────────────────────
Phase 4: 환자 인터럽트
────────────────────────────────────────────────────────────
  - YOLO 감지 → Nav2 Goal 취소 → TargetManager Lock-on 위임
  - 분류 완료(CONFIRMED) → 파이 쪼개기 재개
"""
import math
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray

# Nav2 action (설치된 환경에 따라 다를 수 있음)
try:
    from nav2_msgs.action import NavigateToPose
    _NAV2_AVAILABLE = True
except ImportError:
    _NAV2_AVAILABLE = False

from ugv_vision.visual_coverage_map import VisualCoverageGrid
from ugv_msgs.msg import TargetDetection


class TacState(Enum):
    IDLE             = auto()
    SECTOR_EXPLORE   = auto()   # 섹터별 미탐색 이동
    SLICE_PIE        = auto()   # CQB 원호 기동
    PATIENT_INTERRUPT = auto()  # 환자 발견, Nav2 일시정지
    CLEAR            = auto()   # 구역 클리어 완료


# ── 튜닝 파라미터 ──────────────────────────────────────────────────────
ARC_RADIUS_M   = 1.5    # 파이 쪼개기 원호 반경 (m)
ARC_STEP_DEG   = 10.0   # 원호 웨이포인트 간격 (°)
NUM_SECTORS    = 8      # 파이 섹터 수 (360/8=45°)
SECTOR_RANGE_M = 6.0    # 섹터 레이캐스팅 최대 거리 (m)
FOV_RAD        = 1.089  # 카메라 수평 FOV (rad ≈ 62°)
CAM_RANGE_M    = 4.0    # 카메라 가시거리 (m)


def euler_from_quaternion(q):
    t3 = 2.0 * (q.w * q.z + q.x * q.y)
    t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(t3, t4)


class TacticalCommanderNode(Node):

    def __init__(self):
        super().__init__('tactical_commander_node')

        # ── 상태 ─────────────────────────────────────────────────────
        self.state = TacState.IDLE

        # ── 로봇 자세 ─────────────────────────────────────────────────
        self.robot_x      = 0.0
        self.robot_y      = 0.0
        self.robot_theta  = 0.0
        self.turret_yaw   = 0.0

        # ── 맵 데이터 ─────────────────────────────────────────────────
        self.slam_map: OccupancyGrid | None = None
        self.cov_map  = VisualCoverageGrid(resolution=0.2, size_m=40.0)

        # ── 파이 쪼개기 상태 ──────────────────────────────────────────
        self.arc_waypoints: list[tuple[float, float]] = []
        self.arc_idx = 0
        self.apex_x  = 0.0
        self.apex_y  = 0.0

        # ── 환자 추적 ─────────────────────────────────────────────────
        self.patient_detected   = False
        self.patient_confirmed  = False   # TargetManager가 등록 완료 신호 후 True

        # ── 구독 ─────────────────────────────────────────────────────
        self.create_subscription(Odometry,     '/odom',              self._odom_cb,    10)
        self.create_subscription(JointState,   '/joint_states',      self._joint_cb,   10)
        self.create_subscription(OccupancyGrid,'/map',               self._map_cb,     10)
        self.create_subscription(OccupancyGrid,'/visual_coverage',   self._vcov_cb,    10)
        self.create_subscription(TargetDetection, '/target_detection', self._target_cb, 10)

        # ── 퍼블리시 ──────────────────────────────────────────────────
        self.goal_pub   = self.create_publisher(PoseStamped, '/goal_pose',       10)
        self.yaw_pub    = self.create_publisher(Float64,     '/turret_yaw_cmd',  10)
        self.pitch_pub  = self.create_publisher(Float64,     '/turret_pitch_cmd',10)
        self.debug_pub  = self.create_publisher(MarkerArray, '/tac_debug',       10)

        # ── Nav2 ActionClient (optional) ──────────────────────────────
        if _NAV2_AVAILABLE:
            self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        else:
            self._nav_client = None
            self.get_logger().warn('nav2_msgs 없음 — /goal_pose 직접 퍼블리시 모드')

        # ── 타이머 ────────────────────────────────────────────────────
        self.create_timer(0.5,  self._state_machine)   # 상태 머신 (2Hz)
        self.create_timer(0.1,  self._turret_sync)     # 포탑 동기화 (10Hz)
        self.create_timer(0.5,  self._cov_update)      # FOV 커버리지 갱신

        self.get_logger().info('TacticalCommanderNode 시작 — CQB 파이 쪼개기 모드')

    # ── 구독 콜백 ──────────────────────────────────────────────────────
    def _odom_cb(self, msg):
        self.robot_x     = msg.pose.pose.position.x
        self.robot_y     = msg.pose.pose.position.y
        self.robot_theta = euler_from_quaternion(msg.pose.pose.orientation)

    def _joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name == 'turret_yaw_joint':
                self.turret_yaw = pos

    def _map_cb(self, msg):
        self.slam_map = msg

    def _vcov_cb(self, _msg):
        pass   # 현재 VisualCoverageGrid는 내부에서 직접 관리

    def _target_cb(self, msg):
        if msg.status == 'TRACKING':
            self.patient_detected = True

    # ── FOV 커버리지 업데이트 ────────────────────────────────────────
    def _cov_update(self):
        self.cov_map.update_fov(
            self.robot_x, self.robot_y,
            self.robot_theta, self.turret_yaw,
        )

    # ──────────────────────────────────────────────────────────────────
    # Phase 1: 섹터별 정보 획득량(Information Gain) 계산
    # ──────────────────────────────────────────────────────────────────
    def _score_sectors(self) -> int:
        """
        8개 섹터(45° 간격)에 레이캐스팅 → 미탐색 셀 수를 점수로 환산.
        가장 높은 섹터 인덱스를 반환.
        """
        if self.slam_map is None:
            return 0

        best_sector = 0
        best_score  = -1

        for sector in range(NUM_SECTORS):
            sector_angle = self.robot_theta + sector * (2 * math.pi / NUM_SECTORS)
            score = self._raycast_score(sector_angle)
            if score > best_score:
                best_score  = score
                best_sector = sector

        return best_sector

    def _raycast_score(self, angle: float) -> int:
        """
        주어진 방향으로 레이를 쏘아 미탐색(Unseen) 셀 수를 반환.
        """
        if self.slam_map is None:
            return 0

        res = self.slam_map.info.resolution
        ox  = self.slam_map.info.origin.position.x
        oy  = self.slam_map.info.origin.position.y
        w   = self.slam_map.info.width

        unseen_count = 0
        step = res
        d    = 0.5

        while d <= SECTOR_RANGE_M:
            wx = self.robot_x + d * math.cos(angle)
            wy = self.robot_y + d * math.sin(angle)
            gx = int((wx - ox) / res)
            gy = int((wy - oy) / res)

            if not (0 <= gx < self.slam_map.info.width and
                    0 <= gy < self.slam_map.info.height):
                break

            cell = self.slam_map.data[gy * w + gx]
            if cell > 50:   # 벽
                break
            if cell == -1:  # 미지 구역
                unseen_count += 2   # 미지 구역은 가중치 2배

            # VisualCoverageMap에서 아직 못 본 자유공간이면 추가 점수
            vcg_x, vcg_y = self.cov_map._w2g(wx, wy)
            if self.cov_map._in_bounds(vcg_x, vcg_y):
                if self.cov_map.grid[vcg_y, vcg_x] == 0:
                    unseen_count += 1

            d += step

        return unseen_count

    # ──────────────────────────────────────────────────────────────────
    # Phase 1-B: Apex 식별 (문틀 코너)
    # ──────────────────────────────────────────────────────────────────
    def _find_apex(self, direction_angle: float) -> tuple[float, float] | None:
        """
        주어진 방향으로 레이를 쏘다가 벽이 끊기는 직전 자유공간 좌표 반환.
        문틀/코너의 Apex를 찾는 용도.
        """
        if self.slam_map is None:
            return None

        res = self.slam_map.info.resolution
        ox  = self.slam_map.info.origin.position.x
        oy  = self.slam_map.info.origin.position.y
        w   = self.slam_map.info.width

        prev_wx, prev_wy = self.robot_x, self.robot_y
        d = 0.5
        while d <= SECTOR_RANGE_M:
            wx = self.robot_x + d * math.cos(direction_angle)
            wy = self.robot_y + d * math.sin(direction_angle)
            gx = int((wx - ox) / res)
            gy = int((wy - oy) / res)

            if not (0 <= gx < self.slam_map.info.width and
                    0 <= gy < self.slam_map.info.height):
                break
            cell = self.slam_map.data[gy * w + gx]
            if cell > 50:               # 벽에 닿음 → 직전 위치가 Apex
                return prev_wx, prev_wy
            prev_wx, prev_wy = wx, wy
            d += res

        return None   # 벽 없음 (열린 공간)

    # ──────────────────────────────────────────────────────────────────
    # Phase 2: Apex 중심 원호 웨이포인트 생성
    # ──────────────────────────────────────────────────────────────────
    def _build_arc_waypoints(
        self,
        apex_x: float, apex_y: float,
        start_angle: float, end_angle: float,
    ) -> list[tuple[float, float]]:
        """
        Apex를 중심으로 반경 ARC_RADIUS_M인 원호 위의 웨이포인트 반환.
        start_angle → end_angle 방향으로 ARC_STEP_DEG 간격.
        """
        waypoints = []
        step = math.radians(ARC_STEP_DEG)
        if end_angle < start_angle:
            step = -step

        angle = start_angle
        while abs(angle - end_angle) > abs(step) / 2:
            wx = apex_x + ARC_RADIUS_M * math.cos(angle)
            wy = apex_y + ARC_RADIUS_M * math.sin(angle)
            waypoints.append((wx, wy))
            angle += step

        return waypoints

    # ──────────────────────────────────────────────────────────────────
    # Phase 3: 포탑 시선 → 파이 쪼개기 방향으로 동기화
    # ──────────────────────────────────────────────────────────────────
    def _turret_sync(self):
        """
        SLICE_PIE 상태일 때, 카메라가 항상 Apex 방향의 사각지대를 향하도록
        포탑 요 각도를 계산해서 명령한다.

          θ_turret_cmd = atan2(apex_y - robot_y, apex_x - robot_x) - robot_theta
        """
        if self.state != TacState.SLICE_PIE:
            return

        target_world_angle = math.atan2(
            self.apex_y - self.robot_y,
            self.apex_x - self.robot_x,
        )
        turret_cmd = target_world_angle - self.robot_theta

        # -π ~ +π 범위로 정규화
        while turret_cmd >  math.pi: turret_cmd -= 2 * math.pi
        while turret_cmd < -math.pi: turret_cmd += 2 * math.pi

        # 포지션 P-제어 → 속도 명령
        KP = 2.0
        yaw_vel = KP * (turret_cmd - self.turret_yaw)
        self.yaw_pub.publish(Float64(data=float(yaw_vel)))
        self.pitch_pub.publish(Float64(data=0.0))

    # ──────────────────────────────────────────────────────────────────
    # 메인 상태 머신
    # ──────────────────────────────────────────────────────────────────
    def _state_machine(self):

        # ── IDLE: 시작 대기 ────────────────────────────────────────────
        if self.state == TacState.IDLE:
            if self.slam_map is not None:
                self.get_logger().info('맵 수신 완료 → SECTOR_EXPLORE 시작')
                self.state = TacState.SECTOR_EXPLORE

        # ── SECTOR_EXPLORE: 섹터별 미탐색 이동 ────────────────────────
        elif self.state == TacState.SECTOR_EXPLORE:
            if self.patient_detected:
                self.get_logger().info('[인터럽트] 환자 감지 → PATIENT_INTERRUPT')
                self.state            = TacState.PATIENT_INTERRUPT
                self.patient_detected = False
                return

            # 가장 정보 획득량 높은 섹터 선택
            best = self._score_sectors()
            angle = self.robot_theta + best * (2 * math.pi / NUM_SECTORS)

            # 해당 방향의 Apex(문틀) 확인 → 있으면 SLICE_PIE로 전환
            apex = self._find_apex(angle)
            if apex is not None:
                self.apex_x, self.apex_y = apex
                robot_to_apex = math.atan2(
                    self.apex_y - self.robot_y,
                    self.apex_x - self.robot_x,
                )
                # 현재 위치 → Apex 반대편까지 원호 생성
                self.arc_waypoints = self._build_arc_waypoints(
                    self.apex_x, self.apex_y,
                    start_angle = robot_to_apex + math.pi * 0.6,
                    end_angle   = robot_to_apex - math.pi * 0.6,
                )
                self.arc_idx = 0
                self.get_logger().info(
                    f'Apex=({self.apex_x:.1f},{self.apex_y:.1f}), '
                    f'웨이포인트 {len(self.arc_waypoints)}개 → SLICE_PIE')
                self.state = TacState.SLICE_PIE
            else:
                # Apex 없음 → 해당 방향으로 직접 이동
                goal_dist = min(SECTOR_RANGE_M * 0.6, 3.0)
                gx = self.robot_x + goal_dist * math.cos(angle)
                gy = self.robot_y + goal_dist * math.sin(angle)
                self._send_nav_goal(gx, gy, angle)

        # ── SLICE_PIE: 원호 기동 ──────────────────────────────────────
        elif self.state == TacState.SLICE_PIE:
            if self.patient_detected:
                self.get_logger().info('[인터럽트] 환자 감지 → PATIENT_INTERRUPT')
                self.state            = TacState.PATIENT_INTERRUPT
                self.patient_detected = False
                return

            if self.arc_idx >= len(self.arc_waypoints):
                self.get_logger().info('원호 완료 → CLEAR')
                self.state = TacState.CLEAR
                return

            wx, wy = self.arc_waypoints[self.arc_idx]
            heading = math.atan2(wy - self.robot_y, wx - self.robot_x)

            # 현재 웨이포인트 도달 확인 (0.5m 이내)
            dist_to_wp = math.hypot(wx - self.robot_x, wy - self.robot_y)
            if dist_to_wp < 0.5:
                self.arc_idx += 1
            else:
                self._send_nav_goal(wx, wy, heading)

        # ── PATIENT_INTERRUPT: 환자 처리 완료 대기 ────────────────────
        elif self.state == TacState.PATIENT_INTERRUPT:
            # TargetManager가 patient_locations.txt에 기록 완료하면 복귀
            # TODO: MarkerArray 구독으로 confirmed 수 변화 감지
            # (현재는 3초 후 EXPLORE 자동 복귀 — 추후 정밀화)
            self.get_logger().info('환자 처리 중... 3초 후 탐색 재개 예정')
            # 실제 구현에서는 /patient_markers 토픽에서 신규 마커 감지
            self.create_timer(3.0, self._resume_from_interrupt)

        # ── CLEAR: 다음 섹터로 ────────────────────────────────────────
        elif self.state == TacState.CLEAR:
            self.get_logger().info('구역 클리어 → SECTOR_EXPLORE')
            self.state = TacState.SECTOR_EXPLORE

    def _resume_from_interrupt(self):
        if self.state == TacState.PATIENT_INTERRUPT:
            self.get_logger().info('탐색 재개 → SLICE_PIE 계속')
            self.state = TacState.SLICE_PIE

    # ── Nav2 Goal 전송 ────────────────────────────────────────────────
    def _send_nav_goal(self, x: float, y: float, heading: float):
        """
        PoseStamped를 /goal_pose로 퍼블리시 (Nav2 Simple Commander 인터페이스).
        ActionClient 사용 시 NavigateToPose로 교체 가능.
        """
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        # heading → quaternion (Z축 회전만)
        goal.pose.orientation.z = math.sin(heading / 2.0)
        goal.pose.orientation.w = math.cos(heading / 2.0)
        self.goal_pub.publish(goal)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TacticalCommanderNode())
    rclpy.shutdown()
