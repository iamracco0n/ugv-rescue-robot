"""
vision_coverage_navigator.py  —  Visibility-Driven Exploration System v2
=========================================================================
목표: "카메라로 실제로 관측했는가(visibility completion)"를
      탐색 완료 기준으로 삼는 자율 탐색 시스템.

v2 개선사항:
    1. costmap-aware goal validation: inflation/footprint 고려한 안전 viewpoint
    2. room-hierarchical exploration: 현재 방 우선 탐색 → 완료 후 다음 방
    3. frontier commitment: progress 기반 stuck 감지, 잦은 goal switching 방지
    4. contextual gaze state machine: SIDE_CLEAR/CORNER_PEEK/DOORWAY_GLANCE/RECHECK/FORWARD_TRAVEL
    5. turret joint-limit 보호 + stagnation 감지 → 포탑 freeze 방지
    6. visibility gain 기반 viewpoint 선택 (ray 기반 가시성 평가)

설계 원칙:
    - 차체는 viewpoint를 바꾸고, 포탑은 지역 안개를 제거 (협조 제어)
    - 모든 동작은 geometry/raycast 기반 — 결정론적, 설명 가능
    - SLAM 벽 occlusion — X-ray vision 방지
    - room-by-room completion 우선 (wandering 방지)
"""
import math
from collections import deque
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from ugv_msgs.msg import TargetDetection
from ugv_vision.visual_coverage_map import VisualCoverageGrid

try:
    from scipy.ndimage import label as _ndlabel, binary_erosion
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False

# ── 카메라 파라미터 ──────────────────────────────────────────────────────
# 시뮬 카메라 수평 FOV (ugv.urdf.xacro: 1.089rad ≈ 62.4°).
# 실기 D435i 는 87° 이지만, 시뮬에서 87° 를 쓰면 커버리지 부채꼴이 실제보다
# 40% 넓게 그려져 못 본 구역을 봤다고 표시한다.
FOV_RAD     = 1.089
CAM_RANGE_M = 4.5               # D435i 최대 유효 거리 (m)

# ── Contextual gaze 임계값 ───────────────────────────────────────────────
GAZE_RESCAN_S       = 1.5
MIN_AIM_CHANGE_DEG  = 7.0
SIDE_MIN_CELLS      = 4
SIDE_RANGE_M        = CAM_RANGE_M
CORNER_DETECT_M     = 2.5     # 전방 ±35~55° 코너 탐지 거리
DOORWAY_DETECT_M    = 3.5     # 전방 문 탐지 거리
DOORWAY_GLANCE_S    = 2.5     # DOORWAY_GLANCE 상태 유지 시간 (s)
RECHECK_CONF        = 0.35    # 이 confidence 이하 셀 → RECHECK
RECHECK_RANGE_M     = 5.0

# ── Turret 안전 ──────────────────────────────────────────────────────────
TURRET_YAW_LIMIT    = math.radians(75)   # 차체 기준 최대 turret 각도 (URDF ±90° 대비 15° 마진)
TURRET_STAGNATION_S = 6.0               # 이 시간 이상 aim 변화 없으면 강제 갱신

# ── Nav goal 파라미터 ────────────────────────────────────────────────────
NAV_GOAL_INTERVAL   = 5.0    # 동일 goal 재발행 간격 (s)
FRONTIER_SEARCH_R   = 18.0
MIN_FRONTIER_CELLS  = 8
GOAL_REACHED_M      = 1.2
APPROACH_STOP_M     = 1.5

# ── Costmap safety ───────────────────────────────────────────────────────
COSTMAP_SAFE_THRESH = 50     # global costmap value < this → navigable
NAV_SAFE_RING_MAX   = 3.0    # 안전 지점 탐색 최대 반경 (m)

# ── Viewpoint planning ───────────────────────────────────────────────────
VG_CANDIDATES  = 14          # viewpoint 후보 최대 수
VG_SAMPLE_RAYS = 10          # 가시성 추정 ray 수 per candidate

# ── Frontier commitment ──────────────────────────────────────────────────
COMMIT_STUCK_MAX  = 5        # 연속 no-progress 허용 횟수 (2Hz 기준 ~2.5s마다)
COMMIT_TIMEOUT    = 45.0     # 강제 재계획 timeout (s)
COMMIT_PROGRESS_M = 0.4      # progress로 인정하는 최소 이동 거리 (m)

# ── Visibility confidence decay ──────────────────────────────────────────
DECAY_LAMBDA = 0.025

# ── Cluster 점수 가중치 ──────────────────────────────────────────────────
W_AREA      =  1.0
W_DIST      = -0.10
W_RECENCY   =  2.0
W_ROOM_PRIO =  6.0   # 현재 방 강력 우선 (wandering 방지)

# ── Room completion ──────────────────────────────────────────────────────
ROOM_COMPLETE_RATIO = 0.15
MIN_ROOM_CELLS      = 30
DOORWAY_EROSION     = 2

# ── 장애물 회피 ──────────────────────────────────────────────────────────
PROX_STOP_M       = 0.35
PROX_CHECK_ARC    = math.radians(40)
RECOVERY_BACK_VEL = -0.25
RECOVERY_TURN_VEL = 0.65
RECOVERY_DURATION = 2.8
RECOVERY_BACKUP_T = 0.9


class ExploreState(Enum):
    IDLE              = auto()
    EXPLORE           = auto()
    PATIENT_INTERRUPT = auto()
    RESUME            = auto()
    RECOVERY          = auto()


class GazeState(Enum):
    FORWARD_TRAVEL  = auto()   # nav goal 방향 주시 (이동 중 기본)
    CORNER_PEEK     = auto()   # 전방 대각선 코너 안쪽 peek
    DOORWAY_GLANCE  = auto()   # 문 너머 공간 glance (타이머 기반)
    SIDE_CLEAR      = auto()   # 측면 근거리 안개 제거
    RECHECK         = auto()   # low-confidence 이전 관측 영역 재확인


def _yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _normalize_angle(a: float) -> float:
    """각도를 [-π, π]로 정규화."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _bfs_label(mask: np.ndarray) -> tuple[np.ndarray, int]:
    """scipy 없을 때 4-연결 BFS 레이블링."""
    labeled = np.zeros(mask.shape, dtype=np.int32)
    lbl = 0
    free_set = set(zip(*np.where(mask)))
    visited: set[tuple[int, int]] = set()
    for start in free_set:
        if start in visited:
            continue
        lbl += 1
        stack = [start]
        visited.add(start)
        labeled[start] = lbl
        while stack:
            r, c = stack.pop()
            for dr, dc in ((1,0),(-1,0),(0,1),(0,-1)):
                nb = (r+dr, c+dc)
                if nb in free_set and nb not in visited:
                    visited.add(nb)
                    labeled[nb] = lbl
                    stack.append(nb)
    return labeled, lbl


class VisibilityExplorationNavigator(Node):
    """
    Visibility-Driven Exploration System v2.

    출력:
        /goal_pose       → Nav2 (차체: costmap-safe cluster viewpoint)
        /apex_aim_point  → target_manager (포탑: context-aware gaze)
    """

    def __init__(self):
        super().__init__('vision_coverage_navigator')

        self.state   = ExploreState.IDLE
        self.slam_map: OccupancyGrid | None = None
        self.cov_map = VisualCoverageGrid(resolution=0.2, size_m=40.0)
        self._slam_dirty_cov = True
        self._rooms_dirty    = True

        # ── 로봇 상태 ─────────────────────────────────────────────────
        self.robot_x     = 0.0
        self.robot_y     = 0.0
        self.robot_theta = 0.0
        self.turret_yaw  = 0.0

        # ── Gaze state machine ────────────────────────────────────────
        self._look_tx: float | None = None
        self._look_ty: float | None = None
        self._last_gaze_t         = 0.0
        self._gaze_state          = GazeState.FORWARD_TRAVEL
        self._doorway_glance_t    = 0.0
        self._doorway_glance_tgt: tuple[float, float] | None = None
        self._gaze_stagnation_t   = 0.0      # 마지막 aim 실제 변경 시각
        self._last_look_tx        = 0.0      # stagnation 감지용 (x)
        self._last_look_ty        = 0.0      # stagnation 감지용 (y)

        # ── Frontier commitment ────────────────────────────────────────
        self._committed_goal: tuple[float, float] | None = None
        self._commitment_start_t   = 0.0
        self._commitment_last_dist = float('inf')
        self._commitment_stuck     = 0
        self._last_nav_pub_t       = 0.0

        # ── Nav goal (gaze forward reference) ─────────────────────────
        self._nav_goal_sent_x: float | None = None
        self._nav_goal_sent_y: float | None = None

        # ── 사용자 Goal 우선순위 ──────────────────────────────────────
        self._user_goal_active    = False
        self._user_goal_x: float | None = None
        self._user_goal_y: float | None = None
        self._last_self_pub_x: float | None = None
        self._last_self_pub_y: float | None = None

        # ── Global costmap ─────────────────────────────────────────────
        self._global_costmap: OccupancyGrid | None = None
        self._costmap_arr: np.ndarray | None = None

        # ── Visibility confidence ──────────────────────────────────────
        _N = self.cov_map.n
        self._last_seen_t = np.zeros((_N, _N), dtype=np.float64)

        # ── Room 세그멘테이션 ─────────────────────────────────────────
        self._room_labels: np.ndarray | None = None
        self._room_cells: dict[int, tuple] = {}
        self._room_complete: set[int] = set()
        self._current_room_id = 0

        # ── 메트릭 ────────────────────────────────────────────────────
        self._t_start           = 0.0
        self._fog_cleared_cells = 0

        # ── 라이다 ────────────────────────────────────────────────────
        self._scan_min_front = float('inf')
        self._scan_left_avg  = 5.0
        self._scan_right_avg = 5.0

        # ── 기타 ─────────────────────────────────────────────────────
        self.patient_flag        = False
        self._resume_timer_set   = False
        self._recovery_start     = 0.0
        self._pre_recovery_state = ExploreState.IDLE

        self.declare_parameter('coverage_enabled_on_boot', True)
        self.coverage_enabled = self.get_parameter('coverage_enabled_on_boot').value

        # ── 구독 ──────────────────────────────────────────────────────
        self.create_subscription(Odometry,        '/odom',             self._odom_cb,     10)
        self.create_subscription(JointState,      '/joint_states',     self._joint_cb,    10)
        self.create_subscription(OccupancyGrid,   '/map',              self._map_cb,      10)
        self.create_subscription(TargetDetection, '/target_detection', self._target_cb,   10)
        self.create_subscription(Bool,            '/coverage_enable',  self._enable_cb,   10)
        self.create_subscription(LaserScan,       '/scan',             self._scan_cb,     10)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._costmap_cb, 1)
        self.create_subscription(
            PoseStamped, '/goal_pose', self._external_goal_cb, 10)

        # ── 퍼블리시 ─────────────────────────────────────────────────
        self.aim_pub     = self.create_publisher(Point,         '/apex_aim_point',    10)
        self.goal_pub    = self.create_publisher(PoseStamped,   '/goal_pose',         10)
        self.cov_pub     = self.create_publisher(OccupancyGrid, '/visual_coverage',   10)
        self.debug_pub   = self.create_publisher(MarkerArray,   '/slice_debug',       10)
        self.cmd_vel_pub = self.create_publisher(Twist,         '/cmd_vel',           10)
        self.status_pub  = self.create_publisher(String,        '/exploration_status', 10)

        self.create_timer(0.5,  self._state_machine)
        self.create_timer(0.5,  self._cov_update)
        self.create_timer(5.0,  self._segment_rooms)
        self.create_timer(1.0,  self._publish_coverage)
        self.create_timer(0.05, self._recovery_tick)
        self.create_timer(5.0,  self._publish_metrics)

        self.get_logger().info(
            f'VisibilityExplorationNavigator v2 시작 '
            f'[scipy={"on" if _HAS_SCIPY else "off"}]')

    # ════════════════════════════════════════════════════════════════════
    # 콜백
    # ════════════════════════════════════════════════════════════════════

    def _odom_cb(self, msg):
        self.robot_x     = msg.pose.pose.position.x
        self.robot_y     = msg.pose.pose.position.y
        self.robot_theta = _yaw_from_quat(msg.pose.pose.orientation)
        if self._room_labels is not None:
            rgx, rgy = self.cov_map._w2g(self.robot_x, self.robot_y)
            N = self.cov_map.n
            if 0 <= rgx < N and 0 <= rgy < N:
                self._current_room_id = int(self._room_labels[rgy, rgx])

    def _joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name == 'turret_yaw_joint':
                self.turret_yaw = pos

    def _map_cb(self, msg):
        self.slam_map        = msg
        self._slam_dirty_cov = True
        self._rooms_dirty    = True

    def _costmap_cb(self, msg: OccupancyGrid):
        self._global_costmap = msg
        self._costmap_arr = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)

    def _external_goal_cb(self, msg: PoseStamped):
        """사용자(RViz 등)가 보낸 /goal_pose 감지 — 자신의 echo는 무시."""
        rx = msg.pose.position.x
        ry = msg.pose.position.y
        # 자신이 방금 발행한 goal과 거의 같으면 무시 (echo 방지)
        if (self._last_self_pub_x is not None
                and math.hypot(rx - self._last_self_pub_x,
                               ry - self._last_self_pub_y) < 0.15):
            return
        # 외부 goal 감지 → 자율 탐색 일시 중단
        self._user_goal_active = True
        self._user_goal_x      = rx
        self._user_goal_y      = ry
        self._committed_goal   = None   # 현재 자율 commitment 취소
        self.get_logger().info(
            f'[USER GOAL] ({rx:.1f},{ry:.1f}) — 자율탐색 일시중지')

    def _scan_cb(self, msg: LaserScan):
        if not msg.ranges:
            return
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = (msg.angle_min
                  + np.arange(len(ranges), dtype=np.float32) * msg.angle_increment)
        valid  = np.isfinite(ranges) & (ranges > 0.01) & (ranges < msg.range_max)
        half   = PROX_CHECK_ARC / 2.0
        front  = valid & (np.abs(angles) <= half)
        left   = valid & (angles >  half) & (angles <= math.pi / 2)
        right  = valid & (angles < -half) & (angles >= -math.pi / 2)
        self._scan_min_front = float(np.min(ranges[front])) if np.any(front) else float('inf')
        self._scan_left_avg  = float(np.mean(ranges[left]))  if np.any(left)  else 5.0
        self._scan_right_avg = float(np.mean(ranges[right])) if np.any(right) else 5.0

    def _target_cb(self, msg):
        if msg.status == 'TRACKING':
            self.patient_flag = True

    def _enable_cb(self, msg: Bool):
        self.coverage_enabled = msg.data
        if msg.data and self.state == ExploreState.IDLE:
            self.state    = ExploreState.EXPLORE
            self._t_start = self.get_clock().now().nanoseconds * 1e-9
        elif not msg.data and self.state == ExploreState.EXPLORE:
            self.state = ExploreState.IDLE

    # ════════════════════════════════════════════════════════════════════
    # cov_map + confidence 갱신
    # ════════════════════════════════════════════════════════════════════

    def _cov_update(self):
        if self._slam_dirty_cov:
            self._project_slam_walls_to_cov()
            self._slam_dirty_cov = False

        prev_seen = self.cov_map.grid.copy()
        self.cov_map.update_fov(
            self.robot_x, self.robot_y,
            self.robot_theta, self.turret_yaw,
            fov_rad=FOV_RAD, max_range=CAM_RANGE_M,
        )

        now       = self.get_clock().now().nanoseconds * 1e-9
        in_fov    = (self.cov_map.grid == 1)
        newly_seen = in_fov & (prev_seen == 0)

        self._last_seen_t[in_fov] = now
        self._fog_cleared_cells  += int(np.sum(newly_seen))

    def _get_confidence(self, now: float) -> np.ndarray:
        """seen 셀: exp(-λ·Δt), 나머지: 0.0."""
        seen = (self.cov_map.grid == 1)
        conf = np.zeros(self.cov_map.grid.shape, dtype=np.float32)
        if not np.any(seen):
            return conf
        dt = (now - self._last_seen_t[seen]).astype(np.float32)
        conf[seen] = np.exp(-DECAY_LAMBDA * dt)
        return conf

    # ════════════════════════════════════════════════════════════════════
    # SLAM 벽 → cov_map 투영
    # ════════════════════════════════════════════════════════════════════

    def _project_slam_walls_to_cov(self):
        if self.slam_map is None:
            return
        res  = self.slam_map.info.resolution
        ox   = self.slam_map.info.origin.position.x
        oy   = self.slam_map.info.origin.position.y
        mw   = self.slam_map.info.width
        mh   = self.slam_map.info.height
        data = np.array(self.slam_map.data, dtype=np.int16).reshape(mh, mw)

        gy_arr, gx_arr = np.where(data > 50)
        if len(gy_arr) == 0:
            return
        wxa = ox + (gx_arr.astype(np.float32) + 0.5) * res
        wya = oy + (gy_arr.astype(np.float32) + 0.5) * res
        cgx = ((wxa - self.cov_map.origin) / self.cov_map.resolution).astype(np.int32)
        cgy = ((wya - self.cov_map.origin) / self.cov_map.resolution).astype(np.int32)
        valid = ((cgx >= 0) & (cgx < self.cov_map.n)
                 & (cgy >= 0) & (cgy < self.cov_map.n))
        self.cov_map.grid[cgy[valid], cgx[valid]] = -1

    # ════════════════════════════════════════════════════════════════════
    # Room 세그멘테이션
    # ════════════════════════════════════════════════════════════════════

    def _segment_rooms(self):
        if self.slam_map is None or not self._rooms_dirty:
            return
        self._rooms_dirty = False

        res  = self.slam_map.info.resolution
        ox   = self.slam_map.info.origin.position.x
        oy   = self.slam_map.info.origin.position.y
        mw   = self.slam_map.info.width
        mh   = self.slam_map.info.height
        data = np.array(self.slam_map.data, dtype=np.int16).reshape(mh, mw)

        N = self.cov_map.n
        cgy, cgx = np.mgrid[0:N, 0:N]
        wx  = (self.cov_map.origin + (cgx + 0.5) * self.cov_map.resolution).astype(np.float32)
        wy  = (self.cov_map.origin + (cgy + 0.5) * self.cov_map.resolution).astype(np.float32)
        sgx = np.clip(((wx - ox) / res).astype(np.int32), 0, mw - 1)
        sgy = np.clip(((wy - oy) / res).astype(np.int32), 0, mh - 1)
        free_mask = (data[sgy, sgx] == 0)

        if _HAS_SCIPY:
            k      = DOORWAY_EROSION * 2 + 1
            struct = np.ones((k, k), dtype=bool)
            eroded = binary_erosion(free_mask, structure=struct)
            labeled_core, num = _ndlabel(eroded)
        else:
            labeled_core, num = _bfs_label(free_mask)

        if num == 0:
            self._room_labels = np.zeros((N, N), dtype=np.int32)
            self._room_cells  = {}
            return

        # Multi-source BFS: core → corridor 확장
        labeled = labeled_core.copy()
        queue   = deque()
        visited = set()
        src_rows, src_cols = np.where(labeled_core > 0)
        for r, c in zip(src_rows.tolist(), src_cols.tolist()):
            pt = (int(r), int(c))
            queue.append(pt)
            visited.add(pt)
        while queue:
            r, c = queue.popleft()
            lbl  = int(labeled[r, c])
            for dr, dc in ((1,0),(-1,0),(0,1),(0,-1)):
                nr, nc = r+dr, c+dc
                pt = (nr, nc)
                if (0 <= nr < N and 0 <= nc < N
                        and free_mask[nr, nc]
                        and pt not in visited):
                    labeled[nr, nc] = lbl
                    visited.add(pt)
                    queue.append(pt)

        new_cells: dict[int, tuple] = {}
        for lbl in range(1, num + 1):
            gy_a, gx_a = np.where(labeled == lbl)
            if len(gy_a) >= MIN_ROOM_CELLS:
                new_cells[lbl] = (gy_a, gx_a)

        self._room_labels = labeled
        self._room_cells  = new_cells
        self.get_logger().info(
            f'[ROOM] {len(new_cells)}개 공간 세그멘테이션 '
            f'(침식 {DOORWAY_EROSION*0.2:.1f}m)')

    def _compute_room_fog_ratio(self, room_id: int) -> float:
        if room_id not in self._room_cells:
            return 1.0
        gy_a, gx_a = self._room_cells[room_id]
        total = len(gy_a)
        if total == 0:
            return 0.0
        return int(np.sum(self.cov_map.grid[gy_a, gx_a] == 0)) / total

    def _is_cluster_in_room(
        self,
        gy_arr: np.ndarray,
        gx_arr: np.ndarray,
        room_id: int,
    ) -> bool:
        """cluster 셀의 50% 이상이 해당 room에 속하면 True."""
        if self._room_labels is None:
            return False
        N = self.cov_map.n
        stride = max(1, len(gy_arr) // 12)
        in_room, total = 0, 0
        for i in range(0, len(gy_arr), stride):
            gy, gx = int(gy_arr[i]), int(gx_arr[i])
            if 0 <= gy < N and 0 <= gx < N:
                total += 1
                if self._room_labels[gy, gx] == room_id:
                    in_room += 1
        return (in_room / max(total, 1)) >= 0.5

    # ════════════════════════════════════════════════════════════════════
    # Costmap safety helpers
    # ════════════════════════════════════════════════════════════════════

    def _is_nav_safe(self, wx: float, wy: float) -> bool:
        """costmap 기반 navigation 안전성 확인. costmap 없으면 SLAM free 사용."""
        if self._global_costmap is not None and self._costmap_arr is not None:
            cm  = self._global_costmap
            res = cm.info.resolution
            ox  = cm.info.origin.position.x
            oy  = cm.info.origin.position.y
            mw  = cm.info.width
            mh  = cm.info.height
            gx  = int((wx - ox) / res)
            gy  = int((wy - oy) / res)
            if not (0 <= gx < mw and 0 <= gy < mh):
                return False
            val = int(self._costmap_arr[gy, gx])
            return 0 <= val < COSTMAP_SAFE_THRESH

        # Fallback: SLAM free
        if self.slam_map is None:
            return True
        res = self.slam_map.info.resolution
        ox  = self.slam_map.info.origin.position.x
        oy  = self.slam_map.info.origin.position.y
        gx  = int((wx - ox) / res)
        gy  = int((wy - oy) / res)
        mw, mh = self.slam_map.info.width, self.slam_map.info.height
        if not (0 <= gx < mw and 0 <= gy < mh):
            return False
        data = np.array(self.slam_map.data, dtype=np.int16).reshape(mh, mw)
        return bool(data[gy, gx] == 0)

    def _find_nav_safe_point(
        self, wx: float, wy: float
    ) -> tuple[float, float] | None:
        """(wx,wy) 근처에서 costmap-safe 지점 탐색 (최대 NAV_SAFE_RING_MAX m)."""
        if self._is_nav_safe(wx, wy):
            return wx, wy

        # costmap 기반 ring search
        if self._global_costmap is not None and self._costmap_arr is not None:
            cm  = self._global_costmap
            res = cm.info.resolution
            ox  = cm.info.origin.position.x
            oy  = cm.info.origin.position.y
            mw  = cm.info.width
            mh  = cm.info.height
            sgx = int((wx - ox) / res)
            sgy = int((wy - oy) / res)
            max_r = int(NAV_SAFE_RING_MAX / res) + 1
            for r in range(1, max_r):
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        if abs(dx) != r and abs(dy) != r:
                            continue
                        ny, nx = sgy + dy, sgx + dx
                        if not (0 <= nx < mw and 0 <= ny < mh):
                            continue
                        val = int(self._costmap_arr[ny, nx])
                        if 0 <= val < COSTMAP_SAFE_THRESH:
                            return (ox + (nx + 0.5) * res,
                                    oy + (ny + 0.5) * res)
            return None

        # Fallback: SLAM ring search
        return self._nearest_slam_free(wx, wy)

    def _nearest_slam_free(
        self, wx: float, wy: float
    ) -> tuple[float, float] | None:
        """SLAM free space 내 가장 가까운 지점 (최대 2m)."""
        if self.slam_map is None:
            return wx, wy
        res  = self.slam_map.info.resolution
        ox   = self.slam_map.info.origin.position.x
        oy   = self.slam_map.info.origin.position.y
        mw   = self.slam_map.info.width
        mh   = self.slam_map.info.height
        data = np.array(self.slam_map.data, dtype=np.int16).reshape(mh, mw)

        sgx = int((wx - ox) / res)
        sgy = int((wy - oy) / res)
        if (0 <= sgx < mw and 0 <= sgy < mh and data[sgy, sgx] == 0):
            return wx, wy
        max_r = int(2.0 / res) + 1
        for r in range(1, max_r):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if abs(dx) != r and abs(dy) != r:
                        continue
                    ny, nx = sgy + dy, sgx + dx
                    if 0 <= nx < mw and 0 <= ny < mh and data[ny, nx] == 0:
                        return (ox + (nx + 0.5) * res,
                                oy + (ny + 0.5) * res)
        return None

    # ════════════════════════════════════════════════════════════════════
    # Viewpoint planning — visibility gain 기반
    # ════════════════════════════════════════════════════════════════════

    def _estimate_visibility_gain(
        self,
        vp_wx: float,
        vp_wy: float,
        gy_arr: np.ndarray,
        gx_arr: np.ndarray,
        n_samples: int = VG_SAMPLE_RAYS,
    ) -> int:
        """viewpoint → cluster unseen 셀 가시성 ray 추정."""
        res = self.cov_map.resolution
        org = self.cov_map.origin
        stride = max(1, len(gy_arr) // n_samples)
        visible = 0

        for i in range(0, len(gy_arr), stride):
            tgy = int(gy_arr[i])
            tgx = int(gx_arr[i])
            t_wx = org + (tgx + 0.5) * res
            t_wy = org + (tgy + 0.5) * res

            dx   = t_wx - vp_wx
            dy   = t_wy - vp_wy
            dist = math.hypot(dx, dy)
            if dist < 0.05:
                visible += 1
                continue

            march_step = res * 0.9
            d = 0.1
            blocked = False
            while d < dist - march_step:
                rx = vp_wx + (d / dist) * dx
                ry = vp_wy + (d / dist) * dy
                cgx, cgy = self.cov_map._w2g(rx, ry)
                if not self.cov_map._in_bounds(cgx, cgy):
                    blocked = True
                    break
                if self.cov_map.grid[cgy, cgx] == -1:
                    blocked = True
                    break
                d += march_step

            if not blocked:
                visible += 1

        return visible

    def _select_viewpoint(
        self,
        gy_arr: np.ndarray,
        gx_arr: np.ndarray,
        cluster_wx: float,
        cluster_wy: float,
    ) -> tuple[float, float] | None:
        """
        Cluster 관측을 위한 최적 viewpoint 선택.

        전략 A: frontier 경계(seen 인접 unseen) → costmap-safe 후보 → visibility gain 최고
        전략 B: approach point (fallback)
        """
        N   = self.cov_map.n
        res = self.cov_map.resolution
        org = self.cov_map.origin

        # 1. Frontier boundary candidates (seen cells adjacent to unseen)
        seen_set: set[tuple[int, int]] = set()
        stride = max(1, len(gy_arr) // (VG_CANDIDATES * 2))
        for i in range(0, len(gy_arr), stride):
            gy, gx = int(gy_arr[i]), int(gx_arr[i])
            for dy, dx in ((0,1),(0,-1),(1,0),(-1,0)):
                ny, nx = gy+dy, gx+dx
                if (0 <= ny < N and 0 <= nx < N
                        and self.cov_map.grid[ny, nx] == 1
                        and (ny, nx) not in seen_set):
                    seen_set.add((ny, nx))

        # 2. Filter by costmap safety + min distance from robot
        safe_candidates: list[tuple[float, float]] = []
        dedup_set: list[tuple[float, float]] = []
        for (ny, nx) in seen_set:
            cwx = org + (nx + 0.5) * res
            cwy = org + (ny + 0.5) * res
            if math.hypot(cwx - self.robot_x, cwy - self.robot_y) < 0.5:
                continue
            safe = self._find_nav_safe_point(cwx, cwy)
            if safe is None:
                continue
            # Deduplicate within 0.4m
            if any(math.hypot(safe[0]-p[0], safe[1]-p[1]) < 0.4 for p in dedup_set):
                continue
            dedup_set.append(safe)
            safe_candidates.append(safe)
            if len(safe_candidates) >= VG_CANDIDATES:
                break

        # 3. Score candidates: visibility_gain - 0.05*dist
        best_score = -1.0
        best_vp: tuple[float, float] | None = None

        for vp_wx, vp_wy in safe_candidates:
            gain  = self._estimate_visibility_gain(vp_wx, vp_wy, gy_arr, gx_arr)
            dist  = math.hypot(vp_wx - self.robot_x, vp_wy - self.robot_y)
            score = float(gain) - 0.05 * dist
            if score > best_score:
                best_score = score
                best_vp    = (vp_wx, vp_wy)

        if best_vp is not None:
            return best_vp

        # 4. Fallback: approach point (APPROACH_STOP_M 앞에서 정지)
        dx   = cluster_wx - self.robot_x
        dy   = cluster_wy - self.robot_y
        dist = math.hypot(dx, dy)
        if dist < 0.1:
            return None
        t     = max(0.5, dist - APPROACH_STOP_M) / dist
        vp_wx = self.robot_x + dx * t
        vp_wy = self.robot_y + dy * t
        return self._find_nav_safe_point(vp_wx, vp_wy)

    # ════════════════════════════════════════════════════════════════════
    # Frontier 추출
    # ════════════════════════════════════════════════════════════════════

    def _extract_frontier_clusters(
        self, now: float
    ) -> list[tuple[float, float, float, np.ndarray, np.ndarray]]:
        """
        unseen cluster 추출 + 점수 계산 + viewpoint 반환.
        room-hierarchical 필터링: 현재 방이 미완료면 현재 방 cluster 우선.
        반환: [(nav_wx, nav_wy, score, gy_arr, gx_arr), ...]
        """
        N      = self.cov_map.n
        origin = self.cov_map.origin
        res    = self.cov_map.resolution
        max_c  = int(FRONTIER_SEARCH_R / res)

        rgx, rgy = self.cov_map._w2g(self.robot_x, self.robot_y)
        x0 = max(0, rgx - max_c);  x1 = min(N, rgx + max_c)
        y0 = max(0, rgy - max_c);  y1 = min(N, rgy + max_c)

        sub         = self.cov_map.grid[y0:y1, x0:x1]
        unseen_mask = (sub == 0)
        if not np.any(unseen_mask):
            return []

        if _HAS_SCIPY:
            labeled, num = _ndlabel(unseen_mask)
        else:
            labeled, num = _bfs_label(unseen_mask)

        conf    = self._get_confidence(now)
        results = []

        for lbl in range(1, num + 1):
            local_gy, local_gx = np.where(labeled == lbl)
            if len(local_gy) < MIN_FRONTIER_CELLS:
                continue

            gy_arr = local_gy + y0
            gx_arr = local_gx + x0

            cluster_wx = origin + (float(np.mean(gx_arr)) + 0.5) * res
            cluster_wy = origin + (float(np.mean(gy_arr)) + 0.5) * res

            dist = math.hypot(cluster_wx - self.robot_x, cluster_wy - self.robot_y)
            if dist < 0.5:
                continue

            # Viewpoint: visibility gain + costmap-safe
            vp = self._select_viewpoint(gy_arr, gx_arr, cluster_wx, cluster_wy)
            if vp is None:
                continue
            nav_wx, nav_wy = vp
            nav_dist = math.hypot(nav_wx - self.robot_x, nav_wy - self.robot_y)

            area_score = len(local_gy) * (res ** 2)

            # Recency bonus: 인접 seen 셀 confidence 낮을수록 재방문 우선
            nbr_sum, nbr_cnt = 0.0, 0
            sample = min(20, len(gy_arr))
            stride = max(1, len(gy_arr) // sample)
            for k in range(0, len(gy_arr), stride):
                gy, gx = int(gy_arr[k]), int(gx_arr[k])
                for dy, dx in ((0,1),(0,-1),(1,0),(-1,0)):
                    ny, nx = gy+dy, gx+dx
                    if 0 <= ny < N and 0 <= nx < N and self.cov_map.grid[ny, nx] == 1:
                        nbr_sum += float(conf[ny, nx])
                        nbr_cnt += 1
            recency_bonus = 1.0 - (nbr_sum / nbr_cnt if nbr_cnt > 0 else 1.0)

            # Room priority
            room_prio = 0.0
            if (self._room_labels is not None
                    and self._current_room_id > 0
                    and self._is_cluster_in_room(gy_arr, gx_arr, self._current_room_id)):
                room_prio = 1.0

            score = (W_AREA    * area_score
                   + W_DIST    * nav_dist
                   + W_RECENCY * recency_bonus
                   + W_ROOM_PRIO * room_prio)

            results.append((nav_wx, nav_wy, score, gy_arr, gx_arr))

        results.sort(key=lambda t: t[2], reverse=True)

        # Room-hierarchical 필터: 현재 방이 미완료면 현재 방 cluster만 허용
        if (self._room_labels is not None
                and self._current_room_id > 0
                and self._compute_room_fog_ratio(self._current_room_id) > ROOM_COMPLETE_RATIO):
            room_results = [r for r in results
                            if self._is_cluster_in_room(r[3], r[4], self._current_room_id)]
            if room_results:
                return room_results

        return results

    # ════════════════════════════════════════════════════════════════════
    # Nav2 Goal — commitment state
    # ════════════════════════════════════════════════════════════════════

    def _check_commitment(self, now: float) -> bool:
        """
        현재 commitment 유효성 확인.
        False 반환 시 새 goal 필요.
        True 반환 시 commitment 유지 (필요하면 republish).
        """
        if self._committed_goal is None:
            return False

        cx, cy = self._committed_goal
        dist   = math.hypot(self.robot_x - cx, self.robot_y - cy)

        # 도달
        if dist < GOAL_REACHED_M:
            self.get_logger().info(f'[NAV] 목표 도달 ({cx:.1f},{cy:.1f})')
            self._committed_goal = None
            self._commitment_stuck = 0
            return False

        # Timeout
        if now - self._commitment_start_t > COMMIT_TIMEOUT:
            self.get_logger().warn(f'[NAV] timeout → 재계획')
            self._committed_goal = None
            self._commitment_stuck = 0
            return False

        # Stuck 감지
        progress = self._commitment_last_dist - dist
        if progress < COMMIT_PROGRESS_M:
            self._commitment_stuck += 1
        else:
            self._commitment_stuck = 0
        self._commitment_last_dist = dist

        if self._commitment_stuck >= COMMIT_STUCK_MAX:
            self.get_logger().warn(
                f'[NAV] stuck {self._commitment_stuck}회 → 재계획')
            self._committed_goal = None
            self._commitment_stuck = 0
            return False

        return True

    def _update_nav_goal(self, now: float):
        if self._slam_dirty_cov or self.slam_map is None:
            return

        # 사용자 Goal 우선순위: user goal 활성 중이면 자율 탐색 발행 금지
        if self._user_goal_active and self._user_goal_x is not None:
            dist_to_user = math.hypot(
                self.robot_x - self._user_goal_x,
                self.robot_y - self._user_goal_y)
            if dist_to_user > GOAL_REACHED_M:
                return  # 미완료 → 자율 발행 금지
            # 완료
            self._user_goal_active = False
            self.get_logger().info('[USER GOAL] 완료 → 자율탐색 재개')

        # 방 완료 체크
        if self._room_labels is not None and self._current_room_id > 0:
            fog = self._compute_room_fog_ratio(self._current_room_id)
            if fog < ROOM_COMPLETE_RATIO and \
                    self._current_room_id not in self._room_complete:
                self._room_complete.add(self._current_room_id)
                self._committed_goal = None   # 방 완료 시 즉시 재계획
                self.get_logger().info(
                    f'[ROOM 완료] ID={self._current_room_id} fog={fog*100:.1f}%')

        valid = self._check_commitment(now)

        if valid:
            # 동일 goal 주기적 재발행 (Nav2 갱신)
            if now - self._last_nav_pub_t >= NAV_GOAL_INTERVAL:
                self._publish_nav_goal(*self._committed_goal)
            return

        # 새 goal 계획
        clusters = self._extract_frontier_clusters(now)
        if not clusters:
            self.get_logger().debug('[NAV] frontier 없음')
            return

        nav_wx, nav_wy, score, _, _ = clusters[0]

        self._committed_goal       = (nav_wx, nav_wy)
        self._commitment_start_t   = now
        self._commitment_last_dist = math.hypot(
            nav_wx - self.robot_x, nav_wy - self.robot_y)
        self._commitment_stuck     = 0
        self._nav_goal_sent_x      = nav_wx
        self._nav_goal_sent_y      = nav_wy

        self._publish_nav_goal(nav_wx, nav_wy)
        self.get_logger().info(
            f'[NAV] → ({nav_wx:.1f},{nav_wy:.1f}) '
            f'score={score:.1f} '
            f'dist={self._commitment_last_dist:.1f}m')

    def _publish_nav_goal(self, wx: float, wy: float):
        # echo 감지를 위해 자신이 발행할 좌표를 미리 기록
        self._last_self_pub_x = wx
        self._last_self_pub_y = wy
        yaw = math.atan2(wy - self.robot_y, wx - self.robot_x)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.position.x = wx
        goal.pose.position.y = wy
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(goal)
        self._last_nav_pub_t = self.get_clock().now().nanoseconds * 1e-9

    # ════════════════════════════════════════════════════════════════════
    # Contextual Gaze State Machine (5-state)
    # ════════════════════════════════════════════════════════════════════

    def _check_side_unseen(self) -> tuple[float, float] | None:
        """SIDE_CLEAR: ±60°~±120° 측면 4m 이내 unseen 영역."""
        side_offsets = [65, 80, 95, 110, -65, -80, -95, -110]
        best_count   = 0
        best_target  = None
        step         = self.cov_map.resolution * 0.8

        for offset_deg in side_offsets:
            angle = self.robot_theta + math.radians(offset_deg)
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)
            count = 0
            last_x = last_y = 0.0

            d = 0.5
            while d <= SIDE_RANGE_M:
                wx = self.robot_x + d * cos_a
                wy = self.robot_y + d * sin_a
                gx, gy = self.cov_map._w2g(wx, wy)
                if not self.cov_map._in_bounds(gx, gy):
                    break
                cell = self.cov_map.grid[gy, gx]
                if cell == -1:
                    break
                if cell == 0:
                    count += 1
                    last_x, last_y = wx, wy
                d += step

            if count >= SIDE_MIN_CELLS and count > best_count:
                best_count  = count
                best_target = (last_x, last_y)

        return best_target

    def _detect_corner_ahead(self) -> tuple[float, float] | None:
        """CORNER_PEEK: 전방 ±35°~±55° 대각선 근거리 unseen 코너 탐지."""
        best_count  = 0
        best_target = None
        step        = self.cov_map.resolution * 0.6

        for offset_deg in (38, 48, -38, -48):
            angle = self.robot_theta + math.radians(offset_deg)
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)
            count = 0
            last_x = last_y = 0.0

            d = 0.3
            while d <= CORNER_DETECT_M:
                wx = self.robot_x + d * cos_a
                wy = self.robot_y + d * sin_a
                gx, gy = self.cov_map._w2g(wx, wy)
                if not self.cov_map._in_bounds(gx, gy):
                    break
                cell = self.cov_map.grid[gy, gx]
                if cell == -1:
                    break
                if cell == 0:
                    count += 1
                    last_x, last_y = wx, wy
                d += step

            if count >= 2 and count > best_count:
                best_count  = count
                best_target = (last_x, last_y)

        return best_target

    def _detect_doorway_ahead(self) -> tuple[float, float] | None:
        """DOORWAY_GLANCE: 전방 ±15° 내 seen 구간 이후 unseen 공간 = 문 너머."""
        step  = self.cov_map.resolution * 0.7
        cos_a = math.cos(self.robot_theta)
        sin_a = math.sin(self.robot_theta)

        seen_before   = 0
        unseen_count  = 0
        last_ux = last_uy = 0.0

        d = 0.4
        while d <= DOORWAY_DETECT_M:
            wx = self.robot_x + d * cos_a
            wy = self.robot_y + d * sin_a
            gx, gy = self.cov_map._w2g(wx, wy)
            if not self.cov_map._in_bounds(gx, gy):
                break
            cell = self.cov_map.grid[gy, gx]
            if cell == -1:
                break
            elif cell == 1:
                seen_before += 1
            elif cell == 0:
                unseen_count += 1
                last_ux, last_uy = wx, wy
            d += step

        # 복도 통과 후 방 개구부: seen ≥ 2개 + unseen ≥ 4개
        if seen_before >= 2 and unseen_count >= 4:
            return last_ux, last_uy
        return None

    def _find_recheck_target(self, now: float) -> tuple[float, float] | None:
        """RECHECK: 인근 RECHECK_RANGE_M 내 low-confidence seen 영역."""
        N   = self.cov_map.n
        org = self.cov_map.origin
        res = self.cov_map.resolution
        conf = self._get_confidence(now)

        rgx, rgy = self.cov_map._w2g(self.robot_x, self.robot_y)
        r  = int(RECHECK_RANGE_M / res)
        x0 = max(0, rgx - r);  x1 = min(N, rgx + r)
        y0 = max(0, rgy - r);  y1 = min(N, rgy + r)

        sub_seen = (self.cov_map.grid[y0:y1, x0:x1] == 1)
        sub_conf = conf[y0:y1, x0:x1]
        low_mask = sub_seen & (sub_conf < RECHECK_CONF)

        if not np.any(low_mask):
            return None

        gy_loc, gx_loc = np.where(low_mask)
        if len(gy_loc) < 4:
            return None

        cx = org + (float(np.mean(gx_loc + x0)) + 0.5) * res
        cy = org + (float(np.mean(gy_loc + y0)) + 0.5) * res
        return cx, cy

    def _get_forward_gaze(self) -> tuple[float, float]:
        """FORWARD_TRAVEL: nav goal 방향 또는 정면 (항상 유효)."""
        if self._nav_goal_sent_x is not None:
            return self._nav_goal_sent_x, self._nav_goal_sent_y
        dist = 5.0
        return (self.robot_x + dist * math.cos(self.robot_theta),
                self.robot_y + dist * math.sin(self.robot_theta))

    def _determine_gaze_state(
        self, now: float
    ) -> tuple[GazeState, float, float]:
        """
        문맥 기반 gaze state 결정. 항상 유효한 target 반환.

        우선순위:
            1. SIDE_CLEAR   — 측면 즉각 가능한 안개
            2. CORNER_PEEK  — 전방 근거리 코너
            3. DOORWAY_GLANCE — 전방 문 너머 (타이머 기반)
            4. RECHECK      — 저신뢰 재확인
            5. FORWARD_TRAVEL — nav goal 방향 (기본)
        """
        # 1. SIDE_CLEAR
        side = self._check_side_unseen()
        if side is not None:
            return GazeState.SIDE_CLEAR, side[0], side[1]

        # 2. CORNER_PEEK
        corner = self._detect_corner_ahead()
        if corner is not None:
            return GazeState.CORNER_PEEK, corner[0], corner[1]

        # 3. DOORWAY_GLANCE (타이머 유지 또는 신규 감지)
        if (self._gaze_state == GazeState.DOORWAY_GLANCE
                and self._doorway_glance_tgt is not None
                and now - self._doorway_glance_t < DOORWAY_GLANCE_S):
            tx, ty = self._doorway_glance_tgt
            return GazeState.DOORWAY_GLANCE, tx, ty

        doorway = self._detect_doorway_ahead()
        if doorway is not None:
            self._doorway_glance_t   = now
            self._doorway_glance_tgt = doorway
            return GazeState.DOORWAY_GLANCE, doorway[0], doorway[1]

        # 4. RECHECK
        recheck = self._find_recheck_target(now)
        if recheck is not None:
            return GazeState.RECHECK, recheck[0], recheck[1]

        # 5. FORWARD_TRAVEL
        tx, ty = self._get_forward_gaze()
        return GazeState.FORWARD_TRAVEL, tx, ty

    # ── gaze 갱신 ──────────────────────────────────────────────────────
    def _update_look_target(self, now: float):
        if now - self._last_gaze_t < GAZE_RESCAN_S:
            return

        # Stagnation 감지: aim point가 TURRET_STAGNATION_S 동안 변하지 않으면 강제 갱신
        if self._look_tx is not None:
            moved = math.hypot(
                self._look_tx - self._last_look_tx,
                self._look_ty - self._last_look_ty)
            if moved > 0.15:
                self._gaze_stagnation_t = now
                self._last_look_tx = self._look_tx
                self._last_look_ty = self._look_ty

            if now - self._gaze_stagnation_t > TURRET_STAGNATION_S:
                tx, ty = self._get_forward_gaze()
                self._look_tx           = tx
                self._look_ty           = ty
                self._last_gaze_t       = now
                self._gaze_stagnation_t = now
                self._last_look_tx      = tx
                self._last_look_ty      = ty
                return

        self._last_gaze_t = now
        state, new_tx, new_ty = self._determine_gaze_state(now)

        # Joint limit: 필요 turret yaw > TURRET_YAW_LIMIT 이면 클램핑
        target_abs = math.atan2(new_ty - self.robot_y, new_tx - self.robot_x)
        req_yaw    = _normalize_angle(target_abs - self.robot_theta)
        if abs(req_yaw) > TURRET_YAW_LIMIT:
            clamped_yaw = math.copysign(TURRET_YAW_LIMIT, req_yaw)
            abs_clamped = self.robot_theta + clamped_yaw
            dist   = 4.0
            new_tx = self.robot_x + dist * math.cos(abs_clamped)
            new_ty = self.robot_y + dist * math.sin(abs_clamped)

        # 최소 각도 변화 미만이면 유지 (지터 방지)
        if self._look_tx is not None:
            new_a = math.atan2(new_ty - self.robot_y, new_tx - self.robot_x)
            old_a = math.atan2(self._look_ty - self.robot_y,
                               self._look_tx - self.robot_x)
            if abs(_normalize_angle(new_a - old_a)) < math.radians(MIN_AIM_CHANGE_DEG):
                return

        self._look_tx           = new_tx
        self._look_ty           = new_ty
        self._gaze_state        = state
        self._gaze_stagnation_t = now
        self._last_look_tx      = new_tx
        self._last_look_ty      = new_ty

    def _publish_aim(self):
        if self._look_tx is None:
            return
        p = Point()
        p.x = self._look_tx
        p.y = self._look_ty
        p.z = 0.0
        self.aim_pub.publish(p)

    # ════════════════════════════════════════════════════════════════════
    # 상태 머신 (2 Hz)
    # ════════════════════════════════════════════════════════════════════

    def _state_machine(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.state == ExploreState.IDLE:
            if self.slam_map is not None and self.coverage_enabled:
                self.state    = ExploreState.EXPLORE
                self._t_start = now
                self.get_logger().info('[FSM] EXPLORE 시작')
            return

        if (self._scan_min_front < PROX_STOP_M
                and self.state not in (ExploreState.RECOVERY,
                                       ExploreState.PATIENT_INTERRUPT)):
            self.get_logger().warn(f'[RECOVERY] 전방 {self._scan_min_front:.2f}m')
            self._pre_recovery_state = self.state
            self._recovery_start     = now
            self.state = ExploreState.RECOVERY
            return

        if self.patient_flag and self.state not in (
                ExploreState.PATIENT_INTERRUPT, ExploreState.RESUME):
            self.patient_flag = False
            self.state        = ExploreState.PATIENT_INTERRUPT
            return

        if self.state == ExploreState.EXPLORE:
            self._update_nav_goal(now)
            self._update_look_target(now)
            self._publish_aim()

        elif self.state == ExploreState.PATIENT_INTERRUPT:
            if not self._resume_timer_set:
                self._resume_timer_set = True
                self.create_timer(5.0, self._resume)

        elif self.state == ExploreState.RESUME:
            self.state = ExploreState.EXPLORE

        elif self.state == ExploreState.RECOVERY:
            if now - self._recovery_start > RECOVERY_DURATION:
                self.get_logger().info('[RECOVERY] 완료')
                self._committed_goal = None   # recovery 후 재계획
                self.state = self._pre_recovery_state

    def _recovery_tick(self):
        if self.state != ExploreState.RECOVERY:
            return
        elapsed = self.get_clock().now().nanoseconds * 1e-9 - self._recovery_start
        twist   = Twist()
        if elapsed < RECOVERY_BACKUP_T:
            twist.linear.x = RECOVERY_BACK_VEL
        else:
            turn = 1.0 if self._scan_left_avg >= self._scan_right_avg else -1.0
            twist.angular.z = turn * RECOVERY_TURN_VEL
        self.cmd_vel_pub.publish(twist)

    def _resume(self):
        if self.state == ExploreState.PATIENT_INTERRUPT:
            self._resume_timer_set = False
            self.state = ExploreState.RESUME

    # ════════════════════════════════════════════════════════════════════
    # 메트릭 퍼블리시
    # ════════════════════════════════════════════════════════════════════

    def _publish_metrics(self):
        now          = self.get_clock().now().nanoseconds * 1e-9
        seen_cells   = int(np.sum(self.cov_map.grid == 1))
        unseen_cells = int(np.sum(self.cov_map.grid == 0))
        free_cells   = seen_cells + unseen_cells
        cov_ratio    = seen_cells / max(free_cells, 1)
        elapsed      = now - self._t_start if self._t_start > 0 else 1.0

        conf     = self._get_confidence(now)
        avg_conf = float(np.mean(conf[self.cov_map.grid == 1])) if seen_cells > 0 else 0.0

        n_rooms    = len(self._room_cells)
        n_complete = len(self._room_complete)

        room_parts = []
        for rid in sorted(self._room_cells.keys())[:6]:
            r   = self._compute_room_fog_ratio(rid)
            sym = '✓' if rid in self._room_complete else ' '
            room_parts.append(f'[{rid}{sym}:{r*100:.0f}%]')

        gaze_name = self._gaze_state.name if hasattr(self, '_gaze_state') else '?'

        self.status_pub.publish(String(data=(
            f'coverage={cov_ratio*100:.1f}% seen={seen_cells} unseen={unseen_cells} '
            f'avg_conf={avg_conf:.2f} rooms={n_complete}/{n_rooms} '
            f'fog_cleared={self._fog_cleared_cells} elapsed={elapsed:.0f}s '
            f'state={self.state.name} gaze={gaze_name}')))

        self.get_logger().info(
            f'[메트릭] cov={cov_ratio*100:.1f}% conf={avg_conf:.2f} '
            f'rooms={n_complete}/{n_rooms} gaze={gaze_name} '
            + ' '.join(room_parts))

    # ════════════════════════════════════════════════════════════════════
    # 시각화
    # ════════════════════════════════════════════════════════════════════

    def _publish_coverage(self):
        msg = self.cov_map.to_occupancy_grid(
            self.get_clock().now().to_msg(), frame_id='map')
        self.cov_pub.publish(msg)
        self._publish_debug_markers()

    def _publish_debug_markers(self):
        now_msg = self.get_clock().now().to_msg()
        ma      = MarkerArray()
        rx, ry  = self.robot_x, self.robot_y

        # FOV 콘
        cone = Marker()
        cone.header.frame_id = 'map'; cone.header.stamp = now_msg
        cone.ns = 'fov_cone'; cone.id = 0
        cone.type = Marker.TRIANGLE_LIST; cone.action = Marker.ADD
        cone.scale.x = cone.scale.y = cone.scale.z = 1.0
        cone.color.r = 1.0; cone.color.g = 1.0; cone.color.b = 0.0; cone.color.a = 0.15
        cone.lifetime.sec = 1
        cam = self.robot_theta + self.turret_yaw
        for i in range(14):
            a1 = cam - FOV_RAD/2 + i       * FOV_RAD/14
            a2 = cam - FOV_RAD/2 + (i+1)   * FOV_RAD/14
            p0 = Point(); p0.x = rx;                             p0.y = ry;                             p0.z = 0.05
            p1 = Point(); p1.x = rx+CAM_RANGE_M*math.cos(a1);   p1.y = ry+CAM_RANGE_M*math.sin(a1);   p1.z = 0.05
            p2 = Point(); p2.x = rx+CAM_RANGE_M*math.cos(a2);   p2.y = ry+CAM_RANGE_M*math.sin(a2);   p2.z = 0.05
            cone.points.extend([p0, p1, p2])
        ma.markers.append(cone)

        # Gaze target
        if self._look_tx is not None:
            gaze_colors = {
                GazeState.SIDE_CLEAR:     (0.2, 1.0, 0.2),
                GazeState.CORNER_PEEK:    (1.0, 0.6, 0.0),
                GazeState.DOORWAY_GLANCE: (0.0, 0.8, 1.0),
                GazeState.RECHECK:        (0.8, 0.2, 1.0),
                GazeState.FORWARD_TRAVEL: (0.2, 0.8, 1.0),
            }
            cr, cg, cb = gaze_colors.get(self._gaze_state, (0.2, 0.8, 1.0))
            sph = Marker()
            sph.header.frame_id = 'map'; sph.header.stamp = now_msg
            sph.ns = 'gaze_target'; sph.id = 1
            sph.type = Marker.SPHERE; sph.action = Marker.ADD
            sph.pose.position.x = self._look_tx
            sph.pose.position.y = self._look_ty
            sph.pose.position.z = 0.3
            sph.pose.orientation.w = 1.0
            sph.scale.x = sph.scale.y = sph.scale.z = 0.35
            sph.color.r = cr; sph.color.g = cg; sph.color.b = cb; sph.color.a = 0.9
            sph.lifetime.sec = 3
            line = Marker()
            line.header.frame_id = 'map'; line.header.stamp = now_msg
            line.ns = 'gaze_line'; line.id = 2
            line.type = Marker.LINE_STRIP; line.action = Marker.ADD
            line.scale.x = 0.04
            line.color.r = cr; line.color.g = cg; line.color.b = cb; line.color.a = 0.5
            line.lifetime.sec = 3
            ps = Point(); ps.x = rx; ps.y = ry; ps.z = 0.3
            pe = Point(); pe.x = self._look_tx; pe.y = self._look_ty; pe.z = 0.3
            line.points = [ps, pe]
            ma.markers.extend([sph, line])

        # Nav2 committed goal
        if self._committed_goal is not None:
            cx, cy = self._committed_goal
            gm = Marker()
            gm.header.frame_id = 'map'; gm.header.stamp = now_msg
            gm.ns = 'nav_viewpoint'; gm.id = 3
            gm.type = Marker.CYLINDER; gm.action = Marker.ADD
            gm.pose.position.x = cx; gm.pose.position.y = cy; gm.pose.position.z = 0.1
            gm.pose.orientation.w = 1.0
            gm.scale.x = 0.7; gm.scale.y = 0.7; gm.scale.z = 0.25
            gm.color.r = 0.0; gm.color.g = 1.0; gm.color.b = 0.4; gm.color.a = 0.85
            gm.lifetime.sec = 12
            gl = Marker()
            gl.header.frame_id = 'map'; gl.header.stamp = now_msg
            gl.ns = 'nav_line'; gl.id = 4
            gl.type = Marker.LINE_STRIP; gl.action = Marker.ADD
            gl.scale.x = 0.07
            gl.color.r = 0.0; gl.color.g = 1.0; gl.color.b = 0.4; gl.color.a = 0.45
            gl.lifetime.sec = 12
            gs = Point(); gs.x = rx;  gs.y = ry;  gs.z = 0.15
            ge = Point(); ge.x = cx;  ge.y = cy;  ge.z = 0.15
            gl.points = [gs, ge]
            ma.markers.extend([gm, gl])

        self.debug_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = VisibilityExplorationNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
