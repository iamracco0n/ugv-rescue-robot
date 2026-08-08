"""
visibility_overlay_node.py  —  Door Kickers Style Visibility Overlay
====================================================================
이벤트 기반 FOV 갱신: 뎁스 수신 / yaw 변화 / 위치 변화 시 즉시 갱신.
저속 타이머(2Hz)는 시간 등급 메모리 · 벽 · 블라인드 코너만 담당.

퍼블리시 토픽:
  /viz/seen_persistent  OccupancyGrid  — Fog of War: 미수색=검정
  /viz/seen_recent      OccupancyGrid  — 시간 등급 수색 이력 (0-5s 투명→30s+ 회색)
  /viz/walls            OccupancyGrid  — SLAM 벽 = 흰색 (관측된 벽만)
  /viz/active_fov       MarkerArray    — 현재 카메라 시야 콘 (밝은 노란색)
  /viz/blind_corners    MarkerArray    — 위험 사각지대
  /viz/robot_trail      Path           — 이동 궤적
"""
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import tf2_ros

from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Image as RosImage, JointState
from visualization_msgs.msg import Marker, MarkerArray

# ── 커버리지 격자 파라미터 ────────────────────────────────────────────────
_RES    = 0.2
_SIZE   = 40.0
_N      = int(_SIZE / _RES)      # 200
_ORIGIN = -_SIZE / 2.0           # -20.0 m

# ── 카메라 파라미터 (D435i) ───────────────────────────────────────────────
# 시뮬 카메라 수평 FOV (ugv.urdf.xacro: 1.089rad ≈ 62.4°).
# 실기 D435i 는 87° 이지만, 시뮬에서 87° 를 쓰면 커버리지 부채꼴이 실제보다
# 40% 넓게 그려져 못 본 구역을 봤다고 표시한다.
_FOV_RAD      = 1.089
_CAM_RANGE    = 8.0               # Gazebo far clip = D435i 최대 사거리 (m)
_DEPTH_RAYS   = 48
_IMG_W        = 640
_IMG_H        = 480
_HORIZON_ROWS = 5                 # 수평선 밴드 폭 (중심 ±2픽셀, 5행)

# ── 이벤트 기반 갱신 임계값 ──────────────────────────────────────────────
_YAW_THRESH_RAD  = math.radians(2.5)   # 이 각도 이상 변화 시 즉시 갱신
_POSE_THRESH_M   = 0.07               # 이 거리 이상 이동 시 즉시 갱신
_MIN_FOV_INTERVAL = 0.05              # 비(非)뎁스 트리거 최대 20Hz 캡

# ── 시간 등급 메모리 ──────────────────────────────────────────────────────
_T_FRESH = 5.0
_T_FADE1 = 15.0
_T_FADE2 = 30.0
_V_FADE1 = 10
_V_FADE2 = 20
_V_OLD   = 35

# ── 블라인드 코너 ────────────────────────────────────────────────────────
_BLIND_R   = 6.0
_MAX_MARKS = 250

# ── QoS 프로파일 ─────────────────────────────────────────────────────────
_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class VisibilityOverlayNode(Node):

    def __init__(self):
        super().__init__('visibility_overlay_node')

        # ── 커버리지 격자 ─────────────────────────────────────────────
        self._seen   = np.zeros((_N, _N), dtype=np.uint8)
        self._last_t = np.zeros((_N, _N), dtype=np.float64)

        # ── 로봇 상태 (TF 폴백용) ────────────────────────────────────
        self.robot_x     = 0.0
        self.robot_y     = 0.0
        self.robot_theta = 0.0
        self.turret_yaw  = 0.0

        # ── 이동 궤적 ─────────────────────────────────────────────────
        self._trail: deque[tuple[float, float]] = deque(maxlen=2000)
        self._trail_last_x = None

        # ── SLAM 맵 + 캐시 ─────────────────────────────────────────────
        self.slam_map: OccupancyGrid | None = None
        self._slam_proj_free: np.ndarray | None = None
        self._slam_proj_wall: np.ndarray | None = None
        self._slam_dirty = True

        # ── D435i 뎁스 ────────────────────────────────────────────────
        self._bridge        = CvBridge()
        self._latest_depth: RosImage | None = None
        self._depth_rays: list[float] = []

        # ── TF2 버퍼 / 리스너 ────────────────────────────────────────
        # cache_time=30s: Gazebo 일시정지/재개 시 클럭 점프 후 버퍼 부족 방지
        self._tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=30))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        # ── 이벤트 기반 갱신 상태 ─────────────────────────────────────
        self._last_fov_t   = 0.0       # 마지막 FOV 갱신 시각 (초)
        self._last_depth_t = 0.0       # 마지막 뎁스 수신 시각 (초, 0=미수신)

        # ── 구독 ──────────────────────────────────────────────────────
        self.create_subscription(Odometry,      '/odom',         self._odom_cb,  10)
        self.create_subscription(JointState,    '/joint_states', self._joint_cb, 10)
        self.create_subscription(OccupancyGrid, '/map',          self._map_cb,   10)
        self.create_subscription(
            RosImage,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self._depth_cb,
            _SENSOR_QOS,
        )

        # ── 퍼블리시 ─────────────────────────────────────────────────
        self.pub_fog   = self.create_publisher(OccupancyGrid, '/viz/seen_persistent', _RELIABLE_QOS)
        self.pub_grey  = self.create_publisher(OccupancyGrid, '/viz/seen_recent',     _RELIABLE_QOS)
        self.pub_walls = self.create_publisher(OccupancyGrid, '/viz/walls',           _RELIABLE_QOS)
        self.pub_fov   = self.create_publisher(MarkerArray,   '/viz/active_fov',      _RELIABLE_QOS)
        self.pub_blind = self.create_publisher(MarkerArray,   '/viz/blind_corners',   _RELIABLE_QOS)
        self.pub_trail = self.create_publisher(Path,          '/viz/robot_trail',     _RELIABLE_QOS)

        # ── 타이머: 시간 등급 메모리 + 벽 + 블라인드 코너만 (2Hz) ────
        self.create_timer(0.5, self._tick_slow)

        self.get_logger().info(
            'VisibilityOverlayNode 시작 — 이벤트 기반 FOV (depth/yaw/pose 트리거)')

    # ════════════════════════════════════════════════════════════════════
    # 콜백 — 이벤트 트리거
    # ════════════════════════════════════════════════════════════════════

    def _odom_cb(self, msg: Odometry):
        prev_x, prev_y = self.robot_x, self.robot_y

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        if (self._trail_last_x is None or
                math.hypot(self.robot_x - self._trail_last_x[0],
                           self.robot_y - self._trail_last_x[1]) > 0.15):
            self._trail.append((self.robot_x, self.robot_y))
            self._trail_last_x = (self.robot_x, self.robot_y)

    def _joint_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name == 'turret_yaw_joint':
                self.turret_yaw = pos

    def _map_cb(self, msg: OccupancyGrid):
        self.slam_map    = msg
        self._slam_dirty = True

    def _depth_cb(self, msg: RosImage):
        """새 뎁스 프레임 → 즉시 FOV 갱신 (15Hz 자연 throttle). 뎁스만 FOV 트리거."""
        self._latest_depth = msg
        self._last_depth_t = self.get_clock().now().nanoseconds * 1e-9
        self._do_fov_update()

    # ── 실제 FOV 갱신 + 즉시 발행 ────────────────────────────────────
    def _do_fov_update(self):
        now   = self.get_clock().now().nanoseconds * 1e-9
        stamp = self.get_clock().now().to_msg()
        self._last_fov_t = now

        if self._slam_dirty:
            self._rebuild_slam_proj()

        self._update_fov(now)

        self.pub_fog.publish(self._make_fog_grid(stamp))
        self.pub_fov.publish(self._make_fov_marker(stamp))
        self.pub_trail.publish(self._make_trail_path(stamp))

    # ════════════════════════════════════════════════════════════════════
    # TF 유틸 — map → camera_link 변환
    # ════════════════════════════════════════════════════════════════════

    def _get_camera_pose(self):
        """map 프레임에서 카메라 위치(x,y)와 방향(yaw) 조회.
        TF 실패 시 None 반환 — odom 폴백 없음 (odom≠map 좌표 혼용 방지)."""
        try:
            tf = self._tf_buf.lookup_transform(
                'map', 'camera_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
            cam_x = tf.transform.translation.x
            cam_y = tf.transform.translation.y
            q = tf.transform.rotation
            cam_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return cam_x, cam_y, cam_yaw
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF map→camera_link 실패, FOV 갱신 건너뜀: {e}',
                throttle_duration_sec=5.0)
            return None

    # ════════════════════════════════════════════════════════════════════
    # 격자 유틸
    # ════════════════════════════════════════════════════════════════════

    def _w2g(self, wx, wy):
        return int((wx - _ORIGIN) / _RES), int((wy - _ORIGIN) / _RES)

    def _in_bounds(self, gx, gy):
        return 0 <= gx < _N and 0 <= gy < _N

    def _make_og(self, stamp, data: np.ndarray) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'map'
        msg.info.resolution = _RES
        msg.info.width      = _N
        msg.info.height     = _N
        msg.info.origin.position.x = _ORIGIN
        msg.info.origin.position.y = _ORIGIN
        msg.info.origin.orientation.w = 1.0
        msg.data = data.astype(np.int8).flatten().tolist()
        return msg

    def _rebuild_slam_proj(self):
        """SLAM 맵을 커버리지 격자에 투영 (맵 변경 시만 재계산)."""
        if self.slam_map is None:
            return
        res  = self.slam_map.info.resolution
        ox   = self.slam_map.info.origin.position.x
        oy   = self.slam_map.info.origin.position.y
        mw   = self.slam_map.info.width
        mh   = self.slam_map.info.height
        data = np.array(self.slam_map.data, dtype=np.int16).reshape(mh, mw)

        cgy, cgx = np.mgrid[0:_N, 0:_N]
        wx = (_ORIGIN + (cgx + 0.5) * _RES).astype(np.float32)
        wy = (_ORIGIN + (cgy + 0.5) * _RES).astype(np.float32)

        # in_map을 클리핑 전에 계산해야 SLAM 맵 외부 셀을 올바르게 분류함.
        # 클리핑 후 계산하면 항상 True가 되어 외부 셀이 엣지 셀 값으로 잘못 분류됨.
        raw_sgx = ((wx - ox) / res).astype(np.int32)
        raw_sgy = ((wy - oy) / res).astype(np.int32)
        in_map  = (raw_sgx >= 0) & (raw_sgx < mw) & (raw_sgy >= 0) & (raw_sgy < mh)
        sgx = np.clip(raw_sgx, 0, mw - 1)
        sgy = np.clip(raw_sgy, 0, mh - 1)

        slam_val = data[sgy, sgx]
        self._slam_proj_free = in_map & (slam_val == 0)

        # 벽 마스크: SLAM 맵 안에 있으면서 점유값 > 50인 셀.
        # 1셀(0.2m) 팽창: SLAM(0.05m) vs 커버리지(0.2m) 해상도 불일치로 생기는
        # 얇은 벽 누락 방지 (레이가 벽 셀을 건너뛰는 현상 억제).
        wall_raw = in_map & (slam_val > 50)
        w = wall_raw.copy()
        w[1:,  :] |= wall_raw[:-1, :]
        w[:-1, :] |= wall_raw[1:,  :]
        w[:,  1:] |= wall_raw[:, :-1]
        w[:, :-1] |= wall_raw[:, 1: ]
        self._slam_proj_wall = w
        self._slam_dirty     = False

    # ════════════════════════════════════════════════════════════════════
    # FOV 업데이트 — D435i 뎁스 전용 (폴백 없음)
    # ════════════════════════════════════════════════════════════════════

    def _update_fov(self, now: float):
        """뎁스 없으면 안개를 건드리지 않고 대기."""
        if self._latest_depth is None:
            return
        self._update_fov_depth(now)

    def _update_fov_depth(self, now: float):
        """
        D435i 뎁스 기반 정확한 FOV 마킹.
        - 열별 25th 퍼센타일 깊이 (소음/작은 장애물 강인성)
        - TF에서 카메라 실제 위치/방향 획득
        - 뎁스 디코딩 실패 시 안개 갱신 건너뜀 (폴백 없음)
        """
        try:
            img = self._bridge.imgmsg_to_cv2(
                self._latest_depth, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(
                f'depth decode 실패, FOV 갱신 건너뜀: {e}', throttle_duration_sec=5.0)
            return
        finally:
            self._latest_depth = None   # 소비 완료

        h, w = img.shape[:2]
        pose = self._get_camera_pose()
        if pose is None:
            return   # TF 미확립 — odom 좌표로 map 안개 갱신하면 위치 오류
        cam_x, cam_y, cam_yaw = pose
        half = _FOV_RAD / 2.0

        # Gazebo depth camera는 float32(m), RealSense HW는 uint16(mm)
        depth_scale = 0.001 if img.dtype == np.uint16 else 1.0

        # 수평선(중심행) 밴드만 사용.
        # 전체 수직 FOV를 쓰면 상단 픽셀이 벽 꼭대기나 벽 너머를 찍어
        # 2D 맵에 X-ray vision이 생긴다. 중심 ±2픽셀(5행)만 추출하면
        # 카메라와 같은 높이의 수평 거리만 반영된다.
        cy = h // 2
        r0 = max(0, cy - _HORIZON_ROWS // 2)
        r1 = min(h, r0 + _HORIZON_ROWS)
        row_block = img[r0:r1, :].astype(np.float32)
        row_block[row_block == 0] = np.nan
        col_depth_raw = np.nanmin(row_block, axis=0)   # (w,) — 좁은 밴드이므로 min 사용

        stride = max(1, w // _DEPTH_RAYS)
        step   = _RES * 0.5   # 0.1m — 벽 셀 건너뜀 방지
        self._depth_rays = []

        for col in range(0, w, stride):
            d_raw = col_depth_raw[col]
            if not np.isfinite(d_raw):
                continue   # 유효하지 않은 뎁스 → 이 레이 전체 건너뜀 (폴백 없음)
            d_m = float(d_raw) * depth_scale
            d_m = min(max(d_m, 0.1), _CAM_RANGE)
            self._depth_rays.append(d_m)

            angle = cam_yaw - half + (col / w) * _FOV_RAD
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)

            d = 0.25
            while d <= d_m:
                wx = cam_x + d * cos_a
                wy = cam_y + d * sin_a
                gx, gy = self._w2g(wx, wy)
                if not self._in_bounds(gx, gy):
                    break
                self._seen[gy, gx]   = 1
                self._last_t[gy, gx] = now
                d += step

            # 뎁스 히트 포인트도 표시
            wx = cam_x + d_m * cos_a
            wy = cam_y + d_m * sin_a
            gx, gy = self._w2g(wx, wy)
            if self._in_bounds(gx, gy):
                self._seen[gy, gx]   = 1
                self._last_t[gy, gx] = now

    # ════════════════════════════════════════════════════════════════════
    # OccupancyGrid 레이어
    # ════════════════════════════════════════════════════════════════════

    def _make_fog_grid(self, stamp) -> OccupancyGrid:
        """
        카메라 기반 안개.
        카메라가 아직 보지 않은 모든 셀 → 100 (검정 안개)
        카메라가 본 셀                  → -1  (투명, 배경 노출)

        SLAM free/unknown 구분 없이 카메라 시야로만 안개 제거.
        라이다는 Nav2 코스트맵용이고, 안개는 카메라 탐색으로만 걷힌다.
        """
        out = np.full((_N, _N), np.int8(100), dtype=np.int8)   # 전부 검정 안개
        out[self._seen.astype(bool)] = np.int8(-1)              # 카메라가 본 곳만 투명
        return self._make_og(stamp, out)

    def _make_grey_grid(self, stamp, now: float) -> OccupancyGrid:
        """
        시간 등급별 수색 이력 (2Hz 타이머에서만 호출).
          0  ~ 5s  → -1   (투명)
          5  ~15s  → 10   (옅은 회색)
         15  ~30s  → 20   (회색)
         30s+      → 35   (짙은 회색 영구)
        """
        out      = np.full((_N, _N), np.int8(-1), dtype=np.int8)
        observed = self._last_t > 0.0
        if not np.any(observed):
            return self._make_og(stamp, out)

        age = np.where(observed, now - self._last_t, 0.0).astype(np.float32)
        out[observed & (age >= _T_FADE2)]                        = np.int8(_V_OLD)
        out[observed & (age >= _T_FADE1) & (age < _T_FADE2)]    = np.int8(_V_FADE2)
        out[observed & (age >= _T_FRESH) & (age < _T_FADE1)]    = np.int8(_V_FADE1)
        # 0~5s: 이미 -1 (투명)
        return self._make_og(stamp, out)

    def _make_walls_grid(self, stamp) -> OccupancyGrid:
        """관측된 SLAM 벽 → 0 (흰색), 미관측 벽 → -1 (투명)."""
        out = np.full((_N, _N), np.int8(-1), dtype=np.int8)
        if self._slam_proj_wall is None:
            return self._make_og(stamp, out)

        s = self._seen.astype(bool)
        s_exp = s.copy()
        s_exp[1:,  :] |= s[:-1, :]
        s_exp[:-1, :] |= s[1:,  :]
        s_exp[:,  1:] |= s[:, :-1]
        s_exp[:, :-1] |= s[:, 1: ]

        out[self._slam_proj_wall & s_exp] = np.int8(0)
        return self._make_og(stamp, out)

    # ════════════════════════════════════════════════════════════════════
    # FOV 콘 마커
    # ════════════════════════════════════════════════════════════════════

    def _make_fov_marker(self, stamp) -> MarkerArray:
        ma = MarkerArray()
        pose = self._get_camera_pose()
        if pose is None:
            return ma
        cam_x, cam_y, cam_yaw = pose

        cone = Marker()
        cone.header.frame_id = 'map'; cone.header.stamp = stamp
        cone.ns = 'active_fov'; cone.id = 0
        cone.type   = Marker.TRIANGLE_LIST
        cone.action = Marker.ADD
        cone.scale.x = cone.scale.y = cone.scale.z = 1.0
        cone.color.r = 1.0; cone.color.g = 0.95
        cone.color.b = 0.5; cone.color.a = 0.40
        cone.lifetime.sec = 3   # 3s: depth 일시 드롭 시 콘이 깜빡이지 않게

        half_fov = _FOV_RAD / 2.0

        if self._depth_rays and len(self._depth_rays) >= 2:
            rays = self._depth_rays
            n    = len(rays)
            for i in range(n - 1):
                a1 = cam_yaw - half_fov + (i / n)       * _FOV_RAD
                a2 = cam_yaw - half_fov + ((i + 1) / n) * _FOV_RAD
                r1 = rays[i]; r2 = rays[i + 1]
                p0 = Point(); p0.x = cam_x;                          p0.y = cam_y;                          p0.z = 0.02
                p1 = Point(); p1.x = cam_x + r1 * math.cos(a1);     p1.y = cam_y + r1 * math.sin(a1);     p1.z = 0.02
                p2 = Point(); p2.x = cam_x + r2 * math.cos(a2);     p2.y = cam_y + r2 * math.sin(a2);     p2.z = 0.02
                cone.points.extend([p0, p1, p2])
        # 뎁스 없으면 콘 마커 비워둠 (기하 폴백 없음)
        ma.markers.append(cone)

        arrow = Marker()
        arrow.header.frame_id = 'map'; arrow.header.stamp = stamp
        arrow.ns = 'turret_los'; arrow.id = 1
        arrow.type   = Marker.ARROW
        arrow.action = Marker.ADD
        s_pt = Point(); s_pt.x = cam_x; s_pt.y = cam_y; s_pt.z = 0.4
        los_r = self._depth_rays[len(self._depth_rays) // 2] if self._depth_rays else _CAM_RANGE
        e_pt = Point()
        e_pt.x = cam_x + los_r * math.cos(cam_yaw)
        e_pt.y = cam_y + los_r * math.sin(cam_yaw)
        e_pt.z = 0.4
        arrow.points = [s_pt, e_pt]
        arrow.scale.x = 0.07; arrow.scale.y = 0.16; arrow.scale.z = 0.0
        arrow.color.r = 1.0; arrow.color.g = 1.0
        arrow.color.b = 0.0; arrow.color.a = 1.0
        arrow.lifetime.sec = 3
        ma.markers.append(arrow)

        return ma

    # ════════════════════════════════════════════════════════════════════
    # 이동 궤적
    # ════════════════════════════════════════════════════════════════════

    def _make_trail_path(self, stamp) -> Path:
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp    = stamp
        for x, y in self._trail:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        return path

    # ════════════════════════════════════════════════════════════════════
    # 2Hz 타이머 — 시간 등급 메모리 + 벽 + 블라인드 코너
    # ════════════════════════════════════════════════════════════════════

    def _tick_slow(self):
        now   = self.get_clock().now().nanoseconds * 1e-9
        stamp = self.get_clock().now().to_msg()

        # 뎁스 미수신 경고 (최초 수신 이후 2초 이상 끊기면 WARN)
        if self._last_depth_t > 0.0 and (now - self._last_depth_t) > 2.0:
            self.get_logger().warn(
                f'뎁스 미수신 {now - self._last_depth_t:.1f}s — FOV 안개 갱신 중단',
                throttle_duration_sec=5.0)

        self.pub_grey.publish(self._make_grey_grid(stamp, now))

        if self.slam_map is None:
            return
        self.pub_walls.publish(self._make_walls_grid(stamp))
        self._publish_blind_corners(stamp)

        # depth 드롭 시 FOV 콘이 사라지는 것 방지: 마지막 레이로 2Hz 재발행
        if self._depth_rays:
            self.pub_fov.publish(self._make_fov_marker(stamp))

    def _publish_blind_corners(self, stamp):
        res  = self.slam_map.info.resolution
        ox_m = self.slam_map.info.origin.position.x
        oy_m = self.slam_map.info.origin.position.y
        mw   = self.slam_map.info.width
        mh   = self.slam_map.info.height

        data = np.array(self.slam_map.data, dtype=np.int16).reshape(mh, mw)
        unk  = (data == -1)
        free = (data == 0)
        wall = (data > 50)

        def _nbr(mask):
            out = np.zeros_like(mask, dtype=bool)
            out[1:,  :] |= mask[:-1, :];  out[:-1, :] |= mask[1:,  :]
            out[:,  1:] |= mask[:, :-1];  out[:, :-1] |= mask[:, 1: ]
            return out

        occ_edge  = unk & _nbr(free)
        door_edge = occ_edge & _nbr(wall)

        rgx = int((self.robot_x - ox_m) / res)
        rgy = int((self.robot_y - oy_m) / res)
        rc  = int(_BLIND_R / res) + 1

        x0 = max(0, rgx - rc);  x1 = min(mw, rgx + rc + 1)
        y0 = max(0, rgy - rc);  y1 = min(mh, rgy + rc + 1)

        local_door = door_edge[y0:y1, x0:x1]
        local_occ  = occ_edge[y0:y1, x0:x1] & ~door_edge[y0:y1, x0:x1]

        def _cells_to_world(gy_arr, gx_arr, y_off, x_off):
            if len(gy_arr) == 0:
                return np.array([]), np.array([])
            wxa = ox_m + (gx_arr + x_off + 0.5) * res
            wya = oy_m + (gy_arr + y_off + 0.5) * res
            d   = np.sqrt((wxa - self.robot_x)**2 + (wya - self.robot_y)**2)
            return wxa[d <= _BLIND_R], wya[d <= _BLIND_R]

        def _ds(wx, wy, n):
            if len(wx) > n:
                idx = np.random.choice(len(wx), n, replace=False)
                return wx[idx], wy[idx]
            return wx, wy

        door_gy, door_gx = np.where(local_door)
        occ_gy,  occ_gx  = np.where(local_occ)
        door_wx, door_wy = _ds(*_cells_to_world(door_gy, door_gx, y0, x0), _MAX_MARKS)
        occ_wx,  occ_wy  = _ds(*_cells_to_world(occ_gy,  occ_gx,  y0, x0), _MAX_MARKS)

        ma = MarkerArray()
        clr = Marker(); clr.action = Marker.DELETEALL
        ma.markers.append(clr)

        if len(door_wx) > 0:
            m = Marker()
            m.header.frame_id = 'map'; m.header.stamp = stamp
            m.ns = 'door_edge'; m.id = 1
            m.type = Marker.CUBE_LIST; m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = res * 1.6
            m.color.r = 1.0; m.color.g = 0.1; m.color.b = 0.0; m.color.a = 0.85
            m.lifetime.sec = 3
            for wx, wy in zip(door_wx.tolist(), door_wy.tolist()):
                p = Point(); p.x = wx; p.y = wy; p.z = 0.18
                m.points.append(p)
            ma.markers.append(m)

        if len(occ_wx) > 0:
            m = Marker()
            m.header.frame_id = 'map'; m.header.stamp = stamp
            m.ns = 'occ_edge'; m.id = 2
            m.type = Marker.CUBE_LIST; m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = res * 1.2
            m.color.r = 1.0; m.color.g = 0.55; m.color.b = 0.0; m.color.a = 0.60
            m.lifetime.sec = 3
            for wx, wy in zip(occ_wx.tolist(), occ_wy.tolist()):
                p = Point(); p.x = wx; p.y = wy; p.z = 0.12
                m.points.append(p)
            ma.markers.append(m)

        self.pub_blind.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = VisibilityOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
