"""target_manager_node.py
v7 — SEARCH-only + 블라인드코너 지능형 스캔

[타이밍 버그 수정]
  target_cb에서 turret_yaw 스냅샷 보관 → _try_register에서 동일 yaw 사용.
  sine sweep 중 탐지 후 50ms 지나면 yaw가 달라지므로 등록 위치가 틀림.

[지능형 스캔]
  /viz/blind_corners (door_edge + occ_edge) → 로봇 기준 각도 변환
  → 15° 클러스터링 → 거리 우선 정렬 → 순차 조준 (1.2s 드웰)
  → 코너 없으면 ±50° 사인파 폴백
"""
import math

import rclpy
import rclpy.time
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray

from ugv_msgs.msg import TargetDetection

# ── 튜닝 파라미터 ──────────────────────────────────────────────────────
SEARCH_AMP   = math.radians(50)  # 사인파 폴백 진폭
SEARCH_OMEGA = 0.6               # 사인파 각속도 (rad/s) — 낮출수록 위치 오차 줄어듦
KP_SRCH      = 2.0              # 스캔 P 게인 (rad/s per rad)

TURRET_HW_LIMIT  = math.radians(355)  # Yaw 소프트 한계 (URDF ±360° − 5° 마진)
PITCH_SLEW_MAX   = 2.0                # pitch 최대 변화율 (rad/s²)
CAM_FOV_RAD      = math.radians(87)   # D435i 수평 FOV

MERGE_M          = 1.5    # 기발견 환자 중복 판정 반경 (m)
IGNORE_R         = 0.5    # 확인 완료 환자 억제 반경 (m)
MSG_FRESHNESS_S  = 1.0    # YOLO 신선도 한계 (s)
CONFIRM_N        = 3      # 연속 탐지 횟수 — 오탐 방지

# 블라인드코너 스캔 파라미터
SCAN_DWELL_S     = 1.2           # 각 스캔 포인트 체류 시간 (s)
SCAN_AT_RAD      = math.radians(5)   # 도달 판정 임계각
SCAN_UPDATE_S    = 4.0           # 스캔 큐 갱신 주기 (s)
SCAN_CLUSTER_RAD = math.radians(15)  # 클러스터 병합 임계각
N_SCAN_MAX       = 8             # 최대 스캔 포인트 수

_TRIAGE_COLORS = {
    1: (1.0, 0.0,  0.0),
    2: (1.0, 0.55, 0.0),
    3: (0.0, 0.9,  0.0),
}


def euler_from_quaternion(q):
    t3 = 2.0 * (q.w * q.z + q.x * q.y)
    t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(t3, t4)


def get_room_name(x, y):
    if y >  4.0: return 'Room A' if x < -1.0 else 'Room B'
    if y < -4.0: return 'Room C' if x < -1.0 else 'Room D'
    return 'Main Hall'


class TargetManager(Node):
    def __init__(self):
        super().__init__('target_manager_node')

        # ── 구독 ─────────────────────────────────────────────────────
        self.create_subscription(TargetDetection, '/target_detection',  self.target_cb,        10)
        self.create_subscription(Odometry,        '/odom',              self.odom_cb,           10)
        self.create_subscription(Joy,             '/joy',               self.joy_cb,            10)
        self.create_subscription(JointState,      '/joint_states',      self.joint_cb,          10)
        self.create_subscription(JointState,      '/measured_joint_states', self.joint_cb,      10)
        self.create_subscription(Point,           '/apex_aim_point',    self._apex_aim_cb,      10)
        self.create_subscription(MarkerArray,     '/viz/blind_corners', self._blind_corners_cb, 10)

        # ── 퍼블리시 ──────────────────────────────────────────────────
        self.yaw_pub    = self.create_publisher(Float64,     '/turret_yaw_cmd',   10)
        self.pitch_pub  = self.create_publisher(Float64,     '/turret_pitch_cmd', 10)
        self.turret_pub = self.create_publisher(Vector3,     '/turret_cmd',       10)
        self.marker_pub = self.create_publisher(MarkerArray, '/patient_markers',  10)
        self.arrow_pub  = self.create_publisher(Marker,      '/turret_heading',   10)

        # ── TF2 버퍼 (map → base_footprint 로봇 자세) ────────────────
        self._tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=30))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        # ── 로봇 상태 (odom 폴백용) ──────────────────────────────────
        self.robot_x      = 0.0
        self.robot_y      = 0.0
        self.robot_theta  = 0.0
        self.turret_yaw   = 0.0
        self.turret_pitch = 0.0

        # ── 탐지 상태 ─────────────────────────────────────────────────
        self.last_msg    = None   # capture_turret_yaw 필드 포함
        self.last_seen_t = None
        self.last_manual_t = None
        self._detect_streak = 0

        self._prev_pitch_vel = 0.0
        self._loop_dt        = 0.05

        # ── 커버리지 네비게이터 조준점 ────────────────────────────────
        self.apex_aim   = None
        self.apex_aim_t = None

        # ── 블라인드코너 스캔 상태 ────────────────────────────────────
        self._blind_corners: list[tuple[float, float]] = []
        self._scan_queue: list[float] = []   # 순서대로 조준할 turret_yaw 각도 목록
        self._scan_idx       = 0
        self._scan_arrive_t: float | None = None
        self._scan_update_t  = 0.0

        # ── 환자 등록부 ───────────────────────────────────────────────
        self.confirmed: dict[int, tuple] = {}
        self.pid_count  = 0
        self.ignored_targets: list[tuple[float, float]] = []

        self.create_timer(0.05, self.control_loop)
        self.create_timer(0.1,  self._publish_turret_arrow)
        self.create_timer(1.0,  self.republish_markers)
        self.get_logger().info('TargetManager v7 시작 — 블라인드코너 지능형 스캔')

    # ── TF2 유틸 ──────────────────────────────────────────────────────

    def _map_frame_robot_pose(self) -> tuple[float, float, float]:
        """map 프레임에서 로봇 자세(x, y, theta). TF 실패 시 odom 폴백."""
        try:
            tf = self._tf_buf.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            theta = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return x, y, theta
        except Exception:
            return self.robot_x, self.robot_y, self.robot_theta

    # ── 구독 콜백 ──────────────────────────────────────────────────────

    def odom_cb(self, msg):
        self.robot_x     = msg.pose.pose.position.x
        self.robot_y     = msg.pose.pose.position.y
        self.robot_theta = euler_from_quaternion(msg.pose.pose.orientation)

    def joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name == 'turret_yaw_joint':  self.turret_yaw   = pos
            if name == 'turret_pitch_joint': self.turret_pitch = pos

    def _apex_aim_cb(self, msg: Point):
        self.apex_aim   = msg
        self.apex_aim_t = self.get_clock().now()

    def joy_cb(self, msg):
        if any(abs(a) > 0.1 for a in msg.axes) or any(b == 1 for b in msg.buttons):
            self.last_manual_t = self.get_clock().now()

    def target_cb(self, msg: TargetDetection):
        """탐지 수신 — msg.capture_turret_yaw는 yolo_pose_node가 frame 도착
        시점에 기록한 값. YOLO 추론 지연(50-200ms) 동안의 포탑 이동을 보정.
        map 프레임 로봇 자세로 위치 추정 (odom 드리프트 보정)."""
        if msg.distance < 0.1:
            return
        rx, ry, rtheta = self._map_frame_robot_pose()
        pixel_angle = ((msg.x - 320.0) / 320.0) * (CAM_FOV_RAD / 2.0)
        cam_hdg     = rtheta + msg.capture_turret_yaw + pixel_angle
        est_gx      = rx + msg.distance * math.cos(cam_hdg)
        est_gy      = ry + msg.distance * math.sin(cam_hdg)
        if self._is_ignored(est_gx, est_gy):
            return
        self.last_msg    = msg
        self.last_seen_t = self.get_clock().now()

    def _blind_corners_cb(self, msg: MarkerArray):
        """/viz/blind_corners에서 도어·오클루전 엣지 좌표 추출."""
        pts: list[tuple[float, float]] = []
        for m in msg.markers:
            if m.action == Marker.DELETEALL:
                continue
            if m.type == Marker.CUBE_LIST and m.ns in ('door_edge', 'occ_edge'):
                for p in m.points:
                    pts.append((p.x, p.y))
        self._blind_corners = pts

    # ── 유틸 ──────────────────────────────────────────────────────────

    def _is_known(self, gx, gy) -> bool:
        for px, py, _, _ in self.confirmed.values():
            if math.hypot(gx - px, gy - py) < MERGE_M:
                return True
        return False

    def _is_ignored(self, gx, gy) -> bool:
        for ix, iy in self.ignored_targets:
            if math.hypot(gx - ix, gy - iy) < IGNORE_R:
                return True
        return False

    def _apply_pitch_slew(self, raw_vel: float) -> float:
        max_delta = PITCH_SLEW_MAX * self._loop_dt
        slewed = max(self._prev_pitch_vel - max_delta,
                     min(self._prev_pitch_vel + max_delta, raw_vel))
        self._prev_pitch_vel = slewed
        return slewed

    def _pitch_to_neutral(self) -> float:
        raw = KP_SRCH * (0.0 - self.turret_pitch)
        return self._apply_pitch_slew(raw)

    def _cmd_turret(self, yaw_vel, pitch_vel, z_flag=0.0):
        self.yaw_pub.publish(Float64(data=float(yaw_vel)))
        self.pitch_pub.publish(Float64(data=float(pitch_vel)))
        cmd = Vector3()
        cmd.x = float(-yaw_vel   * 100)
        cmd.y = float(-pitch_vel * 100)
        cmd.z = float(z_flag)
        self.turret_pub.publish(cmd)

    # ── 스캔 큐 갱신 ──────────────────────────────────────────────────

    def _update_scan_queue(self, now_sec: float):
        """블라인드 코너 → 로봇 기준 각도 → 클러스터링 → 순차 스캔 큐."""
        if now_sec - self._scan_update_t < SCAN_UPDATE_S:
            return
        self._scan_update_t = now_sec

        if not self._blind_corners:
            self._scan_queue = []
            return

        rx, ry, rtheta = self._map_frame_robot_pose()

        # 로봇 기준 yaw 각도 변환 + 범위 필터
        angle_pairs: list[tuple[float, float]] = []  # (distance, angle)
        for cx, cy in self._blind_corners:
            d = math.hypot(cx - rx, cy - ry)
            if d < 0.3:
                continue
            aim = math.atan2(cy - ry, cx - rx)
            yaw = aim - rtheta
            while yaw >  math.pi: yaw -= 2 * math.pi
            while yaw < -math.pi: yaw += 2 * math.pi
            if abs(yaw) <= TURRET_HW_LIMIT:
                angle_pairs.append((d, yaw))

        if not angle_pairs:
            self._scan_queue = []
            return

        # 각도 기준 정렬 후 15° 클러스터링 (인접 코너 중복 제거)
        angle_pairs.sort(key=lambda x: x[1])
        clusters: list[tuple[float, float]] = [angle_pairs[0]]
        for d, a in angle_pairs[1:]:
            if abs(a - clusters[-1][1]) > SCAN_CLUSTER_RAD:
                clusters.append((d, a))

        # 가까운 코너 우선 선택 (최대 N_SCAN_MAX개) → 각도 순 재정렬
        clusters.sort(key=lambda x: x[0])
        selected = sorted([a for _, a in clusters[:N_SCAN_MAX]])

        self._scan_queue = selected
        if self._scan_idx >= len(self._scan_queue):
            self._scan_idx      = 0
            self._scan_arrive_t = None

    # ── 환자 등록 ─────────────────────────────────────────────────────

    def _try_register(self):
        msg = self.last_msg
        if msg is None:
            return
        rx, ry, rtheta = self._map_frame_robot_pose()
        pixel_angle = ((msg.x - 320.0) / 320.0) * (CAM_FOV_RAD / 2.0)
        actual_hdg  = rtheta + msg.capture_turret_yaw + pixel_angle
        gx = rx + msg.distance * math.cos(actual_hdg)
        gy = ry + msg.distance * math.sin(actual_hdg)

        if self._is_known(gx, gy) or self._is_ignored(gx, gy):
            return

        pid  = self.pid_count; self.pid_count += 1
        lv   = msg.triage_level
        lbl  = msg.triage_label
        room = get_room_name(gx, gy)
        self.confirmed[pid]  = (gx, gy, lv, lbl)
        self.ignored_targets.append((gx, gy))
        self.republish_markers()

        log = (f'[구조 로그] #{pid} {lbl} | '
               f'거리:{msg.distance:.1f}m | '
               f'위치:({gx:.1f},{gy:.1f}) | {room}')
        self.get_logger().info(log)
        with open('patient_locations.txt', 'a') as f:
            f.write(log + '\n')

    # ── 마커 빌더 ──────────────────────────────────────────────────────

    def _mk_sphere(self, pid, x, y, lv):
        m = Marker()
        m.header.frame_id = 'map'; m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'patient_sphere'; m.id = pid * 3
        m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = 0.3
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.5
        r, g, b = _TRIAGE_COLORS.get(lv, (1, 1, 1))
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
        m.lifetime.sec = 0
        return m

    def _mk_ring(self, pid, x, y, lv):
        m = Marker()
        m.header.frame_id = 'map'; m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'patient_ring'; m.id = pid * 3 + 1
        m.type = Marker.CYLINDER; m.action = Marker.ADD
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = 0.05
        m.pose.orientation.w = 1.0
        m.scale.x = 0.9; m.scale.y = 0.9; m.scale.z = 0.05
        r, g, b = _TRIAGE_COLORS.get(lv, (1, 1, 1))
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.35
        m.lifetime.sec = 0
        return m

    def _mk_text(self, pid, x, y, lv, label):
        m = Marker()
        m.header.frame_id = 'map'; m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'patient_text'; m.id = pid * 3 + 2
        m.type = Marker.TEXT_VIEW_FACING; m.action = Marker.ADD
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = 1.0
        m.pose.orientation.w = 1.0; m.scale.z = 0.35
        r, g, b = _TRIAGE_COLORS.get(lv, (1, 1, 1))
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
        m.text = f'{label}\n[{get_room_name(x, y)}] (CONFIRMED)'
        m.lifetime.sec = 0
        return m

    def republish_markers(self):
        if not self.confirmed:
            return
        ma = MarkerArray()
        for pid, (x, y, lv, lbl) in self.confirmed.items():
            ma.markers.append(self._mk_sphere(pid, x, y, lv))
            ma.markers.append(self._mk_ring(pid, x, y, lv))
            ma.markers.append(self._mk_text(pid, x, y, lv, lbl))
        self.marker_pub.publish(ma)

    def _publish_turret_arrow(self):
        rx, ry, rtheta = self._map_frame_robot_pose()
        cam_angle = rtheta + self.turret_yaw
        m = Marker()
        m.header.frame_id = 'map'; m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'turret_heading'; m.id = 0
        m.type = Marker.ARROW; m.action = Marker.ADD
        start = Point(); start.x = rx; start.y = ry; start.z = 0.5
        end   = Point()
        end.x = rx + 2.0 * math.cos(cam_angle)
        end.y = ry + 2.0 * math.sin(cam_angle)
        end.z = 0.5
        m.points = [start, end]
        m.scale.x = 0.08; m.scale.y = 0.16; m.scale.z = 0.0
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 0.9
        m.lifetime.sec = 1
        self.arrow_pub.publish(m)

    # ── 메인 제어 루프 (20Hz) ─────────────────────────────────────────

    def control_loop(self):
        now     = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        # 조이스틱 수동 입력 우선 (2초)
        if self.last_manual_t is not None:
            if (now - self.last_manual_t).nanoseconds * 1e-9 < 2.0:
                self._prev_pitch_vel = 0.0
                return

        # ── 탐지 신선도 + 연속 등록 ──────────────────────────────────
        if self.last_seen_t is not None:
            data_age = (now - self.last_seen_t).nanoseconds * 1e-9
        else:
            data_age = float('inf')
        target_fresh = (data_age < MSG_FRESHNESS_S) and (self.last_msg is not None)

        if target_fresh:
            self._detect_streak += 1
            if self._detect_streak == CONFIRM_N:
                self._try_register()
        else:
            self._detect_streak = 0

        # ── 포탑 방향 결정 (우선순위: apex_aim > 블라인드코너 > 사인파) ─

        # apex_aim: 커버리지 네비게이터 우선 조준
        if self.apex_aim_t is not None:
            if (now - self.apex_aim_t).nanoseconds * 1e-9 > 1.0:
                self.apex_aim = None

        if self.apex_aim is not None:
            rx, ry, rtheta = self._map_frame_robot_pose()
            aim_angle  = math.atan2(self.apex_aim.y - ry,
                                    self.apex_aim.x - rx)
            target_yaw = aim_angle - rtheta
            while target_yaw >  math.pi: target_yaw -= 2 * math.pi
            while target_yaw < -math.pi: target_yaw += 2 * math.pi
            target_yaw = max(-TURRET_HW_LIMIT, min(TURRET_HW_LIMIT, target_yaw))

        else:
            # 블라인드코너 스캔 큐 갱신
            self._update_scan_queue(now_sec)

            if self._scan_queue:
                # 현재 목표 각도
                target_yaw = self._scan_queue[self._scan_idx % len(self._scan_queue)]

                # 도달 판정 → 드웰 타이머 → 다음 포인트
                if abs(target_yaw - self.turret_yaw) < SCAN_AT_RAD:
                    if self._scan_arrive_t is None:
                        self._scan_arrive_t = now_sec
                    elif now_sec - self._scan_arrive_t >= SCAN_DWELL_S:
                        self._scan_idx = (self._scan_idx + 1) % len(self._scan_queue)
                        self._scan_arrive_t = None
                else:
                    self._scan_arrive_t = None  # 이동 중 → 도달 타이머 리셋
            else:
                # 블라인드코너 없음 → 사인파 폴백
                target_yaw = SEARCH_AMP * math.sin(now_sec * SEARCH_OMEGA)

        yaw_vel   = KP_SRCH * (target_yaw - self.turret_yaw)
        pitch_vel = self._pitch_to_neutral()
        self._cmd_turret(yaw_vel, pitch_vel, -1.0)


def main(args=None):
    rclpy.init(args=args)
    node = TargetManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
