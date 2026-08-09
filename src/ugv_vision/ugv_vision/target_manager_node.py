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
from collections import deque

import rclpy
import rclpy.time
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, Float64
from visualization_msgs.msg import Marker, MarkerArray

from ugv_msgs.msg import TargetDetection

# ── 튜닝 파라미터 ──────────────────────────────────────────────────────
SEARCH_AMP   = math.radians(50)  # 사인파 폴백 진폭
SEARCH_OMEGA = 0.6               # 사인파 각속도 (rad/s) — 낮출수록 위치 오차 줄어듦
KP_SRCH      = 2.0              # 스캔 P 게인 (rad/s per rad)

TURRET_HW_LIMIT  = math.radians(355)  # Yaw 소프트 한계 (URDF ±360° − 5° 마진)
PITCH_SLEW_MAX   = 2.0                # pitch 최대 변화율 (rad/s²)
CAM_FOV_RAD      = 1.089              # 기본값: 시뮬 카메라 수평 FOV (ugv.urdf.xacro)
                                      # 실기 D435i는 87° → 파라미터 cam_fov_rad로 지정

MERGE_M          = 1.5    # 기발견 환자 중복 판정 기본 반경 (m)
# 위치 오차는 관측 거리에 비례한다(실측: 2.2m 관측 → 오차 0.22m,
# 4.8m 관측 → 오차 1.5m). 반경을 1.5m 로 고정하면 멀리서 잡은 등록과
# 가까이서 잡은 등록이 1.64m 벌어져 같은 사람이 둘로 등록됐다.
# 이건 단순 중복이 아니라, 실종자 수를 채운 것처럼 보이게 해 수색을
# 조기 종료시킬 수 있다.
# 반경은 '두 관측 중 더 부정확한 쪽'(먼 쪽)으로 정한다. 평균을 쓰면
# 역설이 생긴다 — 가까이서 재관측해 좌표를 정밀화할수록 저장된 거리가
# 작아져 반경이 줄고, 나중의 먼 관측을 못 묶어 같은 사람이 새로 등록된다.
# 실측: #3 을 1.3m 로 정밀화한 뒤 같은 사람을 3.7m 에서 다시 보자
#       간격 2.55m > 반경 2.13m 가 되어 8번째 조난자로 등록됐다
#       (월드에는 7명뿐. 표본 산포 0.00m 라 잡음이 아니라 계통 오차다).
# 과다 계수는 별개 조난자를 합쳐 '덜 세는' 쪽이고, 과소 계수는 한 사람을
# 둘로 세는 쪽이다. expected_victims 로 수색 종료를 판단하므로
# 덜 세는 편이 안전하다(더 찾으러 다닌다).
MERGE_PER_M      = 0.35   # 더 먼 관측 거리 1m 당 늘려줄 반경 (m)
IGNORE_R         = 0.5    # 확인 완료 환자 억제 반경 (m)
MSG_FRESHNESS_S  = 1.0    # YOLO 신선도 한계 (s)
CONFIRM_N        = 3      # 연속 탐지 횟수 — 오탐 방지

# ── 정지 조준 확인(INSPECT) 파라미터 ──────────────────────────────────
# 이동 중에 등록하면 로봇 자세·포탑 각도가 계속 변해 위치가 튄다.
# → 후보를 보면 즉시 정지 요청하고, 멈춘 뒤 포탑을 조준해서 등록한다.
INSPECT_TRIGGER_N   = 3                   # 정지 요청까지 필요한 연속 '탐지 메시지' 수
# 후보를 믿기 전에 '얼마나 오래·얼마나 안정적으로' 보였는지를 함께 본다.
#
# 유령 후보(겨눴는데 대상 없음)에 탐사 시간의 상당 부분을 쓴다.
# 실측(큰 월드 57분, 예산 240초): 유령 17건 / 정상 등록 7건.
# 접근·정지·조준에 건당 최대 28초라 20분 가까이 낭비된다.
#
# 거리로는 못 거른다 — 유령 중앙 3.0m, 진짜 중앙 3.3m 로 겹친다.
# 차이는 '지속성' 이다. 탐지 메시지 3건이면 14Hz 기준 0.2초에 불과해,
# 잔해에 잠깐 뜬 골격도 그대로 통과한다.
# 그래서 최소 지속 시간과 좌표 안정성을 함께 요구한다. 진짜 사람은
# 접근하는 동안 계속 보이고 좌표도 한 자리에 머문다.
INSPECT_MIN_PERSIST_S = 1.0               # 이 시간 이상 보여야 정지한다
INSPECT_CONSIST_R     = 0.60              # 최근 추정 좌표가 이 반경 안에 모여야 (m)
INSPECT_TRACK_N       = 12                # 안정성 판단에 쓰는 최근 표본 수
# 탐지가 끊겨도 이 시간까지는 같은 후보로 이어 본다.
# 포탑이 ±50도로 스윕하므로 사람은 시야에 들락날락한다. 끊길 때마다
# 기록을 지우면 '연속 1초'를 채우는 것이 구조적으로 불가능해져 진짜
# 조난자까지 전부 기각된다(실측: 기각 67건 중 66건이 '시간 부족',
# 조난자 7/7 → 2/7 로 악화). 끊김을 허용하되, 다른 대상과 섞이는 것은
# 좌표 안정성(INSPECT_CONSIST_R)이 막는다.
INSPECT_TRACK_GAP_S   = 3.0
INSPECT_SETTLE_SPD  = 0.05                # 정지 판정 속도 (m/s)
INSPECT_YAW_TOL     = math.radians(4.0)   # 포탑 조준 완료 판정 오차
INSPECT_SAMPLES     = 5                   # 정지·조준 상태에서 모을 표본 수
# 타임아웃은 2단계로 나눈다. 포탑이 반대편(≈180°)을 봐야 하면 슬루에만 3초 가까이
# 걸리므로 전체 한도는 넉넉히 주되, 일단 멈춰서 겨눈 뒤에도 대상이 안 보이면
# 유령 후보로 보고 곧바로 포기해 순찰 시간을 낭비하지 않는다.
INSPECT_TIMEOUT_S   = 28.0                # 전체 한도 (대상까지 접근 + 정지 + 포탑 슬루)
INSPECT_AFTER_AIM_S = 2.5                 # 조준 완료 후 대상이 안 보일 때 포기까지
MAX_SAMPLE_SPREAD   = 0.30                # 표본이 이보다 흩어지면 등록 보류 (m)
# 등록을 허용하는 최대 관측 거리.
# 먼 거리 관측이 등록으로 이어지면 안 된다. 실제로 6.8~14.8m 에서 잡힌
# 엉터리 투영 4건이 별개 조난자로 등록됐다(모두 같은 사람을 멀리서 잘못
# 투영한 것). 기각된 사례는 전부 7.0~8.0m 구간이었다.
# 값은 조사 거리(patrol_navigator inspect_standoff)에 맞춰야 한다.
# standoff 1.5m 시절엔 4.0 이었으나, 서 있는 사람의 전신이 화면에 들어오도록
# standoff 를 3.0m 로 늘리면서 실제 관측 거리가 4.0m 까지 올라왔다
# (상한과 같아 아슬아슬했다). 접근 오차까지 감안해 5.5m 로 둔다.
MAX_REGISTER_DIST   = 5.5                 # m

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
        # 정지 조준 확인 핸드셰이크 — patrol_navigator 가 소비
        self.inspect_req_pub  = self.create_publisher(PointStamped, '/inspect_request', 10)
        self.inspect_done_pub = self.create_publisher(Bool,         '/inspect_done',    10)

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
        self._last_streak_stamp = 0   # 같은 탐지 메시지를 중복으로 세지 않기 위함
        # 후보 추적: (시각, x, y). 지속 시간·좌표 안정성 판단용.
        self._cand_track: deque = deque(maxlen=INSPECT_TRACK_N)

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

        # 카메라 수평 FOV — 시뮬 1.089rad(62°) / 실기 D435i 87°
        self.declare_parameter('cam_fov_rad', CAM_FOV_RAD)
        self.cam_fov = float(self.get_parameter('cam_fov_rad').value)

        # ── 정지 조준 확인(INSPECT) 상태 ──────────────────────────────
        self.robot_speed          = 0.0
        self._inspect_active      = False
        self._inspect_start_t     = 0.0
        self._inspect_aim: tuple[float, float] | None = None   # 조준 목표 (map)
        self._inspect_samples: list[tuple[float, float]] = []
        self._inspect_settled_t: float | None = None   # 정지+조준이 붙은 시각

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
        v = msg.twist.twist
        self.robot_speed = math.hypot(v.linear.x, v.linear.y) + abs(v.angular.z) * 0.2

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
        est_gx, est_gy = self._estimate_xy(msg)
        if self._is_ignored(est_gx, est_gy):
            return
        self.last_msg    = msg
        self.last_seen_t = self.get_clock().now()

    def _estimate_xy(self, msg: TargetDetection) -> tuple[float, float]:
        """탐지 메시지 → map 프레임 추정 좌표.

        픽셀 x는 오른쪽으로 증가하지만 ROS 방위각(yaw)은 반시계(왼쪽)가 +다.
        따라서 화면 오른쪽(x>중앙)은 방위각이 **감소**해야 한다 → 부호 반전.
        (부호가 +였을 때 추정 위치가 2~3m 어긋났음. 실측으로 확인)
        """
        rx, ry, rtheta = self._map_frame_robot_pose()
        pixel_angle = -((msg.x - 320.0) / 320.0) * (self.cam_fov / 2.0)
        cam_hdg     = rtheta + msg.capture_turret_yaw + pixel_angle
        return (rx + msg.distance * math.cos(cam_hdg),
                ry + msg.distance * math.sin(cam_hdg))

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

    def _match_known(self, gx, gy, dist):
        """같은 사람으로 볼 기존 등록의 pid. 없으면 None.

        중복 반경은 두 관측의 거리에 비례해 넓힌다. 멀리서 잡은 등록일수록
        참값에서 벗어나 있으므로 고정 반경으로는 같은 사람을 못 묶는다.
        """
        for pid, (px, py, _, _, pdist) in self.confirmed.items():
            r = MERGE_M + MERGE_PER_M * max(pdist, dist)
            if math.hypot(gx - px, gy - py) < r:
                return pid
        return None

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

    # ── 정지 조준 확인(INSPECT) ────────────────────────────────────────

    def _candidate_stable(self):
        """정지·접근할 만한 후보인지. (통과여부, 사유).

        유령 후보 한 건에 접근·정지·조준으로 최대 28초를 쓴다. 실측에서
        유령 17건 / 진짜 7건이라 탐사 시간의 상당 부분이 여기로 샜다.
        거리로는 못 거른다(유령 중앙 3.0m vs 진짜 3.3m 로 겹침).
        갈리는 것은 '얼마나 오래, 얼마나 한 자리에서' 보였는가다.
        """
        if len(self._cand_track) < INSPECT_TRIGGER_N:
            return False, '표본부족'
        span = self._cand_track[-1][0] - self._cand_track[0][0]
        if span < INSPECT_MIN_PERSIST_S:
            return False, f'{span:.1f}s만 보임'
        xs = sorted(p[1] for p in self._cand_track)
        ys = sorted(p[2] for p in self._cand_track)
        mx, my = xs[len(xs) // 2], ys[len(ys) // 2]
        spread = max(math.hypot(x - mx, y - my)
                     for _, x, y in self._cand_track)
        if spread > INSPECT_CONSIST_R:
            return False, f'좌표 산포 {spread:.1f}m'
        return True, ''

    def _inspect_step(self, now_sec: float, target_fresh: bool):
        """후보 발견 → 정지 요청 → 조준 완료 후 표본 수집 → 등록.

        이동 중 등록하면 로봇 자세와 포탑 각도가 계속 바뀌어 좌표가 튄다.
        멈추고 조준한 상태에서만 표본을 모아 중앙값으로 등록한다.
        """
        # 1) 시작 조건 — 후보가 연속으로 보이고, 아직 확인 중이 아닐 때
        if (not self._inspect_active and target_fresh
                and self._detect_streak >= INSPECT_TRIGGER_N
                and self.last_msg is not None):
            ok, why = self._candidate_stable()
            if not ok:
                self.get_logger().info(
                    f'후보 불안정({why}) — 정지하지 않고 순찰 계속',
                    throttle_duration_sec=10.0)
                return
            gx, gy = self._estimate_xy(self.last_msg)
            # 이미 아는 사람이어도, 훨씬 가까이서 다시 보게 됐다면 확인을
            # 진행해 좌표를 정밀화한다(멀리서 먼저 잡힌 등록을 고쳐 쓴다).
            known = self._match_known(gx, gy, self.last_msg.distance)
            if known is not None:
                if self.last_msg.distance >= self.confirmed[known][4] - 0.3:
                    return
            elif self._is_ignored(gx, gy):
                return
            self._inspect_active     = True
            self._inspect_start_t    = now_sec
            self._inspect_aim        = (gx, gy)
            self._inspect_samples    = []
            self._inspect_settled_t  = None
            self._publish_inspect_request(gx, gy)
            self.get_logger().info(
                f'후보 발견 ({gx:.1f},{gy:.1f}) — 정지 요청 후 조준 확인 시작')
            return

        if not self._inspect_active:
            return

        # 2) 타임아웃 — 대상을 놓쳤거나 조준이 안 잡히는 경우
        if now_sec - self._inspect_start_t > INSPECT_TIMEOUT_S:
            self.get_logger().warn(
                f'조준 확인 시간초과({INSPECT_TIMEOUT_S:.0f}s) — 표본 '
                f'{len(self._inspect_samples)}개, 순찰 재개 '
                f'[속도={self.robot_speed:.3f}m/s(한계{INSPECT_SETTLE_SPD}), '
                f'조준오차={math.degrees(abs(self._aim_yaw_error())):.1f}°'
                f'(한계{math.degrees(INSPECT_YAW_TOL):.0f}°), '
                f'탐지신선={self._detect_streak > 0}]')
            self._finish_inspect(registered=False)
            return

        # 접근하는 동안에도 조준 목표를 최신 탐지로 계속 갱신한다.
        # 최초 추정만 붙들면, 로봇이 대상 앞으로 이동한 뒤 포탑이 엉뚱한 곳을
        # 겨눠 "겨눴는데 대상 없음(유령 후보)" 으로 버려진다.
        # (실측: 후보 7건 중 5건이 이 경로로 폐기)
        # 포탑은 이 노드가 직접 제어하므로 목표를 바꿔도 충돌하지 않는다.
        if target_fresh and self.last_msg is not None:
            self._inspect_aim = self._estimate_xy(self.last_msg)

        # 3) 정지 + 조준이 붙었는지 확인 (대상이 안 보여도 정착 여부는 판정)
        settled = (self.robot_speed <= INSPECT_SETTLE_SPD
                   and abs(self._aim_yaw_error()) <= INSPECT_YAW_TOL)
        if settled and self._inspect_settled_t is None:
            self._inspect_settled_t = now_sec

        # 겨눴는데도 대상이 안 보이면 유령 후보 → 조기 포기
        if (self._inspect_settled_t is not None and not target_fresh
                and now_sec - self._inspect_settled_t > INSPECT_AFTER_AIM_S):
            self.get_logger().info(
                '조준 완료했는데 대상 없음 — 유령 후보로 판단, 순찰 재개')
            self._finish_inspect(registered=False)
            return

        if not settled or not target_fresh or self.last_msg is None:
            return

        gx, gy = self._estimate_xy(self.last_msg)
        self._inspect_samples.append((gx, gy))
        # 조준점도 최신 추정으로 갱신 (초기 추정이 부정확했을 수 있음)
        self._inspect_aim = (gx, gy)

        if len(self._inspect_samples) >= INSPECT_SAMPLES:
            self._register_from_samples()
            self._finish_inspect(registered=True)

    def _aim_yaw_error(self) -> float:
        """현재 포탑 yaw 와 조준 목표 yaw 의 차이(rad)."""
        if self._inspect_aim is None:
            return math.inf
        rx, ry, rtheta = self._map_frame_robot_pose()
        aim_angle = math.atan2(self._inspect_aim[1] - ry,
                               self._inspect_aim[0] - rx)
        err = (aim_angle - rtheta) - self.turret_yaw
        while err >  math.pi: err -= 2 * math.pi
        while err < -math.pi: err += 2 * math.pi
        return err

    def _publish_inspect_request(self, gx, gy):
        p = PointStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.point.x, p.point.y, p.point.z = float(gx), float(gy), 0.5
        self.inspect_req_pub.publish(p)

    def _finish_inspect(self, registered: bool):
        self._inspect_active    = False
        self._inspect_aim       = None
        self._inspect_samples   = []
        self._inspect_settled_t = None
        self._detect_streak     = 0
        msg = Bool(); msg.data = bool(registered)
        self.inspect_done_pub.publish(msg)

    def _register_from_samples(self):
        """정지 상태에서 모은 표본의 중앙값으로 등록 — 단발 프레임보다 안정적."""
        xs = sorted(s[0] for s in self._inspect_samples)
        ys = sorted(s[1] for s in self._inspect_samples)
        mid = len(xs) // 2
        gx, gy = xs[mid], ys[mid]
        spread = max(math.hypot(x - gx, y - gy)
                     for x, y in self._inspect_samples)
        # 정지·조준 상태의 표본이 흩어져 있으면 관측 자체가 불안정한 것이다.
        # 실측: 실재하지 않는 조난자가 산포 0.58m 로 등록됐다(정상은 0.00~0.05m).
        if spread > MAX_SAMPLE_SPREAD:
            self.get_logger().info(
                f'표본 산포 {spread:.2f}m > {MAX_SAMPLE_SPREAD}m — 관측 불안정으로 등록 보류')
            return
        self._try_register(gx, gy, spread)

    def _try_register(self, gx=None, gy=None, spread=None):
        msg = self.last_msg
        if msg is None:
            return
        # 먼 거리 관측은 등록하지 않는다. 각도 오차가 거리에 비례해 커지고
        # depth 도 사거리(8m) 밖이면 폴백 추정이라 좌표가 크게 튄다.
        if msg.distance > MAX_REGISTER_DIST:
            self.get_logger().info(
                f'관측 거리 {msg.distance:.1f}m > {MAX_REGISTER_DIST}m — '
                '등록 보류 (가까이 접근한 뒤 다시 확인)',
                throttle_duration_sec=5.0)
            return
        if gx is None or gy is None:
            gx, gy = self._estimate_xy(msg)

        known = self._match_known(gx, gy, msg.distance)
        if known is not None:
            # 이미 등록된 사람이다. 다만 이번이 더 가까운 관측이면 위치가
            # 더 정확하므로 갱신한다(멀리서 먼저 잡힌 좌표를 고쳐 쓴다).
            px, py, plv, plbl, pdist = self.confirmed[known]
            if msg.distance < pdist - 0.3:
                self.confirmed[known] = (gx, gy, plv, plbl, msg.distance)
                self.republish_markers()
                self.get_logger().info(
                    f'#{known} 위치 갱신 ({px:.1f},{py:.1f}) → ({gx:.1f},{gy:.1f}) '
                    f'— 더 가까이서 재관측 {pdist:.1f}m → {msg.distance:.1f}m')
            return
        if self._is_ignored(gx, gy):
            return

        pid  = self.pid_count; self.pid_count += 1
        lv   = msg.triage_level
        lbl  = msg.triage_label
        room = get_room_name(gx, gy)
        self.confirmed[pid]  = (gx, gy, lv, lbl, msg.distance)
        self.ignored_targets.append((gx, gy))
        self.republish_markers()

        prec = f' | 표본{len(self._inspect_samples) or INSPECT_SAMPLES}개 산포{spread:.2f}m' \
               if spread is not None else ''
        log = (f'[구조 로그] #{pid} {lbl} | '
               f'거리:{msg.distance:.1f}m | '
               f'위치:({gx:.1f},{gy:.1f}) | {room}{prec}')
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
        for pid, (x, y, lv, lbl, _) in self.confirmed.items():
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

        # streak 은 '새 탐지 메시지' 수여야 한다. 제어루프(20Hz) 틱을 세면
        # 탐지 1건만으로도 1초 안에 20까지 올라가 유령 후보에 멈춰 선다.
        if target_fresh:
            stamp = self.last_seen_t.nanoseconds if self.last_seen_t else 0
            if stamp != self._last_streak_stamp:
                self._last_streak_stamp = stamp
                self._detect_streak += 1
                # 지속 시간·좌표 안정성 판단용으로 추정 좌표를 남긴다
                if not self._inspect_active:
                    gx, gy = self._estimate_xy(self.last_msg)
                    self._cand_track.append((now_sec, gx, gy))
        else:
            self._detect_streak = 0
            self._last_streak_stamp = 0
        # 탐지가 끊겨도 곧바로 지우지 않는다(포탑 스윕으로 시야를 들락날락).
        # 오래 안 보이면 그때 버린다.
        if (self._cand_track
                and now_sec - self._cand_track[-1][0] > INSPECT_TRACK_GAP_S):
            self._cand_track.clear()

        self._inspect_step(now_sec, target_fresh)

        # ── 포탑 방향 결정 (우선순위: apex_aim > 블라인드코너 > 사인파) ─

        # apex_aim: 커버리지 네비게이터 우선 조준
        if self.apex_aim_t is not None:
            if (now - self.apex_aim_t).nanoseconds * 1e-9 > 1.0:
                self.apex_aim = None

        if self._inspect_active and self._inspect_aim is not None:
            # 정지 조준 확인 중 — 스캔·apex보다 최우선으로 대상에 고정
            rx, ry, rtheta = self._map_frame_robot_pose()
            aim_angle  = math.atan2(self._inspect_aim[1] - ry,
                                    self._inspect_aim[0] - rx)
            target_yaw = aim_angle - rtheta
            while target_yaw >  math.pi: target_yaw -= 2 * math.pi
            while target_yaw < -math.pi: target_yaw += 2 * math.pi
            target_yaw = max(-TURRET_HW_LIMIT, min(TURRET_HW_LIMIT, target_yaw))

        elif self.apex_aim is not None:
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
