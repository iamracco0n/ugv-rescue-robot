#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fire_detection_node.py
======================
열화상(Thermal) 카메라로 화재를 '발견'해서 지도에 칠하고, nav 장애물로 등록하는 노드.

  구 fog(visibility_overlay)의 '지우는' 방식은 실시간 동기화 박자 문제가 있었지만,
  이 노드는 '칠하는(누적)' 방식이라 한 번 감지하면 영구히 쌓이면 됨 → 박자 자유.

동작
----
  1) /thermal/image_raw (16bit mono) 에서 고온 픽셀(blob) 검출
  2) 같은 픽셀 위치의 depth로 거리 추정 (열화상·depth 동일 링크/해상도/FOV라 1:1 정렬)
  3) heading = robot_theta + turret_yaw - pixel_offset  로 월드 좌표 투영
     (픽셀 x는 오른쪽 +, 방위각은 왼쪽 + 이므로 부호 반전)
        (target_manager_node 의 환자 투영과 동일한 규약)
  4) 근처 기존 화재와 병합, 신규면 /fire_alert 발행 (경비 순찰 노드가 반응)
  5) 누적 열장(heatmap)·costmap 장애물 구름·마커를 주기 발행

발행 토픽
  /fire_heatmap  nav_msgs/OccupancyGrid       화재 열장 (RViz, fog 자리 대체)
  /fire_cloud    sensor_msgs/PointCloud2       화재 구역 → Nav2 costmap 마킹 장애물
  /fire_markers  visualization_msgs/MarkerArray 불꽃 구 + 라벨
  /fire_alert    geometry_msgs/PointStamped     신규 화재 발견 이벤트
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.duration import Duration
from rclpy.time import Time

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool, Header

import tf2_ros

# ── 열장 격자 (visibility_overlay 와 동일 규약 → RViz 겹침 정합) ──────────
_RES    = 0.2
_SIZE   = 40.0
_N      = int(_SIZE / _RES)      # 200
_ORIGIN = -_SIZE / 2.0           # -20.0 m

_LATCH_QOS = QoSProfile(depth=1)
_LATCH_QOS.durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL
_LATCH_QOS.reliability = QoSReliabilityPolicy.RELIABLE


def _yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class FireDetectionNode(Node):

    def __init__(self):
        super().__init__('fire_detection_node')

        # ── 파라미터 ─────────────────────────────────────────────────
        self.declare_parameter('thermal_topic', '/thermal/image_raw')
        self.declare_parameter('depth_topic',
                               '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('rgb_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('fire_temp_threshold_k', 380.0)   # 이 온도[K] 이상이면 화재
        self.declare_parameter('linear_resolution', 0.01)        # 16bit 픽셀 1 = 0.01K
        self.declare_parameter('min_blob_area', 25)              # 최소 blob 픽셀 수
        self.declare_parameter('cam_fov_rad', 1.089)
        self.declare_parameter('img_width', 640)
        self.declare_parameter('img_height', 480)
        # 열화상은 12m 까지 보지만 RGBD depth 의 far clip 은 8m 다(ugv.urdf.xacro).
        # 그래서 8m 너머 열원은 depth 가 far clip 으로 잘려 8m 지점에 투영되고,
        # 화재가 실제보다 앞쪽(벽·허공)에 저장됐다. depth 로 실제 잰 범위 안에서만
        # 등록한다. 더 가까이 가면 순찰 중에 정확히 다시 잡는다.
        self.declare_parameter('max_fire_range', 7.5)            # m, 이보다 멀면 무시
        self.declare_parameter('min_fire_range', 0.3)
        self.declare_parameter('depth_max_valid', 7.8)           # 이상은 far clip 포화로 간주
        # blob 픽셀 중 depth 가 실제로 측정된 비율이 이보다 낮으면 신뢰 안 함
        self.declare_parameter('blob_depth_ratio', 0.5)
        self.declare_parameter('bloom_radius', 1.4)              # m, 열장이 번지는 반경
        self.declare_parameter('obstacle_radius', 1.3)          # m, nav 회피 반경
        # 열원 자체가 1m 안팎 크기라 각도·면에 따라 추정이 2m 넘게 흩어진다.
        # 1.2m 로는 화재 하나가 3~4건으로 중복 등록됐다.
        self.declare_parameter('merge_dist', 2.0)               # m, 이 안이면 같은 화재
        self.declare_parameter('confirm_hits', 2)               # 몇 번 봐야 확정
        # ── 정지 조준 확인 ────────────────────────────────────────────
        # 이동·포탑 스윕 중에 등록하면 각도 스냅샷과 실제 렌더 시각이 어긋나
        # 위치가 크게 튄다(실측: 실제 화재 2곳인데 5m 이상 벗어난 항목 2건 발생).
        # 조난자와 같은 방식으로, 멈춰서 겨눈 상태에서만 등록한다.
        self.declare_parameter('inspect_enabled',   True)
        self.declare_parameter('inspect_samples',   5)
        self.declare_parameter('inspect_settle_spd', 0.05)      # 정지 판정(m/s)
        # 조준 완료 판정. 열원은 사람보다 크고(약 1m) 화면상 blob 중심이
        # 프레임마다 흔들려 4도로는 잘 안 붙는다. 3m 거리에서 8도면
        # 횡방향 0.42m — 열원 크기 안쪽이라 정확도 손해가 크지 않다.
        self.declare_parameter('inspect_yaw_tol',   0.14)       # rad, ≈8°
        self.declare_parameter('inspect_timeout_s', 28.0)
        self.declare_parameter('inspect_after_aim_s', 2.5)      # 겨눴는데 안 보이면 포기
        self.declare_parameter('inspect_retry_s', 120.0)        # 확인 실패한 열원 재시도 유예

        g = self.get_parameter
        self.thermal_topic = g('thermal_topic').value
        self.depth_topic   = g('depth_topic').value
        self.rgb_topic     = g('rgb_topic').value
        self.temp_thresh_k = g('fire_temp_threshold_k').value
        self.lin_res       = g('linear_resolution').value
        self.min_blob_area = int(g('min_blob_area').value)
        self.fov           = g('cam_fov_rad').value
        self.iw            = int(g('img_width').value)
        self.ih            = int(g('img_height').value)
        self.max_range     = g('max_fire_range').value
        self.min_range     = g('min_fire_range').value
        self.depth_max_valid  = float(g('depth_max_valid').value)
        self.blob_depth_ratio = float(g('blob_depth_ratio').value)
        self.bloom_r       = g('bloom_radius').value
        self.obst_r        = g('obstacle_radius').value
        self.merge_d       = g('merge_dist').value
        self.confirm_hits  = int(g('confirm_hits').value)
        self.insp_enabled   = bool(g('inspect_enabled').value)
        self.insp_samples   = int(g('inspect_samples').value)
        self.insp_settle    = float(g('inspect_settle_spd').value)
        self.insp_yaw_tol   = float(g('inspect_yaw_tol').value)
        self.insp_timeout   = float(g('inspect_timeout_s').value)
        self.insp_after_aim = float(g('inspect_after_aim_s').value)
        self.insp_retry_s   = float(g('inspect_retry_s').value)

        # 원본 픽셀 임계값 (16bit: K/lin_res)
        self.raw_thresh = self.temp_thresh_k / self.lin_res

        # ── 상태 ─────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_fire_boxes = []   # [(x1,y1,x2,y2,peak_k), ...] (iw×ih 픽셀)
        self._fire_box_t = 0.0        # 박스 갱신 시각(sec)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.turret_yaw = 0.0
        self.robot_speed = 0.0

        # 정지 조준 확인(INSPECT) 상태
        self._ci_active   = False                 # 확인 진행 중
        self._ci_aim      = None                  # 조준 목표 (map x,y)
        self._ci_peak     = 0.0
        self._ci_start    = 0.0
        self._ci_settled  = None                  # 정지+조준이 붙은 시각
        self._ci_samples  = []
        self._ci_seen_t   = 0.0                   # 마지막으로 열원이 보인 시각
        # 확인에 실패한 열원 — 바로 다시 시도하면 정지·재개를 무한 반복한다
        self._ci_failed: list[tuple] = []         # [(x, y, 실패시각)]
        # 화재 소스: {'x','y','peak_k','hits','confirmed'}
        self.fires: list[dict] = []
        # 누적 열장 (0..100, 0=미검출)
        self.heat = np.zeros((_N, _N), dtype=np.float32)

        # ── TF (map → base_footprint), odom 폴백 ─────────────────────
        self._tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=10))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        # ── 구독 ─────────────────────────────────────────────────────
        from sensor_msgs.msg import JointState
        self.create_subscription(Image,      self.thermal_topic, self.thermal_cb, 5)
        self.create_subscription(Image,      self.depth_topic,   self.depth_cb,   5)
        self.create_subscription(Image,      self.rgb_topic,     self.rgb_cb,     5)
        self.create_subscription(Odometry,   '/odom',            self.odom_cb,    10)
        # SLAM 맵 — 화재 추정 위치가 벽 안쪽으로 찍히는 것을 걸러내는 데 사용
        self.create_subscription(OccupancyGrid, '/map',           self.slam_map_cb, 1)
        self.create_subscription(JointState, '/joint_states',    self.joint_cb,   10)

        # ── 발행 ─────────────────────────────────────────────────────
        self.pub_heat   = self.create_publisher(OccupancyGrid, '/fire_heatmap', _LATCH_QOS)
        self.pub_cloud  = self.create_publisher(PointCloud2,   '/fire_cloud',   10)
        self.pub_marker = self.create_publisher(MarkerArray,   '/fire_markers', 10)
        self.pub_alert  = self.create_publisher(PointStamped,  '/fire_alert',   10)
        # 정지 조준 확인 핸드셰이크 — patrol_navigator 가 소비
        self.pub_ci_req  = self.create_publisher(PointStamped, '/fire_candidate',    10)
        self.pub_ci_done = self.create_publisher(Bool,         '/fire_inspect_done', 10)
        # 불 박스 오버레이 이미지 → rqt_image_view / RViz Image
        self.pub_img    = self.create_publisher(Image,         '/fire/image_annotated', 5)

        self.create_timer(0.5, self.publish_all)   # 2 Hz
        self.create_timer(0.5, self._ci_timer)     # 조준 확인 타임아웃 감시

        self.get_logger().info(
            f'fire_detection_node 시작 — thermal={self.thermal_topic}, '
            f'임계 {self.temp_thresh_k:.0f}K (raw>{self.raw_thresh:.0f})')

    # ── 콜백 ─────────────────────────────────────────────────────────
    def depth_cb(self, msg: Image):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'depth 디코드 실패: {e}', throttle_duration_sec=5.0)

    def rgb_cb(self, msg: Image):
        """RGB 영상에 열화상으로 잡은 불 박스를 그려서 발행."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        if frame.shape[1] != self.iw or frame.shape[0] != self.ih:
            frame = cv2.resize(frame, (self.iw, self.ih))
        now_s = self.get_clock().now().nanoseconds * 1e-9
        boxes = self.latest_fire_boxes if (now_s - self._fire_box_t) < 1.0 else []
        for (x1, y1, x2, y2, pk) in boxes:
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 3)
            cv2.putText(frame, f'FIRE {pk:.0f}K', (int(x1), max(0, int(y1) - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        n = len(self.fires)
        cv2.putText(frame, f'FIRES: {n}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 140, 255), 2)
        try:
            out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            out.header = msg.header
            self.pub_img.publish(out)
        except Exception:
            pass

    def odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_theta = _yaw_from_quat(msg.pose.pose.orientation)
        v = msg.twist.twist
        self.robot_speed = math.hypot(v.linear.x, v.linear.y) + abs(v.angular.z) * 0.2

    def joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name == 'turret_yaw_joint':
                self.turret_yaw = pos
                break

    def _robot_pose(self):
        """map 프레임 로봇 자세 (TF), 실패 시 odom 값."""
        try:
            tf = self._tf_buf.lookup_transform('map', 'base_footprint', Time())
            t = tf.transform.translation
            return t.x, t.y, _yaw_from_quat(tf.transform.rotation)
        except Exception:
            return self.robot_x, self.robot_y, self.robot_theta

    # ── 화재 검출 핵심 ────────────────────────────────────────────────
    def thermal_cb(self, msg: Image):
        # 프레임 도착 시점의 포탑 각도를 먼저 확보한다.
        # 투영 직전에 self.turret_yaw 를 읽으면, 포탑이 스윕 중일 때
        # (최대 1.15 rad/s) 처리 지연만큼 각도가 어긋나 화재 위치가 크게 틀어진다.
        # yolo_pose_node 가 capture_turret_yaw 로 잡는 것과 같은 문제.
        capture_yaw = self.turret_yaw

        try:
            therm = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'thermal 디코드 실패: {e}', throttle_duration_sec=5.0)
            return
        if therm is None or therm.size == 0:
            return

        # 8bit로 떨어지는 환경 대비: 8bit면 min/max 매핑 상 상위값 기준
        if therm.dtype == np.uint8:
            thresh = 180  # 8bit(0..255)에서 매우 뜨거운 픽셀
        else:
            thresh = self.raw_thresh

        now_s = self.get_clock().now().nanoseconds * 1e-9
        hot = (therm.astype(np.float32) >= thresh).astype(np.uint8)
        if hot.sum() < self.min_blob_area:
            self.latest_fire_boxes = []       # 불 안 보이면 박스 지움
            self._fire_box_t = now_s
            return

        # 해상도 정규화 (혹시 다른 크기로 들어와도)
        if hot.shape[1] != self.iw or hot.shape[0] != self.ih:
            hot = cv2.resize(hot, (self.iw, self.ih), interpolation=cv2.INTER_NEAREST)
            therm = cv2.resize(therm, (self.iw, self.ih), interpolation=cv2.INTER_NEAREST)

        contours, _ = cv2.findContours(hot, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rx, ry, rtheta = self._robot_pose()

        # 오버레이용 박스 수집 (RGB에 그릴 용도, 등록 로직과 별개)
        boxes = []
        for c in contours:
            if cv2.contourArea(c) < self.min_blob_area:
                continue
            bx, by, bw, bh = cv2.boundingRect(c)
            pk = float(therm[by:by + bh, bx:bx + bw].max())
            pk = pk * self.lin_res if therm.dtype != np.uint8 else pk
            boxes.append((bx, by, bx + bw, by + bh, pk))
        self.latest_fire_boxes = boxes
        self._fire_box_t = now_s

        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_blob_area:
                continue
            mask = np.zeros(hot.shape, np.uint8)
            cv2.drawContours(mask, [c], -1, 255, -1)
            ys, xs = np.nonzero(mask)
            if xs.size == 0:
                continue

            # 기준 픽셀 = blob 안에서 가장 뜨거운 점.
            # blob 중심을 쓰면, 문틈으로 볼 때 중심이 앞쪽 벽에 걸쳐
            # 벽까지의 depth 로 투영돼 화재가 '벽 속'에 저장된다.
            vals = therm[ys, xs]
            k = int(np.argmax(vals))
            cx, cy = float(xs[k]), float(ys[k])

            peak_raw = float(vals[k])
            peak_k = peak_raw * self.lin_res if therm.dtype != np.uint8 else peak_raw

            dist = self._depth_of_blob(xs, ys)
            if dist is None or not (self.min_range < dist < self.max_range):
                continue

            # 픽셀 x는 오른쪽으로 증가, ROS 방위각은 반시계(왼쪽)가 + → 부호 반전
            pixel_angle = -((cx - self.iw / 2.0) / (self.iw / 2.0)) * (self.fov / 2.0)
            hdg = rtheta + capture_yaw + pixel_angle
            fx = rx + dist * math.cos(hdg)
            fy = ry + dist * math.sin(hdg)

            if self._on_wall(fx, fy):
                self.get_logger().info(
                    f'화재 추정 ({fx:.1f},{fy:.1f}) 이 벽 위 — 가림 판단, 무시',
                    throttle_duration_sec=5.0)
                continue

            self._observe_fire(fx, fy, peak_k, now_s)

    def slam_map_cb(self, msg: OccupancyGrid):
        self._slam_map = msg

    def _cell(self, g, x, y):
        ix = int((x - g.info.origin.position.x) / g.info.resolution)
        iy = int((y - g.info.origin.position.y) / g.info.resolution)
        if not (0 <= ix < g.info.width and 0 <= iy < g.info.height):
            return None
        return g.data[iy * g.info.width + ix]

    def _on_wall(self, x, y) -> bool:
        """추정 위치가 '벽 속 깊숙이' 인지 판정.

        주의: 단순히 점유 셀이라고 버리면 안 된다. 화재는 대개 방 구석에 있어
        추정 위치가 벽에 인접하고, 맵 두께·해상도 때문에 점유 셀로 찍히기
        쉽다. 실제로 GT (-12,-8) 화재의 정확한 추정 (-11.6,-7.6) 이
        '벽 위'로 오판돼 버려졌다.

        그래서 사방이 모두 막힌 경우 — 즉 방 안쪽으로도 트인 데가 없는
        진짜 벽 내부 — 만 배제한다. 벽에 붙은 화재는 통과시킨다.
        """
        g = getattr(self, '_slam_map', None)
        if g is None:
            return False                      # 맵 없으면 판단 보류(통과)
        v = self._cell(g, x, y)
        if v is None or v < 65:
            return False                      # 점유 자체가 아니면 통과
        r = 0.35
        for dx, dy in ((r, 0), (-r, 0), (0, r), (0, -r),
                       (r, r), (r, -r), (-r, r), (-r, -r)):
            n = self._cell(g, x + dx, y + dy)
            if n is not None and 0 <= n < 25:
                return False                  # 한쪽이라도 트여 있으면 벽 표면
        return True                           # 사방이 막힘 = 벽 내부

    def _depth_of_blob(self, xs, ys):
        """열원 blob **전체 픽셀**의 depth 로 거리 추정. 없으면 None.

        단일 픽셀만 보면 가림 경계에서 1픽셀만 어긋나도 앞쪽 문틀·벽의
        거리를 집어온다. blob 안에서 실제로 측정된 픽셀이 충분히 많을 때만
        신뢰하고, 그 중앙값을 쓴다.

        열원이 depth 사거리(8m) 밖이면 blob 픽셀 대부분이 무효가 되므로
        여기서 자연스럽게 걸러진다 — 멀리 있는 불이 앞쪽 벽에 찍히던 원인.
        """
        d = self.latest_depth
        if d is None or xs.size == 0:
            return None
        h, w = d.shape[:2]
        sx = w / float(self.iw)
        sy = h / float(self.ih)
        px = np.clip(np.round(xs * sx).astype(int), 0, w - 1)
        py = np.clip(np.round(ys * sy).astype(int), 0, h - 1)

        vals = d[py, px].astype(np.float32)
        if d.dtype == np.uint16:
            vals = vals * 0.001
        valid = vals[(vals > 0.05) & np.isfinite(vals)
                     & (vals < self.depth_max_valid)]
        # 유효 픽셀이 blob 의 일부에 불과하면 대부분 사거리 밖이거나 가려진
        # 상황이다. 소수의 앞쪽 물체 거리로 투영하면 안 된다.
        if valid.size < max(4, int(xs.size * self.blob_depth_ratio)):
            return None
        return float(np.median(valid))

    def _depth_at(self, cx, cy):
        """(cx,cy) 주변 창의 중앙값 거리[m]. 없으면 None."""
        d = self.latest_depth
        if d is None:
            return None
        h, w = d.shape[:2]
        # depth 해상도가 열화상과 다르면 스케일
        sx = w / float(self.iw)
        sy = h / float(self.ih)
        px = int(round(cx * sx))
        py = int(round(cy * sy))
        if not (0 <= px < w and 0 <= py < h):
            return None

        scale = 0.001 if d.dtype == np.uint16 else 1.0

        def ok(v):
            # far clip(8m) 포화값은 '측정된 거리'가 아니다.
            return 0.05 < v < self.depth_max_valid and np.isfinite(v)

        # ① 반드시 '그 픽셀' 의 depth 를 먼저 본다.
        #    창 전체의 중앙값을 쓰면, 열원이 depth 사거리 밖일 때 창에 걸친
        #    문틀·벽(≈4m) 이 중앙값이 되어 화재가 그 벽 위에 저장된다.
        center = float(d[py, px]) * scale
        if not ok(center):
            return None

        # ② 잡음 완화용으로만 창을 쓰되, 중앙값과 가까운 값만 평균낸다.
        r = 4
        x0, x1 = max(0, px - r), min(w, px + r + 1)
        y0, y1 = max(0, py - r), min(h, py + r + 1)
        patch = d[y0:y1, x0:x1].astype(np.float32) * scale
        near = patch[(patch > 0.05) & np.isfinite(patch)
                     & (np.abs(patch - center) < 0.5)]
        return float(np.median(near)) if near.size else center

    # ── 정지 조준 확인(INSPECT) ────────────────────────────────────────

    def _aim_yaw_error(self):
        """조준 목표와 현재 포탑 각도의 차이(rad)."""
        if self._ci_aim is None:
            return math.inf
        rx, ry, rtheta = self._robot_pose()
        aim = math.atan2(self._ci_aim[1] - ry, self._ci_aim[0] - rx)
        err = (aim - rtheta) - self.turret_yaw
        while err >  math.pi: err -= 2 * math.pi
        while err < -math.pi: err += 2 * math.pi
        return err

    def _observe_fire(self, fx, fy, peak_k, now_s):
        """열원 관측 → 정지·조준 후에만 등록.

        이동·포탑 스윕 중에 등록하면 위치가 크게 튄다. 조난자와 동일하게
        후보를 보면 정지를 요청하고, 멈춰서 겨눈 상태에서 모은 표본의
        중앙값으로 등록한다.
        """
        if not self.insp_enabled:
            self._register_fire(fx, fy, peak_k)
            return

        # 이미 확정된 화재 근처면 재확인 불필요 — 기존 병합 경로로
        for f in self.fires:
            if f['confirmed'] and math.hypot(f['x'] - fx, f['y'] - fy) < self.merge_d:
                self._register_fire(fx, fy, peak_k)
                return

        # 방금 확인에 실패한 열원이면 잠시 건너뛴다. 안 그러면 타임아웃 →
        # 순찰 재개 → 같은 열원 재발견 → 다시 정지 를 무한 반복하며
        # 로봇이 그 앞에서 영영 못 벗어난다(실측: 28초 주기로 반복).
        for fx0, fy0, t0 in self._ci_failed:
            if (now_s - t0 < self.insp_retry_s
                    and math.hypot(fx - fx0, fy - fy0) < self.merge_d):
                return

        self._ci_seen_t = now_s

        if not self._ci_active:
            self._ci_active  = True
            self._ci_aim     = (fx, fy)
            self._ci_peak    = peak_k
            self._ci_start   = now_s
            self._ci_settled = None
            self._ci_samples = []
            p = PointStamped()
            p.header.frame_id = 'map'
            p.header.stamp = self.get_clock().now().to_msg()
            p.point.x, p.point.y, p.point.z = float(fx), float(fy), 0.5
            self.pub_ci_req.publish(p)
            self.get_logger().info(
                f'열원 후보 ({fx:.1f},{fy:.1f}) — 정지 요청 후 조준 확인 시작')
            return

        # 정지 + 조준이 붙었는지
        settled = (self.robot_speed <= self.insp_settle
                   and abs(self._aim_yaw_error()) <= self.insp_yaw_tol)
        if settled and self._ci_settled is None:
            self._ci_settled = now_s
        if not settled:
            return

        self._ci_samples.append((fx, fy))
        # _ci_aim 은 갱신하지 않는다. 포탑을 실제로 돌리는 쪽은
        # patrol_navigator 이고, 그쪽은 /fire_candidate 로 받은 **최초** 위치를
        # 계속 겨눈다. 여기서 조준 목표만 최신 추정치로 옮기면 두 노드의
        # 기준이 어긋나 조준오차가 영영 안 좁혀진다.
        # (실측: 표본 1개 모은 뒤 오차 12.2° 로 벌어져 타임아웃)
        self._ci_peak = max(self._ci_peak, peak_k)

        if len(self._ci_samples) >= self.insp_samples:
            xs = sorted(s[0] for s in self._ci_samples)
            ys = sorted(s[1] for s in self._ci_samples)
            m = len(xs) // 2
            gx, gy = xs[m], ys[m]
            spread = max(math.hypot(x - gx, y - gy) for x, y in self._ci_samples)
            self.get_logger().info(
                f'열원 확인 완료 ({gx:.1f},{gy:.1f}) 표본{len(xs)}개 산포{spread:.2f}m')
            self._register_fire(gx, gy, self._ci_peak)
            self._finish_ci(True)

    def _ci_timer(self):
        """확인 상태 타임아웃 감시 (열원이 시야에서 사라져도 풀려야 한다)."""
        if not self._ci_active:
            return
        now_s = self.get_clock().now().nanoseconds * 1e-9
        lost = now_s - self._ci_seen_t
        if now_s - self._ci_start > self.insp_timeout:
            self.get_logger().warn(
                f'열원 조준 확인 시간초과 — 순찰 재개 '
                f'[표본 {len(self._ci_samples)}개, 속도 {self.robot_speed:.3f}m/s'
                f'(한계 {self.insp_settle}), 조준오차 '
                f'{math.degrees(abs(self._aim_yaw_error())):.1f}°'
                f'(한계 {math.degrees(self.insp_yaw_tol):.0f}°), '
                f'정착 {"O" if self._ci_settled else "X"}]')
            self._finish_ci(False)
        elif self._ci_settled is not None and lost > self.insp_after_aim:
            self.get_logger().info('겨눴는데 열원 없음 — 유령 후보로 판단, 순찰 재개')
            self._finish_ci(False)

    def _finish_ci(self, ok: bool):
        if not ok and self._ci_aim is not None:
            now_s = self.get_clock().now().nanoseconds * 1e-9
            self._ci_failed = [f for f in self._ci_failed
                               if now_s - f[2] < self.insp_retry_s]
            self._ci_failed.append((self._ci_aim[0], self._ci_aim[1], now_s))
        self._ci_active  = False
        self._ci_aim     = None
        self._ci_settled = None
        self._ci_samples = []
        m = Bool(); m.data = bool(ok)
        self.pub_ci_done.publish(m)

    def _register_fire(self, fx, fy, peak_k):
        # 기존 화재와 병합
        for f in self.fires:
            if math.hypot(f['x'] - fx, f['y'] - fy) < self.merge_d:
                # 이동평균으로 위치 안정화
                w = min(f['hits'], 10)
                f['x'] = (f['x'] * w + fx) / (w + 1)
                f['y'] = (f['y'] * w + fy) / (w + 1)
                f['peak_k'] = max(f['peak_k'], peak_k)
                f['hits'] += 1
                if not f['confirmed'] and f['hits'] >= self.confirm_hits:
                    f['confirmed'] = True
                    self._paint_bloom(f['x'], f['y'])
                    self._emit_alert(f['x'], f['y'])
                return
        # 신규
        f = {'x': fx, 'y': fy, 'peak_k': peak_k, 'hits': 1,
             'confirmed': self.confirm_hits <= 1}
        self.fires.append(f)
        if f['confirmed']:
            self._paint_bloom(fx, fy)
            self._emit_alert(fx, fy)

    def _emit_alert(self, fx, fy):
        p = PointStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.point.x = fx
        p.point.y = fy
        p.point.z = 0.0
        self.pub_alert.publish(p)
        self.get_logger().warn(f'🔥 화재 발견! map ({fx:.1f}, {fy:.1f}) — 총 {len(self.fires)}건')

    # ── 열장 페인팅 ──────────────────────────────────────────────────
    def _world_to_cell(self, x, y):
        col = int((x - _ORIGIN) / _RES)
        row = int((y - _ORIGIN) / _RES)
        return row, col

    def _paint_bloom(self, fx, fy):
        """화재 주변으로 가우시안 열장(0..100)을 누적(max)."""
        sigma = self.bloom_r / 2.0
        rad_cells = int(math.ceil(self.bloom_r / _RES)) + 1
        r0, c0 = self._world_to_cell(fx, fy)
        for dr in range(-rad_cells, rad_cells + 1):
            for dc in range(-rad_cells, rad_cells + 1):
                r, c = r0 + dr, c0 + dc
                if not (0 <= r < _N and 0 <= c < _N):
                    continue
                d = math.hypot(dr, dc) * _RES
                if d > self.bloom_r:
                    continue
                val = 100.0 * math.exp(-(d * d) / (2.0 * sigma * sigma))
                if val > self.heat[r, c]:
                    self.heat[r, c] = val

    # ── 주기 발행 ────────────────────────────────────────────────────
    def publish_all(self):
        stamp = self.get_clock().now().to_msg()
        self._publish_heatmap(stamp)
        self._publish_cloud(stamp)
        self._publish_markers(stamp)

    def _publish_heatmap(self, stamp):
        og = OccupancyGrid()
        og.header.stamp = stamp
        og.header.frame_id = 'map'
        og.info.resolution = _RES
        og.info.width = _N
        og.info.height = _N
        og.info.origin.position.x = _ORIGIN
        og.info.origin.position.y = _ORIGIN
        og.info.origin.orientation.w = 1.0
        data = np.where(self.heat >= 1.0, self.heat, -1.0)
        og.data = data.astype(np.int8).flatten().tolist()
        self.pub_heat.publish(og)

    def _publish_cloud(self, stamp):
        """확정 화재마다 obstacle_radius 원반의 점들을 map 프레임으로 발행 → costmap 마킹."""
        pts = []
        step = 0.15
        n = int(self.obst_r / step)
        for f in self.fires:
            if not f['confirmed']:
                continue
            for i in range(-n, n + 1):
                for j in range(-n, n + 1):
                    dx, dy = i * step, j * step
                    if math.hypot(dx, dy) > self.obst_r:
                        continue
                    pts.append((f['x'] + dx, f['y'] + dy, 0.3))
        header = Header()
        header.stamp = stamp
        header.frame_id = 'map'
        cloud = point_cloud2.create_cloud_xyz32(header, pts)
        self.pub_cloud.publish(cloud)

    def _publish_markers(self, stamp):
        ma = MarkerArray()
        # 오래된 마커 삭제
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        for idx, f in enumerate(self.fires):
            if not f['confirmed']:
                continue
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = 'fire'
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = f['x']
            m.pose.position.y = f['y']
            m.pose.position.z = 0.6
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.8
            m.color.r = 1.0
            m.color.g = 0.35
            m.color.b = 0.0
            m.color.a = 0.85
            ma.markers.append(m)

            t = Marker()
            t.header.frame_id = 'map'
            t.header.stamp = stamp
            t.ns = 'fire_label'
            t.id = 1000 + idx
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = f['x']
            t.pose.position.y = f['y']
            t.pose.position.z = 1.4
            t.pose.orientation.w = 1.0
            t.scale.z = 0.5
            t.color.r = 1.0
            t.color.g = 0.9
            t.color.b = 0.2
            t.color.a = 1.0
            t.text = f"FIRE {f['peak_k']:.0f}K"
            ma.markers.append(t)
        self.pub_marker.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = FireDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
