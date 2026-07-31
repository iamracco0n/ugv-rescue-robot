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
  3) heading = robot_theta + turret_yaw + pixel_angle  로 월드 좌표 투영
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
from std_msgs.msg import Header

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
        self.declare_parameter('max_fire_range', 10.0)           # m, 이보다 멀면 무시
        self.declare_parameter('min_fire_range', 0.3)
        self.declare_parameter('bloom_radius', 1.4)              # m, 열장이 번지는 반경
        self.declare_parameter('obstacle_radius', 1.3)          # m, nav 회피 반경
        self.declare_parameter('merge_dist', 1.2)               # m, 이 안이면 같은 화재
        self.declare_parameter('confirm_hits', 2)               # 몇 번 봐야 확정

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
        self.bloom_r       = g('bloom_radius').value
        self.obst_r        = g('obstacle_radius').value
        self.merge_d       = g('merge_dist').value
        self.confirm_hits  = int(g('confirm_hits').value)

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
        self.create_subscription(JointState, '/joint_states',    self.joint_cb,   10)

        # ── 발행 ─────────────────────────────────────────────────────
        self.pub_heat   = self.create_publisher(OccupancyGrid, '/fire_heatmap', _LATCH_QOS)
        self.pub_cloud  = self.create_publisher(PointCloud2,   '/fire_cloud',   10)
        self.pub_marker = self.create_publisher(MarkerArray,   '/fire_markers', 10)
        self.pub_alert  = self.create_publisher(PointStamped,  '/fire_alert',   10)
        # 불 박스 오버레이 이미지 → rqt_image_view / RViz Image
        self.pub_img    = self.create_publisher(Image,         '/fire/image_annotated', 5)

        self.create_timer(0.5, self.publish_all)   # 2 Hz

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
            M = cv2.moments(c)
            if M['m00'] <= 0:
                continue
            cx = M['m10'] / M['m00']
            cy = M['m01'] / M['m00']

            dist = self._depth_at(cx, cy)
            if dist is None or not (self.min_range < dist < self.max_range):
                continue

            pixel_angle = ((cx - self.iw / 2.0) / (self.iw / 2.0)) * (self.fov / 2.0)
            hdg = rtheta + self.turret_yaw + pixel_angle
            fx = rx + dist * math.cos(hdg)
            fy = ry + dist * math.sin(hdg)

            # blob 최고온도 → Kelvin
            mask = np.zeros(hot.shape, np.uint8)
            cv2.drawContours(mask, [c], -1, 255, -1)
            peak_raw = float(therm[mask > 0].max())
            peak_k = peak_raw * self.lin_res if therm.dtype != np.uint8 else peak_raw

            self._register_fire(fx, fy, peak_k)

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
        r = 4
        x0, x1 = max(0, px - r), min(w, px + r + 1)
        y0, y1 = max(0, py - r), min(h, py + r + 1)
        patch = d[y0:y1, x0:x1].astype(np.float32)
        if d.dtype == np.uint16:
            patch = patch * 0.001            # mm → m
        valid = patch[(patch > 0.05) & np.isfinite(patch)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

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
