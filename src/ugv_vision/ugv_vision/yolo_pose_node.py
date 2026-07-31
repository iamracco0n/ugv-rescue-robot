import os
import math
import time
import pickle
from collections import deque, Counter

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
import message_filters
from cv_bridge import CvBridge
from ultralytics import YOLO

from ugv_msgs.msg import TargetDetection

# 카메라 파라미터 (ugv.urdf.xacro: fov=1.089rad, 640x480)
# focal_px = (320 / tan(1.089/2)) ≈ 535
_FOCAL_PX   = 535.0
# 바운딩박스 대각선 기반 폴백: 사람 몸통 대각선 ≈ sqrt(1.7²+0.4²) ≈ 1.75m
# 누운 자세 / 서 있는 자세 모두 대각선 길이는 유사하게 유지됨
_BODY_DIAG  = math.sqrt(1.7**2 + 0.4**2)

_SKELETON_EDGES = [
    (0,1),(0,2),(1,3),(2,4),
    (5,6),(5,11),(6,12),(11,12),
    (5,7),(7,9),(6,8),(8,10),
    (11,13),(13,15),(12,14),(14,16),
]
_TRIAGE_MAP = {
    1: ('L1:Critical', (0, 0, 255)),
    2: ('L2:Urgent',   (0, 165, 255)),
    3: ('L3:Normal',   (0, 255, 0)),
}


class YoloPoseNode(Node):
    def __init__(self):
        super().__init__('yolo_pose_node')

        model_dir = os.path.dirname(os.path.realpath(__file__))

        self.model  = YOLO(os.path.join(model_dir, 'yolov8n-pose.pt'))
        self.bridge = CvBridge()

        # 추론 디바이스 자동 감지 (CPU 데스크탑 / Jetson GPU 양쪽 대응)
        try:
            import torch
            self.device = 0 if torch.cuda.is_available() else 'cpu'
        except Exception:
            self.device = 'cpu'
        self.get_logger().info(f'YOLO 추론 device = {self.device}')

        ml_path     = os.path.join(model_dir, 'triage_model_rf_robust.pkl')
        scaler_path = os.path.join(model_dir, 'triage_scaler_robust.pkl')
        self.classifier = None
        self.scaler = None
        if os.path.exists(ml_path) and os.path.exists(scaler_path):
            try:
                with open(ml_path, 'rb') as f:
                    self.classifier = pickle.load(f)
                with open(scaler_path, 'rb') as f:
                    self.scaler = pickle.load(f)
                self.get_logger().info('트리아지 모델 로드 완료')
            except Exception as e:
                # sklearn 미설치 등 → 노드 전체가 죽지 않게 규칙 기반으로 폴백
                self.classifier = None
                self.scaler = None
                self.get_logger().warn(f'트리아지 모델 로드 실패({e}) — 규칙 기반(L3)으로 동작')
        else:
            self.get_logger().warn('트리아지 모델 없음 — 규칙 기반으로 동작')

        self.label_history  = deque(maxlen=10)
        self.prev_time      = time.time()
        self._turret_yaw    = 0.0   # frame 캡처 시점의 turret_yaw 보관용

        self.pub = self.create_publisher(TargetDetection, '/target_detection', 10)
        # 감지 오버레이 이미지(사람 박스+골격+트리아지) → rqt_image_view / RViz Image
        self.img_pub = self.create_publisher(Image, '/detection/image_annotated', 5)
        # 로컬 OpenCV 창 표시 여부 (SSH/headless면 False 권장 — imshow 크래시 방지)
        self.declare_parameter('show_window', False)
        self.show_window = self.get_parameter('show_window').value

        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)

        # RGB + Depth 동기화 구독
        rgb_sub   = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw',
            qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw',
            qos_profile=qos_profile_sensor_data)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)

        self.get_logger().info('YoloPoseNode 시작 — RGB+Depth 동기화 구독 중')

    def _joint_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name == 'turret_yaw_joint':
                self._turret_yaw = pos

    # ── Depth 거리 측정 (실제 Z 값, 미터 단위) ───────────────────────
    def get_depth(self, depth_img, cx, cy):
        """
        depth_img: float32(m) 또는 uint16(mm).
        윈도우 내 유효값의 중앙값을 미터로 반환.
        """
        win = 4
        h, w = depth_img.shape
        cx, cy = int(cx), int(cy)

        patch = depth_img[
            max(0, cy-win):min(h, cy+win+1),
            max(0, cx-win):min(w, cx+win+1)
        ].astype(np.float32)

        if depth_img.dtype == np.uint16:
            patch *= 0.001          # mm → m
        vals = patch[(patch > 0.1) & (patch < 8.0)]
        return float(np.median(vals)) if vals.size > 0 else 0.0

    # ── 대각선 기반 거리 폴백 (depth=0일 때) ─────────────────────────
    def estimate_depth_diagonal(self, x1, y1, x2, y2):
        """
        누운 사람·서 있는 사람 모두 바운딩박스 대각선 길이는
        몸통 대각선(_BODY_DIAG≈1.75m)에 비례한다는 가정 사용.
        """
        w = float(x2 - x1)
        h = float(y2 - y1)
        pixel_diag = math.sqrt(w*w + h*h)
        if pixel_diag < 5.0:
            return 0.0
        return (_FOCAL_PX * _BODY_DIAG) / pixel_diag

    # ── 스켈레톤 정규화 → 34차원 ─────────────────────────────────────
    def extract_skeleton_features(self, kpts):
        valid = [p for p in kpts if p[0] != 0 and p[1] != 0]
        if not valid:
            return np.zeros((1, 34))
        l_hip, r_hip = kpts[11], kpts[12]
        if l_hip[0] != 0 and r_hip[0] != 0:
            cx = (l_hip[0] + r_hip[0]) / 2.0
            cy = (l_hip[1] + r_hip[1]) / 2.0
        else:
            cx = np.mean([p[0] for p in valid])
            cy = np.mean([p[1] for p in valid])
        trans = [[p[0]-cx, p[1]-cy] if p[0] != 0 else [0, 0] for p in kpts]
        dmax  = max(math.hypot(p[0], p[1]) for p in trans) or 1.0
        feat  = []
        for p in trans:
            feat.extend([p[0]/dmax, p[1]/dmax])
        return np.array([feat])

    # ── 트리아지 분류 ────────────────────────────────────────────────
    def classify(self, kpts):
        feat = self.extract_skeleton_features(kpts)
        if self.classifier and self.scaler:
            pred  = self.classifier.predict(self.scaler.transform(feat))[0]
            level = int(str(pred).split('_')[0])
        else:
            level = 3
        label, color = _TRIAGE_MAP.get(level, ('Unknown', (255,255,255)))
        return level, label, color

    # ── 메인 콜백 ────────────────────────────────────────────────────
    def sync_callback(self, rgb_msg, depth_msg):
        # frame 도착 시점의 turret_yaw 스냅샷 — YOLO 추론 지연(50-200ms) 보정용.
        # YOLO 결과가 나올 때 joint_state를 읽으면 포탑이 이미 돌아있어 위치 오차 발생.
        capture_yaw = self._turret_yaw

        try:
            frame      = self.bridge.imgmsg_to_cv2(rgb_msg,   'bgr8')
            depth_img  = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception:
            return

        frame     = cv2.resize(frame,     (640, 480))
        depth_img = cv2.resize(depth_img, (640, 480), interpolation=cv2.INTER_NEAREST)

        now = time.time()
        fps = 1.0 / max(now - self.prev_time, 1e-6)
        self.prev_time = now

        results  = self.model(frame, verbose=False, device=self.device, conf=0.25)
        best     = None
        min_dx   = float('inf')

        for r in results:
            if r.boxes is None or r.keypoints is None:
                continue
            for box, kpts in zip(r.boxes.xyxy.cpu().numpy(),
                                  r.keypoints.xy.cpu().numpy()):
                if len(kpts) < 17 or kpts[0][0] == 0:
                    continue
                x1, y1, x2, y2 = map(int, box)
                cx, cy = (x1+x2)/2.0, (y1+y2)/2.0

                # 1순위: 실제 depth 픽셀값 (미터)
                dist = self.get_depth(depth_img, cx, cy)
                # 폴백: depth=0이면 대각선 공식 (누운 사람도 정상 동작)
                if dist == 0.0:
                    dist = self.estimate_depth_diagonal(x1, y1, x2, y2)

                level, label, color = self.classify(kpts)

                # 시각화
                cv2.rectangle(frame, (x1,y1), (x2,y2), color, 2)
                cv2.putText(frame, f'{label} {dist:.1f}m',
                            (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                for e in _SKELETON_EDGES:
                    p1, p2 = kpts[e[0]], kpts[e[1]]
                    if p1[0] and p2[0]:
                        cv2.line(frame,
                                 (int(p1[0]),int(p1[1])),
                                 (int(p2[0]),int(p2[1])), (255,0,255), 2)
                for kp in kpts:
                    if kp[0]:
                        cv2.circle(frame, (int(kp[0]),int(kp[1])), 3, (0,255,0), -1)

                if abs(cx - 320.0) < min_dx:
                    min_dx = abs(cx - 320.0)
                    best   = (cx, cy, dist, level, label, color, x1, y1, x2, y2)

        # 다수결 필터 → 퍼블리시
        if best:
            cx, cy, dist, raw_lv, _, _, bx1, by1, bx2, by2 = best
            self.label_history.append(raw_lv)
            final_lv              = Counter(self.label_history).most_common(1)[0][0]
            final_label, final_color = _TRIAGE_MAP.get(final_lv, ('Unknown',(255,255,255)))

            cv2.rectangle(frame, (bx1,by1), (bx2,by2), final_color, 4)
            cv2.putText(frame, f'[TARGET] {final_label}',
                        (bx1, by1-30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, final_color, 3)

            out = TargetDetection()
            out.x, out.y, out.distance    = float(cx), float(cy), float(dist)
            out.triage_level              = int(final_lv)
            out.triage_label              = final_label
            out.status                    = 'TRACKING'
            out.capture_turret_yaw        = float(capture_yaw)
            self.pub.publish(out)
        else:
            self.label_history.clear()

        cv2.putText(frame, f'FPS:{fps:.1f}', (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

        # ROS 토픽으로 오버레이 이미지 발행 (rqt/RViz에서 확인)
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header = rgb_msg.header
            self.img_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().warn(f'오버레이 발행 실패: {e}', throttle_duration_sec=5.0)

        if self.show_window:
            try:
                cv2.imshow('A.R.G.U.S Vision', frame)
                cv2.waitKey(1)
            except Exception:
                self.show_window = False


def main(args=None):
    rclpy.init(args=args)
    node = YoloPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
