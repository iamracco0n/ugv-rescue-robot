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

        # ── 오탐(벽·기둥·잔해) 억제 게이트 ────────────────────────────
        # 벽이 사람으로 잡히던 원인: conf가 낮고(0.25) 키포인트 신뢰도를
        # 전혀 보지 않아, 코 키포인트 x!=0 하나만 통과하면 등록됐음.
        self.declare_parameter('det_conf',        0.50)  # YOLO 박스 신뢰도 하한
        self.declare_parameter('min_kpt_conf',    0.50)  # 키포인트 신뢰도 하한
        self.declare_parameter('min_valid_kpts',  6)     # 위 기준 통과 키포인트 최소 개수
        # depth vs 박스크기 추정 허용 상대오차.
        # 박스 대각선 추정은 '서 있는 사람(1.75m)' 을 가정한다. 앉은 사람,
        # 침대에 누운 사람, 잔해에 가린 사람은 박스가 작아 추정 거리가 크게
        # 어긋나므로 0.6 으로 잡으면 진짜 조난자가 대량 기각된다.
        # (실측: 15초당 depth불일치 기각 42건, 같은 구간 키포인트 기각은 3건)
        # 이 검사는 '벽을 사람으로 보는' 극단적 경우만 걸러내면 충분하다.
        # 사람 여부의 실질적 판별은 키포인트 신뢰도가 담당한다.
        self.declare_parameter('depth_tol',       2.00)
        self.declare_parameter('min_box_diag_px', 40.0)  # 너무 작은 박스 제거
        self.declare_parameter('max_box_diag_px', 900.0) # 화면 전체를 덮는 박스 제거
        self.det_conf        = float(self.get_parameter('det_conf').value)
        self.min_kpt_conf    = float(self.get_parameter('min_kpt_conf').value)
        self.min_valid_kpts  = int(self.get_parameter('min_valid_kpts').value)
        self.depth_tol       = float(self.get_parameter('depth_tol').value)
        self.min_box_diag_px = float(self.get_parameter('min_box_diag_px').value)
        self.max_box_diag_px = float(self.get_parameter('max_box_diag_px').value)
        self._reject_counts  = {'conf': 0, 'kpt': 0, 'geom': 0, 'depth': 0}

        # 골격 기하로 자세(누움/앉음)를 판정해 트리아지 모델 결과를 보정할지.
        # 모델이 앉은 자세를 학습하지 않아 휠체어 환자가 L3(정상)로 나왔다.
        self.declare_parameter('posture_rule', True)
        self.posture_rule = bool(self.get_parameter('posture_rule').value)

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

        self.create_timer(15.0, self._log_rejects)

        self.get_logger().info(
            f'YoloPoseNode 시작 — RGB+Depth 동기화 구독 중 '
            f'(오탐게이트: conf≥{self.det_conf}, 키포인트 {self.min_valid_kpts}개'
            f'≥{self.min_kpt_conf}, depth오차≤{self.depth_tol:.0%})')

    def _log_rejects(self):
        """기각 통계 — 게이트가 과하게/모자라게 걸리는지 튜닝용."""
        if not any(self._reject_counts.values()):
            return
        c = self._reject_counts
        self.get_logger().info(
            f'[오탐 게이트] 기각 15s: 키포인트={c.get("kpt",0)} '
            f'박스크기={c.get("geom",0)} depth불일치={c.get("depth",0)}')
        for k in c:
            c[k] = 0

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

    # ── 사람 판정 게이트 (벽·기둥 오탐 억제) ─────────────────────────
    def _person_gate(self, kconf, x1, y1, x2, y2, dist_depth, dist_diag):
        """사람으로 인정할지 판정. (통과여부, 사유) 반환.

        벽·기둥은 사람 실루엣이 없으므로 키포인트 신뢰도가 전반적으로 낮고,
        depth로 잰 거리와 박스 크기로 추정한 거리가 크게 어긋난다.
        (평평한 벽은 가까워도 박스가 작게/크게 제멋대로 잡힘)
        """
        # ① 키포인트 신뢰도 — 가장 강한 신호
        if kconf is not None:
            n_valid = int((kconf >= self.min_kpt_conf).sum())
            if n_valid < self.min_valid_kpts:
                return False, 'kpt'

        # ② 박스 크기 상식 범위
        w, h = float(x2 - x1), float(y2 - y1)
        diag = math.hypot(w, h)
        if not (self.min_box_diag_px <= diag <= self.max_box_diag_px):
            return False, 'geom'

        # ③ depth 거리 vs 박스크기 추정 거리 정합성
        #    둘 다 유효할 때만 검사(폴백 상황은 통과시킴)
        if dist_depth > 0.1 and dist_diag > 0.1:
            rel = abs(dist_depth - dist_diag) / max(dist_depth, 1e-6)
            if rel > self.depth_tol:
                return False, 'depth'

        return True, ''

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
    def _posture(self, kpts, kconf):
        """골격 기하로 자세를 판정. 'lying' | 'sitting' | 'standing' | None.

        트리아지 RandomForest 는 서있음/누움 위주로 학습돼 있어, 앉은 자세를
        넣으면 가까운 쪽(대개 L3:Normal)으로 떨어진다. 실제로 휠체어 환자가
        초록(정상)으로 표시됐다. 모델을 다시 학습시킬 데이터가 없으므로,
        판별이 명확한 기하 특징으로 자세를 먼저 가른다.

        · 몸통(어깨중점→엉덩이중점)이 수평에 가까우면  → 누움
        · 몸통과 다리(엉덩이→발목)가 이루는 각이 90도 근처면 → 앉음
          (서 있으면 몸통과 다리가 거의 일직선 = 180도)
        """
        def ok(i):
            return (kconf is None or kconf[i] >= self.min_kpt_conf) and kpts[i][0] > 0

        def mid(a, b):
            if ok(a) and ok(b):
                return ((kpts[a][0] + kpts[b][0]) / 2.0,
                        (kpts[a][1] + kpts[b][1]) / 2.0)
            if ok(a):
                return (kpts[a][0], kpts[a][1])
            if ok(b):
                return (kpts[b][0], kpts[b][1])
            return None

        sh, hp = mid(5, 6), mid(11, 12)
        kn, an = mid(13, 14), mid(15, 16)
        if sh is None or hp is None:
            return None

        # 이미지 y 는 아래로 증가 → 위쪽이 음수
        tx, ty = hp[0] - sh[0], hp[1] - sh[1]
        tl = math.hypot(tx, ty)
        if tl < 8.0:
            return None
        torso_from_vertical = math.degrees(math.acos(min(1.0, abs(ty) / tl)))
        if torso_from_vertical > 55.0:
            return 'lying'

        # 다리 방향은 반드시 '허벅지'(엉덩이→무릎)로 잰다.
        # 발목까지 쓰면 앉은 자세에서 종아리가 아래로 내려가 벡터가 대각선이
        # 되고, 각이 충분히 안 벌어져 서있음으로 오판한다(실측 36도).
        # 허벅지로 재면 같은 자세가 85도로 명확히 갈린다.
        leg_end = kn or an
        if leg_end is None:
            return None
        lx, ly = leg_end[0] - hp[0], leg_end[1] - hp[1]
        ll = math.hypot(lx, ly)
        if ll < 8.0:
            return None
        cosang = (tx * lx + ty * ly) / (tl * ll)
        bend = math.degrees(math.acos(max(-1.0, min(1.0, cosang))))

        # 정강이 길이 — 정면 뷰에서 앉음을 가리는 핵심 단서
        shin = 0.0
        if kn is not None and an is not None:
            shin = math.hypot(an[0] - kn[0], an[1] - kn[1])

        self.get_logger().info(
            f'[자세] 몸통기울기 {torso_from_vertical:.0f}° 몸통-허벅지 {bend:.0f}° '
            f'몸통 {tl:.0f}px 허벅지 {ll:.0f}px 정강이 {shin:.0f}px '
            f'허벅지/몸통 {ll/tl:.2f} 정강이/허벅지 {shin/max(ll,1e-6):.2f}',
            throttle_duration_sec=3.0)

        # 옆에서 보면 몸통과 허벅지가 크게 꺾인다
        if bend > 45.0:
            return 'sitting'
        # 정면에서는 허벅지가 카메라 쪽으로 향해 원근 단축된다. 2D 각도는
        # 거의 안 벌어지지만, 허벅지가 정강이·몸통보다 뚜렷하게 짧아진다.
        #
        # 같은 자리·같은 거리에서 두 자세를 실측한 값 (시뮬):
        #            몸통-허벅지   허벅지/몸통   정강이/허벅지
        #   서있음        3°       0.72~0.74     0.98~1.00
        #   앉음      45~62°       0.42~0.56     1.96~2.60
        # 정강이/허벅지가 가장 확실히 갈리고, 허벅지/몸통은 서있음(0.74)과
        # 가까우므로 0.65 로 여유를 둔다.
        if shin > 0 and ll > 0:
            if shin / ll > 1.5 and ll / tl < 0.65 and torso_from_vertical < 35.0:
                return 'sitting'
        elif ll > 0 and torso_from_vertical < 35.0 and tl >= 35.0:
            # 발목이 안 잡히는 경우(정강이 0px)가 흔하다. 휠체어는 다리가
            # 프레임 아래로 잘리거나 바퀴에 가려 발목 키포인트가 자주 빠진다.
            # 이때는 허벅지/몸통 비율만으로 판단한다.
            # 실측: 서있음 0.72~0.74 / 앉음 0.42~0.56.
            # 다만 이 단서 하나로는 약해서, 부분 관측된 서있는 사람이
            # 앉음으로 넘어가 L2 로 과대평가되는 사례가 나왔다.
            # → 임계를 0.58 로 좁히고, 몸통이 35px 이상 제대로 잡힌
            #   관측에서만 적용한다(작게 잡힌 골격은 비율이 부정확).
            if ll / tl < 0.58:
                return 'sitting'
        return 'standing'

    def classify(self, kpts, kconf=None):
        feat = self.extract_skeleton_features(kpts)
        if self.classifier and self.scaler:
            pred  = self.classifier.predict(self.scaler.transform(feat))[0]
            level = int(str(pred).split('_')[0])
        else:
            level = 3

        # 기하 판정이 명확한 경우 모델 결과를 덮어쓴다.
        # 앉은 자세는 학습 데이터에 없어 모델이 L3 로 흘려보내지만,
        # 스스로 못 걷는 상태이므로 최소 L2(Urgent) 로 본다.
        if self.posture_rule:
            p = self._posture(kpts, kconf)
            if p == 'sitting' and level > 2:
                level = 2
            elif p == 'lying' and level > 1:
                level = 1

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

        results  = self.model(frame, verbose=False, device=self.device,
                              conf=self.det_conf)
        best     = None
        min_dx   = float('inf')

        for r in results:
            if r.boxes is None or r.keypoints is None:
                continue
            kp_xy   = r.keypoints.xy.cpu().numpy()
            kp_conf = (r.keypoints.conf.cpu().numpy()
                       if r.keypoints.conf is not None else [None] * len(kp_xy))
            for box, kpts, kconf in zip(r.boxes.xyxy.cpu().numpy(), kp_xy, kp_conf):
                if len(kpts) < 17:
                    continue
                x1, y1, x2, y2 = map(int, box)
                cx, cy = (x1+x2)/2.0, (y1+y2)/2.0

                # 1순위: 실제 depth 픽셀값 (미터)
                dist_depth = self.get_depth(depth_img, cx, cy)
                # 폴백: depth=0이면 대각선 공식 (누운 사람도 정상 동작)
                dist_diag  = self.estimate_depth_diagonal(x1, y1, x2, y2)
                dist = dist_depth if dist_depth > 0.0 else dist_diag

                ok, why = self._person_gate(kconf, x1, y1, x2, y2,
                                            dist_depth, dist_diag)
                if not ok:
                    self._reject_counts[why] = self._reject_counts.get(why, 0) + 1
                    # 기각된 후보는 회색 점선 박스로만 표시 (튜닝용)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (128, 128, 128), 1)
                    cv2.putText(frame, f'rej:{why}', (x1, y1 - 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)
                    continue

                level, label, color = self.classify(kpts, kconf)

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
            # 다수결이 아니라 '2프레임 이상 지지받은 것 중 가장 위중한 등급'.
            # 구조에서는 과대평가보다 과소평가가 위험하다. 다수결을 쓰면
            # 앉은 자세가 몇 프레임만 잡혀도 L3(정상)에 묻혀 사라진다.
            # (실측: 휠체어 환자가 몸통-허벅지 46~56도로 잡히는 프레임이
            #  있었는데도 L3 로 등록됐다)
            counts = Counter(self.label_history)
            supported = [lv for lv, n in counts.items() if n >= 2]
            final_lv = min(supported) if supported else counts.most_common(1)[0][0]
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
