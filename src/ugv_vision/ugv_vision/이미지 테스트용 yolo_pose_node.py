import rclpy
from rclpy.node import Node
from ugv_msgs.msg import TargetDetection
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
import cv2
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO
import pickle
import os
import math
import time
from collections import deque, Counter

class YoloPoseNode(Node):
    def __init__(self):
        super().__init__('yolo_pose_node')
        self.publisher_ = self.create_publisher(TargetDetection, '/target_detection', 10)  
        model_dir = "/home/user/ugv_ws/src/ugv_vision/ugv_vision/"
        
        yolo_path = os.path.join(model_dir, "yolov8n-pose.pt")
        self.model = YOLO(yolo_path)
        self.bridge = CvBridge()

        self.ml_model_path = os.path.join(model_dir, "triage_model_rf_robust.pkl")
        self.scaler_path = os.path.join(model_dir, "triage_scaler_robust.pkl")
        
        if os.path.exists(self.ml_model_path) and os.path.exists(self.scaler_path):
            with open(self.ml_model_path, 'rb') as f:
                self.triage_classifier = pickle.load(f)
            with open(self.scaler_path, 'rb') as f:
                self.scaler = pickle.load(f)
            self.get_logger().info('스켈레톤 트리아지 모델 로드 완료')
        else:
            self.triage_classifier = None
            self.scaler = None
            self.get_logger().warn('모델 파일이 없음. 규칙 기반 더미 분류기로 작동합니다.')

        self.label_history = deque(maxlen=10)

        # 논문 캡처용: Color 이미지만 받음
        self.rgb_sub = self.create_subscription(
            Image, 
            '/camera/color/image_raw', 
            self.image_callback, 
            10
        )

        self.prev_time = time.time()

        self.skeleton_edges = [
            (0, 1), (0, 2), (1, 3), (2, 4), 
            (5, 6), (5, 11), (6, 12), (11, 12), 
            (5, 7), (7, 9), (6, 8), (8, 10), 
            (11, 13), (13, 15), (12, 14), (14, 16) 
        ]

    def get_average_depth(self, depth_img, cx, cy):
        return 0.0

    def extract_pure_skeleton(self, person):
        valid_points = [p for p in person if p[0] != 0 and p[1] != 0]
        if len(valid_points) == 0:
            return np.zeros(34)

        l_hip, r_hip = person[11], person[12]
        if l_hip[0] != 0 and r_hip[0] != 0:
            cx, cy = (l_hip[0] + r_hip[0]) / 2.0, (l_hip[1] + r_hip[1]) / 2.0
        else:
            cx, cy = np.mean([p[0] for p in valid_points]), np.mean([p[1] for p in valid_points])

        translated = [[p[0] - cx, p[1] - cy] if p[0] != 0 else [0, 0] for p in person]
        max_dist = max([math.hypot(p[0], p[1]) for p in translated])
        if max_dist == 0: max_dist = 1.0

        normalized = []
        for p in translated:
            normalized.extend([p[0] / max_dist, p[1] / max_dist]) 

        return np.array([normalized])

    def classify_triage(self, person, width, height):
        features = self.extract_pure_skeleton(person)

        if self.triage_classifier is not None and self.scaler is not None:
            scaled_features = self.scaler.transform(features)
            prediction = self.triage_classifier.predict(scaled_features)[0]
            level = int(str(prediction).split('_')[0])
        else:
            level = 3 

        triage_map = {
            1: ("L1:Critical", (0, 0, 255)),
            2: ("L2:NeedHelp", (0, 165, 255)),
            3: ("L3:Normal", (0, 255, 0))
        }
        label, color = triage_map.get(level, ("Unknown", (255, 255, 255)))
        return level, label, color

    def image_callback(self, rgb_msg):
        try:
            raw_frame = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image Error: {e}")
            return

        # [비율 유지 리사이즈 로직] 화면에 들어오도록 비율 맞추기 (최대 높이 800 기준)
        h, w = raw_frame.shape[:2]
        target_h = 800
        target_w = int(w * (target_h / h))
        frame = cv2.resize(raw_frame, (target_w, target_h))

        curr_time = time.time()
        fps = 1.0 / (curr_time - self.prev_time) if (curr_time - self.prev_time) > 0 else 0.0
        self.prev_time = curr_time

        results = self.model(frame, verbose=False, device=0)
        best_target = None
        min_dist_to_center = float('inf')

        for r in results:
            if r.boxes is None or r.keypoints is None: continue
            boxes = r.boxes.xyxy.cpu().numpy()
            keypoints = r.keypoints.xy.cpu().numpy()

            for box, person in zip(boxes, keypoints):
                if len(person) < 17 or person[0][0] == 0: continue

                x1, y1, x2, y2 = map(int, box)
                cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                
                distance = 0.0 
                level, label, color = self.classify_triage(person, x2-x1, y2-y1)

                if abs(cx - (target_w/2.0)) < min_dist_to_center:
                    min_dist_to_center = abs(cx - (target_w/2.0))
                    best_target = (cx, cy, distance, level, label, color, x1, y1, x2, y2)

                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f"{label}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                for edge in self.skeleton_edges:
                    pt1, pt2 = person[edge[0]], person[edge[1]]
                    if pt1[0] != 0 and pt2[0] != 0:
                        cv2.line(frame, (int(pt1[0]), int(pt1[1])), (int(pt2[0]), int(pt2[1])), (255, 0, 255), 2)
                for kpt in person:
                    if kpt[0] != 0: cv2.circle(frame, (int(kpt[0]), int(kpt[1])), 3, (0, 255, 0), -1)

        if best_target is not None:
            cx, cy, dist, raw_lv, raw_lbl, raw_clr, bx1, by1, bx2, by2 = best_target
            
            self.label_history.append(raw_lv)
            final_level = Counter(self.label_history).most_common(1)[0][0]
            
            triage_map = {
                1: ("L1:Critical", (0, 0, 255)),
                2: ("L2:NeedHelp", (0, 165, 255)),
                3: ("L3:Normal", (0, 255, 0))
            }
            final_label, final_color = triage_map.get(final_level, ("Unknown", (255, 255, 255)))

            cv2.rectangle(frame, (bx1, by1), (bx2, by2), final_color, 4)
            cv2.putText(frame, f"[TARGET] {final_label}", (bx1, by1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, final_color, 3)

            msg = TargetDetection()
            msg.x, msg.y, msg.distance = float(cx), float(cy), float(dist)
            msg.triage_level, msg.triage_label, msg.status = int(final_level), final_label, "TRACKING"
            self.publisher_.publish(msg)
        else:
            self.label_history.clear()

        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("A.R.G.U.S Vision", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
