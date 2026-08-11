#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLO 재학습용 데이터 자동 수집 노드.

    ros2 run ugv_vision dataset_capture_node \
        --ros-args -p truth_json:=/tmp/truth.json -p out_dir:=~/ugv_dataset

무엇을 모으나
-------------
유령 후보(잔해를 사람으로 오인)가 탐사 시간의 30%를 먹는다. 거리·지속성
어느 신호로도 진짜와 구분되지 않았다(README 참조). 남은 방법은 인식기
자체를 이 환경에 맞게 고치는 것이다.

시뮬은 정답 위치를 알고 있으므로 라벨을 사람이 붙일 필요가 없다.
  · 검출의 추정 좌표가 정답 조난자 근처(match_r) → 양성
  · 그렇지 않으면 → 유령(음성). 그 프레임은 '사람 없음' 으로 학습시킨다

YOLO 형식으로 저장한다(images/*.jpg + labels/*.txt).
음성 프레임은 빈 라벨 파일을 만든다 — 학습에서 배경(하드 네거티브)이 된다.

주의
----
투영 계산은 target_manager_node._estimate_xy 와 반드시 같아야 한다.
픽셀 x 는 오른쪽이 +, ROS yaw 는 왼쪽이 + 라 부호를 뒤집는다.
"""
import json
import math
import os
from datetime import datetime

import cv2
import numpy as np
import rclpy
import rclpy.time
import tf2_ros
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from ugv_msgs.msg import TargetDetection

CAM_FOV_RAD = 1.089


class DatasetCapture(Node):
    def __init__(self):
        super().__init__('dataset_capture_node')

        self.declare_parameter('truth_json', '')
        self.declare_parameter('out_dir', os.path.expanduser('~/ugv_dataset'))
        self.declare_parameter('match_r', 1.5)      # 정답과 이 안이면 양성(m)
        # 저장 주기를 양성/유령으로 나눈다.
        # 로봇은 진짜 조난자를 조사하는 데 시간을 많이 쓰므로 같은 주기로
        # 저장하면 양성만 쌓인다(실측 211 : 16). 유령을 막으려면 '사람이
        # 아니다' 를 가르칠 표본이 많아야 하는데 그게 부족해진다.
        # 유령은 드물게 생기므로 촘촘히, 양성은 성기게 담는다.
        self.declare_parameter('min_gap_pos_s', 1.0)
        self.declare_parameter('min_gap_neg_s', 0.1)
        self.declare_parameter('cam_fov_rad', CAM_FOV_RAD)

        self.match_r = float(self.get_parameter('match_r').value)
        self.gap_pos = float(self.get_parameter('min_gap_pos_s').value)
        self.gap_neg = float(self.get_parameter('min_gap_neg_s').value)
        self.cam_fov = float(self.get_parameter('cam_fov_rad').value)

        tj = self.get_parameter('truth_json').value
        self.victims = []
        if tj and os.path.exists(tj):
            with open(tj, encoding='utf-8') as f:
                self.victims = [(v['x'], v['y'])
                                for v in json.load(f).get('victims', [])]
        if not self.victims:
            self.get_logger().warn(
                'truth_json 이 없거나 비었다 — 모든 검출을 유령으로 본다. '
                '의도한 것이 아니면 -p truth_json:=... 을 줄 것')

        base = os.path.expanduser(str(self.get_parameter('out_dir').value))
        stamp = datetime.now().strftime('%m%d_%H%M%S')
        self.img_dir = os.path.join(base, stamp, 'images')
        self.lbl_dir = os.path.join(base, stamp, 'labels')
        os.makedirs(self.img_dir, exist_ok=True)
        os.makedirs(self.lbl_dir, exist_ok=True)

        self.bridge = CvBridge()
        self._tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=30))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        self._frame = None
        self._last_pos = 0.0
        self._last_neg = 0.0
        self.n_pos = self.n_neg = 0

        self.create_subscription(
            Image, '/camera/camera/color/image_raw', self._img_cb,
            qos_profile=qos_profile_sensor_data)
        self.create_subscription(
            TargetDetection, '/target_detection', self._det_cb, 10)
        self.create_timer(20.0, self._report)

        self.get_logger().info(
            f'데이터 수집 시작 — 정답 조난자 {len(self.victims)}명, '
            f'저장 위치 {os.path.dirname(self.img_dir)}')

    def _img_cb(self, msg):
        try:
            self._frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            self._frame = None

    def _robot_pose(self):
        try:
            tf = self._tf_buf.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            q = tf.transform.rotation
            th = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return tf.transform.translation.x, tf.transform.translation.y, th
        except Exception:
            return None

    def _det_cb(self, msg: TargetDetection):
        if self._frame is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        # 주기 판단은 양성/유령을 가른 뒤에 한다(아래). 여기서 한 번에
        # 걸러버리면 드문 유령이 양성에 밀려 저장되지 않는다.
        pose = self._robot_pose()
        if pose is None:
            return
        rx, ry, rtheta = pose

        # target_manager 와 동일한 투영식이어야 한다
        pixel_angle = -((msg.x - 320.0) / 320.0) * (self.cam_fov / 2.0)
        cam_hdg = rtheta + msg.capture_turret_yaw + pixel_angle
        gx = rx + msg.distance * math.cos(cam_hdg)
        gy = ry + msg.distance * math.sin(cam_hdg)

        near = min((math.hypot(gx - vx, gy - vy) for vx, vy in self.victims),
                   default=float('inf'))
        is_person = near <= self.match_r
        gap = self.gap_pos if is_person else self.gap_neg
        last = self._last_pos if is_person else self._last_neg
        if now - last < gap:
            return

        frame = self._frame
        h, w = frame.shape[:2]
        name = f'{int(now * 1000) % 10**10}'
        cv2.imwrite(os.path.join(self.img_dir, f'{name}.jpg'), frame)

        lines = []
        if is_person:
            # YOLO 형식: class cx cy w h (0~1 정규화). class 0 = person
            bw = max(4.0, float(msg.w)) / w
            bh = max(4.0, float(msg.h)) / h
            cx = float(msg.x) / w
            cy = float(msg.y) / h
            cx = min(max(cx, 0.0), 1.0); cy = min(max(cy, 0.0), 1.0)
            bw = min(bw, 1.0); bh = min(bh, 1.0)
            lines.append(f'0 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}')
            self.n_pos += 1
        else:
            self.n_neg += 1   # 빈 라벨 = 배경(하드 네거티브)

        with open(os.path.join(self.lbl_dir, f'{name}.txt'), 'w') as f:
            f.write('\n'.join(lines))
        if is_person:
            self._last_pos = now
        else:
            self._last_neg = now

    def _report(self):
        self.get_logger().info(
            f'수집 현황 — 양성 {self.n_pos}장 / 유령(배경) {self.n_neg}장')


def main(args=None):
    rclpy.init(args=args)
    node = DatasetCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'종료 — 양성 {node.n_pos}장 / 유령 {node.n_neg}장')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
