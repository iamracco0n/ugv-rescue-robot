#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""여러 로봇의 SLAM 지도를 하나로 겹쳐 공용 /map 으로 낸다.

왜 정렬을 탐색하지 않나
-----------------------
일반적인 다중로봇 지도 병합은 겹치는 부분을 찾아 정합해야 한다. 여기서는
그럴 이유가 없다 — 두 지도가 이미 같은 원점을 쓴다.

gz 의 OdometryPublisher 가 월드 원점 기준으로 odom 을 내기 때문이다
(실측: 스폰 (0,0.8) 인 로봇의 첫 odom 이 (0,0.8) 로 나온다). 그래서 두
로봇의 SLAM 지도가 같은 좌표계에 놓인다.

처음엔 'map 프레임 원점은 로봇이 출발한 자리' 라고 보고 스폰 좌표 차를
오프셋으로 넣었는데, 그게 오히려 어긋나게 만들었다 — 한쪽이 벽이라고 본
칸의 99.2% 가 상대 지도에서는 자유공간에 떨어졌다. 오프셋은 0 이 맞다.
파라미터는 남겨 둔다(스폰 방식이 바뀌면 필요해진다).

겹치는 규칙은 tools/test_map_merge.py 에 고정해 두었다.
  점유가 이기고 미탐사가 진다. 안 그러면 늦게 온 로봇의 미탐사가 먼저 만든
  지도를 지운다.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)
from nav_msgs.msg import OccupancyGrid


def merge_cells(base, add):
    """두 격자값 배열을 겹친다. 값이 큰 쪽이 이기고 미탐사(-1)는 진다.

    점유(100)가 이겨야 하는 이유: 한쪽 로봇만 벽을 봤어도 벽이다.
    미탐사가 져야 하는 이유: 상대가 이미 본 곳을 '모른다'로 덮으면 안 된다.

    둘 다 미탐사일 때만 미탐사로 남는다.
    """
    both_unknown = (base < 0) & (add < 0)
    b = np.where(base < 0, -1, base)
    a = np.where(add < 0, -1, add)
    out = np.maximum(b, a)
    out[both_unknown] = -1
    return out.astype(np.int16)


class MapMerge(Node):
    """로봇별 지도를 구독해 공용 /map 을 낸다."""

    def __init__(self):
        super().__init__('map_merge_node')
        self.declare_parameter('robots', ['ugv1', 'ugv2'])
        # 각 로봇 지도를 공용 프레임으로 옮길 때 더할 오프셋(m).
        # robots 와 같은 순서.
        #
        # ★ 보통은 0 이다. gz 의 OdometryPublisher 가 월드 원점 기준으로
        #   odom 을 내기 때문에(실측: 스폰 (0,0.8) 인 로봇의 첫 odom 이
        #   (0,0.8)), 두 로봇의 SLAM 지도가 이미 같은 원점을 공유한다.
        #   여기에 스폰 좌표 차를 넣으면 오히려 어긋난다 — 실제로 그렇게
        #   했다가 한쪽 벽의 99.2% 가 상대 지도에서 자유공간에 떨어졌다.
        self.declare_parameter('offset_x', [0.0, 0.0])
        self.declare_parameter('offset_y', [0.0, 0.0])
        self.declare_parameter('publish_period_s', 2.0)
        self.declare_parameter('map_frame', 'map')

        self.robots = list(self.get_parameter('robots').value)
        sx = list(self.get_parameter('offset_x').value)
        sy = list(self.get_parameter('offset_y').value)
        self.map_frame = self.get_parameter('map_frame').value

        if not (len(self.robots) == len(sx) == len(sy)):
            raise SystemExit('robots / offset_x / offset_y 길이가 다르다')

        self.offset = {r: (sx[i], sy[i]) for i, r in enumerate(self.robots)}
        self.grids: dict[str, OccupancyGrid] = {}

        latched = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        for r in self.robots:
            self.create_subscription(
                OccupancyGrid, f'/{r}/map',
                lambda m, r=r: self.grids.__setitem__(r, m), latched)

        self.pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.create_timer(
            float(self.get_parameter('publish_period_s').value), self.tick)
        self.get_logger().info(
            f'지도 병합 시작 — {", ".join(self.robots)} '
            f'(기준 {self.robots[0]}, 오프셋 {self.offset})')

    def tick(self):
        have = [r for r in self.robots if r in self.grids]
        if not have:
            return

        # 공용 격자의 범위를 모든 지도가 들어가도록 잡는다.
        res = self.grids[have[0]].info.resolution
        xs, ys, xe, ye = [], [], [], []
        for r in have:
            g = self.grids[r]
            ox = g.info.origin.position.x + self.offset[r][0]
            oy = g.info.origin.position.y + self.offset[r][1]
            xs.append(ox)
            ys.append(oy)
            xe.append(ox + g.info.width * res)
            ye.append(oy + g.info.height * res)
        ox, oy = min(xs), min(ys)
        W = int(np.ceil((max(xe) - ox) / res))
        H = int(np.ceil((max(ye) - oy) / res))
        if W <= 0 or H <= 0 or W * H > 40_000_000:
            self.get_logger().warn(f'병합 격자 크기가 이상하다 {W}x{H} — 건너뜀')
            return

        out = np.full((H, W), -1, dtype=np.int16)
        for r in have:
            g = self.grids[r]
            a = np.asarray(g.data, dtype=np.int16).reshape(
                g.info.height, g.info.width)
            gx = g.info.origin.position.x + self.offset[r][0]
            gy = g.info.origin.position.y + self.offset[r][1]
            cx = int(round((gx - ox) / res))
            cy = int(round((gy - oy) / res))
            sub = out[cy:cy + g.info.height, cx:cx + g.info.width]
            if sub.shape != a.shape:        # 경계 반올림으로 어긋나면 건너뜀
                continue
            out[cy:cy + g.info.height, cx:cx + g.info.width] = merge_cells(sub, a)

        m = OccupancyGrid()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = res
        m.info.width = W
        m.info.height = H
        m.info.origin.position.x = ox
        m.info.origin.position.y = oy
        m.info.origin.orientation.w = 1.0
        m.data = out.reshape(-1).astype(np.int8).tolist()
        self.pub.publish(m)


def main():
    rclpy.init()
    node = MapMerge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
