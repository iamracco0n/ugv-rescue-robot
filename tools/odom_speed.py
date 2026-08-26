#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""/odom 을 직접 받아 실제 주행 속도를 잰다.

    python3 tools/odom_speed.py <초> <로봇수>

왜 로그가 아니라 odom 인가
--------------------------
앞서 궤적 로그(1Hz)의 연속 표본 사이 변위로 속도를 쟀는데, 그 방법이
편향돼 있었다. 로그 타이머가 CPU 에 굶주리면 표본이 듬성해지고 중간 경로를
놓쳐 거리와 속도가 실제보다 작게 나온다. 실측으로 표본 간격이

    2대 1.29초 · 3대 1.78초 (3대는 99.7%가 1.5초 초과)

였으므로 '3대가 40% 덜 움직인다' 는 결과는 측정 탓일 수 있다.

odom 은 구동 플러그인이 30~50Hz 로 직접 내보내므로 로깅 타이머와 무관하다.
여기서 나온 값이 진짜 주행 속도다.
"""
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class Sampler(Node):
    def __init__(self, n_robots):
        super().__init__('odom_speed_sampler')
        self.vals = {f'ugv{i+1}': [] for i in range(n_robots)}
        for name in self.vals:
            self.create_subscription(
                Odometry, f'/{name}/odom',
                lambda msg, r=name: self.vals[r].append(msg.twist.twist.linear.x),
                20)


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 120
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    rclpy.init()
    node = Sampler(n)
    end = node.get_clock().now().nanoseconds * 1e-9 + secs
    while rclpy.ok() and node.get_clock().now().nanoseconds * 1e-9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)

    allv = []
    for r, v in node.vals.items():
        allv += v
        if v:
            mv = sorted(x for x in v if x > 0.05)
            print(f'{r}: 표본 {len(v)}개 · 주행 중앙값 '
                  f'{mv[len(mv)//2]:.3f} m/s' if mv else f'{r}: 주행 표본 없음')
    if not allv:
        print('표본 없음 — 런이 안 돌거나 도메인이 다르다')
    else:
        moving = sorted(x for x in allv if x > 0.05)
        stopped = len(allv) - len(moving)
        print(f'\n전체 {len(allv)}개 · 정지 {100*stopped/len(allv):.1f}%')
        if moving:
            print(f'주행 중앙값 {moving[len(moving)//2]:.3f} m/s · '
                  f'평균 {sum(moving)/len(moving):.3f}')
            for lo, hi in ((0.05, 0.15), (0.15, 0.25), (0.25, 0.35),
                           (0.35, 0.45), (0.45, 99)):
                c = sum(1 for x in moving if lo <= x < hi)
                lab = f'{lo:.2f}~{hi:.2f}' if hi < 90 else f'{lo:.2f} 이상'
                print(f'  {lab:>12} {100*c/len(moving):5.1f}%')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
