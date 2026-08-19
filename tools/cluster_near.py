#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""특정 자리 근처의 실제 검출 좌표를 모아 평균을 낸다.

    python3 cluster_near.py <x> <y> <반경> <로그...>

누운 사람은 메쉬 원점이 발밑이라 몸이 yaw 방향으로 약 1.75m 뻗는다.
어느 쪽으로 뻗는지 부호를 헷갈리면 정답표가 1.75m 어긋나고, 멀쩡한
추정이 오차로 잡힌다. 실제 검출이 어디 모이는지 보면 부호가 정해진다.
"""
import math
import re
import sys

LOG = re.compile(r'\[구조 로그\].*?위치:\(([^,]+),([^)]+)\)')


def main():
    tx, ty, r = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    xs, ys = [], []
    for p in sys.argv[4:]:
        try:
            f = open(p, encoding='utf-8', errors='replace')
        except OSError:
            continue
        with f:
            for line in f:
                m = LOG.search(line)
                if not m:
                    continue
                x, y = float(m.group(1)), float(m.group(2))
                if math.hypot(x - tx, y - ty) <= r:
                    xs.append(x)
                    ys.append(y)
    if not xs:
        print(f'({tx}, {ty}) 반경 {r}m — 검출 없음')
        return
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    print(f'({tx}, {ty}) 반경 {r}m — 검출 {len(xs)}건')
    print(f'  평균 위치 ({mx:.2f}, {my:.2f})')
    print(f'  기준점에서 ({mx - tx:+.2f}, {my - ty:+.2f}) 만큼 떨어짐')


if __name__ == '__main__':
    main()
