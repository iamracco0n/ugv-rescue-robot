#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""주행 중 속도 분포를 본다(궤적 1Hz).

    python3 tools/speed_hist.py <로그...>

왜 필요한가
-----------
명령(cmd_vel)은 2대·3대 모두 0.5 m/s 최대치인데, 실제 주행 속도는 2대
0.37 · 3대 0.26 m/s 다. 명령대로 못 가고 있고 3대가 더 못 간다.

평균만 보면 이유를 모른다. 분포를 보면 갈린다.

  최고 속도 구간이 줄었다      -> 가다 서다를 반복한다(가감속 손실)
  중간 속도 구간이 늘었다      -> 계속 느리게 간다(회피·좁은 길)

3대는 목표가 더 가깝다(9.9m 대 12.1m). 짧게 가면 가속하다 끝나서 최고
속도에 못 닿는다 — 그 몫이 얼마인지 이 분포가 답한다.
"""
import math
import os
import re
import sys
from collections import defaultdict

RE_T = re.compile(r'\[(\d{10})\.')
RE_R = re.compile(r'\[(ugv\d)\.')
RE_TR = re.compile(r'\[궤적\] \((-?\d+\.\d+),(-?\d+\.\d+)\)')
WINDOW = 600
JUMP = 5.0
BINS = [(0.05, 0.15), (0.15, 0.25), (0.25, 0.35), (0.35, 0.45), (0.45, 99)]


def speeds(path):
    t0 = None
    last = {}
    out = []
    for line in open(path, encoding='utf-8', errors='replace'):
        mt, mr, mp = RE_T.search(line), RE_R.search(line), RE_TR.search(line)
        if not (mt and mr and mp):
            continue
        t = int(mt.group(1))
        t0 = t0 if t0 is not None else t
        if t - t0 > WINDOW:
            break
        r = mr.group(1)
        x, y = float(mp.group(1)), float(mp.group(2))
        if r in last:
            px, py, pt = last[r]
            dt = t - pt
            if 0 < dt <= 3:
                d = math.hypot(x - px, y - py)
                if d < JUMP:
                    out.append(d / dt)
        last[r] = (x, y, t)
    return out


def main(paths):
    allv = []
    for p in paths:
        if os.path.exists(p):
            allv += speeds(p)
    moving = [v for v in allv if v > 0.05]
    if not moving:
        print('표본 없음')
        return
    print(f'주행 표본 {len(moving)}개 (정지 제외)')
    for lo, hi in BINS:
        n = sum(1 for v in moving if lo <= v < hi)
        label = f'{lo:.2f}~{hi:.2f}' if hi < 90 else f'{lo:.2f} 이상'
        print(f'  {label:>12}  {100*n/len(moving):5.1f}%')
    moving.sort()
    print(f'  중앙값 {moving[len(moving)//2]:.2f} m/s')


if __name__ == '__main__':
    args = []
    for a in sys.argv[1:]:
        if a.endswith('.txt'):
            args += [l.strip() for l in open(a, encoding='utf-8') if l.strip()]
        else:
            args.append(a)
    main(args)
