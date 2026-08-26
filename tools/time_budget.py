#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""로봇이 시간을 어디에 쓰는지 분해한다(궤적 1Hz).

    python3 tools/time_budget.py <로그...>

왜 필요한가
-----------
3대일 때 로봇당 이동거리가 40% 줄었다(10분에 186m -> 112m). 그런데 로봇이
노는 것은 아니다 — 구역을 다 훑어 동료를 도우러 간 런이 24런 중 0런이다.
Nav2 제어 주기 놓침도 로봇당 1~2회뿐이라 연산 부족도 아니다.

그러면 시간이 어디로 가는지 직접 봐야 한다. 궤적에 위치와 방향이 1초마다
남으므로, 한 칸씩 보고 세 가지로 가른다.

  주행    자리가 눈에 띄게 바뀜
  회전    자리는 그대로인데 방향만 바뀜 (제자리 회전은 거리를 못 번다)
  정지    둘 다 그대로 (조난자 조준·계획 대기 등)

구역이 좁아지면 목표가 가까워지고(실측 12.1m -> 9.9m) 짧게 가다 서고 도는
일이 잦아진다. 그 몫이 실제로 얼마인지 이 분해가 답한다.
"""
import math
import os
import re
import sys
from collections import defaultdict

RE_T = re.compile(r'\[(\d{10})\.')
RE_R = re.compile(r'\[(ugv\d)\.')
RE_TR = re.compile(r'\[궤적\] \((-?\d+\.\d+),(-?\d+\.\d+)\) yaw=(-?\d+\.\d+)')

MOVE_M = 0.05        # 1초에 이만큼 넘게 움직이면 주행
TURN_RAD = 0.05      # 1초에 이만큼 넘게 돌면 회전
JUMP_M = 5.0         # TF 튐은 버린다
WINDOW = 600         # 앞 10분만 본다


def budget(path):
    t0 = None
    last = {}
    out = defaultdict(lambda: [0, 0, 0])     # robot -> [주행, 회전, 정지]
    for line in open(path, encoding='utf-8', errors='replace'):
        mt, mr, mp = RE_T.search(line), RE_R.search(line), RE_TR.search(line)
        if not (mt and mr and mp):
            continue
        t = int(mt.group(1))
        t0 = t0 if t0 is not None else t
        if t - t0 > WINDOW:
            break
        r = mr.group(1)
        x, y, yaw = (float(mp.group(1)), float(mp.group(2)),
                     float(mp.group(3)))
        if r in last:
            px, py, pyaw, pt = last[r]
            dt = t - pt
            if 0 < dt <= 3:
                d = math.hypot(x - px, y - py)
                dyaw = abs((yaw - pyaw + math.pi) % (2 * math.pi) - math.pi)
                if d < JUMP_M:
                    if d > MOVE_M * dt:
                        out[r][0] += dt
                    elif dyaw > TURN_RAD * dt:
                        out[r][1] += dt
                    else:
                        out[r][2] += dt
        last[r] = (x, y, yaw, t)
    return out


def main(paths):
    tot = [0, 0, 0]
    n_robot = 0
    for p in paths:
        if not os.path.exists(p):
            continue
        for r, (drive, turn, stop) in budget(p).items():
            if drive + turn + stop < 60:      # 너무 짧은 로그는 뺀다
                continue
            tot[0] += drive
            tot[1] += turn
            tot[2] += stop
            n_robot += 1
    s = sum(tot)
    if not s:
        print('궤적이 있는 로그가 없다')
        return
    print(f'로봇 {n_robot}대분 · 앞 {WINDOW // 60}분')
    for label, v in zip(('주행', '회전', '정지'), tot):
        print(f'  {label} {100 * v / s:5.1f}%')


if __name__ == '__main__':
    args = []
    for a in sys.argv[1:]:
        if a.endswith('.txt'):
            args += [l.strip() for l in open(a, encoding='utf-8') if l.strip()]
        else:
            args.append(a)
    main(args)
