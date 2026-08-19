#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""조난자를 '몇 미터 앞에서' 처음 알아봤는지 잰다.

    python3 tools/first_det.py <로그...>

왜 필요한가
-----------
오탐 관문을 풀거나 조이면 바뀌는 것은 딱 하나다 — 사람이 화면에 들어왔을
때 통과시키느냐. 그런데 그 효과를 '찾았다/못 찾았다' 로 재면 동선 운에
묻힌다. 런당 18% 짜리 차이를 가르려면 수십 런이 필요하다.

관문이 풀리면 더 멀리서도 통과하므로 첫 검출 거리가 늘어난다. 이건 런마다
연속값으로 나와서 다섯 런으로도 움직임이 보인다. 관문 실험은 이 지표로
봐야 한다.

궤적 계측(1Hz)이 있어야 돌아간다.

읽는 법
-------
lying_s2 는 실측에서 5m 안으로 들어가야 처음 잡혔고, 나머지 여섯은 5m
밖에서 이미 잡혔다. 관문 완화가 통했다면 lying_s2 의 이 값이 커진다.
"""
import math
import os
import re
import sys
from collections import defaultdict

VICTIMS = {
    'lying_n3':    (-1.0 - 0.875 * math.cos(2.0),   15.5 - 0.875 * math.sin(2.0)),
    'lying_s2':    (-14.0 - 0.875 * math.cos(0.5), -16.0 - 0.875 * math.sin(0.5)),
    'lying_s4':    (9.0 - 0.875 * math.cos(0.3),   -14.0 - 0.875 * math.sin(0.3)),
    'standing_n1': (-22.0, 16.0),
    'occluded_s1': (-23.5, -16.0),
    'sitting_n4':  (12.0, 10.0),
    'corridor_e':  (24.0, 0.5),
}
FOUND = 3.0
MAX_DT = 3.0

RE_T = re.compile(r'\[(\d{10}\.\d+)\]')
RE_ROBOT = re.compile(r'\[(ugv\d)\.')
RE_TRACE = re.compile(r'\[궤적\] \((-?\d+\.\d+),(-?\d+\.\d+)\)')
RE_CAND = re.compile(r'후보 발견 \((-?\d+\.?\d*),(-?\d+\.?\d*)\)')


def scan(path):
    """로그 하나 → 조난자별 첫 검출 시 로봇-조난자 거리."""
    traces = defaultdict(list)
    out = {}
    pending = []
    with open(path, encoding='utf-8', errors='replace') as fh:
        for line in fh:
            mt = RE_T.search(line)
            mr = RE_ROBOT.search(line)
            if not mt or not mr:
                continue
            t, rob = float(mt.group(1)), mr.group(1)
            mtr = RE_TRACE.search(line)
            if mtr:
                traces[rob].append((t, float(mtr.group(1)), float(mtr.group(2))))
                continue
            mc = RE_CAND.search(line)
            if mc:
                cx, cy = float(mc.group(1)), float(mc.group(2))
                for name, (vx, vy) in VICTIMS.items():
                    if math.hypot(cx - vx, cy - vy) <= FOUND and name not in out:
                        pending.append((name, rob, t))
                        out[name] = None
                        break
    for name, rob, t in pending:
        best, gap = None, MAX_DT
        for tt, x, y in traces.get(rob, []):
            if abs(tt - t) <= gap:
                best, gap = (x, y), abs(tt - t)
        if best is None:
            out.pop(name, None)
            continue
        vx, vy = VICTIMS[name]
        out[name] = math.hypot(best[0] - vx, best[1] - vy)
    return {k: v for k, v in out.items() if v is not None}


def main(paths):
    per = defaultdict(list)
    runs = 0
    for p in paths:
        if not os.path.exists(p):
            continue
        got = scan(p)
        if not got:
            continue
        runs += 1
        for k, v in got.items():
            per[k].append(v)
    if not runs:
        raise SystemExit('궤적이 있는 로그가 없다')

    def med(v):
        v = sorted(v)
        return v[len(v) // 2]

    print(f'첫 검출 거리 ({runs}런)')
    print(f'{"조난자":<14}{"중앙값":>8}{"최소":>7}{"최대":>7}{"검출런":>8}')
    print('-' * 46)
    rows = [(med(v), k, min(v), max(v), len(v)) for k, v in per.items()]
    for m, k, lo, hi, n in sorted(rows):
        print(f'{k:<14}{m:>8.1f}{lo:>7.1f}{hi:>7.1f}{n:>6}/{runs}')
    print()
    print('관문을 풀었으면 이 값이 커져야 한다. 안 커지면 관문이 원인이 아니다.')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    args = []
    for a in sys.argv[1:]:
        if a.endswith('.txt'):
            with open(a, encoding='utf-8') as fh:
                args += [ln.strip() for ln in fh if ln.strip()]
        else:
            args.append(a)
    main(args)
