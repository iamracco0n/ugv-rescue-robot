#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""조난자별로 '얼마나 오래 붙어 있어야 찾았나' 를 잰다.

    python3 tools/dwell.py <로그...>

왜 필요한가
-----------
발견률은 찾았다/못 찾았다 두 값뿐이라, 성공한 런에서는 아무 정보도 안 준다.
그런데 실패는 드물어서(런당 18%) 실패 사례만 기다리면 표본이 안 모인다.

대신 성공런에서도 여유는 잴 수 있다. 로봇이 5m 안에 1초만 있어도 찾는
조난자와, 40초를 붙어 있어야 겨우 찾는 조난자는 둘 다 '100% 발견' 으로
찍히지만 후자는 조건이 조금만 나빠지면 통째로 사라진다.

이 값이 유독 큰 대상이 다음에 놓칠 대상이다. 성공런만으로도 잡힌다.

읽는 법
-------
  체류(초)   첫 후보 검출까지 5m 안에 머문 시간. 클수록 빠듯하다
  헛체류     5m 안에 있었는데 끝내 후보를 못 만든 시간. 크면 시야각 문제다
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
NEAR = 5.0        # 검출이 가능한 거리대(실측 2.1~4.9m)를 덮는 반경
FOUND = 3.0       # 후보 좌표가 이 안이면 그 조난자를 본 것

RE_T = re.compile(r'\[(\d{10}\.\d+)\]')
RE_ROBOT = re.compile(r'\[(ugv\d)\.')
RE_TRACE = re.compile(r'\[궤적\] \((-?\d+\.\d+),(-?\d+\.\d+)\)')
RE_CAND = re.compile(r'후보 발견 \((-?\d+\.?\d*),(-?\d+\.?\d*)\)')


def parse(path):
    traces = defaultdict(list)
    first_seen = {}
    with open(path, encoding='utf-8', errors='replace') as fh:
        for line in fh:
            mt = RE_T.search(line)
            if not mt:
                continue
            t = float(mt.group(1))
            mtr = RE_TRACE.search(line)
            if mtr:
                mr = RE_ROBOT.search(line)
                if mr:
                    traces[mr.group(1)].append(
                        (t, float(mtr.group(1)), float(mtr.group(2))))
                continue
            mc = RE_CAND.search(line)
            if mc:
                cx, cy = float(mc.group(1)), float(mc.group(2))
                for name, (vx, vy) in VICTIMS.items():
                    if math.hypot(cx - vx, cy - vy) <= FOUND:
                        first_seen.setdefault(name, t)
                        break
    return traces, first_seen


def main(paths):
    dwell = defaultdict(list)
    wasted = defaultdict(list)
    for path in paths:
        if not os.path.exists(path):
            continue
        traces, first_seen = parse(path)
        if not traces:
            continue
        for name, (vx, vy) in VICTIMS.items():
            t_find = first_seen.get(name)
            before = 0
            after = 0
            for track in traces.values():
                for t, x, y in track:
                    if math.hypot(x - vx, y - vy) > NEAR:
                        continue
                    if t_find is None or t <= t_find:
                        before += 1
                    else:
                        after += 1
            if t_find is not None:
                dwell[name].append(before)
            else:
                wasted[name].append(before)

    def med(vals):
        vals = sorted(vals)
        return vals[len(vals) // 2] if vals else None

    print(f'조난자별 발견까지 붙어 있던 시간 ({len(paths)}런, 반경 {NEAR}m)')
    print(f'{"조난자":<14}{"체류(초)":>10}{"최대":>7}{"못찾은런":>10}')
    print('-' * 44)
    rows = []
    for name in VICTIMS:
        m = med(dwell[name])
        if m is None:
            rows.append((9999, name, '-', '-', len(wasted[name])))
        else:
            rows.append((m, name, m, max(dwell[name]), len(wasted[name])))
    for key, name, m, hi, miss in sorted(rows):
        print(f'{name:<14}{str(m):>10}{str(hi):>7}{miss:>8}')
    print()
    print('체류가 유독 큰 대상이 다음에 놓칠 대상이다 — 지금 100% 라도 그렇다.')


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
