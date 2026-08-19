#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""조난자를 '몇 번째로, 어느 로봇이, 언제' 찾았는지 본다.

    python3 tools/find_order.py <로그...>

왜 필요한가
-----------
한 조난자만 유독 못 찾을 때 원인은 두 갈래고 처방이 다르다.

    그 자리가 어렵다          -> 검출·시야각을 고친다
    그 자리가 늘 마지막이다    -> 시간이 모자란 것이다. 일감 배분을 고친다

늘 마지막에 찾는 대상은, 시간이 조금만 모자라면 그 하나만 빠진 채로 런이
끝난다. 발견률만 보면 '그 자리가 어렵다' 와 구분이 안 된다. 순서를 봐야
갈린다.

담당 로봇도 같이 본다. 2대는 구역을 나눠 맡으므로, 한 로봇이 조난자를 더
많이 맡으면 그 구역의 마지막 사람이 늘 밀린다.
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

RE_T = re.compile(r'\[(\d{10}\.\d+)\]')
RE_ROBOT = re.compile(r'\[(ugv\d)\.')
RE_CAND = re.compile(r'후보 발견 \((-?\d+\.?\d*),(-?\d+\.?\d*)\)')


def scan(path):
    t0 = None
    first = {}
    with open(path, encoding='utf-8', errors='replace') as fh:
        for line in fh:
            mt = RE_T.search(line)
            if not mt:
                continue
            t = float(mt.group(1))
            if t0 is None:
                t0 = t
            mc = RE_CAND.search(line)
            if not mc:
                continue
            mr = RE_ROBOT.search(line)
            cx, cy = float(mc.group(1)), float(mc.group(2))
            for name, (vx, vy) in VICTIMS.items():
                if math.hypot(cx - vx, cy - vy) <= FOUND and name not in first:
                    first[name] = (t - t0, mr.group(1) if mr else '?')
                    break
    return first


def main(paths):
    order_rank = defaultdict(list)
    times = defaultdict(list)
    robots = defaultdict(lambda: defaultdict(int))
    last_count = defaultdict(int)
    runs = 0
    for p in paths:
        if not os.path.exists(p):
            continue
        first = scan(p)
        if not first:
            continue
        runs += 1
        ranked = sorted(first.items(), key=lambda kv: kv[1][0])
        for i, (name, (t, rob)) in enumerate(ranked, 1):
            order_rank[name].append(i)
            times[name].append(t)
            robots[name][rob] += 1
        if ranked:
            last_count[ranked[-1][0]] += 1

    if not runs:
        raise SystemExit('로그를 못 읽었다')

    def med(v):
        v = sorted(v)
        return v[len(v) // 2]

    print(f'발견 순서·시각 ({runs}런)')
    print(f'{"조난자":<14}{"순서":>6}{"시각(초)":>10}{"꼴찌":>7}{"찾은 로봇":>12}')
    print('-' * 52)
    rows = []
    for name in VICTIMS:
        if not order_rank[name]:
            rows.append((99, name, '-', '-', 0, '-', 0))
            continue
        r = med(order_rank[name])
        t = med(times[name])
        who = robots[name]
        wtxt = ' '.join(f'{k}:{v}' for k, v in sorted(who.items()))
        rows.append((r, name, r, f'{t:.0f}', last_count[name], wtxt,
                     len(order_rank[name])))
    for key, name, r, t, lc, wtxt, n in sorted(rows):
        print(f'{name:<14}{str(r):>6}{t:>10}{lc:>5}/{n}{wtxt:>14}')
    print()
    print('순서가 늘 뒤쪽이고 꼴찌가 잦으면 시간 문제다 — 일감 배분을 봐야 한다.')


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
