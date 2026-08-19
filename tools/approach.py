#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""특정 자리에 로봇이 실제로 얼마나 바싹 갔는지 잰다.

    python3 tools/approach.py <x> <y> <로그...>

왜 필요한가
-----------
'찾았다/못 찾았다' 는 런당 한 번뿐인 이분값이라 둔하다. lying_s2 는 런당
18% 로 놓치는데, 이 정도를 이분값으로 재려면 조건당 수십 런이 필요하다.

탐사를 얼마나 꼼꼼하게 만들었는지는 훨씬 예민하게 잴 수 있다 — 로봇이 그
자리에 얼마나 가까이, 얼마나 오래 있었는지는 런마다 연속값으로 나온다.
탐사 설정을 바꿨을 때 이 값이 안 움직이면 발견률도 안 움직인다.

궤적 계측(1Hz)이 있어야 돌아간다.

읽는 법
-------
  최근접   그 런에서 가장 가까이 간 거리[m]
  3m체류   3m 안에 머문 시간[s]
  5m체류   5m 안에 머문 시간[s]
"""
import math
import os
import re
import sys

RE_T = re.compile(r'\[(\d{10}\.\d+)\]')
RE_TRACE = re.compile(r'\[궤적\] \((-?\d+\.\d+),(-?\d+\.\d+)\)')


def scan(path, vx, vy):
    near = float('inf')
    d3 = d5 = 0
    n = 0
    with open(path, encoding='utf-8', errors='replace') as fh:
        for line in fh:
            m = RE_TRACE.search(line)
            if not m:
                continue
            n += 1
            d = math.hypot(float(m.group(1)) - vx, float(m.group(2)) - vy)
            near = min(near, d)
            if d <= 3.0:
                d3 += 1
            if d <= 5.0:
                d5 += 1
    return near, d3, d5, n


def main(vx, vy, paths):
    rows = []
    for p in paths:
        if not os.path.exists(p):
            continue
        near, d3, d5, n = scan(p, vx, vy)
        if n == 0:
            print(f'{os.path.basename(p):<16} 궤적 계측 없음 — 건너뜀')
            continue
        rows.append((os.path.basename(p)[:-4], near, d3, d5))
    if not rows:
        raise SystemExit('궤적이 있는 로그가 없다')
    print(f'({vx:.1f}, {vy:.1f}) 접근 정도 — {len(rows)}런')
    print(f'{"런":<16}{"최근접":>8}{"3m체류":>8}{"5m체류":>8}')
    print('-' * 42)
    for name, near, d3, d5 in rows:
        print(f'{name:<16}{near:>8.1f}{d3:>8}{d5:>8}')
    print('-' * 42)

    def med(v):
        v = sorted(v)
        return v[len(v) // 2]
    print(f'{"중앙값":<16}{med([r[1] for r in rows]):>8.1f}'
          f'{med([r[2] for r in rows]):>8}{med([r[3] for r in rows]):>8}')
    got3 = sum(1 for r in rows if r[2] > 0)
    print(f'\n3m 안까지 간 런 {got3}/{len(rows)}')


if __name__ == '__main__':
    if len(sys.argv) < 4:
        raise SystemExit(__doc__)
    args = []
    for a in sys.argv[3:]:
        if a.endswith('.txt'):
            with open(a, encoding='utf-8') as fh:
                args += [ln.strip() for ln in fh if ln.strip()]
        else:
            args.append(a)
    main(float(sys.argv[1]), float(sys.argv[2]), args)
