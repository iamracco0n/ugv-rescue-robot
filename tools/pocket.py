#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""방 안쪽 구석까지 실제로 들어갔는지 잰다.

    python3 tools/pocket.py <x0> <x1> <로그...>

왜 필요한가
-----------
lying_s2 는 방2 의 남서 구석에 있다. 방2 는 남쪽 다섯 방 중 **유일하게
내부 칸막이가 있는 방**이고(inner_s2, x=-9.3), 그 칸막이가 방을 가른 결과
문이 서쪽 칸의 구석에 붙어 버렸다. 로봇은 구석에서 들어와 15m 깊이의
주머니를 훑어야 한다.

같은 깊이(11~13m)인 다른 방들은 칸막이가 없어 문이 방 한가운데에 있다.

그래서 '방에 들어갔나' 가 아니라 '주머니 끝까지 내려갔나' 를 봐야 한다.
들어가긴 했는데 입구 근처만 돌고 나오면 조난자는 못 찾는다.

출력
----
    최남단  그 x 구간 안에서 로봇이 내려간 가장 남쪽 y
    도달    조난자 y(-16.4) 기준 5m 안까지 내려간 런
"""
import math
import os
import re
import sys

RE_ROBOT = re.compile(r'\[(ugv\d)\.')
RE_TRACE = re.compile(r'\[궤적\] \((-?\d+\.\d+),(-?\d+\.\d+)\)')
RE_CAND = re.compile(r'후보 발견 \((-?\d+\.?\d*),(-?\d+\.?\d*)\)')
VX, VY = -14.77, -16.42
FOUND = 3.0


def scan(path, x0, x1):
    south = 99.0
    found = False
    n = 0
    with open(path, encoding='utf-8', errors='replace') as fh:
        for line in fh:
            m = RE_TRACE.search(line)
            if m:
                x, y = float(m.group(1)), float(m.group(2))
                n += 1
                if x0 <= x <= x1:
                    south = min(south, y)
                continue
            c = RE_CAND.search(line)
            if c and math.hypot(float(c.group(1)) - VX,
                                float(c.group(2)) - VY) <= FOUND:
                found = True
    return (south, found) if n else None


def main(x0, x1, paths):
    rows = []
    for p in paths:
        if not os.path.exists(p):
            continue
        r = scan(p, x0, x1)
        if r:
            rows.append((os.path.basename(p)[:-4], r[0], r[1]))
    if not rows:
        raise SystemExit('궤적이 있는 로그가 없다')

    print(f'x {x0} ~ {x1} 구간 침투 깊이 — {len(rows)}런')
    print(f'{"런":<16}{"최남단 y":>10}{"발견":>7}')
    print('-' * 36)
    for name, s, f in sorted(rows, key=lambda r: r[1]):
        s_txt = '안 들어감' if s > 90 else f'{s:.1f}'
        print(f'{name:<16}{s_txt:>10}{"O" if f else "X":>6}')
    print('-' * 36)

    deep = [r for r in rows if r[1] <= VY + 5.0]
    shallow = [r for r in rows if r[1] > VY + 5.0]
    df = sum(1 for r in deep if r[2])
    sf = sum(1 for r in shallow if r[2])
    print(f'조난자 5m 안까지 내려간 런  {len(deep):>3}런 중 발견 {df:>3}런')
    print(f'거기까지 못 내려간 런       {len(shallow):>3}런 중 발견 {sf:>3}런')
    print()
    print('못 내려간 런이 곧 못 찾은 런이면, 고칠 곳은 검출이 아니라 이 주머니다.')


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
