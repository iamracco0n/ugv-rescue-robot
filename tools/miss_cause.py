#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""놓친 원인이 '안 감' 인지 '가고도 못 알아봄' 인지 런마다 가른다.

    python3 tools/miss_cause.py <x> <y> <로그...>

왜 필요한가
-----------
처방이 정반대라 이 둘은 반드시 갈라야 한다.

    안 갔다            -> 탐사를 고친다
    갔는데 못 알아봤다  -> 관문·시야각을 고친다

지금까지는 '탐사 목표' 좌표로 어림했는데, 목표는 가려던 곳이지 간 곳이
아니다. 1Hz 궤적이 붙은 뒤로는 실제로 간 곳을 쓸 수 있다.

이 표가 중요한 이유
-------------------
'가까이 간 런은 거의 다 찾는다' 면 관문을 아무리 풀어도 소용없다. 못 찾은
런에서 로봇이 애초에 거기 없었기 때문이다. 반대로 '가까이 갔는데도 자주
놓친다' 면 탐사를 더 꼼꼼히 해봐야 헛일이다.

출력
----
    접근  = 그 자리 5m 안까지 간 런
    발견  = 그 중 실제로 찾은 런
"""
import math
import os
import re
import sys

RE_TRACE = re.compile(r'\[궤적\] \((-?\d+\.\d+),(-?\d+\.\d+)\)')
RE_CAND = re.compile(r'후보 발견 \((-?\d+\.?\d*),(-?\d+\.?\d*)\)')
NEAR = 5.0
FOUND = 3.0


def scan(path, vx, vy):
    near = float('inf')
    found = False
    n = 0
    with open(path, encoding='utf-8', errors='replace') as fh:
        for line in fh:
            m = RE_TRACE.search(line)
            if m:
                n += 1
                d = math.hypot(float(m.group(1)) - vx, float(m.group(2)) - vy)
                near = min(near, d)
                continue
            c = RE_CAND.search(line)
            if c and math.hypot(float(c.group(1)) - vx,
                                float(c.group(2)) - vy) <= FOUND:
                found = True
    return (near, found) if n else None


def main(vx, vy, paths):
    rows = []
    for p in paths:
        if not os.path.exists(p):
            continue
        r = scan(p, vx, vy)
        if r:
            rows.append((os.path.basename(p)[:-4], r[0], r[1]))
    if not rows:
        raise SystemExit('궤적이 있는 로그가 없다')

    went = [r for r in rows if r[1] <= NEAR]
    away = [r for r in rows if r[1] > NEAR]
    wf = sum(1 for r in went if r[2])
    af = sum(1 for r in away if r[2])

    print(f'({vx:.1f}, {vy:.1f}) 놓친 원인 — {len(rows)}런')
    print()
    print(f'{NEAR:.0f}m 안까지 간 런   {len(went):>3}런 중 발견 {wf:>3}런'
          f'  ({100 * wf / len(went) if went else 0:.0f}%)')
    print(f'그보다 멀리서 끝난 런 {len(away):>3}런 중 발견 {af:>3}런'
          f'  ({100 * af / len(away) if away else 0:.0f}%)')
    print()
    miss_far = len(away) - af
    miss_near = len(went) - wf
    tot = miss_far + miss_near
    if tot == 0:
        print('놓친 런이 없다.')
        return
    print(f'놓친 {tot}런의 내역')
    print(f'  안 감           {miss_far:>3}런  -> 탐사 문제')
    print(f'  가고도 못 알아봄 {miss_near:>3}런  -> 인식 문제')
    print()
    print('많은 쪽이 진짜 원인이다. 적은 쪽을 고치면 시간만 쓴다.')


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
