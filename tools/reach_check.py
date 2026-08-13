#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""못 찾은 조난자 근처까지 로봇이 갔는지 본다.

    python3 reach_check.py <x> <y> <로그...>

왜 필요한가
-----------
'못 찾았다' 에는 두 가지가 섞여 있다.

  안 갔다   -> 커버리지 문제. 탐사를 고쳐야 한다
  갔는데 못 알아봤다 -> 인식 문제. 검출을 고쳐야 한다

처방이 정반대라 반드시 갈라야 한다. 탐사 목표 좌표로 로봇이 그 자리에
얼마나 가까이 갔는지 재면 갈린다.

주의: 목표는 '가려고 한 곳' 이지 '간 곳' 이 아니다. 목표를 냈지만 못 갔을
수도 있다. 그래서 이 값은 '최소한 이만큼은 가까이 가려 했다' 로 읽는다.
가까이 가려 한 적조차 없으면 커버리지 문제인 것은 확실하다.
"""
import math
import re
import sys

GOAL = re.compile(r'탐사 목표 → \(([^,]+),([^)]+)\)')
LOG = re.compile(r'\[구조 로그\].*?위치:\(([^,]+),([^)]+)\)')


def main():
    tx, ty = float(sys.argv[1]), float(sys.argv[2])
    print(f'대상 ({tx}, {ty}) 까지 얼마나 가까이 갔나')
    print(f'{"로그":<14}{"발견":>6}{"최근접목표":>12}{"목표수":>8}')
    print('-' * 42)
    miss_near, miss_far, hit = [], [], []
    for p in sys.argv[3:]:
        try:
            lines = open(p, encoding='utf-8', errors='replace').readlines()
        except OSError:
            continue
        best = 1e9
        n = 0
        found = False
        for line in lines:
            m = GOAL.search(line)
            if m:
                n += 1
                d = math.hypot(float(m.group(1)) - tx, float(m.group(2)) - ty)
                best = min(best, d)
            g = LOG.search(line)
            if g and math.hypot(float(g.group(1)) - tx,
                                float(g.group(2)) - ty) <= 3.0:
                found = True
        name = p.rsplit('/', 1)[-1].replace('.log', '')
        mark = 'O' if found else 'X'
        print(f'{name:<14}{mark:>6}{best:>12.1f}{n:>8}')
        if found:
            hit.append(best)
        elif best <= 5.0:
            miss_near.append(best)
        else:
            miss_far.append(best)
    print('-' * 42)
    print(f'발견        {len(hit)}런')
    print(f'못찾음·근접 {len(miss_near)}런  (5m 안까지 갔는데 못 알아봄 = 인식 문제)')
    print(f'못찾음·원거리 {len(miss_far)}런  (5m 안에 간 적 없음 = 커버리지 문제)')


if __name__ == '__main__':
    main()
