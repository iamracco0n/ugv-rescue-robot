#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""누운 조난자 검출까지 걸린 시간을 잰다(집중 시험용).

    python3 focus_check.py <로그...>

왜 시간을 재나
--------------
로봇을 같은 방(Room D)에서 시작시키면 반드시 마주치므로 양쪽 조건 다
결국 찾는다. 찾았나/못찾았나로만 보면 100% 대 100% 가 되어 아무것도
구분하지 못한다.

관문을 풀면 '더 흐릿하게 보여도 통과' 하므로, 효과가 있다면 **더 일찍**
잡혀야 한다. 시간은 연속값이라 이분법보다 정보가 훨씬 많고, 런 몇 개로도
차이가 드러난다.

기준점은 첫 탐사 목표 발행 시각이다. 노드 기동 시간(가제보 로딩, Nav2
lifecycle, YOLO/CUDA 적재)이 머신마다 달라서 로그 첫 줄을 쓰면 그 차이가
섞인다. 로봇이 실제로 움직이기 시작한 시점부터 재야 공정하다.
"""
import math
import re
import sys

STAMP = re.compile(r'\[(1[0-9]{9})\.([0-9]+)\]')
GOAL = re.compile(r'탐사 목표')
LOG = re.compile(r'\[구조 로그\].*?(L[0-9]):.*?위치:\(([^,]+),([^)]+)\)')

# 누운 사람 몸 중심. 발밑 원점(10,-7)이 아니라 중심을 써야 한다.
LX, LY = 9.23, -7.42
MATCH_R = 3.0


def t_of(line):
    m = STAMP.search(line)
    if not m:
        return None
    return float(m.group(1)) + float('0.' + m.group(2))


def scan(path):
    t0 = None
    found = None
    try:
        f = open(path, encoding='utf-8', errors='replace')
    except OSError:
        return None
    with f:
        for line in f:
            if t0 is None and GOAL.search(line):
                t0 = t_of(line)
            g = LOG.search(line)
            if g and found is None:
                x, y = float(g.group(2)), float(g.group(3))
                if math.hypot(x - LX, y - LY) <= MATCH_R:
                    found = (t_of(line), g.group(1), math.hypot(x - LX, y - LY))
    if t0 is None:
        return ('기동실패', None, None)
    if found is None:
        return ('못찾음', None, None)
    return (found[0] - t0, found[1], found[2])


def main():
    print('누운 조난자 검출까지 걸린 시간 (첫 탐사 목표 시각 기준)')
    print(f'{"로그":<16}{"시간":>10}{"등급":>8}{"오차m":>8}')
    print('-' * 44)
    groups = {}
    for p in sys.argv[1:]:
        r = scan(p)
        if r is None:
            continue
        name = p.rsplit('/', 1)[-1].replace('.log', '')
        cond = 'fix' if 'fix' in name else 'base'
        dt, lv, err = r
        if isinstance(dt, str):
            print(f'{name:<16}{dt:>10}{"":>8}{"":>8}')
            groups.setdefault(cond, []).append(None)
        else:
            print(f'{name:<16}{dt:>10.0f}s{lv:>7}{err:>8.2f}')
            groups.setdefault(cond, []).append(dt)
    print('-' * 44)
    for cond in ('base', 'fix'):
        vals = [v for v in groups.get(cond, []) if v is not None]
        n = len(groups.get(cond, []))
        if not vals:
            print(f'{cond:<8} 유효 0 / {n}')
            continue
        vals.sort()
        med = vals[len(vals) // 2]
        print(f'{cond:<8} n={len(vals)}/{n}  중앙값 {med:.0f}s  '
              f'평균 {sum(vals)/len(vals):.0f}s  최소 {vals[0]:.0f}s  '
              f'최대 {vals[-1]:.0f}s')
    print('\n관문을 풀면 더 흐릿해도 통과하므로, 효과가 있다면 더 짧아야 한다.')


if __name__ == '__main__':
    main()
