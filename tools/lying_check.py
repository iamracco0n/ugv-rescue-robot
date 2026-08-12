#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""작은 맵에서 누운 조난자를 제대로 찾는지 검사한다.

    python3 lying_check.py <로그...>

왜 만드나
---------
작은 맵 조난자 3명 중 하나(person_l1_critical)는 Standing person 메쉬를
pitch -90도 돌려 바닥에 눕힌 것이다. 실제 구조 현장에서 가장 흔한 자세인데
지금까지 따로 재본 적이 없다.

과거 기록(patient_locations.txt)을 보면 탐지 자체는 된다. 문제는 다른 데
있었다.

  · 누운 사람: 등급은 L1 로 맞는데 위치가 (10,-7) 대신 (8.8,-7.5) 로
    약 1.3m 어긋난다. 서있는 사람들이 산포 0.01~0.03m 로 정확한 것과 대비된다.
    누우면 몸이 가로로 길어져 중심 추정이 밀리는 것으로 보인다.
  · 서있는 여성(6,1)은 위치는 맞는데 등급이 L2 여야 할 것이 L3 로 나오고,
    어떤 런에서는 Main Hall 에 없는 L1 이 찍혔다.

그래서 '찾았나' 가 아니라 '얼마나 정확히, 등급까지 맞게 찾았나' 를 센다.
찾은 것만 세면 1.3m 어긋난 것도 성공으로 잡혀 문제가 안 보인다.
"""
import math
import re
import sys

# 정답표 — rescue_building.sdf 의 <pose> 에서 그대로 옮겼다
#
# 등급은 자세로 정해진다 (yolo_pose_node.classify)
#
#   서있음 → L3 정상
#   앉음   → L2 도움필요   (휠체어 등, 스스로 못 걸음)
#   누움   → L1 긴급
#
# person_l2_urgent 는 이름이 L2 지만 실제로는 '서 있는' Casual female 이다.
# 서있는 사람에게서 L3 말고 다른 값이 나올 수가 없고, 그게 맞는 동작이다.
# 개체 이름을 정답으로 삼으면 멀쩡한 동작이 결함으로 잡히므로 None 을 준다.
# (작은 맵에는 앉은 사람이 아예 없어서 L2 는 검사 대상이 아니다.)
# 위치 정답은 '몸 중심' 이어야 한다. 비전이 내놓는 값이 몸 중심이기 때문이다.
#
# SDF <pose> 는 메쉬 원점이고 Standing person 메쉬의 원점은 발밑이다.
# 서 있으면 발과 몸 중심이 지도상 거의 같은 (x,y) 라 원점을 그냥 써도 된다.
# 그런데 누운 사람은 pitch -90 도로 눕혀 몸이 원점에서 옆으로 뻗는다.
#
#   <pose>10 -7 0.15  0 -1.5708 0.5</pose>
#   키 1.75m -> 몸 중심은 원점에서 약 0.875m, 방향은 yaw 0.5rad
#   10 - 0.875*cos(0.5) = 9.23,  -7 - 0.875*sin(0.5) = -7.42
#
# 원점(10,-7)을 정답으로 놓고 채점하면 멀쩡한 추정이 1.4m 오차로 잡힌다.
# 실제로 그렇게 재서 '위치 정확도 문제' 로 보고할 뻔했다.
TRUTH = [
    ('person_l3_normal',   -9.0,  7.0, 'L3', '서있음'),
    ('person_l2_urgent',    6.0,  1.0, None, '서있음(등급 판정불가)'),
    ('person_l1_critical',  9.23, -7.42, 'L1', '누움'),
]
MATCH_R = 3.0      # 이 반경 안이면 그 사람을 가리킨 것으로 본다

LOG = re.compile(r'\[구조 로그\].*?(L[0-9]):.*?위치:\(([^,]+),([^)]+)\)')


def check(path):
    """로그 하나에서 사람별 최선의 검출을 뽑는다."""
    best = {t[0]: None for t in TRUTH}
    try:
        f = open(path, encoding='utf-8', errors='replace')
    except OSError:
        return None
    extra = 0
    with f:
        for line in f:
            m = LOG.search(line)
            if not m:
                continue
            lv, x, y = m.group(1), float(m.group(2)), float(m.group(3))
            # 가장 가까운 정답에 붙인다
            near, nd = None, 1e9
            for name, tx, ty, _, _ in TRUTH:
                d = math.hypot(x - tx, y - ty)
                if d < nd:
                    near, nd = name, d
            if nd > MATCH_R:
                extra += 1          # 아무에게도 안 붙는 검출 = 유령
                continue
            if best[near] is None or nd < best[near][0]:
                best[near] = (nd, lv)
    return best, extra


def main():
    print('누운 조난자 검사 — 위치 오차와 등급까지 본다')
    print(f'{"로그":<18}{"사람":<20}{"발견":>5}{"오차m":>8}{"등급":>10}')
    print('-' * 62)
    agg = {t[0]: [] for t in TRUTH}
    ghosts = 0
    runs = 0
    for p in sys.argv[1:]:
        r = check(p)
        if r is None:
            continue
        best, extra = r
        runs += 1
        ghosts += extra
        short = p.rsplit('/', 1)[-1].replace('.log', '')
        for name, tx, ty, want, posture in TRUTH:
            b = best[name]
            if b is None:
                print(f'{short:<18}{name+"("+posture+")":<20}{"✗":>5}{"":>8}{"":>10}')
                agg[name].append(None)
            else:
                d, lv = b
                if want is None:
                    mark = f'{lv}(판정외)'
                else:
                    mark = '맞음' if lv == want else f'{lv}(≠{want})'
                print(f'{short:<18}{name+"("+posture+")":<20}{"✓":>5}{d:>8.2f}{mark:>10}')
                agg[name].append((d, None if want is None else lv == want))
        print()

    if not runs:
        return
    print('=' * 62)
    print(f'{"사람":<24}{"발견률":>10}{"평균오차":>10}{"등급정확":>10}')
    print('-' * 62)
    for name, tx, ty, want, posture in TRUTH:
        got = [a for a in agg[name] if a]
        rate = f'{len(got)}/{runs}'
        if got:
            err = sum(g[0] for g in got) / len(got)
            if want is None:
                lvl = '판정외'
            else:
                lvl = f'{sum(1 for g in got if g[1])}/{len(got)}'
            print(f'{name+"("+posture+")":<24}{rate:>10}{err:>10.2f}{lvl:>10}')
        else:
            print(f'{name+"("+posture+")":<24}{rate:>10}{"-":>10}{"-":>10}')
    print(f'\n정답에 안 붙는 검출(유령): {ghosts}건 / {runs}런')


if __name__ == '__main__':
    main()
