#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""큰 월드에서 어느 조난자를 못 찾는지 센다.

    python3 which_missed.py <로그...>

왜 이걸 보나
------------
2대 구성이 1500초 안에 7/7 을 절반만 달성한다. 마지막 1~2명을 못 찾는데,
그게 매번 같은 사람이면 특정 자리에 커버리지 구멍이 있는 것이고, 매번
다른 사람이면 그냥 시간이 모자란 것이다. 처방이 완전히 다르다.

정답 위치는 rescue_building_large.sdf 의 <pose> 에서 옮겼다. 누운 사람은
메쉬 원점이 발밑이라 몸 중심으로 보정한다 — 원점으로 채점하면 멀쩡한
추정이 1.4m 오차로 잡힌다(작은 맵에서 실제로 그렇게 착각했다).
"""
import math
import re
import sys

# (이름, x, y, 설명)
#
# 누운 사람은 메쉬 원점이 발밑이라 몸이 yaw '반대' 방향으로 약 1.75m 뻗는다.
# 정답은 몸 중심(원점 - 0.875*(cos yaw, sin yaw))으로 잡는다.
# 부호는 실측으로 확인했다 — 원점(-14,-16)/yaw 0.5 인 조난자의 검출 38건이
# 평균 (-14.77,-16.74), 즉 원점에서 (-0.77,-0.74) 였다. 예측값 (-0.77,-0.42)
# 와 x 가 정확히 맞는다. 부호를 반대로 잡으면 정답이 1.75m 어긋나 멀쩡한
# 추정이 오차로 잡힌다(작은 맵에서 실제로 그렇게 착각했다).
TRUTH = [
    ('standing_n1', -22.0,  16.0, '서있음 북서'),
    ('lying_n3',     -0.64, 14.70, '누움 북중앙'),    # 원점(-1,15.5) yaw 2.0
    ('lying_s2',    -14.77, -16.42, '누움 남서'),     # 원점(-14,-16) yaw 0.5
    ('sitting_n4',   12.0,  10.0, '앉음 북동(휠체어)'),
    ('lying_s4',      8.16, -14.26, '누움 남동'),     # 원점(9,-14) yaw 0.3
    ('occluded_s1', -23.5, -16.0, '서있음 남서구석(가려짐)'),
    ('lying_e',      23.13,  0.50, '누움 동쪽복도'),  # 원점(24,0.5) yaw 0
]
MATCH_R = 3.0

LOG = re.compile(r'\[구조 로그\].*?위치:\(([^,]+),([^)]+)\)')


def found_in(path):
    got = set()
    try:
        f = open(path, encoding='utf-8', errors='replace')
    except OSError:
        return None
    with f:
        for line in f:
            m = LOG.search(line)
            if not m:
                continue
            x, y = float(m.group(1)), float(m.group(2))
            for name, tx, ty, _ in TRUTH:
                if math.hypot(x - tx, y - ty) <= MATCH_R:
                    got.add(name)
                    break
    return got


def main():
    runs = 0
    tally = {t[0]: 0 for t in TRUTH}
    for p in sys.argv[1:]:
        got = found_in(p)
        if got is None:
            continue
        runs += 1
        for name in got:
            tally[name] += 1
    if not runs:
        print('로그 없음')
        return
    print(f'조난자별 발견률 ({runs}런)')
    print(f'{"조난자":<16}{"설명":<16}{"발견":>8}{"비율":>8}')
    print('-' * 50)
    for name, tx, ty, desc in sorted(TRUTH, key=lambda t: tally[t[0]]):
        n = tally[name]
        print(f'{name:<16}{desc:<16}{f"{n}/{runs}":>8}{n / runs:>8.0%}')
    print('\n매번 같은 사람을 놓치면 그 자리에 커버리지 구멍이 있는 것이고,')
    print('매번 다른 사람이면 시간이 모자란 것이다.')


if __name__ == '__main__':
    main()
