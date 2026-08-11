#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""고정 시간에 몇 명 찾았는지 센다.

    python3 victims_at.py <초> <로그...>

왜 이 지표인가
--------------
'전원 발견 시각' 은 조난자 7명 중 가장 늦게 찾은 한 명이 정한다. 최댓값
통계라 본질적으로 편차가 크다 — 실측으로 같은 조건에서 700초와 2672초가
나왔다. 알고리즘 차이보다 '마지막 한 명이 어느 방에 있었나' 가 더 크게
작용한다.

게다가 시간 안에 못 끝낸 런은 숫자가 아예 안 나와 표본에서 빠진다. 그러면
운 좋게 완주한 런만 비교하게 되어 편향이 생긴다(실측: 1대 5런 중 2런,
2대 6런 중 3런만 완주).

고정 시간에 센 인원수는 그 둘을 다 피한다. 모든 런이 값을 내고, 최댓값이
아니라 누적량이라 편차가 작다.

로봇마다 등록번호가 0 부터라 로그 줄을 그대로 세면 중복된다. 위치로 묶는다.
"""
import math
import re
import sys

STAMP = re.compile(r'\[(1[0-9]{9})\.')
LOG = re.compile(r'\[구조 로그\].*위치:\(([^,]+),([^)]+)\)')
MERGE_R = 2.2      # patrol_navigator 의 victim_merge_r 기본값


def count_at(path, limit_s):
    t0 = None
    uniq = []
    try:
        f = open(path, encoding='utf-8', errors='replace')
    except OSError:
        return None
    with f:
        for line in f:
            m = STAMP.search(line)
            if not m:
                continue
            t = int(m.group(1))
            if t0 is None:
                t0 = t
            if t - t0 > limit_s:
                break
            g = LOG.search(line)
            if not g:
                continue
            x, y = float(g.group(1)), float(g.group(2))
            if not any(math.hypot(x - a, y - b) < MERGE_R for a, b in uniq):
                uniq.append((x, y))
    return len(uniq)


def main():
    limit = int(sys.argv[1])
    print(f'{limit}초 시점의 발견 인원 (위치로 중복 제거)')
    print(f'{"로그":<16}{"인원":>6}')
    print('-' * 24)
    vals = []
    for p in sys.argv[2:]:
        n = count_at(p, limit)
        name = p.rsplit('/', 1)[-1].replace('.log', '')
        if n is None:
            print(f'{name:<16}{"없음":>6}')
        else:
            print(f'{name:<16}{n:>6}')
            vals.append(n)
    if vals:
        vals.sort()
        med = vals[len(vals) // 2]
        print('-' * 24)
        print(f'{"중앙값":<16}{med:>6}   (n={len(vals)}, 평균 {sum(vals)/len(vals):.1f})')


if __name__ == '__main__':
    main()
