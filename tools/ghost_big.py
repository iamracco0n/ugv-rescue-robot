#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""큰 월드에서 유령(정답 조난자에 안 붙는 검출) 건수.

    python3 ghost_big.py <로그...>

which_missed.py 와 같은 정답표를 쓴다. lying_check.py 는 작은 맵(3명)용이라
큰 월드에 쓰면 조난자 7명이 전부 '정답 밖' 으로 잡혀 유령이 폭증한 것처럼
보인다 — 실제로 그렇게 잘못 셀 뻔했다.
"""
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from which_missed import TRUTH, MATCH_R          # noqa: E402

LOG = re.compile(r'\[구조 로그\].*?위치:\(([^,]+),([^)]+)\)')


def main():
    runs = 0
    ghosts = 0
    spots = []
    for p in sys.argv[1:]:
        try:
            f = open(p, encoding='utf-8', errors='replace')
        except OSError:
            continue
        runs += 1
        with f:
            for line in f:
                m = LOG.search(line)
                if not m:
                    continue
                x, y = float(m.group(1)), float(m.group(2))
                if any(math.hypot(x - tx, y - ty) <= MATCH_R
                       for _, tx, ty, _ in TRUTH):
                    continue
                ghosts += 1
                spots.append((round(x, 1), round(y, 1)))
    print(f'유령 검출 {ghosts}건 / {runs}런  (런당 {ghosts / max(runs, 1):.1f})')
    if spots:
        print('  자리:', ' '.join(f'({x},{y})' for x, y in spots[:8]))


if __name__ == '__main__':
    main()
