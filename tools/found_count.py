#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""런마다 '몇 명을 찾았는지' 센다(정답 파일 기준).

    python3 tools/found_count.py <truth.json> <로그...>

왜 완주율로 재면 안 되나
------------------------
조난자가 13명인 XL 맵에서는 45분 안에 전원 발견이 드물다. 실측 21런에서
완주는 1건뿐이었다. 그러면 완주율은 대부분 0 이라 대수 비교가 안 된다.

발견 인원은 모든 런이 값을 낸다. 최댓값 통계가 아니라 누적량이라 편차도
작다 — 예전에 '전원 발견까지 걸린 시간' 을 쓰다 같은 이유로 버렸다.

세는 방법
---------
target_manager 의 '후보 발견 (x,y)' 를 정답 좌표에 붙인다. 같은 사람을
여러 번 봐도 한 명으로 센다. 정답에서 3m 넘게 떨어진 검출은 유령으로
따로 센다.
"""
import json
import math
import os
import re
import sys

RE_CAND = re.compile(r'후보 발견 \((-?\d+\.?\d*),(-?\d+\.?\d*)\)')
RE_GOAL = re.compile(r'patrol_navigator.*탐사 목표')
NEAR = 3.0
# 이 횟수보다 목표를 적게 낸 런은 중간에 끊긴 것으로 보고 뺀다.
#
# 크래시가 나면 반쪽 로그가 남는다. 무효 판정(목표>5 이면서 키포인트=0)은
# 목표를 5회도 못 낸 로그를 안 걸러서, '발견 0명' 이 정상 런처럼 집계에
# 섞인다. 실제로 오로라가 GPU Xid 45 로 죽었을 때 그런 로그가 남았다.
MIN_GOALS = 5


def load_truth(path):
    d = json.load(open(path, encoding='utf-8'))
    return [(v['name'], v['x'], v['y']) for v in d['victims']]


def scan(path, truth):
    found = set()
    ghost = 0
    goals = 0
    with open(path, encoding='utf-8', errors='replace') as fh:
        for line in fh:
            if RE_GOAL.search(line):
                goals += 1
            m = RE_CAND.search(line)
            if not m:
                continue
            x, y = float(m.group(1)), float(m.group(2))
            hit = None
            for name, vx, vy in truth:
                if math.hypot(x - vx, y - vy) <= NEAR:
                    hit = name
                    break
            if hit:
                found.add(hit)
            else:
                ghost += 1
    if goals < MIN_GOALS:
        return None
    return found, ghost


def main(truth_path, paths):
    truth = load_truth(truth_path)
    print(f'정답 {len(truth)}명 기준')
    print(f'{"런":<16}{"발견":>6}{"유령":>6}')
    print('-' * 30)
    counts = []
    for p in paths:
        if not os.path.exists(p):
            continue
        res = scan(p, truth)
        if res is None:
            print(f'{os.path.basename(p)[:-4]:<16}  중간에 끊긴 로그 — 제외')
            continue
        found, ghost = res
        counts.append(len(found))
        print(f'{os.path.basename(p)[:-4]:<16}{len(found):>6}{ghost:>6}')
    if counts:
        s = sorted(counts)
        med = s[len(s) // 2]
        print('-' * 30)
        print(f'{"중앙값":<16}{med:>6}   (n={len(counts)})')


if __name__ == '__main__':
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    args = []
    for a in sys.argv[2:]:
        if a.endswith('.txt'):
            with open(a, encoding='utf-8') as fh:
                args += [ln.strip() for ln in fh if ln.strip()]
        else:
            args.append(a)
    main(sys.argv[1], args)
