#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""시점별로 '그때까지 몇 명 찾았는지' 센다(정답 파일 기준).

    python3 tools/found_at.py <truth.json> <로그...>

왜 시점별로 보나
----------------
XL 맵(조난자 13명, 45분)에서 1·2·3대가 모두 12명을 찾았다. 대수 차이가
안 보이는데, 원인이 둘 중 어느 쪽인지 이 지표가 가른다.

  (a) 시간이 남아돈다   -> 이른 시점에서는 대수가 갈린다.
                          1대가 30분 걸릴 것을 3대는 12분에 끝낸다
  (b) 마지막 한둘이 어렵다 -> 어느 시점에서도 안 갈린다.
                          대수와 무관한 병목이다

완주율은 이걸 못 가른다 — 13명 전원은 드물어서 대부분 0 이다. 시점별
인원은 모든 런이 값을 내고, 한 런에서 여러 시점을 뽑으므로 표본도 아낀다.

시각은 시뮬 시간이다(로그 타임스탬프, use_sim_time).
"""
import json
import math
import os
import re
import sys

RE_T = re.compile(r'\[(\d{10}\.\d+)\]')
RE_CAND = re.compile(r'후보 발견 \((-?\d+\.?\d*),(-?\d+\.?\d*)\)')
NEAR = 3.0
MARKS = (600, 1200, 1800, 2700)      # 10·20·30·45분


def load_truth(path):
    d = json.load(open(path, encoding='utf-8'))
    return [(v['name'], v['x'], v['y']) for v in d['victims']]


def scan(path, truth):
    """시점별 누적 발견 인원."""
    t0 = None
    first = {}
    with open(path, encoding='utf-8', errors='replace') as fh:
        for line in fh:
            mt = RE_T.search(line)
            if not mt:
                continue
            t = float(mt.group(1))
            if t0 is None:
                t0 = t
            mc = RE_CAND.search(line)
            if not mc:
                continue
            x, y = float(mc.group(1)), float(mc.group(2))
            for name, vx, vy in truth:
                if math.hypot(x - vx, y - vy) <= NEAR and name not in first:
                    first[name] = t - t0
                    break
    return [sum(1 for v in first.values() if v <= m) for m in MARKS]


def main(truth_path, paths):
    truth = load_truth(truth_path)
    rows = []
    for p in paths:
        if os.path.exists(p):
            rows.append(scan(p, truth))
    if not rows:
        raise SystemExit('읽을 로그가 없다')

    def med(vals):
        vals = sorted(vals)
        return vals[len(vals) // 2]

    print(f'정답 {len(truth)}명 · {len(rows)}런')
    head = ''.join(f'{m // 60:>7}분' for m in MARKS)
    print(f'{"":<10}{head}')
    print('-' * (10 + 8 * len(MARKS)))
    line = ''.join(f'{med([r[i] for r in rows]):>8}' for i in range(len(MARKS)))
    print(f'{"중앙값":<10}{line}')


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
