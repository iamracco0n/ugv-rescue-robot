#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""로봇별 Nav2 경로 탐색 건강도를 센다.

    python3 tools/nav_health.py <대수> <로그...>

왜 로봇 단위로 보나
-------------------
XL 맵에서 3대가 2대보다 나빴다(45분 평균 12.08 → 11.35명). 런 단위 지표로는
'가끔 망한다' 까지만 보이고 왜 망하는지가 안 보인다.

로봇 단위로 펴 보면 갈린다. 3대에서는 **한 대가 혼자 무너지는** 런이 있다 —
그 로봇이 planner Abort 를 200회 넘게 내며 복구행동만 돌고, 담당 구역이 통째로
안 훑린다. 나머지 두 대는 멀쩡하다. 런 평균을 보면 이게 '전체가 조금씩 나쁨'
으로 뭉개진다.

'기회가 많아서' 와 구분하는 법
------------------------------
로봇이 3대면 사고 날 기회도 1.5배다. 그것뿐이라면 **로봇당** 발생률은 2대와
같아야 한다. 실측은 1.9% → 7.7% 로 4배였다. 즉 로봇을 늘리면 로봇 하나하나가
더 위험해진다(복도 하나를 셋이 나눠 쓰며 서로를 장애물로 본다).

그래서 런 비율이 아니라 **로봇당 발생률**을 같이 낸다. 런 비율만 보면
1-(1-p)^n 로 커지는 몫과 진짜 악화를 구분할 수 없다.
"""
import os
import re
import sys

# planner 가 이만큼 실패했으면 '루프에 빠졌다' 로 본다. 정상 런의 로봇은
# 수십 회에 머물고, 무너진 로봇은 200회를 넘는다 — 그 사이에 골 없는 골짜기가
# 있어서 경계값에 민감하지 않다.
THRESH = 100


def per_robot(path, n):
    """로그 하나에서 로봇별 planner Abort 횟수."""
    with open(path, encoding='utf-8', errors='replace') as fh:
        text = fh.read()
    return [len(re.findall(r'ugv%d\.planner_server.*Aborting' % i, text))
            for i in range(1, n + 1)]


def main(n, paths):
    rows = []
    for p in paths:
        if os.path.exists(p):
            rows.append((os.path.basename(p), per_robot(p, n)))
    if not rows:
        raise SystemExit('읽을 로그가 없다')

    counts = [c for _, cs in rows for c in cs]
    thrashed = sum(1 for c in counts if c >= THRESH)
    bad_runs = sum(1 for _, cs in rows if any(c >= THRESH for c in cs))

    print('%d대 · %d런 · 로봇 %d개' % (n, len(rows), len(counts)))
    print('로봇당 planner Abort  평균 %.1f · 최대 %d'
          % (sum(counts) / len(counts), max(counts)))
    print('실패 루프(Abort >= %d)  로봇 %d개 = 로봇당 %.1f%% · 해당 런 %d개 = %.0f%%'
          % (THRESH, thrashed, thrashed / len(counts) * 100,
             bad_runs, bad_runs / len(rows) * 100))

    worst = sorted(rows, key=lambda r: -max(r[1]))[:5]
    print('\n최악 5런 (로봇별 Abort)')
    for name, cs in worst:
        mark = '  <- 한 대만 무너짐' if max(cs) >= THRESH and sorted(cs)[-2] < THRESH else ''
        print('  %-16s %s%s' % (name, ' '.join('%4d' % c for c in cs), mark))


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
    main(int(sys.argv[1]), args)
