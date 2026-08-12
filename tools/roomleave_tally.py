#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""방 이탈·재진입 집계.

    python3 roomleave_tally.py <로그...>

왜 시간대를 가르나
------------------
수색 초반에는 지도가 통째로 미관측이라, 방에 들어가자마자 나와도 '크게
남기고 나감' 으로 잡힌다. 그때 나가는 건 오히려 정상이다 — 아직 못 가본
방이 널렸으니까. 실제로 1차 계측에서 이탈당 평균 50m^2 가 나왔는데,
그 값은 초반이 끌어올린 것이었다.

고칠 대상이 '이탈 전부' 인지 '후반 몇 건' 인지에 따라 규칙 설계가 달라진다.
무조건 방을 다 훑게 만들면 예전 고장으로 돌아간다 — 반경 필터를 썼을 때
방 하나를 1.4~4.3m 잔걸음으로 갉아먹으며 나가질 못했다.

왜 재진입을 따로 세나
---------------------
덜 보고 나왔다가 되돌아오는 왕복이 진짜 낭비다. 한 번에 끝냈으면 안 했을
이동이기 때문이다. '한번에 방 다 보고 다시 안 들어오기' 가 목표라면
이 숫자가 직접적인 지표다.
"""
import re
import sys

LEAVE = re.compile(
    r'\[(ugv\d)\.[^\]]*\].*\[방 이탈\] ([0-9.]+)s — 미관측 ([0-9.]+)m')
REENTER = re.compile(r'\[(ugv\d)\.[^\]]*\].*\[방 재진입\] ([0-9.]+)s')
# 로봇 이름이 없는 1대 런도 받는다
LEAVE1 = re.compile(r'\[방 이탈\] ([0-9.]+)s — 미관측 ([0-9.]+)m')
REENTER1 = re.compile(r'\[방 재진입\] ([0-9.]+)s')

EARLY = 400.0      # 이 앞은 '초반' 으로 본다(지도가 아직 비어 있는 구간)


def parse(path):
    leaves, reenters = [], []
    try:
        f = open(path, encoding='utf-8', errors='replace')
    except OSError:
        return None
    with f:
        for line in f:
            m = LEAVE.search(line)
            if m:
                leaves.append((m.group(1), float(m.group(2)), float(m.group(3))))
                continue
            m = REENTER.search(line)
            if m:
                reenters.append((m.group(1), float(m.group(2))))
                continue
            m = LEAVE1.search(line)
            if m:
                leaves.append(('ugv', float(m.group(1)), float(m.group(2))))
                continue
            m = REENTER1.search(line)
            if m:
                reenters.append(('ugv', float(m.group(1))))
    return leaves, reenters


def main():
    print(f'방 이탈·재진입 (초반 = {EARLY:.0f}s 이전)')
    print(f'{"로그":<10}{"이탈":>6}{"초반":>6}{"후반":>6}'
          f'{"후반면적":>10}{"재진입":>8}')
    print('-' * 46)
    tot = [0, 0, 0, 0.0, 0]
    runs = 0
    for p in sys.argv[1:]:
        r = parse(p)
        if r is None:
            continue
        leaves, reenters = r
        runs += 1
        early = sum(1 for _, t, _ in leaves if t < EARLY)
        late = [a for _, t, a in leaves if t >= EARLY]
        name = p.rsplit('/', 1)[-1].replace('.log', '')
        print(f'{name:<10}{len(leaves):>6}{early:>6}{len(late):>6}'
              f'{sum(late):>10.0f}{len(reenters):>8}')
        tot[0] += len(leaves)
        tot[1] += early
        tot[2] += len(late)
        tot[3] += sum(late)
        tot[4] += len(reenters)
    if not runs:
        return
    print('-' * 46)
    print(f'{"런당 평균":<10}{tot[0]/runs:>6.1f}{tot[1]/runs:>6.1f}'
          f'{tot[2]/runs:>6.1f}{tot[3]/runs:>10.0f}{tot[4]/runs:>8.1f}')
    if tot[2]:
        print(f'\n후반 이탈당 평균 남긴 면적: {tot[3]/tot[2]:.1f}m²')
    print('\n후반 이탈과 재진입이 고칠 대상이다. 초반 이탈은 정상 동작이다.')


if __name__ == '__main__':
    main()
