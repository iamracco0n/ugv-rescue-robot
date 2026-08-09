#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""후보 안정성 판정(_candidate_stable) 규칙 검증.

    python3 tools/test_candidate_stable.py

유령 후보 한 건에 접근·정지·조준으로 최대 28초를 쓴다.
실측(큰 월드 57분): 유령 17건 / 진짜 등록 7건 — 20분 가까이 낭비.
거리로는 못 거른다(유령 중앙 3.0m vs 진짜 3.3m). 갈리는 것은
'얼마나 오래, 얼마나 한 자리에서' 보였는가다.

노드를 띄우지 않고 규칙만 떼어 확인한다.
"""
import math
import sys

TRIGGER_N   = 3
MIN_PERSIST = 1.0
CONSIST_R   = 0.60


def candidate_stable(track):
    """track = [(t, x, y), ...] → (통과, 사유)"""
    if len(track) < TRIGGER_N:
        return False, '표본부족'
    span = track[-1][0] - track[0][0]
    if span < MIN_PERSIST:
        return False, f'{span:.1f}s만 보임'
    xs = sorted(p[1] for p in track)
    ys = sorted(p[2] for p in track)
    mx, my = xs[len(xs) // 2], ys[len(ys) // 2]
    spread = max(math.hypot(x - mx, y - my) for _, x, y in track)
    if spread > CONSIST_R:
        return False, f'좌표 산포 {spread:.1f}m'
    return True, ''


def seq(n, dt, x, y, jitter=0.0):
    """n개 표본을 dt 간격으로. jitter 만큼 좌표를 흔든다."""
    out = []
    for i in range(n):
        s = jitter * (1 if i % 2 else -1)
        out.append((i * dt, x + s, y - s))
    return out


CASES = [
    # (설명, track, 기대)
    ('진짜 사람 — 1.4초 안정 관측',
     seq(12, 0.13, 12.0, 10.0, 0.05), True),
    ('진짜 사람 — 접근 중 약간 흔들림(0.2m)',
     seq(12, 0.13, 8.5, -14.0, 0.20), True),
    ('유령 — 0.2초 반짝(표본 3개, 14Hz)',
     seq(3, 0.07, 19.8, 4.5, 0.05), False),
    ('유령 — 오래 보이지만 좌표가 1.5m 튐',
     seq(12, 0.13, 19.8, 4.5, 1.50), False),
    ('유령 — 표본 2개뿐',
     seq(2, 0.5, 5.0, 5.0), False),
    ('경계 — 정확히 1.0초, 산포 0.5m → 통과',
     [(0.0, 0.0, 0.0), (0.5, 0.35, 0.35), (1.0, 0.0, 0.0)], True),
    ('경계 — 0.9초로 부족 → 기각',
     [(0.0, 0.0, 0.0), (0.45, 0.1, 0.1), (0.9, 0.0, 0.0)], False),
    ('경계 — 산포 0.7m 로 초과 → 기각',
     [(0.0, 0.0, 0.0), (0.5, 0.5, 0.5), (1.2, 0.0, 0.0)], False),
]


def main():
    print(f'{"사례":<40}{"기대":>7}{"결과":>7}  사유')
    bad = 0
    for name, track, want in CASES:
        got, why = candidate_stable(track)
        ok = got == want
        if not ok:
            bad += 1
        print(f'{name:<40}{"통과" if want else "기각":>7}'
              f'{"통과" if got else "기각":>7}  {why or "-"}'
              f'{"" if ok else "   ← 실패"}')
    print()
    if bad:
        print(f'실패 {bad}건')
        return 1
    print(f'{len(CASES)}개 사례 전부 통과')
    print('\n효과는 시뮬 채점(tools/run_eval.sh)의 유령 건수로 확인할 것.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
