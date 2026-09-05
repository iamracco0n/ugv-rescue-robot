#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""두 묶음의 런을 시점별로 비교한다 — 평균만이 아니라 오차와 p 까지.

    python3 tools/compare_runs.py <truth.json> --a <로그...> --b <로그...>
    LABEL_A=2대 LABEL_B=3대 python3 tools/compare_runs.py ...

왜 이 도구가 필요했나
---------------------
`found_at.py` 는 시점별 중앙값·평균을 낸다. 그것만 보고 결론을 내다가 두 번
헛짚었다.

  · 표본 8/10런일 때 '3대가 초·중반에서 앞선다' 로 읽었다.
    52/49런으로 늘리자 그 우세가 사라졌다(10분 +0.63 p=0.10 -> +0.42 p=0.17).
    표본이 늘수록 유의성에서 **멀어졌으니** 애초에 노이즈였다.

런당 흩어짐이 1.5~1.7명이라 0.2~0.6명 차이는 표본을 다시 뽑을 때마다 뒤집힌다.
그래서 평균 옆에 표준오차를 붙이고 Welch t 로 p 를 같이 낸다.

읽는 법
-------
p 가 크면 '차이가 없다' 가 아니라 **'이 표본으로는 못 가른다'** 는 뜻이다.
그리고 네 시점을 한꺼번에 검정하므로, p=0.03 같은 경계값은 다중비교를
감안해 조심해서 읽어야 한다(본페로니면 문턱이 0.0125 다).

파싱은 `found_at.py` 의 scan 을 그대로 쓴다. 같은 것을 두 번 구현하면
두 도구가 다른 답을 내는 사고가 난다.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import found_at                                          # noqa: E402


def stats(vals):
    """평균 · 표준편차 · 표준오차."""
    n = len(vals)
    if n == 0:
        return 0.0, 0.0, 0.0
    m = sum(vals) / n
    if n < 2:
        return m, 0.0, 0.0
    var = sum((v - m) ** 2 for v in vals) / (n - 1)
    return m, math.sqrt(var), math.sqrt(var / n)


def welch_p(a, b):
    """Welch t 의 양측 p. 자유도가 크므로 정규근사로 충분하다."""
    ma, _, sea = stats(a)
    mb, _, seb = stats(b)
    se = math.hypot(sea, seb)
    if se == 0:
        return 1.0
    t = (ma - mb) / se
    return 2.0 * (1.0 - 0.5 * (1.0 + math.erf(abs(t) / math.sqrt(2.0))))


def main():
    la = os.environ.get('LABEL_A', 'A')
    lb = os.environ.get('LABEL_B', 'B')
    truth = found_at.load_truth(sys.argv[1])
    args = sys.argv[2:]
    try:
        ia, ib = args.index('--a'), args.index('--b')
    except ValueError:
        raise SystemExit(__doc__)
    pa = [p for p in args[ia + 1:ib] if os.path.exists(p)]
    pb = [p for p in args[ib + 1:] if os.path.exists(p)]
    if not pa or not pb:
        raise SystemExit('양쪽 모두 로그가 있어야 한다')

    A = [found_at.scan(p, truth) for p in pa]
    B = [found_at.scan(p, truth) for p in pb]
    print('%s %d런 · %s %d런 · 정답 %d명\n' % (la, len(A), lb, len(B), len(truth)))
    print('%-5s %-18s %-18s %-8s %s'
          % ('시점', la + ' 평균±오차', lb + ' 평균±오차', '차이', '판정'))
    print('-' * 74)
    for k, mk in enumerate(found_at.MARKS):
        a = [r[k] for r in A]
        b = [r[k] for r in B]
        ma, _, sea = stats(a)
        mb, _, seb = stats(b)
        p = welch_p(a, b)
        d = mb - ma
        verdict = (lb + ' 우세' if (p < 0.05 and d > 0)
                   else la + ' 우세' if (p < 0.05 and d < 0)
                   else '못 가름')
        print('%-5s %6.2f ± %-9.2f %6.2f ± %-9.2f %+5.2f   %s (p=%.2f)'
              % ('%d분' % (mk // 60), ma, sea, mb, seb, d, verdict, p))


if __name__ == '__main__':
    if len(sys.argv) < 4:
        raise SystemExit(__doc__)
    main()
