#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Nav2 가 목표를 포기한 뒤, 순찰기가 알아채기까지 로봇이 서 있는 시간.

    python3 tools/dead_time.py <대수> <로그...>

무엇을 재나
-----------
순찰기는 목표를 `/goal_pose` **토픽**으로 던진다. 액션 클라이언트가 아니라서
Nav2 의 결과를 못 받았다. 그래서 Nav2 가 복구행동(backup·spin·wait)을 다 쓰고
`navigate_to_pose Aborting` 으로 손을 든 뒤에도, 순찰기는 자기 제한시간
(거리 비례, 최대 150초)이 다 흐를 때까지 그 목표를 붙들고 있었다.

그동안 로봇은 아무 데도 안 간다. 박힘 감지도 이 구간은 못 잡는다 — Nav2 가
속도를 안 내고 있으므로 `stuck_decision` 이 '일부러 선 것' 으로 보고 넘긴다
(복구행동 중에는 옳은 판단이다). 빠져나올 길이 타임아웃 하나뿐이었다.

이 스크립트는 그 죽은 시간을 잰다. Nav2 의 포기 시각과, 그 로봇의 순찰기가
다음 행동을 한 시각의 차다.

읽는 법
-------
`nav_abort_react` 를 켜기 전 실측:

    구성   포기(런당)  회당 대기   런당 누적
    2대       2.7회      60초       2.7분
    3대       5.9회     101초       9.9분

45분 런에서 3대가 서 있기만 한 시간이 9.9분이었다. 고친 뒤에는 회당 대기가
순찰기 tick 주기까지 내려가야 한다 — 그게 이 수정의 합격 기준이다.

주의
----
포기 횟수는 반드시 `bt_navigator` 의 `Aborting handle` 로 센다. 액션 상태
토픽의 `ABORTED` 로 세면 안 된다 — Nav2 는 우리가 목표를 갈아끼울 때도 밀려난
목표를 `ABORTED` 로 끝내므로 실제보다 부풀려진다.
"""
import os
import re
import sys

ABORT = r'\[(\d+)\.\d+\]\s*\[ugv%d\.bt_navigator\].*\[navigate_to_pose\].*Aborting'
# 순찰기가 다음 행동으로 넘어간 순간 = 실패를 인지했거나 새 목표를 던진 때
NOTICE = (r'\[(\d+)\.\d+\]\s*\[ugv%d\.patrol_navigator\].*'
          r'(?:도달 실패|포기 통보|탐사 목표 →|조난자 후보|순찰 재개|후진 탈출)')


def per_log(path, n):
    """로그 하나에서 (포기 → 인지) 간격들."""
    with open(path, encoding='utf-8', errors='replace') as fh:
        lines = fh.read().splitlines()
    out = []
    for i in range(1, n + 1):
        ab = re.compile(ABORT % i)
        no = re.compile(NOTICE % i)
        aborts, notices = [], []
        for ln in lines:
            m = ab.search(ln)
            if m:
                aborts.append(int(m.group(1)))
                continue
            m = no.search(ln)
            if m:
                notices.append(int(m.group(1)))
        for t in aborts:
            nxt = [u for u in notices if u >= t]
            if nxt:
                out.append(nxt[0] - t)
    return out


def main(n, paths):
    gaps, nlog = [], 0
    for p in paths:
        if not os.path.exists(p):
            continue
        nlog += 1
        gaps += per_log(p, n)
    if not nlog:
        raise SystemExit('읽을 로그가 없다')
    if not gaps:
        print('%d대 · 로그 %d개 — Nav2 포기가 한 건도 없다' % (n, nlog))
        return
    gaps.sort()
    tot = sum(gaps)
    print('%d대 · 로그 %d개' % (n, nlog))
    print('Nav2 포기 %d회 · 런당 %.1f회' % (len(gaps), len(gaps) / nlog))
    print('죽은 시간  평균 %.0fs · 중앙 %ds · 최대 %ds'
          % (tot / len(gaps), gaps[len(gaps) // 2], gaps[-1]))
    print('런당 누적  %.1f분' % (tot / nlog / 60.0))
    over = sum(1 for g in gaps if g > 30)
    print('30s 넘게 서 있던 경우 %d회 (%.0f%%)'
          % (over, over / len(gaps) * 100))


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
