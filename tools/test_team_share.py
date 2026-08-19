#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""팀 공유 규칙 단위 테스트 — 목표 선점과 조난자 명부 합산.

    python3 tools/test_team_share.py

왜 필요한가
-----------
2대가 지도를 공유해도 목표와 명부를 안 나누면 둘이 같은 구역으로 간다.
실측 로그:

    ugv2  탐사 목표 → (-0.4, 12.1)
    ugv1  탐사 목표 → ( 0.1, 11.9)     거의 같은 곳

그리고 같은 조난자를 둘이 각각 등록하면 실종자 수가 채워진 것처럼 보여
수색이 조기 종료된다. 1대에서 겪었던 중복 등록 사고와 같은 종류다.
"""
import ast
import math
import os
import sys

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'src', 'ugv_vision', 'ugv_vision',
                   'patrol_navigator.py')


def load(name):
    tree = ast.parse(open(SRC, encoding='utf-8').read())
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            mod = ast.Module(body=[node], type_ignores=[])
            ns = {'math': math}
            exec(compile(mod, SRC, 'exec'), ns)
            return ns[name]
    raise SystemExit(f'{SRC} 에 {name} 함수가 없다')


def main():
    claimed_by_peer = load('claimed_by_peer')
    count_unique_victims = load('count_unique_victims')
    fails = 0

    def check(desc, got, want):
        nonlocal fails
        ok = got == want
        print(f'{desc:<50}{"통과" if ok else "실패":>8}')
        if not ok:
            print(f'    got={got}  want={want}')
            fails += 1

    # ── 목표 선점 ───────────────────────────────────────────────
    peers = {'ugv2': (10.0, 10.0)}
    check('상대 목표 코앞은 선점됨',
          claimed_by_peer(10.5, 10.0, peers, 6.0), True)
    check('상대 목표에서 멀면 자유',
          claimed_by_peer(30.0, 10.0, peers, 6.0), False)
    check('반경 경계 바로 밖은 자유',
          claimed_by_peer(16.5, 10.0, peers, 6.0), False)
    check('상대가 목표를 안 잡았으면 항상 자유',
          claimed_by_peer(10.0, 10.0, {}, 6.0), False)
    # 상대가 여럿이면 하나라도 걸리면 선점
    check('상대가 여럿일 때 하나만 걸려도 선점',
          claimed_by_peer(10.0, 10.0,
                          {'a': (100.0, 100.0), 'b': (11.0, 10.0)}, 6.0), True)

    # ── 조난자 명부 합산 ────────────────────────────────────────
    # 두 로봇이 같은 사람을 각각 등록해도 하나로 세야 한다. 안 그러면
    # 실종자 수가 채워진 것처럼 보여 수색이 조기 종료된다.
    both = [('ugv1', 0, 12.0, 10.0), ('ugv2', 0, 12.3, 10.2)]
    check('두 로봇이 본 같은 사람은 1명', count_unique_victims(both, 1.5), 1)

    apart = [('ugv1', 0, 12.0, 10.0), ('ugv2', 0, 20.0, 10.0)]
    check('떨어진 두 사람은 2명', count_unique_victims(apart, 1.5), 2)

    # 한 로봇이 여럿을 본 경우도 그대로 세야 한다
    solo = [('ugv1', 0, 0.0, 0.0), ('ugv1', 1, 5.0, 0.0),
            ('ugv1', 2, 10.0, 0.0)]
    check('한 로봇이 본 3명은 3명', count_unique_victims(solo, 1.5), 3)

    check('빈 명부는 0명', count_unique_victims([], 1.5), 0)

    # 사슬 병합을 하지 않는다 — 1.4m 간격으로 늘어선 3명.
    #
    # 이웃끼리 이어 붙이면(합집합 방식) 셋이 통째로 1명이 된다. 나란히 누운
    # 별개 조난자를 하나로 지워버리는 셈이라 더 나쁘다. 그래서 새 등록은
    # '이미 만들어진 군집 중심' 하고만 견준다.
    #   0.0 -> 새 군집
    #   1.4 -> 0.0 과 1.5 이내라 합쳐짐
    #   2.8 -> 0.0 과 2.8 떨어져 새 군집
    # 결과 2명. 처음엔 1명을 기대했는데 그 기대가 틀렸다.
    chain = [('ugv1', 0, 0.0, 0.0), ('ugv2', 0, 1.4, 0.0),
             ('ugv1', 1, 2.8, 0.0)]
    check('사슬은 이어 붙이지 않는다(별개 조난자 보호)',
          count_unique_victims(chain, 1.5), 2)

    print()
    if fails:
        print(f'실패 {fails}건')
        return 1
    print('10개 사례 전부 통과')
    return 0


if __name__ == '__main__':
    sys.exit(main())
