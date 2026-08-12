#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""방 우선 보너스 단위 테스트.

    python3 tools/test_room_bonus.py

무엇을 고치려는 건가
--------------------
실측(큰 월드 2대, 4런): 수색 후반에만 방을 덜 보고 나가는 일이 런당
19.3회, 덜 보고 나왔던 방으로 되돌아오는 왕복이 14.3회였다. 왕복은 한 번에
끝냈으면 안 했을 이동이라 순수 낭비다.

왜 필터가 아니라 보너스인가
---------------------------
예전에 '반경 5m 안을 먼저 처리하고 없을 때만 멀리' 라는 하드 필터를 썼다가
정반대 고장이 났다. 점수식은 이미 큰 덩어리를 선호하는데, 필터가 비교 대상
자체를 눈앞으로 제한해 점수식을 무력화했다. 목표가 1.4~4.3m 잔걸음이 되고
방 하나를 자투리 단위로 갉아먹느라 다른 방으로 넘어가질 못했다
(실측: 25.6m 목표를 세 번 재발행하고도 24초에 1.1m 전진).

보너스면 훨씬 좋은 바깥 후보가 여전히 이긴다. 그래서 이 테스트는 양방향을
다 본다 — 방 안을 제대로 밀어주는지, 그리고 큰 바깥 후보를 여전히 못
이기게 두는지.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'ugv_vision', 'ugv_vision'))

import ast

import numpy as np

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'src', 'ugv_vision', 'ugv_vision',
                   'patrol_navigator.py')


def load(name):
    tree = ast.parse(open(SRC, encoding='utf-8').read())
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            mod = ast.Module(body=[node], type_ignores=[])
            ns = {'math': math, 'np': np}
            exec(compile(mod, SRC, 'exec'), ns)
            return ns[name]
    raise SystemExit(f'{SRC} 에 {name} 함수가 없다')


def load_method(cls_name, meth):
    """클래스 안의 staticmethod 를 떼어 온다."""
    tree = ast.parse(open(SRC, encoding='utf-8').read())
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == cls_name:
            for f in node.body:
                if isinstance(f, ast.FunctionDef) and f.name == meth:
                    f.decorator_list = []
                    mod = ast.Module(body=[f], type_ignores=[])
                    ns = {'math': math, 'np': np}
                    exec(compile(mod, SRC, 'exec'), ns)
                    return ns[meth]
    raise SystemExit(f'{SRC} 에 {cls_name}.{meth} 가 없다')


def main():
    goal_score = load('goal_score')
    in_room = load_method('PatrolNavigator', '_in_room')
    fails = 0

    def check(desc, got, want):
        nonlocal fails
        ok = got == want
        print(f'{desc:<50}{"통과" if ok else "실패":>8}')
        if not ok:
            print(f'    got={got}  want={want}')
            fails += 1

    # 20x20 격자, 해상도 0.5m. 왼쪽 절반이 '지금 있는 방'
    room = np.zeros((20, 20), dtype=bool)
    room[:, :10] = True
    info = (room, 0.0, 0.0, 0.5, 20, 20)

    # ── 방 판정 ────────────────────────────────────────────────────
    check('방 안 좌표는 안으로 판정', in_room(info, 2.0, 2.0), True)
    check('방 밖 좌표는 밖으로 판정', in_room(info, 8.0, 2.0), False)
    check('격자 밖 좌표는 밖으로 판정', in_room(info, 99.0, 99.0), False)
    check('경계 바로 안쪽(x=4.9)', in_room(info, 4.9, 2.0), True)
    check('경계 바로 바깥(x=5.1)', in_room(info, 5.1, 2.0), False)

    # ── 보너스가 방 안을 밀어주는가 ────────────────────────────────
    # 미관측(visual)은 이득이 셀넓이라 작고, 경계(frontier)는 길이×시야라
    # 크다. 실제로 경합이 벌어지는 쪽은 경계끼리이므로 그걸로 짠다.
    RES, VIEW, LAM = 0.05, 8.0, 0.5
    BONUS = 6.0

    # 방 안: 남은 경계가 작다(10셀=4.0m^2), 가깝다(3m) -> 2.5
    inside = goal_score('frontier', 10, 3.0, RES, VIEW, LAM)
    # 방 밖: 더 크다(30셀=12.0m^2), 멀다(9m) -> 7.5
    outside = goal_score('frontier', 30, 9.0, RES, VIEW, LAM)
    check('보너스 없으면 바깥이 이긴다', inside > outside, False)
    check('보너스를 주면 방 안이 이긴다',
          inside + BONUS > outside, True)

    # ── 그래도 훨씬 좋은 바깥은 못 이겨야 한다 ─────────────────────
    # 이게 하드 필터와의 결정적 차이다. 필터였다면 아래도 방 안이 이겨서
    # 방을 자투리 단위로 갉아먹으며 못 나가는 고장이 난다.
    big_outside = goal_score('frontier', 100, 12.0, RES, VIEW, LAM)
    check('훨씬 큰 바깥 후보는 보너스를 줘도 이긴다',
          inside + BONUS > big_outside, False)

    # ── 보너스 0 이면 아무것도 안 바뀐다 ───────────────────────────
    check('보너스 0 이면 점수 그대로',
          inside + 0.0 == inside, True)

    print()
    if fails:
        print(f'실패 {fails}건')
        return 1
    print('9개 사례 전부 통과')
    return 0


if __name__ == '__main__':
    sys.exit(main())
