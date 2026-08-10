#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""박힘 판정 단위 테스트.

    python3 tools/test_stuck_decision.py

왜 필요한가
-----------
박힘 판정이 위치만 봤다. 제자리 회전은 위치가 안 변하므로, 목표 방향이
크게 바뀌어 로봇이 180도 돌아서는 동안 '8초간 못 움직임' 으로 오판됐다.
그러면 후진 탈출이 돌아 방금 온 길을 되돌아가고, 다시 앞으로 가다 또
돌아서고 — 앞뒤로 왔다 갔다만 하게 된다. 실측 로그:

    목표 (2.5,-1.1) → 박힘 → 목표 (-0.9,0.8) → 도달실패
    목표 (2.4,-1.2) → 박힘 → 목표 (1.3,0.0)  → 박힘
    목표 (-10.5,0)  → 박힘 → 목표 (1.1,0)

박힘 메시지에 찍힌 로봇 위치는 (0,0) → (-0.9,0) → (-1.8,0) → (-2.6,0) 로
실제로는 계속 전진 중이었다. 박히지 않았는데 박혔다고 본 것이다.

회전도 '진전' 으로 쳐야 한다. 시뮬을 돌리지 않고 확인할 수 있는 규칙이다.
"""
import ast
import math
import os
import sys

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'src', 'ugv_vision', 'ugv_vision',
                   'patrol_navigator.py')

EPS_M    = 0.15    # 이만큼 움직이면 진전
EPS_RAD  = 0.30    # 이만큼 돌면 진전 (약 17도)
CONFIRM  = 8.0     # 진전 없이 이 시간이 지나면 박힘


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
    stuck_decision = load('stuck_decision')
    fails = 0

    def check(desc, got, want):
        nonlocal fails
        ok = got == want
        print(f'{desc:<50}{"통과" if ok else "실패":>8}')
        if not ok:
            print(f'    got={got}  want={want}')
            fails += 1

    # ref = (x, y, yaw, t)
    ref = (0.0, 0.0, 0.0, 100.0)

    # ── 1. 전진 중이면 박힘이 아니고 기준이 갱신된다 ────────────────
    stuck, new_ref = stuck_decision(ref, 1.0, 0.0, 0.0, 109.0,
                                    EPS_M, EPS_RAD, CONFIRM)
    check('1m 전진 — 박힘 아님', stuck, False)
    check('   기준이 새 위치로 갱신됨', new_ref, (1.0, 0.0, 0.0, 109.0))

    # ── 2. 이번에 고치는 것: 제자리 회전도 진전이다 ─────────────────
    # 180도 돌아서는 데 8초가 넘게 걸려도 박힘이 아니어야 한다.
    stuck, new_ref = stuck_decision(ref, 0.0, 0.0, math.pi, 109.0,
                                    EPS_M, EPS_RAD, CONFIRM)
    check('제자리에서 180도 회전 — 박힘 아님', stuck, False)
    check('   회전했으므로 기준 갱신됨', new_ref is not None, True)

    # 작은 회전도 진전으로 친다(계속 돌고 있는 중)
    stuck, _ = stuck_decision(ref, 0.0, 0.0, 0.35, 109.0,
                              EPS_M, EPS_RAD, CONFIRM)
    check('20도 회전 — 박힘 아님', stuck, False)

    # ── 3. 진짜로 아무것도 안 하면 박힘 ────────────────────────────
    stuck, new_ref = stuck_decision(ref, 0.05, 0.0, 0.05, 109.0,
                                    EPS_M, EPS_RAD, CONFIRM)
    check('위치도 자세도 그대로 + 8초 초과 — 박힘', stuck, True)
    check('   박힘일 때는 기준을 갱신하지 않음', new_ref, None)

    # ── 4. 시간이 아직 안 됐으면 박힘 아님 ─────────────────────────
    stuck, _ = stuck_decision(ref, 0.05, 0.0, 0.05, 105.0,
                              EPS_M, EPS_RAD, CONFIRM)
    check('가만히 있지만 아직 5초 — 박힘 아님', stuck, False)

    # ── 5. 각도 wrap 을 제대로 다루는가 ────────────────────────────
    # 3.10 rad 과 -3.10 rad 은 실제로 0.08 rad 차이(약 5도)다. 단순 뺄셈이면
    # 6.2 rad 으로 잘못 커져 '크게 돌았다' 고 오판한다.
    ref_wrap = (0.0, 0.0, 3.10, 100.0)
    stuck, _ = stuck_decision(ref_wrap, 0.0, 0.0, -3.10, 109.0,
                              EPS_M, EPS_RAD, CONFIRM)
    check('3.10 → -3.10 rad 은 5도 차이 — 박힘', stuck, True)

    # 반대로 진짜 큰 회전은 wrap 을 건너도 진전으로 잡혀야 한다
    stuck, _ = stuck_decision(ref_wrap, 0.0, 0.0, 1.0, 109.0,
                              EPS_M, EPS_RAD, CONFIRM)
    check('3.10 → 1.0 rad 은 120도 — 박힘 아님', stuck, False)

    print()
    if fails:
        print(f'실패 {fails}건')
        return 1
    print('10개 사례 전부 통과')
    return 0


if __name__ == '__main__':
    sys.exit(main())
