#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""반복 유령 자리 기억 단위 테스트.

    python3 tools/test_ghost_spots.py

왜 자리로 기억하나
------------------
유령 하나에 정지·조준·포기로 10~15초를 쓴다. 실측으로 같은 조건의 두 런이
유령 25건과 41건이었고, 그게 700초와 2672초 차이의 일부였다. 2대면 유령도
두 배다.

아침에 '프레임 단위 검출 필터' 를 시도했다가 되돌렸다. 한 프레임을 보고
거르면 진짜 조난자까지 걸러진다 — 후보는 다른 프레임에서 또 뜨는데 진짜는
한 번 놓치면 끝이라 손해가 크다.

자리로 기억하는 건 다르다. '조준까지 했는데 아무것도 없었던' 자리만 쌓으므로
진짜 조난자는 애초에 들어오지 않는다(진짜였다면 등록됐을 테니). 한 번으로
막지 않고 같은 자리에서 여러 번 반복될 때만 막아, 잠깐 가려졌던 진짜를
영구히 버리는 일도 피한다.
"""
import ast
import math
import os
import sys

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'src', 'ugv_vision', 'ugv_vision',
                   'target_manager_node.py')


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
    add_ghost = load('add_ghost_spot')
    is_blocked = load('ghost_blocked')
    fails = 0

    def check(desc, got, want):
        nonlocal fails
        ok = got == want
        print(f'{desc:<50}{"통과" if ok else "실패":>8}')
        if not ok:
            print(f'    got={got}  want={want}')
            fails += 1

    R, NEED = 1.5, 3          # 반경 1.5m, 3번 반복되면 막는다

    spots = []
    check('처음엔 아무것도 안 막는다', is_blocked(0.0, 0.0, spots, R, NEED), False)

    add_ghost(10.0, 5.0, spots, R)
    check('유령 1번으로는 안 막는다',
          is_blocked(10.0, 5.0, spots, R, NEED), False)

    add_ghost(10.3, 5.1, spots, R)     # 같은 자리로 묶임
    check('2번도 아직 안 막는다', is_blocked(10.0, 5.0, spots, R, NEED), False)

    add_ghost(9.8, 4.9, spots, R)
    check('3번째부터 막는다', is_blocked(10.0, 5.0, spots, R, NEED), True)
    check('   같은 자리는 하나로 묶였다', len(spots), 1)

    # 떨어진 자리는 영향을 받지 않는다 — 진짜 조난자가 근처에 있어도 안전
    check('2m 밖은 안 막힌다', is_blocked(12.5, 5.0, spots, R, NEED), False)

    # 흩어진 유령은 쌓이지 않는다 — 매번 다른 곳이면 막지 않는다
    spread = []
    for i in range(5):
        add_ghost(i * 4.0, 0.0, spread, R)
    check('흩어진 유령 5건은 아무 데도 안 막는다',
          any(is_blocked(i * 4.0, 0.0, spread, R, NEED) for i in range(5)),
          False)
    check('   각각 따로 쌓였다', len(spread), 5)

    print()
    if fails:
        print(f'실패 {fails}건')
        return 1
    print('8개 사례 전부 통과')
    return 0


if __name__ == '__main__':
    sys.exit(main())
