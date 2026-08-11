#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""완료 판정이 세는 미관측 면적 단위 테스트.

    python3 tools/test_actionable_unseen.py

왜 필요한가
-----------
계획기는 일정 크기 이상의 미관측 군집만 목표로 삼는다(visual_min_local,
기본 40셀). 그런데 완료 판정은 자투리까지 전부 세고 있었다. 그러면 로봇이
절대 지울 수 없는 면적이 남아 수색이 영원히 안 끝난다.

실측(큰 월드, 조난자 7/7 다 찾은 뒤):
    미관측 군집 5794개, 총 206.0 m^2
      계획기가 갈 수 있음(>=40셀)   61개  174.0 m^2
      너무 작아 목표가 못 됨(<40셀) 5733개  32.0 m^2

저 32 m^2 가 완료 판정을 영원히 막는다. '회차 완료' 보고가 지금까지 한
번도 안 나온 이유로 보인다.
"""
import ast
import os
import sys

import numpy as np
from scipy import ndimage

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'src', 'ugv_vision', 'ugv_vision',
                   'patrol_navigator.py')


def load(name):
    tree = ast.parse(open(SRC, encoding='utf-8').read())
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            mod = ast.Module(body=[node], type_ignores=[])
            ns = {'np': np, 'ndimage': ndimage}
            exec(compile(mod, SRC, 'exec'), ns)
            return ns[name]
    raise SystemExit(f'{SRC} 에 {name} 함수가 없다')


def main():
    f = load('actionable_cells')
    fails = 0

    def check(desc, got, want):
        nonlocal fails
        ok = got == want
        print(f'{desc:<52}{"통과" if ok else "실패":>8}')
        if not ok:
            print(f'    got={got}  want={want}')
            fails += 1

    # 10x10 격자에 5x5(25셀) 덩어리 하나 + 흩뿌린 한 칸짜리 셋
    m = np.zeros((10, 10), dtype=bool)
    m[0:5, 0:5] = True          # 25셀
    m[9, 0] = True              # 1셀
    m[9, 5] = True              # 1셀
    m[0, 9] = True              # 1셀

    check('하한 20 이면 큰 덩어리만 센다', f(m, 20), 25)
    check('하한 1 이면 전부 센다', f(m, 1), 28)
    check('하한이 덩어리보다 크면 0', f(m, 30), 0)

    # 대각선으로 이어진 것도 한 덩어리로 본다(8방향 연결)
    d = np.zeros((5, 5), dtype=bool)
    d[0, 0] = d[1, 1] = d[2, 2] = True
    check('대각선 연결도 한 덩어리(8방향)', f(d, 3), 3)
    check('   그 덩어리가 하한 미만이면 0', f(d, 4), 0)

    # 빈 격자
    check('미관측이 없으면 0', f(np.zeros((5, 5), dtype=bool), 1), 0)

    print()
    if fails:
        print(f'실패 {fails}건')
        return 1
    print('6개 사례 전부 통과')
    return 0


if __name__ == '__main__':
    sys.exit(main())
