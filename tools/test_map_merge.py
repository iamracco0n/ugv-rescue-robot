#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""지도 병합 단위 테스트.

    python3 tools/test_map_merge.py

왜 정렬을 탐색하지 않는가
------------------------
두 로봇의 스폰 위치를 우리가 정하므로 두 map 프레임의 오프셋을 이미 안다.
탐색 없이 그대로 겹치면 된다. 문제는 겹치는 규칙이다.

  점유(100) 는 이겨야 한다  — 한쪽만 벽을 봤어도 벽이다
  미탐사(-1) 는 져야 한다   — 상대가 본 곳을 모른다고 덮으면 안 된다

이 우선순위를 뒤집으면, 늦게 온 로봇의 미탐사가 먼저 만든 지도를 지운다.
"""
import ast
import os
import sys

import numpy as np

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'src', 'ugv_vision', 'ugv_vision', 'map_merge_node.py')


def load(name):
    tree = ast.parse(open(SRC, encoding='utf-8').read())
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            mod = ast.Module(body=[node], type_ignores=[])
            ns = {'np': np}
            exec(compile(mod, SRC, 'exec'), ns)
            return ns[name]
    raise SystemExit(f'{SRC} 에 {name} 함수가 없다')


def main():
    merge_cells = load('merge_cells')
    fails = 0

    def check(desc, got, want):
        nonlocal fails
        ok = np.array_equal(got, want)
        print(f'{desc:<46}{"통과" if ok else "실패":>8}')
        if not ok:
            print(f'    got ={list(np.ravel(got))}')
            print(f'    want={list(np.ravel(want))}')
            fails += 1

    U, F, O = -1, 0, 100          # 미탐사 / 자유 / 점유

    # ── 우선순위 ────────────────────────────────────────────────
    base = np.array([U, F, O, F, O, U], dtype=np.int16)
    add  = np.array([F, U, U, O, F, O], dtype=np.int16)
    want = np.array([F, F, O, O, O, O], dtype=np.int16)
    check('점유가 이기고 미탐사가 진다', merge_cells(base, add), want)

    # ── 미탐사끼리는 미탐사로 남는다 ────────────────────────────
    check('둘 다 미탐사면 미탐사',
          merge_cells(np.array([U], dtype=np.int16),
                      np.array([U], dtype=np.int16)),
          np.array([U], dtype=np.int16))

    # ── 자유 대 자유 ────────────────────────────────────────────
    check('둘 다 자유면 자유',
          merge_cells(np.array([F], dtype=np.int16),
                      np.array([F], dtype=np.int16)),
          np.array([F], dtype=np.int16))

    # ── 순서를 바꿔도 결과가 같아야 한다 ────────────────────────
    # 병합은 어느 로봇을 먼저 넣든 같은 지도가 나와야 한다.
    a = merge_cells(base.copy(), add.copy())
    b = merge_cells(add.copy(), base.copy())
    check('넣는 순서가 결과를 바꾸지 않는다', a, b)

    # ── 중간 확률값도 큰 쪽이 이긴다 ────────────────────────────
    check('중간 확률은 큰 쪽',
          merge_cells(np.array([30, 80], dtype=np.int16),
                      np.array([60, 20], dtype=np.int16)),
          np.array([60, 80], dtype=np.int16))

    print()
    if fails:
        print(f'실패 {fails}건')
        return 1
    print('5개 사례 전부 통과')
    print('\n주의: 겹치는 규칙만 본다. 좌표 오프셋이 맞는지는 시뮬에서')
    print('      병합 지도에 벽이 두 겹으로 안 생기는지로 확인해야 한다.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
