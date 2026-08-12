#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""누운 사람 전용 키포인트 관문 단위 테스트.

    python3 tools/test_lying_gate.py

왜 필요한가
-----------
작은 맵 조난자 3명 중 하나는 바닥에 누워 있고 등급이 L1 Critical 이다.
실측(오로라, 4런)에서 3런을 놓쳤다.

  서있는 남성  4/4 발견, 오차 0.17m
  서있는 여성  4/4 발견, 오차 0.21m
  누운 사람    1/4 발견, 오차 1.43m   ← 최우선 등급인데 이렇다

기각 사유는 거의 전부 키포인트였다. 누우면 관절 절반이 가려지고 눌려 보여
'신뢰도 0.5 이상 관절 6개' 를 못 채운다. YOLO 는 박스를 제대로 찾고 있었다
— 재학습으로 풀 문제가 아니라 관문 문제였다.

무엇을 잠그나
-------------
이 관문은 양방향으로 틀릴 수 있어서 둘 다 본다.

  · 느슨해서 벽을 사람으로 들이면  → 유령 조난자가 생긴다
  · 빡빡해서 누운 사람을 버리면    → 최우선 환자를 놓친다

특히 완화가 '가로로 긴 박스' 에만 적용되는지가 핵심이다. 세로로 긴 박스까지
완화되면 벽 오탐을 막아 주던 방어가 통째로 풀린다.
"""
import ast
import math
import os
import sys

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'src', 'ugv_vision', 'ugv_vision',
                   'yolo_pose_node.py')


class FakeConf(list):
    """kconf 는 numpy 배열이라 (kconf >= x).sum() 을 쓴다. 그 부분만 흉내낸다."""

    def __ge__(self, other):
        return FakeConf(1 if v >= other else 0 for v in self)

    def sum(self):
        return sum(self)


class Node:
    """_person_gate 만 떼어 붙인 최소 객체."""

    min_kpt_conf   = 0.50
    min_valid_kpts = 6
    lying_aspect   = 1.15
    lying_min_kpts = 3
    lying_kpt_conf = 0.30
    min_box_diag_px = 40.0
    max_box_diag_px = 900.0
    depth_tol      = 2.00


def load_gate():
    tree = ast.parse(open(SRC, encoding='utf-8').read())
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == '_person_gate':
            mod = ast.Module(body=[node], type_ignores=[])
            ns = {'math': math}
            exec(compile(mod, SRC, 'exec'), ns)
            return ns['_person_gate']
    raise SystemExit(f'{SRC} 에 _person_gate 가 없다')


def main():
    gate = load_gate()
    n = Node()
    fails = 0

    def check(desc, got, want):
        nonlocal fails
        ok = got == want
        print(f'{desc:<52}{"통과" if ok else "실패":>8}')
        if not ok:
            print(f'    got={got}  want={want}')
            fails += 1

    # 박스: (x1,y1,x2,y2). 세로로 긴 = 서있음, 가로로 긴 = 누움
    STAND = (100.0, 50.0, 200.0, 350.0)     # w=100 h=300 → w/h=0.33
    LIE   = (100.0, 200.0, 400.0, 320.0)    # w=300 h=120 → w/h=2.5

    def conf(*vals):
        return FakeConf(vals)

    # 관절 17개 중 몇 개가 어느 신뢰도인지로 상황을 만든다
    strong6 = conf(*([0.9] * 6 + [0.1] * 11))    # 확실한 관절 6개
    weak3   = conf(*([0.35] * 3 + [0.1] * 14))   # 흐릿한 관절 3개 — 누운 사람
    weak1   = conf(*([0.35] * 1 + [0.1] * 16))   # 흐릿한 관절 1개 — 벽에 가까움
    none_   = conf(*([0.1] * 17))                # 사람다움 신호 없음 — 벽

    # ── 서있는 사람: 기존 동작이 그대로여야 한다 ────────────────────
    check('서있음 + 확실한 관절 6개 → 통과',
          gate(n, strong6, *STAND, 3.0, 3.0)[0], True)
    check('서있음 + 흐릿한 관절 3개 → 기각(기존대로 엄격)',
          gate(n, weak3, *STAND, 3.0, 3.0), (False, 'kpt'))

    # ── 누운 사람: 완화가 먹어야 한다 ───────────────────────────────
    check('누움 + 흐릿한 관절 3개 → 통과(완화 적용)',
          gate(n, weak3, *LIE, 3.0, 3.0)[0], True)
    check('누움 + 확실한 관절 6개 → 통과',
          gate(n, strong6, *LIE, 3.0, 3.0)[0], True)

    # ── 벽 방어: 완화해도 사람다움 신호는 계속 요구한다 ─────────────
    check('누움 모양 + 관절 1개 → 기각(벽 방어)',
          gate(n, weak1, *LIE, 3.0, 3.0), (False, 'kpt'))
    check('누움 모양 + 관절 0개 → 기각(벽 방어)',
          gate(n, none_, *LIE, 3.0, 3.0), (False, 'kpt'))

    # ── 완화가 '가로로 긴 박스' 에만 적용되는지 ─────────────────────
    # 여기가 풀리면 벽 오탐 방어가 통째로 무력해진다.
    SQUARE = (100.0, 100.0, 250.0, 240.0)   # w=150 h=140 → w/h=1.07, 문턱 미만
    check('거의 정사각(w/h=1.07) → 완화 안 됨',
          gate(n, weak3, *SQUARE, 3.0, 3.0), (False, 'kpt'))

    # ── 완화되어도 뒤의 관문은 그대로 걸린다 ────────────────────────
    TINY = (100.0, 200.0, 130.0, 210.0)     # 가로로 길지만 대각선 32px < 40
    check('누움 모양이어도 너무 작은 박스는 기각',
          gate(n, weak3, *TINY, 3.0, 3.0), (False, 'geom'))

    # depth 불일치도 완화 대상이 아니다
    check('누움 모양이어도 depth 크게 어긋나면 기각',
          gate(n, weak3, *LIE, 1.0, 9.0), (False, 'depth'))

    print()
    if fails:
        print(f'실패 {fails}건')
        return 1
    print('9개 사례 전부 통과')
    return 0


if __name__ == '__main__':
    sys.exit(main())
