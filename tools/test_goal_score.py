#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""탐사 목표 점수 함수 단위 테스트.

    python3 tools/test_goal_score.py

왜 필요한가
-----------
목표 후보가 두 종류인데 크기(n)의 단위가 서로 다르다.

  라이다 경계  n = 미탐사와 맞닿은 '경계선 길이'(1셀 두께)
  시각 미관측  n = 아직 못 본 바닥 '면적'

5m 짜리 구역이면 전자는 약 100셀, 후자는 약 10000셀로 100배 차이가 난다.
둘을 한 점수식에 그냥 넣으면 시각 후보가 언제나 이겨 라이다 경계가 영영
안 뽑히고, 지도가 안 넓어진다. 그래서 둘 다 '새로 얻을 넓이(m^2)' 로
환산해서 비교한다.

시뮬 없이 확인할 수 있는 규칙이므로 단위 테스트로 고정한다.
rclpy 없이 돌도록 노드 모듈에서 함수만 읽어 실행한다.
"""
import ast
import os
import sys

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'src', 'ugv_vision', 'ugv_vision',
                   'patrol_navigator.py')

RES    = 0.05     # m/셀 — 실제 SLAM 지도와 같게
VIEW_R = 8.0      # 라이다 경계를 넘었을 때 새로 보이는 깊이(m)


def load(name):
    """patrol_navigator 에서 함수 하나만 떼어내 실행 가능하게 만든다."""
    tree = ast.parse(open(SRC, encoding='utf-8').read())
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            mod = ast.Module(body=[node], type_ignores=[])
            ns = {}
            exec(compile(mod, SRC, 'exec'), ns)
            return ns[name]
    raise SystemExit(f'{SRC} 에 {name} 함수가 없다')


def main():
    goal_score = load('goal_score')
    fails = 0

    def check(desc, got, want, cmp='>'):
        nonlocal fails
        ok = got > want if cmp == '>' else got < want
        print(f'{desc:<52}{"통과" if ok else "실패":>8}')
        if not ok:
            print(f'    got={got:.2f}  want {cmp} {want:.2f}')
            fails += 1

    lam = 0.5     # 1m 더 가는 값어치 = 0.5 m^2

    # ── 1. 이번에 고치려는 바로 그 증상 ────────────────────────────
    # 눈앞의 자투리(40셀=0.1m^2)가 20m 밖 미관측 방(10000셀=25m^2)을
    # 이겨서 로봇이 잔걸음만 하고 있었다.
    scrap = goal_score('visual', 40, 1.4, RES, VIEW_R, lam)
    room  = goal_score('visual', 10000, 20.0, RES, VIEW_R, lam)
    check('멀리 있는 큰 미관측 방 > 눈앞의 자투리', room, scrap)

    # ── 2. 단위 환산이 되는가 ──────────────────────────────────────
    # 라이다 경계 100셀 = 길이 5m → 넘어가면 5m x 8m = 40m^2 가 새로 보인다.
    # 시각 자투리 40셀 = 0.1m^2. 거리가 같다면 경계가 압도해야 한다.
    edge = goal_score('frontier', 100, 5.0, RES, VIEW_R, lam)
    near = goal_score('visual',    40, 5.0, RES, VIEW_R, lam)
    check('같은 거리면 라이다 경계 > 시각 자투리', edge, near)

    # 반대로, 아주 큰 시각 미관측(25m^2)은 작은 경계(20셀=1m 길이=8m^2)를
    # 이겨야 한다. 한쪽이 무조건 이기면 환산이 잘못된 것이다.
    big_visual = goal_score('visual', 10000, 5.0, RES, VIEW_R, lam)
    small_edge = goal_score('frontier',  20, 5.0, RES, VIEW_R, lam)
    check('큰 시각 미관측 > 작은 라이다 경계', big_visual, small_edge)

    # ── 3. 거리는 비용이다 ─────────────────────────────────────────
    close = goal_score('visual', 4000, 3.0, RES, VIEW_R, lam)
    far   = goal_score('visual', 4000, 30.0, RES, VIEW_R, lam)
    check('크기가 같으면 가까운 쪽', close, far)

    # ── 4. lam 이 실제로 거리 선호를 조절하는가 ────────────────────
    # lam=0 이면 거리를 무시하므로 큰 쪽이 무조건 이긴다.
    a = goal_score('visual', 10000, 50.0, RES, VIEW_R, 0.0)
    b = goal_score('visual',  9000,  1.0, RES, VIEW_R, 0.0)
    check('lam=0 이면 거리 무시하고 큰 쪽', a, b)

    # lam 을 크게 하면 같은 상황에서 가까운 쪽으로 뒤집혀야 한다.
    a2 = goal_score('visual', 10000, 50.0, RES, VIEW_R, 5.0)
    b2 = goal_score('visual',  9000,  1.0, RES, VIEW_R, 5.0)
    check('lam 이 크면 가까운 쪽으로 뒤집힘', a2, b2, cmp='<')

    # ── 5. 실제 넓이와 맞는가 ──────────────────────────────────────
    # 20x20 셀 = 1m x 1m = 1m^2. 거리 0 이면 점수가 그대로 1.0 이어야 한다.
    v = goal_score('visual', 400, 0.0, RES, VIEW_R, lam)
    ok = abs(v - 1.0) < 1e-9
    print(f'{"400셀(1m^2) 후보의 점수가 실제 1.0 m^2":<52}{"통과" if ok else "실패":>8}')
    if not ok:
        print(f'    got={v}  want 1.0')
        fails += 1

    print()
    if fails:
        print(f'실패 {fails}건')
        return 1
    print('7개 사례 전부 통과')
    return 0


if __name__ == '__main__':
    sys.exit(main())
