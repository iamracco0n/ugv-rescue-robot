#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""같은 방 안 '먼 곳 먼저' 보너스 단위 테스트.

    python3 tools/test_far_first.py

왜 필요한가
-----------
이 보너스는 점수식의 거리 페널티와 정면으로 싸운다. 방 안에서는 먼 쪽을
이기게 하되, 방 밖까지 그러면 로봇이 건물을 가로지르며 왕복한다. 경계
조건이 틀려도 시뮬에서는 '좀 이상한 동선' 으로만 보여서 알아채기 어렵다.

무엇을 잠그나
-------------
  · 꺼져 있으면(계수 0) 아무 영향이 없어야 한다 — 기본값이 0 이다
  · 방 밖 후보에는 절대 안 붙어야 한다
  · 방 안에서는 멀수록 커져야 한다
  · 상한 위로는 안 커져야 한다 — 안 그러면 가장 먼 곳 하나만 계속 이긴다
  · 거리 페널티를 실제로 이길 수 있어야 한다(이걸 못하면 기능이 무의미)
"""
import ast
import os
import sys

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'src', 'ugv_vision', 'ugv_vision',
                   'patrol_navigator.py')


def load(name):
    with open(SRC, encoding='utf-8') as fh:
        tree = ast.parse(fh.read())
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            ns = {}
            exec(compile(ast.Module([node], []), SRC, 'exec'), ns)
            return ns[name]
    raise SystemExit(f'{name} 을 소스에서 못 찾았다')


bonus = load('far_first_bonus')
score = load('goal_score')
fails = []


def check(name, cond):
    if not cond:
        fails.append(name)


# 꺼져 있으면 영향 없음 — 기본값이 0 이므로 이게 깨지면 전 런이 바뀐다
check('계수 0 이면 0', bonus(True, 10.0, 0.0, 12.0) == 0.0)
check('계수 음수면 0', bonus(True, 10.0, -1.0, 12.0) == 0.0)

# 방 밖에는 절대 안 붙는다
check('방 밖은 0', bonus(False, 10.0, 5.0, 12.0) == 0.0)
check('방 밖은 계수 커도 0', bonus(False, 99.0, 100.0, 12.0) == 0.0)

# 방 안에서는 멀수록 크다
check('멀수록 큼', bonus(True, 8.0, 5.0, 12.0) > bonus(True, 3.0, 5.0, 12.0))
check('0m 는 0', bonus(True, 0.0, 5.0, 12.0) == 0.0)

# 상한 — 넘어가면 더 안 커진다
check('상한에서 멈춤', bonus(True, 50.0, 5.0, 12.0) == bonus(True, 12.0, 5.0, 12.0))
check('상한 값 정확', bonus(True, 20.0, 2.0, 12.0) == 24.0)

# 거리 페널티를 이길 수 있어야 한다.
# 같은 크기 덩어리라면 먼 쪽이 이겨야 방 안쪽으로 들어간다.
RES, VIEW, LAM = 0.05, 3.0, 1.0
near = score('visual', 4000, 2.0, RES, VIEW, LAM)
far = score('visual', 4000, 10.0, RES, VIEW, LAM)
check('보너스 없으면 가까운 쪽이 이김', near > far)
check('보너스 있으면 먼 쪽이 이김',
      far + bonus(True, 10.0, 5.0, 12.0) > near + bonus(True, 2.0, 5.0, 12.0))

# 다만 훨씬 큰 덩어리는 여전히 이겨야 한다 — 보너스가 점수식을 무력화하면
# 예전 하드 필터 때처럼 잔걸음으로 방을 갉아먹는다.
big_near = score('visual', 40000, 2.0, RES, VIEW, LAM)
small_far = score('visual', 400, 11.0, RES, VIEW, LAM)
check('큰 덩어리는 여전히 이김',
      big_near + bonus(True, 2.0, 5.0, 12.0)
      > small_far + bonus(True, 11.0, 5.0, 12.0))

if fails:
    print(f'실패 {len(fails)}건')
    for f in fails:
        print('  ' + f)
    sys.exit(1)
print('11개 사례 전부 통과')
