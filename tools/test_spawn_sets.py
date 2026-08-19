#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""대수별 스폰 자리 선택 검증.

검증한 2대 구성(-6/+6)이 바뀌지 않는 것과, 3대일 때 각 로봇이 자기 구역
안에서 출발하는 것을 잠근다. 스폰이 자기 구역 밖이면 출발하자마자 건물을
가로질러야 해서 대수를 늘린 값어치가 준다.
"""
import ast, os, sys
SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'src', 'ugv_bringup', 'launch',
                   'multi_robot_sim.launch.py')
tree = ast.parse(open(SRC, encoding='utf-8').read())
sets = None
for node in tree.body:
    if isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', '') == 'ROBOT_SETS':
        sets = ast.literal_eval(node.value)
assert sets, 'ROBOT_SETS 를 못 찾았다'

fails = []
# 검증한 2대 구성은 바뀌면 안 된다
two = [(r['name'], float(r['x']), float(r['y'])) for r in sets[2]]
if two != [('ugv1', -6.0, 0.0), ('ugv2', 6.0, 0.0)]:
    fails.append(f'2대 스폰이 바뀌었다: {two}')

# 3대는 각자 자기 구역 안에서 출발해야 한다
X0, X1 = -27.0, 27.0
for i, r in enumerate(sets[3]):
    w = (X1 - X0) / 3
    lo, hi = X0 + i * w, X0 + (i + 1) * w
    x = float(r['x'])
    if not (lo <= x <= hi):
        fails.append(f'{r["name"]} 스폰 {x} 가 구역 [{lo},{hi}] 밖이다')

# 서로 충분히 떨어져야 한다(코스트맵 인플레이션 0.55m, 실측 1.6m 는 갇혔다)
xs = [float(r['x']) for r in sets[3]]
for i in range(len(xs)):
    for j in range(i + 1, len(xs)):
        if abs(xs[i] - xs[j]) < 4.0:
            fails.append(f'스폰 {xs[i]} 와 {xs[j]} 가 너무 가깝다')

# 복도 잔해와 겹치면 안 된다
DEBRIS = [(-15, 1.5), (-19, -2.0), (16, -1.5), (18, 2.0)]
for r in sets[3]:
    x, y = float(r['x']), float(r['y'])
    for dx, dy in DEBRIS:
        d = ((x - dx) ** 2 + (y - dy) ** 2) ** 0.5
        if d < 3.0:
            fails.append(f'{r["name"]} 스폰이 잔해 ({dx},{dy}) 에서 {d:.1f}m')

if fails:
    print(f'실패 {len(fails)}건')
    for f in fails:
        print('  ' + f)
    sys.exit(1)
print('9개 사례 전부 통과')
