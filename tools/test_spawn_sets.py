#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""스폰 자리가 담당 구역과 맞는지 본다.

    python3 tools/test_spawn_sets.py

왜 필요한가
-----------
구역은 탐사 경계를 대수로 나눠 '순서대로' 배정된다(bounds_for). 스폰이
그 규칙과 어긋나면 로봇이 출발하자마자 자기 구역까지 건너가야 한다.
대수를 늘린 값어치가 그만큼 깎인다.

실제로 어긋난 채 실험을 돌렸다. 84x40 맵(경계 +-41)에서 3대 스폰이
-12/0/+12 로 박혀 있어 1번·3번이 담당 구역 밖에서 출발했고, '3대가 2대를
못 이긴다' 는 결과를 냈다. 그때 이 테스트는 56x40 경계(+-27)로만 검사해서
통과했다 — 잘못된 맵을 검증하고 있었다.

그래서 이제 맵마다 그 맵의 경계로 검사한다.

무엇을 잠그나
-------------
  · 검증한 2대 구성(-6/+6)이 바뀌지 않는가
  · 맵·대수 조합마다 각 로봇이 자기 구역 안에서 출발하는가
  · 서로 충분히 떨어졌는가(1.6m 로 붙였다 둘 다 갇힌 적이 있다)
  · 복도 잔해와 겹치지 않는가
"""
import ast
import os
import sys

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'src', 'ugv_bringup', 'launch',
                   'multi_robot_sim.launch.py')

# 맵별 탐사 경계 [x0, x1] — 런 스크립트가 UGV_BOUNDS 로 넘기는 값과 같다
MAPS = {'large(56x40)': (-27.0, 27.0),
        'xl(84x40)': (-41.0, 41.0),
        'xxl(168x40)': (-84.0, 84.0)}
DEBRIS_X = (-15, -19, 16, 18, -39, 39)   # 복도(|y|<3) 안 잔해의 x


def load():
    tree = ast.parse(open(SRC, encoding='utf-8').read())
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == 'spawn_points':
            ns = {}
            exec(compile(ast.Module([node], []), SRC, 'exec'), ns)
            return ns['spawn_points']
    raise SystemExit('spawn_points 를 소스에서 못 찾았다')


spawn = load()
fails = []

# 검증한 2대 구성은 바뀌면 안 된다 — 유효 100런의 근거가 이 배치다
two = [(p['name'], float(p['x']), float(p['y']))
       for p in spawn(2, -27.0, 27.0)]
if two != [('ugv1', -6.0, 0.0), ('ugv2', 6.0, 0.0)]:
    fails.append(f'검증한 2대 스폰이 바뀌었다: {two}')

for label, (x0, x1) in MAPS.items():
    for n in (1, 2, 3):
        pts = spawn(n, x0, x1)
        if len(pts) != n:
            fails.append(f'{label} {n}대: 자리 {len(pts)}개')
            continue
        w = (x1 - x0) / n
        xs = [float(p['x']) for p in pts]
        for i, x in enumerate(xs):
            lo, hi = x0 + i * w, x0 + (i + 1) * w
            if not (lo <= x <= hi):
                fails.append(
                    f'{label} {n}대 ugv{i+1}: 스폰 {x} 가 구역 [{lo:.1f},'
                    f'{hi:.1f}] 밖')
        for i in range(len(xs)):
            for j in range(i + 1, len(xs)):
                if abs(xs[i] - xs[j]) < 4.0:
                    fails.append(f'{label} {n}대: {xs[i]} 와 {xs[j]} 가 붙음')
            for bx in DEBRIS_X:
                if abs(xs[i] - bx) < 2.5:
                    fails.append(f'{label} {n}대: 스폰 {xs[i]} 가 잔해 {bx} 옆')

if fails:
    print(f'실패 {len(fails)}건')
    for f in fails:
        print('  ' + f)
    sys.exit(1)
print(f'{1 + len(MAPS) * 3}개 사례 전부 통과')
