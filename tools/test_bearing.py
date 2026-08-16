#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""픽셀 → 방위 변환 단위 테스트.

    python3 tools/test_bearing.py

왜 필요한가
-----------
이 변환은 부호를 틀려도 그럴듯한 값이 나온다. 좌우가 뒤집힌 채로 거리도
맞고 크기도 맞으니, 결과만 보고는 틀린 줄을 모른다. 실제로 조난자 위치가
2~3m 씩 어긋났던 원인이 정확히 이 부호였고, 찾는 데 오래 걸렸다.

픽셀 x 는 오른쪽이 +, ROS 요는 왼쪽이 + 다. 이 한 줄을 잠가 둔다.

무엇을 잠그나
-------------
  · 화면 오른쪽에 보이면 방위가 음수여야 한다(로봇 기준 오른쪽)
  · 화면 중앙이면 포탑 요 그대로여야 한다
  · 포탑이 돌아가 있으면 그만큼 더해져야 한다
  · 화면 가장자리는 화각의 절반만큼 벌어져야 한다
"""
import ast
import math
import os
import sys

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'src', 'ugv_vision', 'ugv_vision',
                   'yolo_pose_node.py')


def load_fn():
    """소스에서 함수 하나만 떼어 온다 — ROS 를 띄우지 않기 위해서다."""
    with open(SRC, encoding='utf-8') as fh:
        tree = ast.parse(fh.read())
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == 'bearing_from_pixel':
            ns = {'math': math, '_CAM_FOV': 1.089}
            exec(compile(ast.Module([node], []), SRC, 'exec'), ns)
            return ns['bearing_from_pixel']
    raise SystemExit('bearing_from_pixel 을 소스에서 못 찾았다')


bearing = load_fn()
FOV = 1.089
W = 640.0
fails = []


def check(name, got, want, tol=1e-6):
    if abs(got - want) > tol:
        fails.append(f'{name}: {got:.4f} (기대 {want:.4f})')


# 중앙이면 포탑 요 그대로
check('중앙·포탑0', bearing(320.0, 0.0), 0.0)
check('중앙·포탑0.7', bearing(320.0, 0.7), 0.7)

# 오른쪽에 보이면 음수 — 여기서 부호가 잡힌다
right = bearing(640.0, 0.0)
if right >= 0:
    fails.append(f'오른쪽 끝인데 방위가 음수가 아니다: {right:.4f}')
check('오른쪽 끝', right, -FOV / 2.0)

# 왼쪽에 보이면 양수
left = bearing(0.0, 0.0)
if left <= 0:
    fails.append(f'왼쪽 끝인데 방위가 양수가 아니다: {left:.4f}')
check('왼쪽 끝', left, FOV / 2.0)

# 좌우 대칭
check('대칭', left + right, 0.0)

# 포탑 요가 더해진다
check('포탑 더하기', bearing(640.0, 1.0), 1.0 - FOV / 2.0)

# 절반 지점은 화각의 1/4
check('오른쪽 절반', bearing(480.0, 0.0), -FOV / 4.0)

if fails:
    print('실패')
    for f in fails:
        print('  ' + f)
    sys.exit(1)
print('8개 사례 전부 통과')
