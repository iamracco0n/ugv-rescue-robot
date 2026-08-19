#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""방 분리(segment_room) 검증 — 합성 지도로 규칙만 확인한다.

    python3 tools/test_room_segment.py

왜 방을 알아야 하나
-------------------
'주변에 안 본 곳이 없으면 나간다' 의 '주변' 이 반경 5m 고정이었다.
방이 그보다 크면 5m 안만 치우고 나가버린다 — 시간 예산이 방 크기에
안 맞는 것과 같은 문제다. 실제로 시간 예산은 거의 걸리지 않았고
(165번 중 3번), 그래서 예산값을 바꿔도 결과가 안 달라졌다.

방법: 자유공간을 침식하면 좁은 통로(문)가 먼저 끊어져 방이 분리된다.
로봇이 있는 덩어리가 '지금 있는 방' 이다.
"""
import sys

import ast
import os

import numpy as np
from scipy import ndimage

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'src', 'ugv_vision', 'ugv_vision',
                   'patrol_navigator.py')


def load(name):
    """실제로 도는 코드에서 함수를 떼어 온다.

    전에는 이 파일이 자기 복사본을 검사했다. 그러면 본 코드가 달라져도
    테스트는 계속 통과한다 — 실제로 본 코드에 팽창 방식이 들어갔는데도
    이 테스트는 초록이었다.
    """
    tree = ast.parse(open(SRC, encoding='utf-8').read())
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            mod = ast.Module(body=[node], type_ignores=[])
            ns = {'np': np, 'ndimage': ndimage}
            exec(compile(mod, SRC, 'exec'), ns)
            return ns[name]
    raise SystemExit(f'{SRC} 에 {name} 함수가 없다')


segment_room = load('segment_room')


RES = 0.05                     # m/셀 — 실제 SLAM 지도와 같게


def m2c(m):
    return int(round(m / RES))


def make_two_rooms(door_w_m):
    """실제 월드 치수로 만든 두 방 + 문 하나.

    방 11 x 17 m, 벽 0.25 m. 앞서 40x60셀·벽 1셀짜리 장난감 지도로
    시험했다가 전부 실패했다 — 외벽이 없어 자유공간의 거리변환이
    무한정 커지고, 1셀 벽으로는 침식 분리가 성립하지 않는다.
    지도 축척을 실제와 맞춰야 이 방법이 되는지 알 수 있다.
    """
    rw, rh, wt = m2c(11.0), m2c(17.0), m2c(0.25)
    W = rw * 2 + wt + wt * 2
    H = rh + wt * 2
    g = np.zeros((H, W), dtype=bool)
    g[wt:wt + rh, wt:wt + rw] = True                      # 왼방
    g[wt:wt + rh, wt + rw + wt:wt + rw + wt + rw] = True  # 오른방
    # 가운데 벽에 문
    dw = m2c(door_w_m)
    c = H // 2
    g[c - dw // 2:c + dw // 2, wt + rw:wt + rw + wt] = True
    return g, wt, rw, rh


def main():
    fails = 0
    ER = m2c(1.0)          # 침식 반경 1.0m — 문 1.8m 의 절반(0.9m)보다 커야 끊긴다
    print(f'해상도 {RES}m/셀 · 침식 반경 {ER}셀({ER*RES:.1f}m)')
    print(f'{"사례":<44}{"기대":>10}{"결과":>10}')

    g, wt, rw, rh = make_two_rooms(1.8)
    ly, lx = wt + rh // 2, wt + rw // 2              # 왼방 중앙
    ry_, rx_ = wt + rh // 2, wt + rw + wt + rw // 2  # 오른방 중앙

    room = segment_room(g, ly, lx, ER)
    got = room is not None and not room[:, wt + rw + wt:].any()
    print(f'{"문 1.8m — 왼방만 잡히고 오른방은 제외":<44}'
          f'{"분리":>10}{"분리" if got else "실패":>10}')
    fails += (not got)

    room = segment_room(g, ry_, rx_, ER)
    got = room is not None and not room[:, :wt + rw].any()
    print(f'{"문 1.8m — 오른방에서는 오른방만":<44}'
          f'{"분리":>10}{"분리" if got else "실패":>10}')
    fails += (not got)

    room = segment_room(g, ly, lx, ER)
    left = g[:, :wt + rw]
    ratio = room[:, :wt + rw].sum() / left.sum()
    ok = ratio > 0.85
    print(f'{"침식 후 원래 방 면적 85% 이상 회복":<44}'
          f'{">0.85":>10}{f"{ratio:.2f}":>10}')
    fails += (not ok)

    g2, wt2, rw2, rh2 = make_two_rooms(4.0)
    room = segment_room(g2, wt2 + rh2 // 2, wt2 + rw2 // 2, ER)
    got = room is not None and room[:, wt2 + rw2 + wt2:].any()
    print(f'{"문 4m(넓은 통로) — 나누지 않고 하나로":<44}'
          f'{"통합":>10}{"통합" if got else "분리됨":>10}')
    fails += (not got)

    room = segment_room(g, 1, 1, ER)                 # 벽(테두리) 위
    got = room is None
    print(f'{"로봇이 벽 위 — None":<44}'
          f'{"None":>10}{"None" if got else "값있음":>10}')
    fails += (not got)

    print()
    if fails:
        print(f'실패 {fails}건')
        return 1
    print('5개 사례 전부 통과')
    print('\n주의: 합성 지도 검증이다. 실제 SLAM 지도는 벽이 두껍고 잡음이')
    print('      있으므로 시뮬 채점으로 효과를 확인해야 한다.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
