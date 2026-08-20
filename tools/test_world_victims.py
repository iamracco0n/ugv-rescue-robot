#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""월드의 조난자가 벽 속이나 잔해 속에 박히지 않았는지 본다.

    python3 tools/test_world_victims.py            # 기존 맵
    UGV_MAP=xl python3 tools/test_world_victims.py # 큰 맵

왜 필요한가
-----------
조난자를 방 기준 좌표로 옮기면서 맵 크기가 진짜 파라미터가 됐다. 그런데
N_ROOMS 를 바꾸면 칸막이 위치가 통째로 이동하므로, 새 맵에서 조난자가
벽 속에 들어가 있어도 SDF 는 멀쩡히 생성된다. 시뮬을 띄워도 조용히
'영원히 못 찾는 조난자' 로만 보인다 — 알고리즘 문제로 오해하기 딱 좋다.

무엇을 잠그나
-------------
  · 건물 밖으로 나가지 않았는가
  · 칸막이·복도벽·외벽에서 충분히 떨어졌는가
  · 잔해와 겹치지 않는가
  · 같은 조난자끼리 겹치지 않는가
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
GEN = os.path.join(HERE, '..', 'src', 'ugv_bringup', 'worlds',
                   'gen_rescue_large.py')

CLEAR_WALL = 0.9      # 벽에서 최소 이 거리(로봇 폭 0.4 + 여유)
CLEAR_OBJ = 1.0       # 잔해·다른 조난자에서 최소 이 거리


def gen(map_name):
    env = dict(os.environ, UGV_MAP=map_name)
    out = subprocess.run([sys.executable, GEN], capture_output=True,
                         text=True, env=env, check=True).stdout
    W, H, N = (84.0, 40.0, 8) if map_name == 'xl' else (56.0, 40.0, 5)
    victims, debris = [], []
    for m in re.finditer(
            r'<name>(victim_[^<]+)</name>.*?<pose>(-?[\d.]+) (-?[\d.]+)',
            out, re.S):
        victims.append((m.group(1), float(m.group(2)), float(m.group(3))))
    for m in re.finditer(
            r'<model name="(debris_\d+)">\s*<static>true</static>\s*'
            r'<pose>(-?[\d.]+) (-?[\d.]+)', out):
        debris.append((m.group(1), float(m.group(2)), float(m.group(3))))
    return victims, debris, W, H, N


def main():
    map_name = os.environ.get('UGV_MAP', 'large')
    victims, debris, W, H, N = gen(map_name)
    HW, HH, COR = W / 2, H / 2, 3.0
    room_w = W / N
    dividers = [-HW + room_w * i for i in range(1, N)]
    fails = []

    if not victims:
        raise SystemExit('조난자를 하나도 못 찾았다 — 생성기 출력이 이상하다')

    for name, x, y in victims:
        if abs(x) > HW - 0.5 or abs(y) > HH - 0.5:
            fails.append(f'{name} ({x},{y}) 건물 밖')
        for d in dividers:
            if abs(x - d) < CLEAR_WALL and abs(y) > COR:
                fails.append(f'{name} ({x},{y}) 칸막이 x={d:.1f} 에 붙음')
        # 복도 벽 — 복도 안(|y|<COR)에 있는 조난자는 통과 대상이 아니다
        if COR < abs(y) < COR + CLEAR_WALL:
            fails.append(f'{name} ({x},{y}) 복도 벽에 붙음')
        for dname, dx, dy in debris:
            if ((x - dx) ** 2 + (y - dy) ** 2) ** 0.5 < CLEAR_OBJ:
                fails.append(f'{name} 이 {dname} 와 겹침')

    for i in range(len(victims)):
        for j in range(i + 1, len(victims)):
            _, x1, y1 = victims[i]
            _, x2, y2 = victims[j]
            if ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5 < CLEAR_OBJ:
                fails.append(f'{victims[i][0]} 와 {victims[j][0]} 가 겹침')

    if fails:
        print(f'[{map_name}] 실패 {len(fails)}건')
        for f in fails:
            print('  ' + f)
        sys.exit(1)
    print(f'{len(victims) + 3}개 사례 전부 통과 '
          f'([{map_name}] 조난자 {len(victims)}명 검사)')


if __name__ == '__main__':
    main()
