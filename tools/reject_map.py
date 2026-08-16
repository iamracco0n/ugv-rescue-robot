#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""기각이 '어느 자리' 에서 났는지 월드 좌표로 찍는다.

    python3 tools/reject_map.py <로그...>

왜 필요한가
-----------
조난자를 놓쳤을 때 처방은 두 갈래고 서로 정반대다.

    안 갔다            -> 커버리지. 탐사를 고친다
    갔는데 못 알아봤다  -> 인식. 관문이나 시야각을 고친다

지금까지는 이 둘을 목표 좌표로 어림잡았는데, 목표는 '가려던 곳' 이지
'간 곳' 이 아니다. 그리고 오탐 게이트 기각은 15초마다 개수만 찍혀서 어느
자리에서 죽었는지 알 수 없었다.

계측을 붙여 둘을 붙인다.

    [궤적] (x,y) yaw=θ                 1Hz, target_manager
    [기각위치] 3.2m/-0.41rad/kpt ...   15초마다, yolo_pose_node

기각의 방위는 로봇 기준이므로 같은 시각의 로봇 자세를 얹으면 월드 좌표가
나온다. 15초 사이 로봇은 멀리 못 가므로 자리를 가르기에는 충분하다.

읽는 법
-------
못 찾은 조난자 자리에 기각이 쌓여 있으면 '보이긴 했는데 관문이 죽였다' 는
뜻이다. 그 자리에 기각도 궤적도 없으면 아예 안 간 것이다. 둘은 고치는
곳이 다르다.
"""
import math
import os
import re
import sys
from collections import defaultdict

# 정답 위치. 누운 사람은 메쉬 원점이 발밑이라 몸 중심이 yaw 반대쪽 0.875m.
VICTIMS = {
    'lying_n3':    (-1.0 - 0.875 * math.cos(2.0),   15.5 - 0.875 * math.sin(2.0)),
    'lying_s2':    (-14.0 - 0.875 * math.cos(0.5), -16.0 - 0.875 * math.sin(0.5)),
    'lying_s4':    (9.0 - 0.875 * math.cos(0.3),   -14.0 - 0.875 * math.sin(0.3)),
    'standing_n1': (-22.0, 16.0),
    'occluded_s1': (-23.5, -16.0),
    'sitting_n4':  (12.0, 10.0),
    'corridor_e':  (24.0, 0.5),
}
NEAR = 3.0        # 이 반경 안이면 그 조난자 자리로 본다
MAX_DT = 8.0      # 궤적과 기각의 시각 차 허용치[s]

RE_T = re.compile(r'\[(\d{10}\.\d+)\]')
RE_ROBOT = re.compile(r'\[(ugv\d)\.')
RE_TRACE = re.compile(r'\[궤적\] \((-?\d+\.\d+),(-?\d+\.\d+)\) yaw=(-?\d+\.\d+)')
RE_REJ = re.compile(r'(\d+\.?\d*)m/(-?\d+\.?\d*)rad/(\w+)')


def parse(path):
    """로그 하나에서 로봇별 궤적과 기각을 뽑는다."""
    traces = defaultdict(list)   # robot -> [(t, x, y, yaw)]
    rejects = defaultdict(list)  # robot -> [(t, dist, bearing, why)]
    with open(path, encoding='utf-8', errors='replace') as fh:
        for line in fh:
            mt = RE_T.search(line)
            mr = RE_ROBOT.search(line)
            if not mt or not mr:
                continue
            t, rob = float(mt.group(1)), mr.group(1)
            mtr = RE_TRACE.search(line)
            if mtr:
                traces[rob].append(
                    (t, float(mtr.group(1)), float(mtr.group(2)),
                     float(mtr.group(3))))
                continue
            if '[기각위치]' in line:
                for d, b, w in RE_REJ.findall(line):
                    rejects[rob].append((t, float(d), float(b), w))
    return traces, rejects


def nearest_pose(track, t):
    """그 시각에 가장 가까운 자세. 너무 멀면 None."""
    best, gap = None, MAX_DT
    for entry in track:
        d = abs(entry[0] - t)
        if d <= gap:
            best, gap = entry, d
    return best


def main(paths):
    hits = defaultdict(int)       # 조난자 -> 기각 수
    visited = defaultdict(int)    # 조난자 -> 5m 안에 간 런 수
    stray = 0
    runs = 0
    for path in paths:
        if not os.path.exists(path):
            continue
        runs += 1
        traces, rejects = parse(path)
        if not traces:
            continue
        seen_here = set()
        for rob, track in traces.items():
            for _, x, y, _ in track:
                for name, (vx, vy) in VICTIMS.items():
                    if math.hypot(x - vx, y - vy) <= 5.0:
                        seen_here.add(name)
        for name in seen_here:
            visited[name] += 1
        for rob, rejs in rejects.items():
            for t, dist, bear, _why in rejs:
                pose = nearest_pose(traces.get(rob, []), t)
                if pose is None:
                    continue
                _, rx, ry, ryaw = pose
                wx = rx + dist * math.cos(ryaw + bear)
                wy = ry + dist * math.sin(ryaw + bear)
                for name, (vx, vy) in VICTIMS.items():
                    if math.hypot(wx - vx, wy - vy) <= NEAR:
                        hits[name] += 1
                        break
                else:
                    stray += 1

    if not runs:
        raise SystemExit('로그를 못 읽었다')
    print(f'기각 위치 분포 ({runs}런, 반경 {NEAR}m)')
    print(f'{"조난자":<14}{"기각":>7}{"근처간런":>10}')
    print('-' * 34)
    for name in VICTIMS:
        print(f'{name:<14}{hits[name]:>7}{visited[name]:>8}/{runs}')
    print(f'{"(조난자 아닌 곳)":<14}{stray:>7}')
    print()
    print('못 찾은 조난자 자리에 기각이 쌓였으면 관문이 죽인 것이고,')
    print('기각도 궤적도 없으면 애초에 안 간 것이다. 고칠 데가 다르다.')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    args = []
    for a in sys.argv[1:]:
        if a.endswith('.txt'):
            with open(a, encoding='utf-8') as fh:
                args += [ln.strip() for ln in fh if ln.strip()]
        else:
            args.append(a)
    main(args)
