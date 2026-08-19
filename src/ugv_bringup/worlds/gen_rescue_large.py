#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rescue_building_large.sdf 생성기.

손으로 1000줄 넘는 SDF 를 쓰는 대신 배치 규칙을 코드로 남긴다.
치수를 바꾸고 싶으면 아래 상수만 고치고 다시 실행하면 된다.

    python3 gen_rescue_large.py > rescue_building_large.sdf

기존 rescue_building.sdf(28x20m, 560m^2) 대비 4배 넓이(56x40m, 2240m^2).
구조를 홀 하나에서 **복도 축 + 남북 10개 방** 으로 바꿨다. 기존 월드는
라이다(360deg 25m)가 스폰 지점에서 대부분을 한 번에 봐서 탐사 난이도가
낮았는데, 방을 문으로 나누면 실제로 들어가 봐야 알 수 있다.
"""

W, H = 56.0, 40.0          # 건물 외곽 (m)
HW, HH = W / 2, H / 2
WALL_T, WALL_H = 0.25, 2.8
COR_HALF = 3.0             # 중앙 복도 반폭 → 복도 폭 6m
DOOR = 1.8                 # 출입구 폭
N_ROOMS = 5                # 남/북 각 5개 방

out = []
# 자동 채점용 정답. 배치 함수가 채우고 --truth 로 JSON 저장한다.
TRUTH = {'world': 'rescue_building_large', 'victims': [], 'fires': []}


def add(s):
    out.append(s)


def box(name, x, y, z, sx, sy, sz, rgb=(0.80, 0.78, 0.72), static=True,
         collision=True, yaw=0.0):
    r, g, b = rgb
    col = (f'<collision name="col"><geometry><box><size>{sx} {sy} {sz}'
           f'</size></box></geometry></collision>') if collision else ''
    add(f'''    <model name="{name}">
      <static>{"true" if static else "false"}</static>
      <pose>{x} {y} {z} 0 0 {yaw}</pose>
      <link name="link">
        {col}
        <visual name="vis">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
        </visual>
      </link>
    </model>''')


def wall_with_doors(name, axis, fixed, span_lo, span_hi, doors):
    """doors = [(중심, 폭), ...] 를 뺀 나머지를 벽 조각으로 채운다."""
    edges = [span_lo]
    for c, w in sorted(doors):
        edges += [c - w / 2, c + w / 2]
    edges.append(span_hi)
    for i in range(0, len(edges) - 1, 2):
        lo, hi = edges[i], edges[i + 1]
        if hi - lo < 0.05:
            continue
        mid, ln = (lo + hi) / 2, hi - lo
        if axis == 'x':      # x 방향으로 뻗은 벽
            box(f'{name}_{i}', mid, fixed, WALL_H / 2, ln, WALL_T, WALL_H)
        else:                # y 방향으로 뻗은 벽
            box(f'{name}_{i}', fixed, mid, WALL_H / 2, WALL_T, ln, WALL_H)


def person(name, uri, x, y, z, roll, pitch, yaw, triage=None):
    """조난자 배치. triage 는 자동 채점용 정답 등급(1/2/3).

    정답을 별도 파일에 손으로 적으면 월드를 고칠 때 반드시 어긋난다.
    배치와 같은 자리에 적어 두고 --truth 로 함께 내보낸다.
    """
    add(f'''    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/{uri}</uri>
      <name>{name}</name>
      <static>true</static>
      <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
    </include>''')
    if triage is not None:
        TRUTH['victims'].append(
            {'name': name, 'x': x, 'y': y, 'triage': triage, 'model': uri})


def fire(name, x, y, z=0.6, sx=0.9, sy=0.9, sz=1.2):
    TRUTH['fires'].append({'name': name, 'x': x, 'y': y})
    add(f'''    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <visual name="vis">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material>
            <ambient>1.0 0.45 0.05 1</ambient><diffuse>1.0 0.45 0.05 1</diffuse>
            <emissive>1.0 0.5 0.1 1</emissive>
          </material>
          <plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal">
            <temperature>600.0</temperature>
          </plugin>
        </visual>
      </link>
    </model>
    <light type="point" name="{name}_light">
      <pose>{x} {y} {z + 0.8} 0 0 0</pose>
      <diffuse>1.0 0.55 0.2 1</diffuse><specular>0.3 0.15 0.05 1</specular>
      <attenuation><range>7</range><constant>0.4</constant><linear>0.15</linear>
        <quadratic>0.02</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>''')


# ── 헤더 ──────────────────────────────────────────────────────────────
add(f'''<?xml version="1.0" ?>
<!--
  rescue_building_large.sdf — {W:.0f} x {H:.0f} m ({W*H:.0f} m^2)
  gen_rescue_large.py 로 생성됨. 직접 고치지 말고 생성기를 고칠 것.

  기존 rescue_building.sdf(28x20m) 의 4배 넓이. 중앙 복도 + 남북 {N_ROOMS}개씩
  총 {N_ROOMS*2}개 방. 각 방은 폭 {DOOR}m 문으로만 복도와 이어진다.
-->
<sdf version="1.7">
  <world name="rescue_building_large">

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <!-- thermal 카메라는 ogre2 에서만 렌더링됨 -->
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 15 0 0 0</pose>
      <diffuse>0.6 0.6 0.6 1</diffuse><specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant>
        <linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>0.1 0.1 -1.0</direction>
    </light>''')

# 복도 조명
for lx in range(-24, 25, 8):
    add(f'''    <light type="point" name="hall_light_{lx+24}">
      <pose>{lx} 0 2.6 0 0 0</pose>
      <diffuse>0.55 0.55 0.5 1</diffuse><specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>14</range><constant>0.5</constant>
        <linear>0.1</linear><quadratic>0.01</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>''')

# ── 바닥 · 외벽 ───────────────────────────────────────────────────────
box('floor', 0, 0, -0.05, W, H, 0.1, rgb=(0.82, 0.80, 0.76))
box('outer_wall_north', 0,  HH, WALL_H / 2, W, WALL_T, WALL_H)
box('outer_wall_south', 0, -HH, WALL_H / 2, W, WALL_T, WALL_H)
box('outer_wall_east',  HW, 0, WALL_H / 2, WALL_T, H + WALL_T, WALL_H)
box('outer_wall_west', -HW, 0, WALL_H / 2, WALL_T, H + WALL_T, WALL_H)

# ── 복도 벽 (남북 각각, 방마다 문 1개) ────────────────────────────────
room_w = W / N_ROOMS
centers = [-HW + room_w * (i + 0.5) for i in range(N_ROOMS)]
wall_with_doors('corridor_wall_north', 'x',  COR_HALF, -HW, HW,
                [(c, DOOR) for c in centers])
wall_with_doors('corridor_wall_south', 'x', -COR_HALF, -HW, HW,
                [(c, DOOR) for c in centers])

# ── 방 사이 칸막이 ────────────────────────────────────────────────────
for i in range(1, N_ROOMS):
    x = -HW + room_w * i
    box(f'divider_north_{i}', x,  (COR_HALF + HH) / 2, WALL_H / 2,
        WALL_T, HH - COR_HALF, WALL_H)
    box(f'divider_south_{i}', x, -(COR_HALF + HH) / 2, WALL_H / 2,
        WALL_T, HH - COR_HALF, WALL_H)

# 일부 방에 막다른 내부 칸막이 — 탐사가 실제로 들어가 봐야 알 수 있게
box('inner_n1', centers[0] + 2.0, 12.0, WALL_H / 2, WALL_T, 10.0, WALL_H)
box('inner_n4', centers[3] - 2.5,  9.0, WALL_H / 2, 6.0, WALL_T, WALL_H)
box('inner_s2', centers[1] + 1.5, -11.0, WALL_H / 2, WALL_T, 9.0, WALL_H)
box('inner_s5', centers[4] - 2.0, -13.0, WALL_H / 2, 7.0, WALL_T, WALL_H)

# ── 잔해 ──────────────────────────────────────────────────────────────
debris = [
    (-22, 11, 1.4, 0.9, 1.1, 0.3), (-18, -9, 1.0, 1.6, 0.9, -0.2),
    (-11,  8, 1.8, 0.7, 1.0, 0.5), (-9, -14, 1.1, 1.2, 1.2, 0.1),
    (-2, 12.5, 1.3, 0.9, 0.9, -0.4), (-3, -7.5, 1.5, 0.8, 1.1, 0.25),
    (4, 9.5, 1.0, 1.4, 0.8, 0.6), (6, -11.5, 1.7, 0.7, 1.0, -0.3),
    (13, 14, 1.2, 1.0, 1.1, 0.15), (11, -16, 1.9, 0.8, 0.9, 0.4),
    (20, 7.5, 1.1, 1.5, 1.0, -0.15), (23, -8, 1.3, 0.9, 1.2, 0.35),
    (-24, -17, 1.0, 1.0, 0.8, 0.0), (25, 17, 1.2, 1.2, 0.9, 0.2),
    (-15, 1.5, 0.8, 0.8, 0.7, 0.3), (16, -1.5, 0.9, 0.9, 0.7, -0.25),
    (-6, 17, 1.4, 0.8, 1.0, 0.1), (8, -18, 1.1, 1.1, 0.9, -0.35),
    (18, 2.0, 0.7, 0.7, 0.6, 0.0), (-19, -2.0, 0.7, 0.7, 0.6, 0.0),
]
for i, (x, y, sx, sy, sz, yaw) in enumerate(debris, 1):
    box(f'debris_{i}', x, y, sz / 2, sx, sy, sz,
        rgb=(0.42, 0.38, 0.33), yaw=yaw)

# ── 조난자 ────────────────────────────────────────────────────────────
# 재난 현장에 맞게 자세를 다시 잡았다. 예전에는 7명 중 4명이 멀쩡히 서
# 있었는데, 건물이 무너진 현장에서 그럴 리가 없다.
# 지금은 누움 3 / 서있음 3(잔해에 가린 1명 포함) / 휠체어 1 이다.
#
# 누운 사람은 메쉬 원점이 발밑이라 몸이 yaw '반대' 방향으로 약 1.75m 뻗는다.
# 실측으로 확인했다 — 원점(-14,-16)/yaw 0.5 인 조난자의 검출 38건이 평균
# (-14.77,-16.74) 였다. 배치할 때 그쪽에 벽이 없는지 봐야 한다.
person('victim_standing_n1', 'Standing%20person', -22.0, 16.0, 0, 0, 0, -1.57, triage=3)
person('victim_lying_n3',    'Casual%20female',    -1.0, 15.5, 0.15, 0, -1.5708, 2.0, triage=1)
person('victim_lying_s2',    'Standing%20person', -14.0, -16.0, 0.15, 0, -1.5708, 0.5, triage=1)
# 앉아있는 사람 — 휠체어 환자. 서있음도 누움도 아닌 애매한 자세.
person('victim_sitting_n4',  'PatientWheelChair',  12.0, 10.0, 0, 0, 0, 1.2, triage=2)
# 원래 병원 이동침대(TrolleyBedPatient)였다. 두 가지 이유로 바닥 누움으로
# 바꿨다.
#   · 재난 현장에 환자용 이동침대가 놓여 있는 것이 부자연스럽다
#   · 실측 46런에서 발견률 41% 로 최악이었고, 못 찾은 29런 중 19런은 YOLO 가
#     사람 박스를 아예 못 만들었다. 침대 프레임이 몸을 가리고 로봇 카메라
#     (약 0.5m)가 60cm 침대의 옆면만 보는 탓으로 보인다. 관문을 고쳐도
#     살릴 수 없는 종류라 시나리오에서 뺐다.
person('victim_lying_s4',    'Standing%20person',   9.0, -14.0, 0.15, 0, -1.5708, 0.3, triage=1)
# 잔해에 반쯤 가린 사람 — 오탐 게이트와 접근 로직 시험용
person('victim_occluded_s1', 'Casual%20female',   -23.5, -16.0, 0, 0, 0, 0.9, triage=3)
# 복도 동쪽 끝 — 이동 중 먼 거리에서 먼저 보이는 대상.
# 원래 'Male visitor' 를 썼는데 이 Fuel 자산만 <model> 이 아니라 <actor> 다
# (걷기 애니메이션 + 자체 trajectory pose 0 1 0). 그래서
#   · 실제 위치가 배치 좌표에서 로컬 +Y 로 1m 밀린다(yaw 3.14 라 월드 -Y).
#     정답 좌표를 (24.0, 0.5) 로 알고 오차 1.0m 라 착각했었다.
#   · 애니메이션이 스켈레톤을 움직여 YOLO 어깨 키포인트가 지면 0.38~0.39m
#     로 잡힌다(거리 2.3~7.4m 전 구간에서 일관). 서 있는 사람인데 누움으로
#     분류돼 L1 로 나왔다.
# 걸어다니는 사람은 애초에 구조 대상이 아니므로 나머지 6명과 같은
# 정적 모델로 통일한다.
person('victim_corridor_e',  'Standing%20person',  24.0, 0.5, 0, 0, 0, 3.14, triage=3)

# 가린 사람 바로 앞 잔해
box('debris_occluder', -22.6, -15.4, 0.55, 1.2, 0.9, 1.1,
    rgb=(0.42, 0.38, 0.33), yaw=0.2)

# ── 화재 ──────────────────────────────────────────────────────────────
fire('fire_source_1', -24.0, 8.0)                 # 북서 방 구석
fire('fire_source_2',  22.0, -17.0)               # 남동 방 구석
fire('fire_source_3',   1.0, -17.5)               # 남측 방 안쪽 (문으로만 보임)
fire('fire_source_4', -26.0, -0.5, sx=0.7, sy=0.7, sz=1.0)   # 복도 서쪽 끝

add('  </world>\n</sdf>')

import sys

if '--truth' in sys.argv:
    # 정답만 JSON 으로 저장(SDF 는 stdout). 자동 채점이 이 파일을 읽는다.
    import json
    path = sys.argv[sys.argv.index('--truth') + 1]
    with open(path, 'w', encoding='utf-8') as fp:
        json.dump(TRUTH, fp, ensure_ascii=False, indent=2)
    sys.stderr.write(
        f"정답 저장: {path} "
        f"(조난자 {len(TRUTH['victims'])}명, 화재 {len(TRUTH['fires'])}건)\n")
else:
    print('\n'.join(out))
