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

import os

# 맵 크기는 환경변수로 바꾼다. 조난자·화재를 방 기준으로 놓으므로 여기만
# 바꾸면 큰 맵이 그대로 만들어진다.
#
#     UGV_MAP=xl python3 gen_rescue_large.py > rescue_building_xl.sdf
#
# 왜 큰 맵이 필요한가: 로봇 대수의 값어치는 맵 크기에 비례한다. 작은 맵
# (28x20)에서는 1대가 2대보다 나았고, 56x40 에서 2대가 1대를 1.4배 앞섰다.
# 그런데 3대는 56x40 에서 2대를 못 이긴다(완주 1566·1664초 대 1050~1143초).
# 복도가 하나뿐이라 로봇끼리 서로 막히고, 방 10개는 3대에게 이미 빽빽하다.
# 대수를 더 늘려 이득을 보려면 방이 더 많아야 한다.
_MAP = os.environ.get('UGV_MAP', 'large')
# SDF 안의 <world name> 은 파일 이름과 같아야 한다.
#
# 다르면 로봇이 조용히 안 생긴다. 스폰이 '-world <이름>' 으로 월드를 지정
# 하는데, 이름이 안 맞아도 create 서비스는 'OK' 를 돌려주고 모델만 안
# 들어간다. 실제로 XL 맵 첫 시도가 이랬다 — 월드는 뜨고 노드도 다 뜨는데
# scan·odom 이 한 건도 안 오고 SLAM 맵이 영영 안 왔다.
WORLD_NAME = ('rescue_building_xxl' if _MAP == 'xxl'
              else 'rescue_building_xl' if _MAP == 'xl'
              else 'rescue_building_large')
if _MAP == 'xxl':
    W, H = 168.0, 40.0     # 방 남북 각 16개
    N_ROOMS = 16
elif _MAP == 'xl':
    W, H = 84.0, 40.0      # 방 남북 각 8개
    N_ROOMS = 8
else:
    W, H = 56.0, 40.0      # 방 남북 각 5개(검증된 기존 맵)
    N_ROOMS = 5
HW, HH = W / 2, H / 2
WALL_T, WALL_H = 0.25, 2.8
COR_HALF = 3.0             # 중앙 복도 반폭 → 복도 폭 6m
DOOR = 1.8                 # 출입구 폭

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


def room_xy(idx, side, dx=0.0, depth=0.0):
    """방 번호와 그 방 안에서의 상대 위치로 월드 좌표를 만든다.

    왜 절대좌표를 쓰면 안 되나
    --------------------------
    파일 첫머리에 '치수를 바꾸고 싶으면 상수만 고치고 다시 실행하면 된다'
    고 적어 뒀지만, 조난자에 대해서는 사실이 아니었다. 좌표가 절대값이라
    N_ROOMS 를 5 에서 8 로 바꾸면 방 중심이 -21.6 에서 -31.5 로 옮겨 가고,
    조난자는 엉뚱한 방이나 벽 속에 떨어진다.

    방 번호 기준으로 적으면 맵 크기가 진짜 파라미터가 된다.

      idx    방 번호(0 부터). 서쪽에서 동쪽으로 센다
      side   'n' 북쪽 / 's' 남쪽
      dx     그 방 문(=방 중심)에서 동서로 얼마나 떨어졌나[m]. +가 동쪽
      depth  복도 안쪽 벽에서 얼마나 깊이 들어갔나[m]. 항상 양수
    """
    room_w = W / N_ROOMS
    cx = -HW + room_w * (idx + 0.5)
    y = (COR_HALF + depth) if side == 'n' else -(COR_HALF + depth)
    return cx + dx, y


def person_in_room(name, uri, idx, side, dx, depth, z, roll, pitch, yaw,
                   triage=None):
    """방 기준으로 조난자를 놓는다. 좌표 계산만 room_xy 에 맡긴다."""
    x, y = room_xy(idx, side, dx, depth)
    person(name, uri, round(x, 2), round(y, 2), z, roll, pitch, yaw, triage)


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
  <world name="{WORLD_NAME}">

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
# XL 맵은 방이 6개 더 늘어난다. 잔해 좌표가 절대값이라 그대로 두면 바깥
# 방들만 깨끗해져서, 넓어진 만큼 오히려 쉬워진다. 난이도가 고르지 않으면
# 대수 비교가 아니라 '어느 로봇이 쉬운 구역을 맡았나' 비교가 된다.
# 기존 맵과 비슷한 밀도(방 10개에 20덩이)로 새 구역에도 깔아 준다.
if _MAP == 'xxl':
    # 방이 32개다. 기존 잔해는 가운데 56m 에만 있어서 그대로 두면 바깥
    # 방들이 통째로 깨끗해진다. 밀도를 맞춰 넓게 깐다.
    #
    # (25,17) 은 XXL 에서 조난자와 겹쳐 비켜 놓는다. 겹치면 SDF 는 멀쩡히
    # 생성되고 시뮬도 뜨지만 '영원히 못 찾는 조난자' 로만 보인다.
    debris = [d for d in debris if not (d[0] == 25 and d[1] == 17)]
    debris += [(21, 17, 1.2, 1.2, 0.9, 0.2)]
    for _bx in (-76, -66, -56, -46, 46, 56, 66, 76):
        debris += [
            (_bx, 11, 1.3, 0.9, 1.0, 0.2),
            (_bx + 4, -12, 1.1, 1.3, 0.9, -0.3),
            (_bx - 3, 4.5, 0.8, 0.8, 0.7, 0.1),
        ]
if _MAP == 'xl':
    debris += [
        (-34, 10, 1.3, 0.9, 1.0, 0.2), (-33, -12, 1.1, 1.3, 0.9, -0.3),
        (-30, 5, 0.8, 0.8, 0.7, 0.1), (31, 12, 1.4, 0.9, 1.1, 0.35),
        (33, -10, 1.0, 1.5, 0.9, -0.2), (36, 6, 0.9, 0.9, 0.8, 0.0),
        (24, -17, 1.2, 1.0, 1.0, 0.15), (-28, -17, 1.1, 1.1, 0.9, -0.1),
        (24, 8, 0.7, 0.7, 0.6, 0.25), (-24, 3, 0.7, 0.7, 0.6, -0.25),
        (39, -2.0, 0.7, 0.7, 0.6, 0.0), (-39, 2.0, 0.7, 0.7, 0.6, 0.0),
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
person_in_room('victim_standing_n1', 'Standing%20person', 0, 'n', 0.4, 13.0,
               0, 0, 0, -1.57, triage=3)
person_in_room('victim_lying_n3', 'Casual%20female', 2, 'n', -1.0, 12.5,
               0.15, 0, -1.5708, 2.0, triage=1)
person_in_room('victim_lying_s2', 'Standing%20person', 1, 's', -2.8, 13.0,
               0.15, 0, -1.5708, 0.5, triage=1)
# 앉아있는 사람 — 휠체어 환자. 서있음도 누움도 아닌 애매한 자세.
person_in_room('victim_sitting_n4', 'PatientWheelChair', 3, 'n', 0.8, 7.0,
               0, 0, 0, 1.2, triage=2)
# 원래 병원 이동침대(TrolleyBedPatient)였다. 두 가지 이유로 바닥 누움으로
# 바꿨다.
#   · 재난 현장에 환자용 이동침대가 놓여 있는 것이 부자연스럽다
#   · 실측 46런에서 발견률 41% 로 최악이었고, 못 찾은 29런 중 19런은 YOLO 가
#     사람 박스를 아예 못 만들었다. 침대 프레임이 몸을 가리고 로봇 카메라
#     (약 0.5m)가 60cm 침대의 옆면만 보는 탓으로 보인다. 관문을 고쳐도
#     살릴 수 없는 종류라 시나리오에서 뺐다.
person_in_room('victim_lying_s4', 'Standing%20person', 3, 's', -2.2, 11.0,
               0.15, 0, -1.5708, 0.3, triage=1)
# 잔해에 반쯤 가린 사람 — 오탐 게이트와 접근 로직 시험용
person_in_room('victim_occluded_s1', 'Casual%20female', 0, 's', -1.1, 13.0,
               0, 0, 0, 0.9, triage=3)
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
person('victim_corridor_e',  'Standing%20person',  HW - 4.0, 0.5, 0, 0, 0, 3.14,
       triage=3)

# ── XL 맵 전용 조난자 ─────────────────────────────────────────────────
# 방이 10개에서 16개로 늘었으므로 조난자도 비례해 늘린다. 안 그러면 방당
# 밀도가 낮아져 '넓어서 어려운' 것이 아니라 '허탕이 많아서 오래 걸리는'
# 맵이 된다. 그러면 대수 비교가 아니라 이동 속도 비교가 되어 버린다.
#
# 자세 비율은 기존 맵과 같게 맞춘다 — 누움 3 : 서있음 3 : 휠체어 1 을
# 누움 6 : 서있음 5 : 휠체어 2 로 늘렸다(7명 → 13명).
# 새로 쓰는 방(4~7번)에 고르게 흩고, 누운 사람은 구석 쪽에 둔다.
if _MAP == 'xl':
    person_in_room('victim_lying_n5', 'Standing%20person', 5, 'n', -2.5, 12.0,
                   0.15, 0, -1.5708, 1.1, triage=1)
    person_in_room('victim_lying_s6', 'Casual%20female', 6, 's', 2.4, 13.5,
                   0.15, 0, -1.5708, 2.6, triage=1)
    person_in_room('victim_lying_n7', 'Standing%20person', 7, 'n', -1.6, 14.0,
                   0.15, 0, -1.5708, 0.2, triage=1)
    person_in_room('victim_standing_s5', 'Casual%20female', 5, 's', 1.8, 8.0,
                   0, 0, 0, 2.2, triage=3)
    person_in_room('victim_standing_n6', 'Standing%20person', 6, 'n', -0.9, 9.5,
                   0, 0, 0, -0.8, triage=3)
    person_in_room('victim_sitting_s7', 'PatientWheelChair', 7, 's', 1.4, 6.5,
                   0, 0, 0, 2.9, triage=2)

# ── XXL 맵 전용 조난자 ─────────────────────────────────────────────────
# 방이 32개라 손으로 적을 수 없다. 방을 돌며 규칙적으로 놓는다.
#
# 자세 비율은 기존 맵과 같게 유지한다(누움 6 : 서있음 5 : 휠체어 2).
# 방마다 다른 자리에 놓아야 '한 자리만 어렵다' 같은 편향이 안 생기므로,
# 방 번호로 오프셋을 돌린다. 난수를 쓰면 실행마다 맵이 달라져서 런끼리
# 비교가 안 되므로 절대 쓰지 않는다.
#
# 밀도는 XL 과 맞춘다 — XL 은 방 16개에 13명(0.81명/방)이었다.
# XXL 은 방 32개이므로 26명이다.
if _MAP == 'xxl':
    _POSES = (['lying'] * 6 + ['standing'] * 5 + ['sitting'] * 2)
    _DX = (-2.8, 1.6, -1.1, 2.4, -2.2, 0.8, -0.4, 1.9)
    _DEPTH = (13.0, 8.5, 11.5, 6.5, 14.0, 9.5, 12.5, 7.5)
    _k = 0
    for _idx in range(N_ROOMS):
        for _side in ('n', 's'):
            # 방 32개에 26명이므로 6개 방은 비운다. 6칸마다 하나씩 건너뛰어
            # 빈 방이 한쪽에 몰리지 않게 한다.
            if _k % 6 == 5 and _k >= 5:
                _k += 1
                continue
            _pose = _POSES[_k % len(_POSES)]
            _dx = _DX[_k % len(_DX)]
            _depth = _DEPTH[_k % len(_DEPTH)]
            _yaw = round(0.3 + 0.37 * _k, 2) % 6.28
            _name = f'victim_{_pose}_{_side}{_idx}'
            if _pose == 'lying':
                person_in_room(_name, 'Standing%20person', _idx, _side,
                               _dx, _depth, 0.15, 0, -1.5708, _yaw, triage=1)
            elif _pose == 'sitting':
                person_in_room(_name, 'PatientWheelChair', _idx, _side,
                               _dx, _depth, 0, 0, 0, _yaw, triage=2)
            else:
                person_in_room(_name, 'Casual%20female', _idx, _side,
                               _dx, _depth, 0, 0, 0, _yaw, triage=3)
            _k += 1

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
