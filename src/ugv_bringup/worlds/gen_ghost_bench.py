#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ghost_bench.sdf 생성기 — 유령 후보 실험용 미니 월드.

    python3 gen_ghost_bench.py > ghost_bench.sdf
    python3 gen_ghost_bench.py --truth truth.json

왜 만드나
---------
유령 후보(사람으로 보고 접근했는데 없음) 대책을 검증하려면 큰 월드에서
70분씩 돌려야 했다. 하루에 몇 번 못 돌리니 시도 한 번에 한 시간이 든다.

유령은 '잔해'에서 나온다 — YOLO-pose 가 상자 더미에 사람 골격을 그린다.
그래서 넓이는 최소로 줄이고 잔해 밀도는 큰 월드보다 높였다.
조난자 3명(서있음·누움·휠체어)으로 트리아지 3등급도 함께 본다.

  큰 월드   56 x 40 m = 2240 m^2, 잔해 21개  → 수색 약 57분
  이 월드   18 x 12 m =  216 m^2, 잔해 16개  → 수색 약 8~10분

주의: 이 월드는 '유령 발생률' 과 '트리아지 정확도' 를 빨리 재기 위한
것이다. 탐사 전략(방 마무리·지역 루프 탈출)은 넓이가 있어야 의미가
있으므로 큰 월드로 검증해야 한다.
"""
import sys

W, H = 18.0, 12.0
HW, HH = W / 2, H / 2
WALL_T, WALL_H = 0.2, 2.5
DOOR = 1.6

out = []
TRUTH = {'world': 'ghost_bench', 'victims': [], 'fires': []}


def add(s):
    out.append(s)


def box(name, x, y, z, sx, sy, sz, rgb=(0.80, 0.78, 0.72), yaw=0.0,
        collision=True):
    r, g, b = rgb
    col = (f'<collision name="col"><geometry><box><size>{sx} {sy} {sz}'
           f'</size></box></geometry></collision>') if collision else ''
    add(f'''    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 {yaw}</pose>
      <link name="link">
        {col}
        <visual name="vis">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
        </visual>
      </link>
    </model>''')


def person(name, uri, x, y, z, roll, pitch, yaw, triage):
    add(f'''    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/{uri}</uri>
      <name>{name}</name>
      <static>true</static>
      <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
    </include>''')
    TRUTH['victims'].append(
        {'name': name, 'x': x, 'y': y, 'triage': triage, 'model': uri})


def fire(name, x, y, z=0.6):
    TRUTH['fires'].append({'name': name, 'x': x, 'y': y})
    add(f'''    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <visual name="vis">
          <geometry><box><size>0.8 0.8 1.1</size></box></geometry>
          <material>
            <ambient>1.0 0.45 0.05 1</ambient><diffuse>1.0 0.45 0.05 1</diffuse>
            <emissive>1.0 0.5 0.1 1</emissive>
          </material>
          <plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal">
            <temperature>600.0</temperature>
          </plugin>
        </visual>
      </link>
    </model>''')


add(f'''<?xml version="1.0" ?>
<!--
  ghost_bench.sdf — {W:.0f} x {H:.0f} m ({W*H:.0f} m^2)
  gen_ghost_bench.py 로 생성됨. 직접 고치지 말고 생성기를 고칠 것.

  유령 후보 실험용. 잔해 밀도를 높여 짧은 시간에 유령을 많이 만든다.
-->
<sdf version="1.7">
  <world name="ghost_bench">

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.6 0.6 0.6 1</diffuse><specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>500</range><constant>0.9</constant>
        <linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>0.2 0.1 -1.0</direction>
    </light>
    <light type="point" name="room_light">
      <pose>0 0 2.3 0 0 0</pose>
      <diffuse>0.6 0.6 0.55 1</diffuse><specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>16</range><constant>0.5</constant>
        <linear>0.1</linear><quadratic>0.01</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>''')

# 바닥·외벽
box('floor', 0, 0, -0.05, W, H, 0.1, rgb=(0.82, 0.80, 0.76))
box('wall_n', 0,  HH, WALL_H / 2, W, WALL_T, WALL_H)
box('wall_s', 0, -HH, WALL_H / 2, W, WALL_T, WALL_H)
box('wall_e',  HW, 0, WALL_H / 2, WALL_T, H, WALL_H)
box('wall_w', -HW, 0, WALL_H / 2, WALL_T, H, WALL_H)

# 방 두 개로 나누는 칸막이 (문 1개씩) — 들어가 봐야 아는 구조
box('div_a', -3.0,  HH / 2 + 0.4, WALL_H / 2, WALL_T, HH - 0.8, WALL_H)
box('div_b',  3.0, -HH / 2 - 0.4, WALL_H / 2, WALL_T, HH - 0.8, WALL_H)

# 잔해 — 유령의 원천. 큰 월드보다 조밀하게 깐다.
debris = [
    (-6.5,  3.8, 1.1, 0.8, 1.0,  0.3), (-5.0, -2.5, 0.9, 1.3, 0.8, -0.2),
    (-1.5,  4.2, 1.4, 0.7, 1.1,  0.5), (-2.0, -4.3, 1.0, 1.1, 0.9,  0.1),
    ( 1.0,  2.0, 1.2, 0.9, 1.0, -0.4), ( 0.5, -1.5, 1.5, 0.8, 1.2,  0.25),
    ( 4.5,  4.0, 0.9, 1.2, 0.8,  0.6), ( 5.5, -3.0, 1.3, 0.7, 1.0, -0.3),
    ( 7.5,  1.5, 1.0, 1.0, 1.1,  0.15), (-7.5, -0.5, 1.1, 0.9, 0.9,  0.4),
    ( 2.5,  5.0, 0.8, 0.8, 1.0,  0.0), (-4.0,  1.0, 1.2, 1.1, 0.8,  0.2),
    ( 6.5, -0.5, 0.9, 0.9, 1.2, -0.15), ( 3.0, -5.0, 1.1, 1.0, 0.9,  0.35),
    (-7.0,  5.0, 1.0, 0.8, 1.1,  0.1), ( 8.0,  4.5, 0.9, 1.0, 1.0, -0.25),
]
for i, (x, y, sx, sy, sz, yaw) in enumerate(debris):
    box(f'debris_{i}', x, y, sz / 2, sx, sy, sz,
        rgb=(0.42, 0.38, 0.33), yaw=yaw)

# 조난자 3명 — 트리아지 3등급을 모두 덮는다
person('v_standing', 'Standing%20person', -7.0,  4.0, 0, 0, 0, -1.2, 3)
person('v_lying',    'Standing%20person',  6.0, -4.5, 0.15, 0, -1.5708, 0.4, 1)
person('v_wheel',    'PatientWheelChair',  0.0,  5.0, 0, 0, 0, 1.4, 2)

fire('fire_1', -8.0, -4.5)

add('  </world>\n</sdf>')

if '--truth' in sys.argv:
    import json
    path = sys.argv[sys.argv.index('--truth') + 1]
    with open(path, 'w', encoding='utf-8') as fp:
        json.dump(TRUTH, fp, ensure_ascii=False, indent=2)
    sys.stderr.write(
        f"정답 저장: {path} "
        f"(조난자 {len(TRUTH['victims'])}명, 화재 {len(TRUTH['fires'])}건)\n")
else:
    print('\n'.join(out))
