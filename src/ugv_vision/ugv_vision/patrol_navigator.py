#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patrol_navigator.py
===================
'경비 아저씨' 자율 순찰 노드.

  · 웨이포인트(방 구석구석)를 무한 루프로 순찰 → Nav2 /goal_pose 발행
  · RViz 2D Nav Goal 이 들어오면 수동 우선 (MANUAL) — 도착 후 순찰 재개
  · /fire_alert (화재 발견) 수신 시:
        멈춤(현재 위치를 goal 로) + 포탑을 화재로 조준(/apex_aim_point) + 경보(로그·마커)
        alarm_duration 후 순찰 재개. 화재 구역은 fire_detection_node 가
        /fire_cloud 로 costmap 에 장애물 마킹 → Nav2 가 알아서 우회.

  기존 vision_coverage_navigator 의 goal 에코 감지 규약을 그대로 사용
  (자기가 쏜 goal 에코는 무시, 외부 goal 이면 MANUAL 전환).

발행 : /goal_pose, /apex_aim_point, /patrol_markers
구독 : /odom, /map, /goal_pose(에코), /fire_alert, /patrol_enable
"""

import math
from collections import deque

import numpy as np
from scipy import ndimage

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point, PointStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool

import tf2_ros


def _yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


# 상태
IDLE, PATROL, MANUAL, FIRE_ALARM = 'IDLE', 'PATROL', 'MANUAL', 'FIRE_ALARM'
INSPECT = 'INSPECT'      # 조난자 후보 확인 — 정지 후 포탑 조준
ESCAPE  = 'ESCAPE'       # 장애물 안에 박힘 — 후진으로 탈출

# 비트 수 표 — 방향 마스크가 0~15 라 표 조회가 가장 빠르다
_POPCOUNT = np.array([bin(i).count('1') for i in range(16)], dtype=np.uint8)


def dir_bit(angle):
    """관측 방향을 4구획 비트로 바꾼다. 동=1, 북=2, 서=4, 남=8.

    한 번 본 칸을 '봤음' 으로만 표시하면, 한 방향에서 스쳐 본 구석도 다시
    안 간다. 가려진 조난자(다른 물체 뒤, 특정 각도에서만 보이는)를 못 찾는
    원인으로 보인다 — 실측으로 2대도 6런 중 3런만 7/7 을 냈고, 미달성 런도
    대부분 6명까지는 찾았다.

    방향을 쌓아 두면 '어느 쪽에서만 봤다' 를 구분할 수 있다. 반대편에서 다시
    보면 가림이 풀린다. 4구획이면 90도씩이라 앞뒤가 확실히 갈린다.

    numpy 불리언이 이미 1바이트라 uint8 비트마스크로 바꿔도 메모리는 같다.
    """
    # -45~45 를 동쪽으로 묶기 위해 45도 밀어서 나눈다
    q = int(math.floor((angle + math.pi / 4) / (math.pi / 2))) % 4
    return 1 << q


def segment_room(free, ry, rx, erode_cells):
    """로봇이 지금 있는 '방' 을 자유공간에서 떼어낸다.

    free        : bool 배열 (자유공간 True)
    ry, rx      : 로봇 셀 좌표
    erode_cells : 침식 반경(셀). **문 폭의 절반보다 커야** 문이 끊긴다.

    반경 고정으로는 안 되는 이유: '주변에 안 본 곳이 없으면 나간다' 의
    '주변' 을 5m 로 두면, 방이 그보다 크면 5m 안만 치우고 나가버린다.
    방 크기는 방마다 다르므로 지도에서 직접 알아내야 한다.

    거리변환을 쓴다. binary_erosion 을 반복하면 4-연결 구조라 마름모꼴로
    깎이고 반복 횟수만큼 느리다. distance_transform_edt 는 한 번에 정확한
    원형 침식을 준다.

    되돌릴 때 팽창을 쓰면 안 된다 — 문틈으로 새어 옆방을 침범한다
    (실측: 왼방 코어를 20셀 팽창하니 오른방을 6셀 침범, 140셀 오염).
    대신 모든 자유공간 셀을 '가장 가까운 속살' 에 귀속시킨다. 이 방식은
    문 한가운데서 자연스럽게 갈려 새지 않는다.
    """
    if free is None or not free[ry, rx]:
        return None
    dist = ndimage.distance_transform_edt(free)
    core = dist > erode_cells
    lbl, n = ndimage.label(core, structure=np.ones((3, 3), bool))
    if n == 0:
        return None
    idx = ndimage.distance_transform_edt(
        lbl == 0, return_distances=False, return_indices=True)
    owner = lbl[idx[0], idx[1]]
    my = owner[ry, rx]
    if my == 0:
        return None
    return (owner == my) & free


def actionable_cells(mask, min_cluster):
    """계획기가 실제로 목표로 삼을 수 있는 셀 수만 센다.

    미관측 격자와 미탐사 경계 둘 다에 쓴다. 두 곳 모두 계획기는 일정 크기
    이상의 군집만 후보로 잡는데, 완료 판정이 자투리까지 세면 로봇이 절대
    지울 수 없는 양이 남아 수색이 영원히 안 끝난다.

    계획기는 min_cluster 셀 이상인 군집만 후보로 잡는다. 그런데 완료 판정이
    자투리까지 전부 세면, 로봇이 절대 지울 수 없는 면적이 남아 수색이
    영원히 안 끝난다.

    실측(큰 월드, 조난자 7/7 을 다 찾은 뒤):
        미관측 군집 5794개, 총 206.0 m^2
          계획기가 갈 수 있음(>=40셀)     61개  174.0 m^2
          너무 작아 목표가 못 됨(<40셀) 5733개   32.0 m^2

    저 32 m^2 가 완료 판정을 영원히 막는다. 두 기준은 반드시 같아야 한다.
    """
    if not mask.any():
        return 0
    lbl, k = ndimage.label(mask, structure=np.ones((3, 3), bool))
    if k == 0:
        return 0
    sizes = np.bincount(lbl.ravel())[1:]
    return int(sizes[sizes >= min_cluster].sum())


def claimed_by_peer(gx, gy, peer_goals, radius):
    """다른 로봇이 이미 그 근처를 목표로 잡았는가.

    지도를 공유해도 목표를 안 나누면 둘이 같은 구역으로 간다(실측: 두 대가
    (-0.4,12.1) 과 (0.1,11.9) 를 각각 잡았다). 그러면 대수를 늘린 값어치가
    없다.

    peer_goals 는 {로봇이름: (x, y)}. 목표가 없는 로봇은 안 들어온다.
    """
    for (px, py) in peer_goals.values():
        if math.hypot(gx - px, gy - py) < radius:
            return True
    return False


def count_unique_victims(entries, merge_r):
    """여러 로봇의 조난자 등록을 합쳐 실제 인원수를 센다.

    entries 는 (로봇, 등록번호, x, y) 목록. 로봇마다 번호가 0 부터 시작하므로
    번호로는 같은 사람인지 알 수 없다. 위치로 묶는다.

    같은 사람을 둘로 세면 실종자 수가 채워진 것처럼 보여 수색이 조기
    종료된다 — 1대에서 겪은 중복 등록 사고와 같은 종류다. 그래서 애매하면
    묶는 쪽(덜 세는 쪽)이 안전하다. 덜 세면 더 찾으러 다닐 뿐이다.
    """
    clusters: list[tuple[float, float]] = []
    for (_, _, x, y) in entries:
        for (cx, cy) in clusters:
            if math.hypot(x - cx, y - cy) < merge_r:
                break
        else:
            clusters.append((x, y))
    return len(clusters)


def stuck_decision(ref, rx, ry, ryaw, now, eps_m, eps_rad, confirm_s,
                   commanded=True):
    """진전이 있었는지 보고 박힘을 판정한다.

    반환: (박힘인가, 새 기준 또는 None)

    회전도 진전으로 쳐야 한다. 예전에는 위치만 봤는데, 목표 방향이 크게
    바뀌어 로봇이 제자리에서 180도 도는 동안에는 위치가 안 변하므로
    '8초간 못 움직임' 으로 오판됐다. 그러면 후진 탈출이 돌아 방금 온 길을
    되짚고, 다시 앞으로 가다 또 돌아서 — 앞뒤로 왕복만 하게 된다.
    목표를 전 지도에서 고르게 되면서 큰 방향 전환이 잦아져 드러난 결함이다.

    각도 차는 반드시 wrap 을 접어야 한다. 3.10 과 -3.10 rad 은 실제로는
    5도 차이인데 그냥 빼면 6.2 rad 이 되어 '크게 돌았다' 고 잘못 본다.
    """
    sx, sy, syaw, st = ref
    dyaw = abs(math.atan2(math.sin(ryaw - syaw), math.cos(ryaw - syaw)))
    if math.hypot(rx - sx, ry - sy) > eps_m or dyaw > eps_rad:
        return False, (rx, ry, ryaw, now)      # 진전 있음 → 기준 갱신
    # Nav2 가 애초에 속도를 안 주고 있으면 박힌 게 아니라 일부러 선 것이다.
    # Nav2 는 경로가 막히면 wait/spin/backup 으로 스스로 복구하는데, 그때
    # 후진 탈출을 걸면 복구를 깨뜨리고 다시 복구가 돌아 무한 반복이 된다.
    # 실측(2대): 박힘 82건, Nav2 복구 wait/spin/backup 각 65/65/64회.
    if not commanded:
        return False, (rx, ry, ryaw, now)      # 기준을 미뤄 누적을 막는다
    if now - st > confirm_s:
        return True, None
    return False, None


def goal_score(kind, n_cells, dist_m, res, view_r, lam):
    """탐사 목표 후보의 점수 — 클수록 좋다. 단위는 m^2.

    후보가 두 종류인데 n 의 단위가 다르다는 것이 함정이다.

      kind='frontier'  n = 미탐사와 맞닿은 '경계선 길이'(1셀 두께)
      kind='visual'    n = 아직 카메라로 못 본 바닥 '면적'

    5m 짜리 구역이면 전자는 약 100셀, 후자는 약 10000셀로 100배 벌어진다.
    그대로 한 점수식에 넣으면 시각 후보가 언제나 이겨 라이다 경계가 영영
    안 뽑히고 지도가 안 넓어진다. 그래서 둘 다 '새로 얻을 넓이' 로 바꾼다.

      시각   넓이 = n * res^2                 (못 본 바닥 그 자체)
      경계   넓이 = n * res * view_r          (경계 길이 x 넘어가면 보일 깊이)

    lam 은 '1m 더 가는 값어치를 몇 m^2 로 볼 것인가' 다. 로봇이 훑으며
    지나가는 폭이 약 1.4m 이므로 그보다 작게 잡아야 먼 곳으로 나간다.
    """
    gain = (n_cells * res * res if kind == 'visual'
            else n_cells * res * view_r)
    return gain - lam * dist_m


def far_first_bonus(in_room, dist_m, coef, cap_m):
    """같은 방 안에서는 '먼 후보' 를 먼저 가도록 얹는 점수[m^2].

    왜 부호를 뒤집나
    ----------------
    기본 점수식은 거리에 페널티를 준다(-lam*d). 그래서 방에 들어가면 입구
    쪽부터 야금야금 훑고, 안쪽이 조금 남은 상태에서 바깥의 더 큰 덩어리에
    져서 방을 뜬다. 남은 안쪽은 나중에 순서가 돌아와야 처리된다.

    실측으로 이게 조난자 한 명을 좌우했다. 방2 는 남쪽 다섯 방 중 유일하게
    내부 칸막이가 있어 문이 주머니 구석에 붙어 있고, 15m 안쪽 끝에 누운
    조난자가 있다. 그 주머니 y<-11 까지 내려간 런은 조난자를 찾았고
    (34런 중 33런), 못 내려간 런은 한 번도 못 찾았다(6런 중 0런).

    방 안에서만 부호를 뒤집으면 끝까지 들어갔다가 나오면서 훑는 동선이 된다.
    같은 방으로 한정하므로 건물을 가로지르는 낭비는 생기지 않는다.

    cap_m 으로 상한을 둔다. 안 두면 방이 클수록 보너스가 무한정 커져서
    '가장 먼 곳' 하나만 계속 이기고 왕복이 는다.
    """
    if not in_room or coef <= 0.0:
        return 0.0
    return coef * min(dist_m, cap_m)


def room_commit_decision(unseen_area, threshold, committed_s, max_s):
    """지금 있는 방에 '눌러앉을' 것인지 정하는 순수 함수.

    왜 보너스가 아니라 커밋인가
    ---------------------------
    앞서 room_bonus(같은 방 후보에 점수를 얹어 주기)를 두 값으로 시험했다.
    1.5 는 점수 단위(m^2)에 비해 너무 작아 아무 일도 안 일어났고, 50 은
    방 이탈 횟수를 확실히 줄였지만(27~35 대 36~44, 12런에서 겹침 없음)
    주머니 깊이도 완주율도 안 움직였다.

    보너스는 경쟁이지 커밋이 아니기 때문이다. 바깥에 더 큰 덩어리가 있으면
    여전히 진다. 실제로 탐욕적 프론티어 선택이 벽·구석·좁은 구조를 놓치는
    것은 알려진 결함이고, 해법으로 제시되는 것은 점수 조정이 아니라
    '커버리지 경로를 만들어 그 계획을 끝까지 수행' 하는 쪽이다.

    안전장치가 둘 필요하다
    ----------------------
    예전에 '반경 5m 안을 먼저 처리' 라는 하드 필터를 썼다가 정반대 고장이
    났다 — 비교 대상을 눈앞으로 제한하니 점수식이 무력화돼 방 하나를
    1.4~4.3m 잔걸음으로 갉아먹으며 나가질 못했다.

    이번 것은 '반경' 이 아니라 '방' 기준이라 방 안에서는 정상 점수식이 그대로
    돌고 큰 덩어리부터 훑는다. 그래도 한 방에 영영 갇히는 것은 막아야 하므로
    체류 시간 상한을 둔다. 남은 미관측이 자투리면 애초에 안 눌러앉는다.

      unseen_area  이 방에 남은 '사람이 숨을 만한' 미관측 넓이[m^2]
      threshold    이보다 커야 눌러앉는다. 0 이면 기능이 꺼진다
      committed_s  이 방에 눌러앉은 시간[s]
      max_s        이 시간을 넘기면 놓아 준다(한 방에 갇히는 것 방지)
    """
    if threshold <= 0.0:
        return False
    if unseen_area < threshold:
        return False
    if max_s > 0.0 and committed_s >= max_s:
        return False
    return True


def sweep_decision(fr_cells, unseen, unseen_budget, done_frontier_cells,
                   goals_done, min_goals, free_area, min_area,
                   victims, expected):
    """수색을 끝낼지·재수색할지·계속할지 정하는 순수 함수.

    노드 상태에 얽혀 있으면 시뮬을 70분씩 돌려야 확인할 수 있다.
    실제로 '회차 완료' 와 '재수색' 두 경로는 커버리지 완료 조건 뒤에
    묶여 있어서, 시간 안에 커버리지가 안 끝나면 둘 다 검증할 수 없었다.
    규칙만 떼어내면 단위 테스트로 확인할 수 있다.

    반환: 'done' | 'resweep' | 'continue'
      done    — 전 구역을 훑었고 인원도 다 찾았다
      resweep — 다 훑었는데 인원이 모자란다 → 놓친 곳이 있다
      continue— 아직 볼 곳이 남았다
    """
    covered = (fr_cells <= done_frontier_cells and unseen <= unseen_budget)
    explored_enough = (goals_done >= min_goals and free_area >= min_area)
    all_found = (expected <= 0 or victims >= expected)
    if covered and explored_enough:
        return 'done' if all_found else 'resweep'
    return 'continue'


# 자기 goal 에코 판별: 같은 좌표가 짧은 시간 안에 되돌아오면 내 것
GOAL_ECHO_TOL    = 0.05   # m
GOAL_ECHO_WINDOW = 5.0    # s
# 수동 goal 을 이 시간 안에 못 가면 포기하고 순찰 복귀(영구 정지 방지)
MANUAL_TIMEOUT_S = 90.0


class PatrolNavigator(Node):

    def __init__(self):
        super().__init__('patrol_navigator')

        # ── 파라미터 ─────────────────────────────────────────────────
        # 순찰 웨이포인트: 메인홀 → RoomA → RoomB → RoomD → RoomC → (반복)
        # 로봇이 둘이면 프레임이 ugv1/map, ugv2/map 으로 갈린다.
        # 기본값은 1대 구성과 같다.
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.map_frame  = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.declare_parameter('waypoints_x', [0.0, -9.0,  9.0,  9.0, -9.0])
        self.declare_parameter('waypoints_y', [0.0,  7.0,  7.0, -7.0, -7.0])
        self.declare_parameter('reach_dist', 0.6)
        self.declare_parameter('alarm_duration', 6.0)
        self.declare_parameter('patrol_enabled_on_boot', True)
        self.declare_parameter('fire_dedup_dist', 1.5)
        self.declare_parameter('wp_timeout', 45.0)   # WP 못 가면 건너뛰기(초)

        # 순찰 방식:
        #   'explore'   — SLAM 맵의 미탐사 경계(프론티어)로 스스로 진출.
        #                 건물 구조를 모른 채 시작해 돌아다니며 맵을 만든다. (기본)
        #   'waypoints' — 아래 절대좌표 웨이포인트 순회. 맵을 이미 아는 전제.
        self.declare_parameter('patrol_mode', 'explore')
        self.declare_parameter('frontier_min_size', 6)      # 프론티어 최소 셀 수
        self.declare_parameter('frontier_replan_s', 6.0)    # 목표 재선정 주기(초)
        self.declare_parameter('inspect_timeout', 30.0)     # 확인 상태 최대 유지(초, 접근 포함)
        # 목표점을 벽에서 떼어놓기 — 벽으로 밀고 드는 것 방지
        self.declare_parameter('frontier_standoff', 0.7)    # 프론티어 경계에서 물러설 거리(m)
        # Nav2 inflation_radius(0.55) 안쪽 지점을 goal 로 주면 플래너가
        # "failed to create plan" 으로 거부한다. 원본 맵에선 자유공간이라
        # 통과하므로, 여유를 inflation 보다 크게 잡아야 한다.
        self.declare_parameter('goal_clearance',   0.70)    # goal 주변이 자유여야 하는 반경(m)
        # 탐사 goal 제한시간 = 기본 + 거리/가정속도.
        # 고정 15초를 쓰다가 맵을 3배(56x40m)로 키운 뒤 완전히 망가졌다.
        # 실측 평균 주행 0.15~0.25 m/s 라 15m 떨어진 프론티어는 60초 이상
        # 걸리는데, 15초에 포기하니 먼 목표는 100% 실패로 처리돼
        # (도달 실패 142회) 로봇이 방을 채우지 못했다.
        self.declare_parameter('explore_goal_timeout', 15.0)   # 기본분(초)
        self.declare_parameter('explore_assumed_speed', 0.25)  # 가정 주행속도(m/s)
        self.declare_parameter('explore_goal_timeout_max', 150.0)
        # 지역 루프 탈출 시 최소 이 거리 이상 떨어진 목표를 고른다(m)
        self.declare_parameter('far_goal_min_dist', 8.0)
        # 목표 점수에서 거리 1m 에 매기는 벌점(m^2). '1m 더 가는 값어치를
        # 몇 m^2 로 보는가'. 로봇이 훑으며 지나가는 폭이 약 1.4m 이므로
        # 그보다 작게 잡아야 먼 미관측 구역으로 나간다.
        # ── 팀 공유(로봇 여러 대) ──────────────────────────────
        # peers 가 비면 1대 구성과 완전히 같다.
        self.declare_parameter('peers', [''])
        # 거짓이면 동료가 있어도 공유를 안 한다. 3단계 효과를 A/B 로 재기
        # 위한 스위치다 — 켠 것과 끈 것을 비교해야 도움이 되는지 알 수 있다.
        self.declare_parameter('team_share', True)
        # 상대 목표에서 이 반경 안의 후보는 고르지 않는다(m).
        self.declare_parameter('peer_claim_radius', 6.0)
        # 두 로봇의 등록을 같은 사람으로 볼 거리(m).
        # 실측으로 두 로봇이 같은 사람을 1.5m 넘게 벌어져 등록해 8/7 이
        # 나왔다. 관측 거리에 따라 오차가 커지므로 조금 넉넉히 잡는다.
        # 덜 세는 쪽이 안전하다 — 더 찾으러 다닐 뿐이다.
        self.declare_parameter('victim_merge_r', 2.2)
        # 탐사 목표를 이 사각형 안으로 제한한다 [xmin, ymin, xmax, ymax].
        # 비우면 제한 없음.
        #
        # ★ 이 로봇은 collision 이 없다(URDF 주석 참조 — 기구학 구동이라
        #   충돌체가 오히려 로봇을 튕겨낸다). 그래서 코스트맵에 벽이 안 찍힌
        #   틈이 있으면 벽을 그냥 통과한다. 실측으로 로봇이 x=-34~-36 에서
        #   관측됐는데 큰 월드 외벽은 x=+-28 이다. 건물 밖으로 나간 것이다.
        #   밖에는 볼 것도 없고 지도도 안 생겨 시간만 버린다.
        self.declare_parameter('explore_bounds', [0.0, 0.0, 0.0, 0.0])
        # 건물 전체 범위. 내 구역을 다 훑으면 여기까지 넓혀 동료를 돕는다.
        # 구역을 하드 필터로만 쓰면 자기 몫을 끝낸 로봇이 그냥 논다.
        # 조난자가 구역마다 고르게 있을 리 없으므로(실측: 중간 월드 서1/동2),
        # 그러면 늦은 쪽이 전체 시간을 정해 2대를 쓴 값어치가 사라진다.
        self.declare_parameter('world_bounds', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('goal_dist_penalty', 0.5)
        # 라이다 경계를 넘었을 때 새로 보이는 깊이(m). 경계 '길이' 를
        # 넓이로 환산할 때 쓴다.
        self.declare_parameter('frontier_view_r', 8.0)
        # 직선 접근이 막혔을 때 대상 주위를 몇 방향까지 뒤질지
        self.declare_parameter('approach_ring_n', 12)
        # 장애물 탈출
        self.declare_parameter('stuck_confirm_s', 8.0)      # 이만큼 안 움직이면 박힘으로 판단
        self.declare_parameter('stuck_move_eps',  0.15)     # 이 이상 움직이면 정상(m)
        # 제자리 회전도 진전으로 친다. 이게 없으면 목표 방향이 크게 바뀌어
        # 로봇이 180도 도는 동안 '안 움직인다' 고 오판해 후진 탈출이 돌고,
        # 앞뒤로 왕복만 하게 된다.
        self.declare_parameter('stuck_turn_eps',  0.30)     # 이 이상 돌면 정상(rad)
        # Nav2 명령 속도가 이보다 작으면 '가라고 안 한 것' 으로 본다.
        self.declare_parameter('stuck_cmd_eps',   0.02)
        self.declare_parameter('escape_speed',    0.35)     # 후진 속도(m/s)
        self.declare_parameter('escape_max_s',    5.0)      # 후진 최대 시간(s)
        self.declare_parameter('escape_min_move', 0.8)      # 이만큼 물러나면 탈출 성공(m)
        # 탈출은 '장애물에 박혔을 때' 를 위한 것이지, Nav2 자체가 죽었을 때
        # 계속 후진하라는 뜻이 아니다. 실제로 Nav2 lifecycle 기동이 실패한
        # 실행에서 탈출이 30회 연속 발동해 로봇을 복도 23m 뒤로 밀어냈다.
        self.declare_parameter('escape_cooldown_s',  25.0)  # 탈출 간 최소 간격
        self.declare_parameter('escape_max_streak',  3)     # 연속 이 횟수 넘으면 중단
        # 조난자에 얼마나 가까이 가서 스캔할지
        # 조난자에서 유지할 거리(m).
        # 카메라가 지면 0.555m, 세로화각 48.8° → 거리 d 에서 보이는 최대 높이는
        # 0.555 + 0.45*d. 1.5m 로 붙으면 1.23m 까지만 보여 서 있는 성인의
        # 머리·어깨가 프레임 위로 잘리고, YOLO 가 어깨를 실제보다 낮게 찍어
        # 서 있는 사람이 '앉음'(L2)으로 과대평가된다.
        # 3.0m 면 1.91m 까지 보여 전신이 들어온다.
        self.declare_parameter('inspect_standoff', 3.0)
        self.declare_parameter('fire_standoff',    2.5)     # 열원에서 유지할 거리(m)
        # 접근 못 하는 대상을 제자리에서 재도 되는 최대 거리. 이보다 멀면
        # 정확도가 급격히 나빠지므로 확인을 보류하고 순찰을 이어간다.
        self.declare_parameter('max_scan_dist',    3.0)
        self.declare_parameter('approach_reach',   0.45)    # 접근 지점 도달 판정(m)
        self.declare_parameter('frontier_reach',   0.8)     # 프론티어 goal 도달 판정(m)
        # ── 시야 커버리지 ────────────────────────────────────────────
        # 라이다(360deg 25m)는 문틈으로 방 안까지 다 그려버린다. 그래서
        # 라이다 프론티어만 쫓으면 "매핑은 됐지만 카메라로는 들여다본 적 없는"
        # 방이 생기고, 조난자를 지나친다. 카메라가 실제로 훑은 격자를 따로
        # 관리해서, 라이다 프론티어가 없어도 안 본 구역으로 계속 들어간다.
        self.declare_parameter('cam_see_range', 4.5)        # 유효 관측 거리(m)
        # 이 거리보다 가까운 바닥은 '봤음' 으로 치지 않는다.
        #
        # 카메라 높이 0.5m, 수직 FOV 48.9도라 수평 조준이면 화면 아래끝이
        # 바닥과 만나는 지점이 1.1m 다. 그보다 가까운 바닥은 아예 안 찍힌다.
        # 누운 사람 검출은 실측으로 2.3m 부터 됐다(2.3m 보다 가까운 성공
        # 0건 / 32건 중).
        #
        # 그런데 지금까지 0.3m 부터 칠하고 있었다. 로봇이 지나가면서 주변
        # 띠를 '봤음' 으로 칠하는데 실제로는 그 바닥을 한 번도 못 본 것이다.
        # 누운 사람이 거기 있으면 영원히 못 찾는다 — 이미 봤다고 표시돼
        # 다시 안 가기 때문이다.
        #
        # 판정이 센서가 못 하는 것을 세면 안 된다. 미탐사 경계·미관측 조각
        # 에서 여섯 번 겪은 것과 같은 종류의 어긋남이다.
        #
        # 기본값 0.3(기존 동작). 2.0 과 비교해 봤지만 **효과가 없어서가
        # 아니라 잴 수 없어서** 그대로 둔다. 포탑을 11.5도 내린 뒤로 누운
        # 조난자를 이미 거의 다 찾아서 더 올릴 여지가 없었다.
        #
        #   큰 월드 2머신 18런(조건당 9런)
        #   누운 3명   2.0m: 22/27   0.3m: 21/27
        #   7/7 달성   4/9 vs 5/9    유령  0건 vs 1건
        #
        # 버그 자체는 실재한다 — 카메라가 못 보는 띠를 '봤음' 으로 칠하면
        # 거기 누운 사람은 영원히 못 찾는다. 검출이 천장에 닿지 않은 조건
        # (더 어두운 월드, 더 작은 대상, 더 빠른 주행)에서는 다시 볼 값어치가
        # 있다. '재봤는데 효과 없었다' 와 '잴 수 없었다' 는 다르다.
        self.declare_parameter('cam_see_min', 0.3)
        self.declare_parameter('cam_fov_rad',   1.089)      # 카메라 수평 FOV
        # 미관측 군집 최소 크기. 카메라 FOV 스윕은 사방에 자잘한 미관측
        # 조각을 항상 남긴다. 작게 잡으면 '주변 미관측' 이 영영 비지 않아
        # 로봇이 제자리에서 조각만 갈아댄다(실측: 목표 35개 중 27개가 주변
        # 미관측, 이동 범위가 56m 건물의 가운데 15m 뿐이었다).
        self.declare_parameter('visual_min_size', 300)      # 셀 수(0.75m^2)
        # 후보로 삼을 미관측 군집의 하한(셀). 구석의 작은 사각지대까지
        # 목표로 삼아야 한다. 이 값이 크면 계획기가 못 잡는 조각이 남아
        # 완료 판정이 영원히 안 선다.
        self.declare_parameter('visual_min_local', 40)      # 셀 수(0.1m^2)
        # 몇 방향에서 봐야 '제대로 봤다' 로 칠지. 1 이면 이 기능이 꺼진다.
        # 2 로 두면 한 방향에서만 스쳐 본 칸을 다시 훑는다 — 가려진 조난자를
        # 찾기 위한 것이다. 대신 수색이 길어진다.
        #
        # 기본값 2(켬). 처음엔 이득이 없어 껐다가 다시 켰다.
        #
        # 처음 쟀을 때는 검출이 병목이었다 — 포탑이 수평이라 누운 조난자를
        # 52% 밖에 못 찾았고, 탐사를 아무리 잘해도 그 벽에 막혀 차이가
        # 안 났다. 포탑을 고친 뒤 다시 재니 값이 드러났다.
        #
        #   메인 15런, 지표는 '7/7 까지 걸린 시간'(발견 인원은 이미 만점)
        #   base  7/7 2/4   1780, 954
        #   dirs  7/7 4/4   874, 1046, 762, 1105    <- 채택
        #   room  7/7 2/3   1267, 1036
        #   both  7/7 3/4   1115, 829, 1098
        #
        # room_bonus 를 같이 켠 both 가 dirs 단독보다 낮다 — 방 우선
        # 보너스는 보태는 것이 없다. 그쪽은 끈 채로 둔다.
        #
        # 아래는 처음 껐을 때의 근거다.
        #   base 5.14  dirs 5.86  (조건당 14런, t≈1.3 → 구분 안 됨)
        # 평균만 보면 앞서지만 머신별로 순위가 뒤집혔다 — 메인에서는 꼴찌
        # (4.8), OMEN 에서는 1위(6.5)였다. 진짜 효과라면 한 머신에서 최고면서
        # 다른 머신에서 최저일 수 없다. 노이즈를 본 것이다.
        # 기능은 남겨 둔다. 가려진 조난자가 실제 문제로 확인되면 켜면 된다.
        self.declare_parameter('seen_min_dirs', 2)
        # 내 구역 밖이라도 이 거리 안이면 간다(0 이면 구역이 하드 경계).
        #
        # 구역을 하드 경계로 두면 코앞의 미탐사도 상대 것이면 안 간다.
        # 도움은 '내 구역을 완전히 비운 뒤' 에야 발동하는데, 그때는 건물을
        # 가로질러야 해서 이동 비용이 이득을 먹는다(실측: 중간 월드에서
        # 12번 넘어간 런이 가장 느렸다 — 866초).
        # 가까우면 경계를 조금 넘도록 두는 편이 자연스럽다.
        self.declare_parameter('cross_border_dist', 6.0)
        # 사각지대(벽 모서리·장애물 뒤)는 여유가 안 나오므로 낮게 잡는다
        self.declare_parameter('visual_clearance', 0.45)
        # 목표 도착 후 그 자리에서 포탑이 훑을 시간(초). 바로 다음 목표로
        # 떠나면 방에 들어갔다 사각지대를 못 보고 나온다.
        self.declare_parameter('arrive_dwell_s', 4.0)
        # '수색 완료' 를 인정하기 위한 최소 근거 (기동 직후 오보 방지)
        self.declare_parameter('min_goals_for_sweep', 8)      # 도달한 목표 수
        self.declare_parameter('min_area_for_sweep', 200.0)   # 매핑된 자유공간 m^2
        # ── 방을 덜 보고 나가는지 측정 (동작은 안 바꾼다) ──────────────
        # 방에 사람이 숨을 만한 미관측이 남았는데 다른 방으로 넘어가면 센다.
        # 벽 뒤 조난자를 놓치는 경로가 이것이라는 관찰이 있었는데, 실제로
        # 몇 번 일어나는지 숫자가 없었다. 고치기 전에 먼저 잰다.
        self.declare_parameter('room_leave_log', True)
        # 침식 반경. **문 폭의 절반보다 커야** 문이 끊겨 방이 갈린다.
        # 이 월드의 문은 1.8m 라 0.9m 초과가 필요하다 — 작게 잡으면 문이
        # 안 끊겨 건물 전체가 한 방으로 나오고, 이탈이 한 번도 안 잡힌다.
        # (tools/test_room_segment.py 가 1.8m 문 / 1.0m 반경으로 확인한다)
        self.declare_parameter('room_erode_m', 1.0)
        # 이보다 작은 미관측은 자투리로 보고 넘어간다. 누운 사람이 약 0.9m^2
        # 라 그보다 조금 작게 잡아 사람이 숨을 수 있는 크기만 센다.
        self.declare_parameter('room_leave_min_area', 0.8)
        # 지금 있는 방 안의 후보에 얹어 주는 점수(m^2). 0 이면 이 기능이 꺼진다.
        #
        # 실측(큰 월드 2대, 4런): 수색 후반에만 방을 덜 보고 나가는 일이
        # 런당 19.3회, 덜 보고 나왔던 방으로 되돌아오는 왕복이 14.3회였다.
        # 왕복은 한 번에 끝냈으면 안 했을 이동이라 순수 낭비다.
        #
        # '보너스' 이지 '필터' 가 아닌 것이 핵심이다. 예전에 '반경 5m 안을
        # 먼저 처리' 라는 하드 필터를 썼다가 정반대 고장이 났다 — 점수식은
        # 이미 큰 덩어리를 선호하는데 필터가 비교 대상을 눈앞으로 제한해
        # 점수식을 무력화했고, 방 하나를 1.4~4.3m 잔걸음으로 갉아먹으며
        # 다른 방으로 넘어가질 못했다. 보너스면 훨씬 좋은 바깥 후보가
        # 여전히 이긴다.
        self.declare_parameter('room_bonus', 0.0)
        # 같은 방 안에서 먼 후보를 먼저 가게 하는 계수[m^2 per m]. 0 이면 꺼짐.
        #
        # room_bonus 와 노리는 것이 다르다. room_bonus 는 '방을 덜 뜨게' 하는데,
        # 실측(12런)에서 이탈 횟수는 확실히 줄었지만(27~35 대 36~44, 겹침 없음)
        # 정작 주머니 침투 깊이도 완주율도 안 움직였다. 이탈 횟수는 성공 런과
        # 실패 런을 안 가른다 — 성공 런도 런당 40회씩 방을 뜬다.
        #
        # 가르는 것은 '방 안쪽 끝까지 들어갔나' 하나였다. 그래서 이 값은
        # 이탈이 아니라 깊이를 직접 겨냥한다.
        # 방에 눌러앉는 기준[m^2]. 0 이면 꺼짐.
        #
        # 이 방에 '사람이 숨을 만한' 미관측이 이보다 많이 남아 있으면, 그
        # 방 밖 후보는 아예 후보에서 뺀다. 보너스가 아니라 커밋이다 —
        # room_bonus 로는 바깥의 더 큰 덩어리에 계속 져서 방을 떴다.
        self.declare_parameter('room_commit_area', 0.0)
        # 한 방에 눌러앉을 수 있는 최대 시간[s]. 갇히는 것을 막는다.
        self.declare_parameter('room_commit_max_s', 240.0)
        self.declare_parameter('room_far_coef', 0.0)
        self.declare_parameter('room_far_cap', 12.0)   # 보너스 상한 거리[m]
        # 실제로 '남은 양' 이 이 이하일 때만 완료로 인정한다
        self.declare_parameter('done_frontier_cells', 40)     # 미탐사 경계 셀
        self.declare_parameter('done_unseen_area', 8.0)       # 미관측 자유공간 m^2
        # 절대 면적만 쓰면 맵 크기에 종속된다. 8m^2 는 작은 월드(320m^2)에선
        # 2.5% 지만 큰 월드(2240m^2)에선 0.36% 라, 전원을 찾고도 완료 보고가
        # 영영 안 났다. 매핑된 자유공간의 비율 기준을 함께 두고 둘 중
        # 느슨한 쪽을 쓴다.
        self.declare_parameter('done_unseen_frac', 0.02)      # 자유공간 대비 2%
        # 구조본부가 알려준 실종자 수. 0이면 모름(면적 기준만 사용).
        # 이 수를 다 찾기 전에는 수색을 끝내지 않고 재수색한다.
        self.declare_parameter('expected_victims', 0)

        xs = list(self.get_parameter('waypoints_x').value)
        ys = list(self.get_parameter('waypoints_y').value)
        self.waypoints = list(zip(xs, ys))
        self.reach_dist     = self.get_parameter('reach_dist').value
        self.alarm_duration = self.get_parameter('alarm_duration').value
        self.enabled        = self.get_parameter('patrol_enabled_on_boot').value
        self.fire_dedup     = self.get_parameter('fire_dedup_dist').value
        self.wp_timeout     = self.get_parameter('wp_timeout').value
        self.patrol_mode    = str(self.get_parameter('patrol_mode').value)
        self.frontier_min   = int(self.get_parameter('frontier_min_size').value)
        self.frontier_replan = float(self.get_parameter('frontier_replan_s').value)
        self.inspect_timeout = float(self.get_parameter('inspect_timeout').value)
        self.frontier_standoff = float(self.get_parameter('frontier_standoff').value)
        self.goal_clearance    = float(self.get_parameter('goal_clearance').value)
        self.inspect_standoff  = float(self.get_parameter('inspect_standoff').value)
        self.fire_standoff     = float(self.get_parameter('fire_standoff').value)
        self.max_scan_dist     = float(self.get_parameter('max_scan_dist').value)
        self.approach_reach    = float(self.get_parameter('approach_reach').value)
        self.frontier_reach    = float(self.get_parameter('frontier_reach').value)
        self.explore_timeout   = float(self.get_parameter('explore_goal_timeout').value)
        self.explore_speed     = float(self.get_parameter('explore_assumed_speed').value)
        self.explore_tmo_max   = float(self.get_parameter('explore_goal_timeout_max').value)
        self.far_goal_min_dist = float(self.get_parameter('far_goal_min_dist').value)
        self.peers = [x for x in self.get_parameter('peers').value if x]
        if not self.get_parameter('team_share').value:
            self.peers = []
        self.peer_claim_r = float(self.get_parameter('peer_claim_radius').value)
        self.victim_merge_r = float(self.get_parameter('victim_merge_r').value)
        b = list(self.get_parameter('explore_bounds').value)
        # 넷이 다 0 이면 제한 없음으로 본다.
        self.explore_bounds = b if len(b) == 4 and any(b) else None
        wb = list(self.get_parameter('world_bounds').value)
        self.world_bounds = wb if len(wb) == 4 and any(wb) else None
        self._helping = False          # 내 구역을 끝내고 동료를 돕는 중인가
        self.goal_dist_penalty = float(self.get_parameter('goal_dist_penalty').value)
        self.frontier_view_r   = float(self.get_parameter('frontier_view_r').value)
        self.approach_ring_n   = int(self.get_parameter('approach_ring_n').value)
        self._explore_budget   = self.explore_timeout   # 현재 goal 의 제한시간(초)
        self.stuck_confirm_s   = float(self.get_parameter('stuck_confirm_s').value)
        self.stuck_move_eps    = float(self.get_parameter('stuck_move_eps').value)
        self.stuck_turn_eps    = float(self.get_parameter('stuck_turn_eps').value)
        self.stuck_cmd_eps     = float(self.get_parameter('stuck_cmd_eps').value)
        self.escape_speed      = float(self.get_parameter('escape_speed').value)
        self.escape_max_s      = float(self.get_parameter('escape_max_s').value)
        self.escape_min_move   = float(self.get_parameter('escape_min_move').value)
        self.escape_cooldown   = float(self.get_parameter('escape_cooldown_s').value)
        self.escape_max_streak = int(self.get_parameter('escape_max_streak').value)
        self.cam_range         = float(self.get_parameter('cam_see_range').value)
        self.cam_min           = float(self.get_parameter('cam_see_min').value)
        self.cam_fov           = float(self.get_parameter('cam_fov_rad').value)
        self.visual_min        = int(self.get_parameter('visual_min_size').value)
        self.visual_min_local  = int(self.get_parameter('visual_min_local').value)
        self.seen_min_dirs     = int(self.get_parameter('seen_min_dirs').value)
        self.cross_border_r    = float(
            self.get_parameter('cross_border_dist').value)
        self.visual_clearance  = float(self.get_parameter('visual_clearance').value)
        self.dwell_s           = float(self.get_parameter('arrive_dwell_s').value)
        self.min_goals_for_sweep = int(self.get_parameter('min_goals_for_sweep').value)
        self.min_area_for_sweep  = float(self.get_parameter('min_area_for_sweep').value)
        self.room_leave_log      = bool(self.get_parameter('room_leave_log').value)
        self.room_erode_m        = float(self.get_parameter('room_erode_m').value)
        self.room_leave_min_area = float(self.get_parameter('room_leave_min_area').value)
        self.room_bonus          = float(self.get_parameter('room_bonus').value)
        self.room_commit_area    = float(
            self.get_parameter('room_commit_area').value)
        self.room_commit_max_s   = float(
            self.get_parameter('room_commit_max_s').value)
        self._commit_room        = None   # 눌러앉은 방의 중심(월드)
        self._commit_since       = None   # 눌러앉기 시작한 시각
        self._commit_logged      = False  # 같은 방 로그 도배 방지
        self.room_far_coef       = float(self.get_parameter('room_far_coef').value)
        self.room_far_cap        = float(self.get_parameter('room_far_cap').value)
        self._room_leaves        = 0      # 덜 보고 나간 횟수
        self._room_left_area     = 0.0    # 그때 남긴 미관측 합계 m^2
        self._t_start            = None   # 첫 계측 시각(경과 시간 기준점)
        # 덜 보고 나온 방들의 중심. 다시 들어오면 왕복한 것이다.
        # 왕복이야말로 진짜 낭비다 — 한 번에 안 끝내서 오가는 것이므로.
        self._left_rooms         = []     # [(cx, cy, 재진입횟수)]
        self._room_reentries     = 0
        self._cur_room           = None   # 직전에 있던 방 중심(바뀜 감지용)
        self.done_frontier_cells = int(self.get_parameter('done_frontier_cells').value)
        self.done_unseen_area    = float(self.get_parameter('done_unseen_area').value)
        self.done_unseen_frac    = float(self.get_parameter('done_unseen_frac').value)
        self.expected_victims    = int(self.get_parameter('expected_victims').value)

        # ── 상태 ─────────────────────────────────────────────────────
        self.state = IDLE
        self.wp_idx = 0
        self.map_ready = False
        self.robot_x = self.robot_y = self.robot_theta = 0.0

        # 자기 goal 에코 필터: 단일 플래그는 발행/수신이 한 번만 어긋나도
        # 영구히 뒤집혀서 자기 goal 을 외부 goal 로 오인한다(→ MANUAL 영구 정지).
        # 좌표+시각으로 대조해 어긋나도 스스로 복구되게 한다.
        # 최근 발행한 goal 들을 (x, y, 시각) 으로 모아둔다. 1개만 기억하면
        # 짧은 간격으로 2개를 쏠 때(_stop_here 직후 탐사 goal 등) 앞의 에코가
        # 뒤 goal 과 대조돼 '외부 goal' 로 오인된다 — 실제로 3회 발생했다.
        self._pub_goals: deque = deque(maxlen=12)
        self._manual_goal = None              # (x,y)
        self._manual_start = None             # MANUAL 진입 시각(초)
        self._far_lock = False                # 원거리 이탈 목표를 붙드는 중
        self._pause_t0 = None                 # 조사/경보로 멈춘 시각
        self._fire_pos = None                 # 현재 경보 대상 (x,y)
        self._alarm_start = None              # 경보 시작 시각(sec)
        self._wp_sent_t = None                # 현재 WP goal 발행 시각(sec)
        self._alarmed_fires: list[tuple] = [] # 이미 경보한 화재들

        # 탐사(frontier) 상태
        self._map_msg = None                  # 최신 OccupancyGrid
        self._frontier_goal = None            # 현재 향하는 goal (벽에서 당긴 점)
        self._frontier_src = None             # 그 goal 을 만든 프론티어 중심 (중복 판정용)
        self._frontier_t = 0.0                # 마지막 목표 선정 시각
        self._dwell_until = None              # 도착 후 훑기 종료 시각
        self._goals_done = 0                  # 실제로 도달한 목표 수
        self._visited_frontiers: list[tuple] = []
        self._explore_done = False
        self._sweeps = 0                      # 건물 전체를 훑은 횟수
        # 확정 조난자 {(로봇, 등록번호): (x, y, label)}.
        # 로봇마다 번호가 0 부터라 로봇 이름까지 키에 넣어야 안 겹친다.
        self._victims: dict[tuple, tuple] = {}
        self._peer_goals: dict[str, tuple] = {}   # 다른 로봇이 향하는 목표
        self._all_found_reported = False      # '전원 발견' 보고를 이미 냈는지
        self._fires_seen: list[tuple] = []    # 확정 화재 [(x, y)]

        # 시야 커버리지 — SLAM 맵과 같은 격자에 정렬해서 유지한다
        self._seen = None                     # uint8 방향 비트마스크 (h, w)
        self._seen_geom = None                # (w, h, res, ox, oy) — 바뀌면 재정렬
        self._turret_yaw = 0.0

        # 조난자 확인(INSPECT) 상태
        self._inspect_pos = None              # 확인 중인 후보 (x,y)
        self._inspect_start = None
        self._state_before_inspect = None
        self._approach_goal = None            # 대상 앞 접근 지점 (x,y)
        self._approach_arrived = False        # 접근 완료 여부

        # 장애물 탈출(ESCAPE) 상태
        self._stuck_ref = None                # (x, y, yaw, t) 마지막 진전 기준점
        self._escape_start = None
        self._escape_from = (0.0, 0.0)
        self._escape_last_t = -1e9            # 마지막 탈출 시각
        self._escape_streak = 0               # 연속 탈출 횟수
        self._nav_down_warned = False

        # ── TF ───────────────────────────────────────────────────────
        self._tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=10))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        # ── 구독 ─────────────────────────────────────────────────────
        self.create_subscription(Odometry,     'odom',        self.odom_cb,   10)
        self.create_subscription(OccupancyGrid, 'map',        self.map_cb,    1)
        self.create_subscription(PoseStamped,   'goal_pose',  self.goal_echo_cb, 10)
        self.create_subscription(PointStamped,  'fire_alert', self.fire_cb,   10)
        self.create_subscription(Bool,          'patrol_enable', self.enable_cb, 10)
        # 조난자 확인 핸드셰이크 (target_manager_node)
        self.create_subscription(PointStamped, 'inspect_request', self.inspect_req_cb,  10)
        self.create_subscription(Bool,         'inspect_done',    self.inspect_done_cb, 10)
        # 열원 확인 핸드셰이크 (fire_detection_node) — 조난자와 같은 흐름,
        # 다만 불에는 너무 가까이 붙지 않도록 standoff 를 따로 둔다
        self.create_subscription(PointStamped, 'fire_candidate',    self.fire_cand_cb, 10)
        # 카메라가 어디를 보는지 알아야 시야 커버리지를 칠할 수 있다
        self.create_subscription(JointState, 'joint_states', self.joint_cb, 10)
        # 수색 결과 요약 보고용 — 확정된 조난자·화재 목록
        self.create_subscription(MarkerArray,  'patient_markers', self.victims_cb, 10)
        self.create_subscription(PointStamped, 'fire_alert',      self.fire_seen_cb, 10)
        self.create_subscription(Bool,         'fire_inspect_done', self.inspect_done_cb, 10)

        # ── 발행 ─────────────────────────────────────────────────────
        self.goal_pub   = self.create_publisher(PoseStamped, 'goal_pose',      10)
        self.aim_pub    = self.create_publisher(Point,       'apex_aim_point', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'patrol_markers', 10)
        # 장애물 탈출용 — 로봇이 장애물 안에 들어가면 Nav2 는 시작 자세가
        # 무효라 경로를 못 낸다. 그때만 직접 후진 명령을 낸다.
        self.cmd_pub    = self.create_publisher(Twist, 'cmd_vel', 10)
        # Nav2 가 지금 실제로 '가라' 고 하는지 본다. 안 가라고 하는 동안
        # 안 움직이는 건 박힌 게 아니라 일부러 선 것이다(복구 동작 중).
        self._cmd_mag = 0.0
        self.create_subscription(Twist, 'cmd_vel', self._cmd_watch, 10)
        # 한 바퀴 수색 완료 신호
        self.sweep_pub  = self.create_publisher(Bool, 'sweep_complete', 10)
        # 카메라로 실제 훑은 구역 / 아직 못 본 구역을 눈으로 구분하기 위한 격자.
        # SLAM 맵은 '라이다가 지나갔나' 만 보여주므로, 방을 통과만 하고 구석을
        # 안 본 경우가 지도상으로는 멀쩡해 보인다. 로그의 '미관측 279m²' 가
        # 어디를 말하는지 화면에서 볼 수 없었다.
        self.cover_pub  = self.create_publisher(OccupancyGrid, 'coverage_map', 1)

        # ── 팀 공유 ────────────────────────────────────────────────
        # 내 목표와 내 관측 격자를 내보내고, 상대 것을 받는다.
        # peers 가 비면(1대 구성) 아무 것도 안 붙는다.
        self.claim_pub = self.create_publisher(PoseStamped, 'explore_claim', 1)
        # 관측 격자는 coverage_map 과 다르다 — 저쪽은 벽과 관측완료가 둘 다
        # -1 이라 '봤다' 를 되읽을 수 없다. 여기선 0/1 로만 낸다.
        self.seen_pub = self.create_publisher(OccupancyGrid, 'seen_grid', 1)
        for peer in self.peers:
            self.create_subscription(
                PoseStamped, f'/{peer}/explore_claim',
                lambda m, p=peer: self.peer_goal_cb(m, p), 1)
            self.create_subscription(
                OccupancyGrid, f'/{peer}/seen_grid',
                lambda m, p=peer: self.peer_seen_cb(m, p), 1)
            self.create_subscription(
                MarkerArray, f'/{peer}/patient_markers',
                lambda m, p=peer: self.victims_cb(m, p), 10)
            # 화재도 합친다. fire_seen_cb 가 이미 2m 안이면 같은 불로 묶으므로
            # 받기만 하면 중복이 안 생긴다. 안 받으면 두 로봇이 같은 불을
            # 각각 세어 정답(4건)보다 많이 보고한다(실측 6/4).
            self.create_subscription(
                PointStamped, f'/{peer}/fire_alert', self.fire_seen_cb, 10)
        if self.peers:
            self.get_logger().info(f'팀 수색 — 동료 {", ".join(self.peers)}')

        self.create_timer(0.5, self.tick)     # 2 Hz FSM
        # 탈출 명령은 20Hz 로 낸다. Nav2 컨트롤러도 /cmd_vel 에 20Hz 로 0을
        # 쏘고 있어서, FSM 주기(2Hz)로 보내면 그 사이 0에 묻혀 로봇이 안 움직인다.
        self.create_timer(0.05, self._escape_cmd_tick)
        # 커버리지 격자는 크고 자주 안 바뀌므로 저주기로만 발행
        self.create_timer(2.0, self._publish_coverage)
        if self.peers:
            self.create_timer(2.0, self._publish_seen)

        if self.patrol_mode == 'explore':
            self.get_logger().info(
                f'patrol_navigator 시작 — 탐사(frontier) 모드, '
                f'enabled={self.enabled}. 맵 없이 시작해 스스로 넓혀갑니다. /map 대기 중...')
        else:
            self.get_logger().info(
                f'patrol_navigator 시작 — 웨이포인트 {len(self.waypoints)}개, '
                f'enabled={self.enabled}. /map 대기 중...')

    # ── 콜백 ─────────────────────────────────────────────────────────
    def odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_theta = _yaw_from_quat(msg.pose.pose.orientation)

    def map_cb(self, msg: OccupancyGrid):
        self._map_msg = msg
        if not self.map_ready:
            self.map_ready = True
            self.get_logger().info('SLAM 맵 수신 — 순찰 준비 완료')

    def inspect_req_cb(self, msg: PointStamped):
        """조난자 후보 발견 → 가까이 접근한 뒤 정지·조준해서 확인."""
        self._inspect_req(msg.point.x, msg.point.y,
                          self.inspect_standoff, '👤 조난자 후보')

    def _inspect_req(self, tx, ty, standoff, label):
        """대상 앞 standoff 지점까지 접근한 뒤 멈춰서 확인.

        멀리서 재면 각도 오차 1°가 거리에 비례해 위치 오차로 커지고, depth 도
        먼 거리에서 부정확하다. 조난자·열원 모두 같은 흐름을 쓰되 서 있는
        거리만 다르다(불에는 더 멀찍이).
        """
        if self.state in (INSPECT, FIRE_ALARM, ESCAPE):
            return
        rx, ry = self._robot_pose()
        self._inspect_pos = (tx, ty)
        self._inspect_start = self._now()
        self._pause_t0 = self._inspect_start      # 가려던 목표를 되살리기 위함
        self._state_before_inspect = self.state
        self.state = INSPECT
        self._approach_arrived = False

        dist = math.hypot(tx - rx, ty - ry)
        approach = None
        if dist > standoff + self.approach_reach:
            approach = self._pull_back(tx, ty, rx, ry, standoff, self.goal_clearance)
        elif dist < standoff - self.approach_reach:
            # 너무 가까우면 물러선다. 화재는 costmap 에 장애물로 칠해지므로
            # (obstacle_radius 1.3m) 그 안에 선 채로 두면 플래너가 막혀
            # "박힘 → 탈출" 을 무한 반복한다(실측: 7분에 11회, 전부 화재 옆).
            approach = self._retreat_point(tx, ty, rx, ry, standoff)
        if approach is None and dist > standoff + self.approach_reach:
            # 직선 접근이 막혔다 → 대상 주위를 둘러 설 자리를 찾는다
            approach = self._approach_ring(tx, ty, rx, ry, standoff,
                                           self.goal_clearance)
            if approach is not None:
                self.get_logger().info(
                    f'{label} 직선 접근 막힘 — 우회 지점 '
                    f'({approach[0]:.1f}, {approach[1]:.1f}) 으로 접근')
        if approach is None and dist > self.max_scan_dist:
            # 접근할 자리를 못 찾았는데 대상이 멀다 → 그 자리에서 재면 부정확하다.
            # 실측: 7.7m 에서 스캔한 건이 산포 0.58m 로 등록됐고 실재하지 않는
            # 조난자였다(정상 등록은 산포 0.00~0.05m). 차라리 넘기고 순찰을
            # 이어가면, 나중에 가까이 지나갈 때 제대로 잡는다.
            self.get_logger().info(
                f'{label} ({tx:.1f}, {ty:.1f}) {dist:.1f}m — 접근 불가하고 너무 멀어 '
                '확인 보류 (순찰 계속)')
            self.state = self._state_before_inspect or PATROL
            self._inspect_pos = None
            return

        if approach is None:
            # 이미 충분히 가까움 → 그 자리에서 스캔
            self._approach_goal = None
            self._approach_arrived = True
            self._stop_here()
            self.get_logger().info(
                f'{label} ({tx:.1f}, {ty:.1f}) {dist:.1f}m — '
                '접근 불필요/불가, 현 위치에서 조준 확인')
        else:
            self._approach_goal = approach
            self._send_goal(approach[0], approach[1],
                            yaw=math.atan2(ty - approach[1], tx - approach[0]))
            self.get_logger().info(
                f'{label} ({tx:.1f}, {ty:.1f}) {dist:.1f}m — '
                f'({approach[0]:.1f}, {approach[1]:.1f}) 까지 접근 후 확인')

    def joint_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name == 'turret_yaw_joint':
                self._turret_yaw = pos

    # ── 시야 커버리지 ────────────────────────────────────────────────
    def _sync_seen(self, g):
        """SLAM 맵 격자가 바뀌면(맵이 자라면) 커버리지 격자를 재정렬한다."""
        geom = (g.info.width, g.info.height, g.info.resolution,
                g.info.origin.position.x, g.info.origin.position.y)
        if self._seen_geom == geom:
            return
        # 방향 비트마스크(동1 북2 서4 남8). 불리언과 같은 1바이트다.
        new = np.zeros((g.info.height, g.info.width), dtype=np.uint8)
        if self._seen is not None and self._seen_geom is not None:
            ow, oh, _ores, oox, ooy = self._seen_geom
            # 기존 커버리지를 새 격자 좌표로 옮긴다 (해상도는 동일 전제)
            dx = int(round((oox - geom[3]) / geom[2]))
            dy = int(round((ooy - geom[4]) / geom[2]))
            h = min(oh, geom[1] - dy)
            w = min(ow, geom[0] - dx)
            if h > 0 and w > 0 and dx >= 0 and dy >= 0:
                new[dy:dy + h, dx:dx + w] = self._seen[:h, :w]
        self._seen = new
        self._seen_geom = geom

    def _update_seen(self, rx, ry):
        """카메라 FOV 부채꼴을 '봤음' 으로 칠한다. 벽에 막히면 거기서 끊는다."""
        g = self._map_msg
        if g is None:
            return
        self._sync_seen(g)
        res = g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        W, H = g.info.width, g.info.height
        occ = np.asarray(g.data, dtype=np.int16).reshape(H, W) >= 65

        cam = self._robot_yaw_map() + self._turret_yaw
        half = self.cam_fov / 2.0
        # 광선이 향하는 쪽이 아니라, 그 칸에서 로봇을 바라보는 쪽을 적는다.
        # '어느 방향에서 이 칸을 봤나' 가 가림을 판단하는 기준이기 때문이다.
        n_rays = 21
        step = res * 0.9
        for i in range(n_rays):
            a = cam - half + i * self.cam_fov / (n_rays - 1)
            ca, sa = math.cos(a), math.sin(a)
            # 가까운 바닥은 카메라에 안 잡히므로 칠하지 않는다.
            # 다만 벽 판정은 로봇 바로 앞부터 해야 한다 — 안 그러면
            # 바로 앞 벽을 건너뛰고 그 너머를 봤다고 칠한다.
            d = 0.3
            while d <= self.cam_range:
                ix = int((rx + d * ca - ox) / res)
                iy = int((ry + d * sa - oy) / res)
                if not (0 <= ix < W and 0 <= iy < H):
                    break
                if occ[iy, ix]:
                    break                      # 벽 뒤는 못 본다
                if d < self.cam_min:
                    d += step
                    continue                   # 너무 가까워 화면에 안 들어옴
                self._seen[iy, ix] |= dir_bit(a + math.pi)
                d += step

    def _seen_enough(self):
        """충분히 본 칸의 불리언 격자. 방향 수가 기준 이상인 칸만 참."""
        if self._seen is None:
            return None
        if self.seen_min_dirs <= 1:
            return self._seen > 0
        # 켜진 비트 수를 센다 (0~15 이므로 표를 쓰는 게 빠르다)
        return _POPCOUNT[self._seen] >= self.seen_min_dirs

    def _coverage_left(self):
        """아직 남은 수색량을 (미탐사 경계 셀 수, 미관측 자유공간 m^2) 로 반환.

        '후보가 없다' 와 '다 훑었다' 는 다르다. _pick_frontier 는 방문 기록·
        거리·여유 조건으로 후보를 걸러내므로, 남은 구역이 있어도 일시적으로
        빈 목록을 반환한다. 그걸 완료로 처리해서 기동 198초 만에
        "수색 완료 — 조난자 0명" 이 나왔고, 바로 다음 tick 에 새 목표가
        잡혔다. 완료 판정은 후보 유무가 아니라 실제 남은 양으로 해야 한다.
        """
        g = self._map_msg
        if g is None:
            return (10 ** 9, 10 ** 9)
        W, H = g.info.width, g.info.height
        res = g.info.resolution
        arr = np.asarray(g.data, dtype=np.int16).reshape(H, W)
        free = (arr >= 0) & (arr < 25)
        unknown = arr < 0
        nb = np.zeros_like(unknown)
        nb[:, :-1] |= unknown[:, 1:]
        nb[:, 1:] |= unknown[:, :-1]
        nb[:-1, :] |= unknown[1:, :]
        nb[1:, :] |= unknown[:-1, :]
        frontier = free & nb
        # 탐사 범위 밖의 경계는 세지 않는다. 목표로 삼는 걸 막아 놓고
        # 완료 판정에서는 세면 영원히 완료가 안 선다 — 자투리 미관측 때와
        # 똑같은 어긋남이다(실측: 미탐사 경계 220셀이 기준 40 아래로 안 내려감).
        if self._active_bounds() is not None:
            x0, y0, x1, y1 = self._active_bounds()
            ox = g.info.origin.position.x
            oy = g.info.origin.position.y
            xs = ox + (np.arange(W) + 0.5) * res
            ys = oy + (np.arange(H) + 0.5) * res
            inx = (xs >= x0) & (xs <= x1)
            iny = (ys >= y0) & (ys <= y1)
            frontier &= iny[:, None] & inx[None, :]
        # 계획기가 목표로 삼을 수 있는 크기의 군집만 센다.
        # _find_frontiers 는 frontier_min 셀 이상인 군집만 후보로 잡는데
        # 완료 판정이 셀을 통째로 세면, 로봇이 절대 지울 수 없는 경계가
        # 남아 수색이 영원히 안 끝난다. 자투리 미관측 때와 같은 어긋남이다
        # (실측: 미니맵에서 경계 143셀이 기준 40 아래로 안 내려가고 오히려
        #  131 -> 143 으로 늘었다).
        # 크기 하한을 넘어도, 계획기가 접근점을 못 찾는 군집은 목표가 될 수
        # 없다. 실측(미니맵): 남은 경계 14군집이 전부 x~+-8, y~+-5 로 벽에
        # 붙은 띠였다. 라이다가 찍은 마지막 자유 줄과 벽 안쪽 미탐사가 맞닿는
        # 자리라, 지우려면 벽 속으로 들어가야 해서 원리적으로 못 지운다.
        # _pick_frontier 는 _pull_back 으로 이런 후보를 이미 걸러낸다.
        # 판정도 같은 필터를 거쳐야 한다.
        frontier_cells = 0
        if frontier.any():
            lbl, k = ndimage.label(frontier, structure=np.ones((3, 3), bool))
            sizes = np.bincount(lbl.ravel())
            big = [i for i in range(1, k + 1) if sizes[i] >= self.frontier_min]
            if big:
                rx, ry = self._robot_pose()
                ox = g.info.origin.position.x
                oy = g.info.origin.position.y
                for c, i in zip(ndimage.center_of_mass(frontier, lbl, big), big):
                    fx = ox + (c[1] + 0.5) * res
                    fy = oy + (c[0] + 0.5) * res
                    # 이미 가본 경계는 계획기가 다시 안 고른다(_pick_frontier
                    # 의 1.5m 중복 판정). 가봤는데도 남아 있다면 지울 수 없는
                    # 것이다 — 벽 안쪽 미탐사처럼. 판정도 같이 빼야 한다.
                    # 실측: 접근가능 필터만으로는 벽 띠가 통과했다. 벽에서
                    # 1m 떨어져 설 수는 있으니 _pull_back 이 성공한다.
                    # '접근이 된다' 와 '지울 수 있다' 는 다르다.
                    if any(math.hypot(fx - vx, fy - vy) < 1.5
                           for vx, vy in self._visited_frontiers):
                        continue
                    if self._pull_back(fx, fy, rx, ry,
                                       self.frontier_standoff,
                                       self.goal_clearance) is not None:
                        frontier_cells += int(sizes[i])
        seen_ok = self._seen_enough()
        if seen_ok is None or seen_ok.shape != free.shape:
            unseen_area = float(free.sum()) * res * res
        else:
            # 계획기가 목표로 삼을 수 있는 크기의 군집만 센다.
            # 자투리까지 세면 로봇이 절대 못 지우는 면적이 남아 수색이
            # 영원히 안 끝난다(실측: 32 m^2 가 5733개 조각으로 흩어져 있었다).
            # 이 하한은 _find_visual_frontiers 에 주는 값과 같아야 한다.
            unseen_mask = free & ~seen_ok
            # 미관측도 탐사 범위 안만 센다. 구역을 갈라 배정하면 상대
            # 구역은 내가 절대 못 가는데, 그걸 세면 완료가 영원히 안 선다.
            # 경계 셀에는 범위를 적용해 놓고 여기만 빠뜨렸다(실측: 구역분할
            # 2대가 조난자를 다 찾고도 '미관측 135m² 남음' 에서 멈췄다).
            if self._active_bounds() is not None:
                bx0, by0, bx1, by1 = self._active_bounds()
                ox = g.info.origin.position.x
                oy = g.info.origin.position.y
                xs_m = ox + (np.arange(W) + 0.5) * res
                ys_m = oy + (np.arange(H) + 0.5) * res
                inx = (xs_m >= bx0) & (xs_m <= bx1)
                iny = (ys_m >= by0) & (ys_m <= by1)
                unseen_mask = unseen_mask & (iny[:, None] & inx[None, :])
            cells = actionable_cells(unseen_mask, self.visual_min_local)
            unseen_area = float(cells) * res * res
        return (frontier_cells, unseen_area)

    def _publish_seen(self):
        """내가 눈으로 훑은 구역을 0/1 격자로 내보낸다(동료가 합칠 수 있게).

        coverage_map 을 재활용하지 않는 이유: 거기선 벽과 관측완료가 둘 다
        -1 이라 되읽으면 '봤다' 와 '벽' 을 구분할 수 없다.
        """
        g = self._map_msg
        if g is None or self._seen is None:
            return
        m = OccupancyGrid()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.info = g.info
        m.data = self._seen.astype(np.int8).reshape(-1).tolist()  # 0~15
        self.seen_pub.publish(m)

    def _publish_coverage(self):
        """수색 커버리지를 /coverage_map 으로 발행 (RViz Map 디스플레이용).

        게임의 전장의 안개처럼 '아직 안 본 곳' 을 어둡게 덮는다.
        RViz 'map' 색상표는 값이 클수록 어둡고 -1 은 투명이므로:
          100 = 미탐사(가본 적 없음)        → 완전한 검정
           55 = 라이다만 지나감, 눈으로 미확인 → 회색 안개
           -1 = 카메라로 확인 완료 / 벽      → 투명 (SLAM 맵이 그대로 보임)
        중간 단계를 둔 이유는 둘이 전혀 다른 상태이기 때문이다. SLAM 맵만
        보면 방을 통과만 해도 다 아는 것처럼 보이지만, 구석을 눈으로 안
        봤으면 조난자를 놓친 것이다.
        """
        g = self._map_msg
        if g is None:
            return
        W, H = g.info.width, g.info.height
        arr = np.asarray(g.data, dtype=np.int16).reshape(H, W)
        free = (arr >= 0) & (arr < 25)
        unknown = arr < 0
        out = np.full((H, W), -1, dtype=np.int8)
        out[unknown] = 100                       # 가본 적 없음 = 짙은 안개
        seen_ok = self._seen_enough()
        if seen_ok is not None and seen_ok.shape == free.shape:
            out[free & ~seen_ok] = 55         # 지나갔지만 눈으로 못 봄
        else:
            out[free] = 55
        m = OccupancyGrid()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.info = g.info
        m.data = out.reshape(-1).tolist()
        self.cover_pub.publish(m)

    def _known_free_area(self) -> float:
        """지금까지 매핑된 자유공간 넓이(m^2). 수색 완료 판정의 최소 근거."""
        g = self._map_msg
        if g is None:
            return 0.0
        arr = np.asarray(g.data, dtype=np.int16)
        free = int(((arr >= 0) & (arr < 25)).sum())
        return free * g.info.resolution ** 2

    def _find_visual_frontiers(self, min_size=None):
        """자유공간인데 카메라로 아직 안 본 구역의 군집 중심.

        라이다 프론티어가 다 없어져도(=매핑 완료) 여기 남아 있으면
        아직 수색이 끝난 게 아니다. 방을 실제로 들여다보게 만드는 핵심.

        min_size 로 군집 크기 하한을 바꿀 수 있다. 방 안을 마무리할 때는
        작은 조각까지 봐야 하고(작은 값), 멀리 나갈 곳을 고를 때는 조각을
        무시해야 한다(큰 값).

        이 하한과 완료 판정(_coverage_left)의 셈이 어긋나면 안 된다.
        하한을 300셀(0.75m^2)로 두면 그보다 작은 조각은 계획기가 아예
        후보로 잡지 않는데 완료 판정은 그 면적을 계속 센다. 로봇이 절대
        못 지우는 바닥이 생겨 미관측이 300~400m^2 에서 멈춘다(실측 3회).
        """
        g = self._map_msg
        if g is None or self._seen is None:
            return []
        W, H = g.info.width, g.info.height
        res = g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        arr = np.asarray(g.data, dtype=np.int16).reshape(H, W)
        seen_ok = self._seen_enough()
        if seen_ok is None or seen_ok.shape != arr.shape:
            return []
        target = (arr >= 0) & (arr < 25) & (~seen_ok)
        if not target.any():
            return []
        lbl, n = ndimage.label(target, structure=np.ones((3, 3), dtype=bool))
        if n == 0:
            return []
        sizes = np.bincount(lbl.ravel())
        lo = self.visual_min if min_size is None else min_size
        idx = [i for i in range(1, n + 1) if sizes[i] >= lo]
        if not idx:
            return []
        cents = ndimage.center_of_mass(target, lbl, idx)
        return [(ox + (c[1] + 0.5) * res, oy + (c[0] + 0.5) * res, int(sizes[i]))
                for c, i in zip(cents, idx)]

    def victims_cb(self, msg: MarkerArray, src: str = 'self'):
        """환자 마커에서 확정 목록을 뽑는다.

        src 로 어느 로봇이 등록했는지 구분한다. 로봇마다 등록번호가 0 부터
        시작하므로 번호만으로는 같은 사람인지 알 수 없다. 인원수는 위치로
        묶어서 센다(count_unique_victims).
        """
        for m in msg.markers:
            if m.ns != 'patient_text' or m.action != Marker.ADD:
                continue
            pid = m.id // 3
            self._victims[(src, pid)] = (
                m.pose.position.x, m.pose.position.y,
                m.text.split('\n')[0] if m.text else '')
        # 실종자 수를 채운 순간 바로 보고한다(커버리지를 기다리지 않는다)
        if self.expected_victims > 0 and self._victim_count() >= self.expected_victims:
            self._report_all_found()

    def _victim_count(self) -> int:
        """팀 전체가 찾은 실제 인원수. 같은 사람을 둘이 등록했으면 하나로 센다."""
        return count_unique_victims(
            [(k[0], k[1], v[0], v[1]) for k, v in self._victims.items()],
            self.victim_merge_r)

    def peer_goal_cb(self, msg: PoseStamped, peer: str):
        """다른 로봇이 지금 향하는 목표. 같은 구역으로 겹쳐 가지 않기 위함."""
        self._peer_goals[peer] = (msg.pose.position.x, msg.pose.position.y)

    def peer_seen_cb(self, msg: OccupancyGrid, peer: str):
        """다른 로봇이 눈으로 훑은 구역. 내 관측 기록에 합친다.

        한 로봇이 이미 들여다본 방을 다른 로봇이 다시 갈 이유가 없다.
        두 로봇이 같은 병합 지도(/map)를 쓰므로 격자 모양이 같아 그대로
        겹칠 수 있다. 모양이 다르면(지도가 막 커진 직후) 건너뛴다.
        """
        if self._seen is None:
            return
        H, W = self._seen.shape
        if msg.info.height != H or msg.info.width != W:
            return
        a = np.asarray(msg.data, dtype=np.uint8).reshape(H, W)
        self._seen |= a          # 방향 비트까지 합쳐진다

    def fire_seen_cb(self, msg: PointStamped):
        fx, fy = msg.point.x, msg.point.y
        for (x, y) in self._fires_seen:
            if math.hypot(x - fx, y - fy) < 2.0:
                return
        self._fires_seen.append((fx, fy))

    def fire_cand_cb(self, msg: PointStamped):
        """열원 후보 — 조난자와 같은 정지·조준 확인. 다만 더 멀찍이 선다."""
        self._inspect_req(msg.point.x, msg.point.y,
                          self.fire_standoff, '🔥 열원 후보')

    def inspect_done_cb(self, msg: Bool):
        if self.state != INSPECT:
            return
        self.get_logger().info(
            '조난자 확인 완료 — 순찰 재개' if msg.data
            else '확인 실패(놓침) — 순찰 재개')
        self._inspect_pos = None
        self._approach_goal = None
        self._approach_arrived = False
        self._resume_patrol()

    def enable_cb(self, msg: Bool):
        self.enabled = msg.data
        self.get_logger().info(f'/patrol_enable = {msg.data}')
        if not self.enabled and self.state == PATROL:
            self.state = IDLE

    def goal_echo_cb(self, msg: PoseStamped):
        gx, gy = msg.pose.position.x, msg.pose.position.y
        now = self._now()
        for (lx, ly, lt) in self._pub_goals:
            if (math.hypot(gx - lx, gy - ly) < GOAL_ECHO_TOL
                    and now - lt < GOAL_ECHO_WINDOW):
                return                         # 내가 쏜 goal 의 에코 → 무시
        # 외부(RViz/CLI) goal → 수동 우선
        self._manual_goal = (gx, gy)
        self._manual_start = self._now()
        self.state = MANUAL
        self.get_logger().info(
            f'외부 goal 수신 → MANUAL ({self._manual_goal[0]:.1f}, {self._manual_goal[1]:.1f})')

    def fire_cb(self, msg: PointStamped):
        fx, fy = msg.point.x, msg.point.y
        for (ax, ay) in self._alarmed_fires:
            if math.hypot(ax - fx, ay - fy) < self.fire_dedup:
                return                          # 이미 경보한 화재
        self._alarmed_fires.append((fx, fy))
        self._fire_pos = (fx, fy)
        self._alarm_start = self._now()
        self._pause_t0 = self._alarm_start       # 가려던 목표를 되살리기 위함
        self.state = FIRE_ALARM
        self.get_logger().warn(
            f'🚨 화재 경보! ({fx:.1f}, {fy:.1f}) — 순찰 정지, 포탑 조준')
        self._stop_here()                       # 현재 위치를 goal 로 → 정지

    # ── 유틸 ─────────────────────────────────────────────────────────
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _robot_pose(self):
        try:
            tf = self._tf_buf.lookup_transform(self.map_frame, self.base_frame, Time())
            t = tf.transform.translation
            return t.x, t.y
        except Exception:
            return self.robot_x, self.robot_y

    def _robot_yaw_map(self):
        """map 프레임 기준 로봇 방위.

        정지 goal 의 위치는 map TF 에서 오는데 방위를 odom(self.robot_theta)에서
        가져오면 SLAM 보정분만큼 어긋나, Nav2 가 자세를 맞추려고 제자리에서
        계속 회전한다(정지 판정이 안 됨).
        """
        try:
            q = self._tf_buf.lookup_transform(
                self.map_frame, self.base_frame, Time()).transform.rotation
            return _yaw_from_quat(q)
        except Exception:
            return self.robot_theta

    def _nav2_alive(self) -> bool:
        """bt_navigator 가 /goal_pose 구독자다 — 0명이면 Nav2 가 안 떴다는 뜻.

        Nav2 lifecycle 전환이 타임아웃(CPU 과부하 등)나면 스택이 unconfigured 로
        남는데, 그때 goal 을 계속 쏘면 '가긴 가는데 못 감'처럼 보여 원인을 놓친다.
        """
        alive = self.goal_pub.get_subscription_count() > 0
        if not alive:
            self.get_logger().error(
                'Nav2 가 /goal_pose 를 구독하지 않음 — 내비게이션 스택이 '
                '올라오지 않았다(lifecycle 미기동 의심). 목표를 보내도 움직일 수 '
                '없으니 nav2 로그와 CPU 부하를 확인할 것.',
                throttle_duration_sec=15.0)
        return alive

    def _send_goal(self, x, y, yaw=None):
        self._nav2_alive()
        if yaw is None:
            rx, ry = self._robot_pose()
            yaw = math.atan2(y - ry, x - rx)
        g = PoseStamped()
        g.header.frame_id = self.map_frame
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = float(x)
        g.pose.position.y = float(y)
        g.pose.orientation.z = math.sin(yaw / 2.0)
        g.pose.orientation.w = math.cos(yaw / 2.0)
        self._pub_goals.append((float(x), float(y), self._now()))
        self.goal_pub.publish(g)

    def _stop_here(self):
        rx, ry = self._robot_pose()
        self._send_goal(rx, ry, yaw=self._robot_yaw_map())

    # ── 맵 조회 ──────────────────────────────────────────────────────
    def _cell_value(self, x, y):
        """world (x,y) 의 맵 셀 값. 맵 밖이면 None."""
        g = self._map_msg
        if g is None:
            return None
        ix = int((x - g.info.origin.position.x) / g.info.resolution)
        iy = int((y - g.info.origin.position.y) / g.info.resolution)
        if not (0 <= ix < g.info.width and 0 <= iy < g.info.height):
            return None
        return g.data[iy * g.info.width + ix]

    def _is_free(self, x, y, clearance=0.0):
        """(x,y) 가 자유공간인지. clearance>0 이면 주변까지 자유여야 한다."""
        v = self._cell_value(x, y)
        if v is None or not (0 <= v < 25):
            return False
        if clearance <= 0.0:
            return True
        c = clearance
        d = c * 0.707                      # 대각선도 확인 — 벽 모서리 대비
        for dx, dy in ((c, 0), (-c, 0), (0, c), (0, -c),
                       (d, d), (d, -d), (-d, d), (-d, -d)):
            v = self._cell_value(x + dx, y + dy)
            if v is None or v >= 25:      # 미탐사(-1)·점유 둘 다 불가
                return False
        return True

    def _pull_back(self, tx, ty, rx, ry, standoff, clearance):
        """목표점을 로봇 쪽으로 standoff 만큼 당겨 자유공간에 놓는다.

        프론티어 중심이나 조난자 위치는 그 자체가 벽에 붙어 있거나 미탐사
        영역이라 그대로 goal 로 쓰면 Nav2 가 도달하지 못하고 벽으로 밀고 든다.
        """
        d = math.hypot(tx - rx, ty - ry)
        if d < 1e-3:
            return None
        ux, uy = (tx - rx) / d, (ty - ry) / d
        # standoff 부터 시작해 조금씩 더 당기며 자유공간을 찾는다
        back = standoff
        while back < d:
            px, py = tx - ux * back, ty - uy * back
            if self._is_free(px, py, clearance):
                return (px, py)
            back += 0.25
        return None

    def _approach_ring(self, tx, ty, rx, ry, standoff, clearance):
        """대상을 중심으로 반경 standoff 원 위에서 접근 가능한 지점을 찾는다.

        _pull_back 은 로봇→대상 '직선' 위만 뒤지므로, 그 선이 잔해나 벽으로
        막히면 곧바로 포기한다. 그러면 조난자를 눈앞에 두고도 확인을 보류하고
        지나간다(실측: 3.2m 앞 후보를 '접근 불가'로 건너뜀).
        대상 주위 어느 방향이든 설 자리가 있으면 거기서 확인하면 된다.
        로봇에 가까운 방향부터 시도해 불필요하게 돌아가지 않게 한다.
        """
        base = math.atan2(ry - ty, rx - tx)      # 대상에서 로봇을 보는 방향
        step_ang = 2.0 * math.pi / self.approach_ring_n
        best = None
        best_d = float('inf')
        # base(정면)부터 시작해 좌우로 대칭으로 벌려간다.
        for k in range(self.approach_ring_n // 2 + 1):
            delta = k * step_ang
            for sign in ((1,) if k == 0 else (1, -1)):
                ang = base + sign * delta
                px = tx + standoff * math.cos(ang)
                py = ty + standoff * math.sin(ang)
                if not self._is_free(px, py, clearance):
                    continue
                d = math.hypot(px - rx, py - ry)
                if d < best_d:
                    best_d, best = d, (px, py)
            if best is not None and delta > math.pi / 2:
                break            # 충분히 돌아봤고 답이 있으면 멈춘다
        return best

    def _report_all_found(self):
        """실종자 수를 다 채운 순간 곧바로 보고한다(커버리지와 무관).

        커버리지까지 끝나야 보고하면, 전원을 찾고도 한참을 말없이 돌아
        운용자는 로봇이 왜 계속 도는지 알 수 없다. 실제로 7명을 다 찾은 뒤
        미관측 279m² 를 메우느라 계속 순찰했다.
        구조 판단에 필요한 정보는 '몇 명을 어디서 찾았나' 이므로 그 시점에
        바로 내보내고, 남은 구역은 '보충 수색' 으로 이어간다(명단에 없는
        조난자가 있을 수 있어 수색 자체는 멈추지 않는다).
        """
        if self._all_found_reported:
            return
        self._all_found_reported = True
        vics = sorted(self._victims.items())
        by_lvl: dict[str, int] = {}
        for _, (_, _, lbl) in vics:
            by_lvl[lbl] = by_lvl.get(lbl, 0) + 1
        _, unseen = self._coverage_left()
        lines = [f'🏁 전원 발견! 조난자 {self._victim_count()}/{self.expected_victims}명 '
                 '확인 — 구조 대기',
                 '  ' + ' / '.join(f'{k} {v}명' for k, v in sorted(by_lvl.items())),
                 f'  화재 {len(self._fires_seen)}건']
        for pid, (x, y, lbl) in vics:
            lines.append(f'   · #{pid} {lbl} ({x:.1f}, {y:.1f})')
        lines.append(f'  (미관측 {unseen:.0f}m² 남음 — 보충 수색 계속)')
        self.get_logger().info('\n'.join(lines))
        m = Bool(); m.data = True
        self.sweep_pub.publish(m)

    def _report_mission(self, sweep_n: int):
        """건물을 한 바퀴 다 훑을 때마다 수색 결과를 요약 보고한다.

        기존에는 미탐사 경계가 없어져도 아무 말 없이 재순찰만 반복해서,
        운용자가 '수색이 끝났는지' 를 알 수 없었다. 구조 임무에서는 이게
        가장 중요한 정보다. 순찰 자체는 감시를 위해 계속 돈다.
        """
        vics = sorted(self._victims.items())
        fires = list(self._fires_seen)
        lines = [f'━━ 수색 {sweep_n}회차 완료 — 미탐사·미관측 구역 없음 ━━',
                 f'  조난자 {self._victim_count()}명, 화재 {len(fires)}건']
        for pid, (x, y, lbl) in vics:
            lines.append(f'   · #{pid} {lbl} ({x:.1f}, {y:.1f})')
        for i, (x, y) in enumerate(fires):
            lines.append(f'   · 🔥 화재{i} ({x:.1f}, {y:.1f})')
        lines.append('  순찰은 감시를 위해 계속합니다.')
        self.get_logger().info('\n'.join(lines))
        m = Bool(); m.data = True
        self.sweep_pub.publish(m)

    def _retreat_point(self, tx, ty, rx, ry, standoff):
        """대상에서 standoff 만큼 떨어진 자리로 물러설 지점.

        로봇이 이미 대상보다 가까이 있을 때 쓴다. 대상→로봇 방향으로
        standoff 지점을 잡고, 막혀 있으면 조금씩 더 물러나며 자유공간을 찾는다.
        """
        d = math.hypot(rx - tx, ry - ty)
        if d < 1e-3:
            return None
        ux, uy = (rx - tx) / d, (ry - ty) / d
        back = standoff
        while back <= standoff + 2.0:
            px, py = tx + ux * back, ty + uy * back
            if self._is_free(px, py, self.goal_clearance):
                return (px, py)
            back += 0.25
        return None

    # ── 프론티어 탐사 ────────────────────────────────────────────────
    def _find_frontiers(self):
        """SLAM 맵에서 '알려진 자유공간 ↔ 미탐사' 경계 셀을 군집화해 반환.

        맵을 미리 알 필요 없이, 지금까지 그린 지도의 가장자리로 나아가며
        스스로 탐사 범위를 넓힌다. 반환: [(x, y, 셀수), ...] (map 좌표)

        numpy/scipy 로 벡터화되어 있다. 순수 파이썬 이중 루프 + BFS 로 짜면
        셀 수에 비례해 급격히 느려져, 맵을 키우면 2Hz FSM 주기를 넘겨버린다.
        (556x396=22만 셀에서 이미 수백 ms) 큰 맵을 쓰려면 이 벡터화가 전제다.
        """
        grid = self._map_msg
        if grid is None:
            return []
        w, h = grid.info.width, grid.info.height
        res  = grid.info.resolution
        ox, oy = grid.info.origin.position.x, grid.info.origin.position.y

        g = np.asarray(grid.data, dtype=np.int16).reshape(h, w)
        free    = (g >= 0) & (g < 25)
        unknown = (g < 0)

        # 상하좌우 중 하나라도 미탐사와 맞닿은 자유 셀 = 프론티어
        nb = np.zeros_like(unknown)
        nb[:, :-1] |= unknown[:, 1:]     # 오른쪽
        nb[:, 1:]  |= unknown[:, :-1]    # 왼쪽
        nb[:-1, :] |= unknown[1:, :]     # 위
        nb[1:, :]  |= unknown[:-1, :]    # 아래
        frontier = free & nb
        if not frontier.any():
            return []

        # 8방향 연결 성분으로 군집화
        lbl, n = ndimage.label(frontier, structure=np.ones((3, 3), dtype=bool))
        if n == 0:
            return []
        sizes = np.bincount(lbl.ravel())
        idx = [i for i in range(1, n + 1) if sizes[i] >= self.frontier_min]
        if not idx:
            return []
        cents = ndimage.center_of_mass(frontier, lbl, idx)   # (row, col) 순
        return [(ox + (c[1] + 0.5) * res, oy + (c[0] + 0.5) * res, int(sizes[i]))
                for c, i in zip(cents, idx)]

    def _pick_frontier(self, rx, ry):
        """가까우면서 충분히 큰 프론티어 선택 (이미 시도한 곳은 제외).

        프론티어 중심은 정의상 미탐사 경계라 그대로 goal 로 쓰면 벽에 붙거나
        미탐사 셀에 놓여 Nav2 가 도달 실패 → 복구 → 벽으로 밀고 드는 일이
        반복된다. 로봇 쪽으로 당겨 자유공간에 놓은 점을 goal 로 쓴다.

        예전에는 '반경 sweep_first_r 안의 미관측' 을 먼저 처리하고, 그게
        없을 때만 멀리 나갔다. 그 반경 필터가 문제였다 — 점수식은 이미
        큰 덩어리를 선호하는데, 필터가 비교 대상 자체를 눈앞으로 제한해
        점수식을 무력화했다. 그래서 목표가 1.4~4.3m 잔걸음이 되고 방
        하나를 자투리 단위로 갉아먹느라 다른 방으로 넘어가질 못했다
        (실측: 25.6m 목표를 세 번 재발행하고도 24초에 1.1m 전진).

        지금은 두 후보를 한 저울에 올려 전 지도에서 고른다. 단위가 다른
        n 을 넓이로 환산하는 몫은 goal_score 가 맡는다.
        """
        cands = ([(fx, fy, n, 'frontier') for fx, fy, n in self._find_frontiers()]
                 + [(fx, fy, n, 'visual')
                    for fx, fy, n in self._find_visual_frontiers(
                        self.visual_min_local)])

        # 지금 있는 방을 한 번만 계산해 점수 보너스와 이탈 계측이 같이 쓴다.
        room_info = None
        if (self.room_bonus > 0.0 or self.room_leave_log
                or self.room_far_coef > 0.0 or self.room_commit_area > 0.0):
            room_info = self._current_room(rx, ry)

        # 이 방에 눌러앉을 것인가. 판정 규칙은 순수 함수로 떼어 테스트한다.
        commit = self._update_commit(room_info)

        # 내 구역에서 먼저 고르고, 없으면 건물 전체로 넓혀 동료를 돕는다.
        tries = [self.explore_bounds]
        if self.world_bounds is not None and self.explore_bounds is not None:
            tries.append(self.world_bounds)
        for bounds in tries:
            best, best_score, best_kind = self._score_cands(
                cands, rx, ry, bounds, room_info, restrict=commit)
            if best is None and commit:
                # 방 안에 갈 만한 후보가 없으면 커밋이 로봇을 세운다.
                # 계획보다 임무가 먼저다 — 제한을 풀고 다시 고른다.
                best, best_score, best_kind = self._score_cands(
                    cands, rx, ry, bounds, room_info, restrict=False)
            if best is not None:
                helping = bounds is not self.explore_bounds
                if helping != self._helping:
                    self._helping = helping
                    self.get_logger().info(
                        '내 구역을 다 훑었다 — 동료 구역으로 넘어가 돕는다'
                        if helping else '내 구역으로 복귀')
                break
        self._last_goal_kind = ('미탐사 경계' if best_kind == 'frontier'
                                else '미관측 구역')
        if best is not None and self.room_leave_log:
            # best 는 (goal, 프론티어중심) 이라 좌표는 best[0] 안에 있다.
            # 계측이 수색을 죽이면 안 된다 — 실제로 여기서 낸 TypeError 로
            # 순찰 노드가 죽어 3런이 목표 0회로 날아갔다. 재는 코드의
            # 실패는 재는 것만 멈추고 임무는 계속돼야 한다.
            try:
                self._check_room_leave(best[0][0], best[0][1], rx, ry, room_info)
            except Exception as e:                      # noqa: BLE001
                if self.room_leave_log:
                    self.room_leave_log = False
                    self.get_logger().error(f'[방 이탈] 계측 중단 — {e!r}')
        return best

    def _current_room(self, rx, ry):
        """로봇이 지금 있는 방 마스크와 좌표 변환 정보.

        거리변환이 들어가 싸지 않으므로 목표를 고를 때 한 번만 계산해
        점수 보너스와 이탈 계측이 함께 쓴다.

        반환: (room, ox, oy, res, W, H) 또는 None
        """
        g = self._map_msg
        if g is None:
            return None
        H, W = g.info.height, g.info.width
        res = g.info.resolution
        if res <= 0.0:
            return None
        ox = g.info.origin.position.x
        oy = g.info.origin.position.y
        ix = int((rx - ox) / res)
        iy = int((ry - oy) / res)
        if not (0 <= ix < W and 0 <= iy < H):
            return None
        arr = np.asarray(g.data, dtype=np.int16).reshape(H, W)
        free = (arr >= 0) & (arr < 25)
        er = max(1, int(round(self.room_erode_m / res)))
        room = segment_room(free, iy, ix, er)
        if room is None:
            return None
        return room, ox, oy, res, W, H

    @staticmethod
    def _in_room(info, x, y):
        """월드 좌표가 그 방 안인지."""
        room, ox, oy, res, W, H = info
        ix = int((x - ox) / res)
        iy = int((y - oy) / res)
        if not (0 <= ix < W and 0 <= iy < H):
            return False
        return bool(room[iy, ix])

    def _room_unseen_area(self, info):
        """이 방에 남은 '사람이 숨을 만한' 미관측 넓이[m^2].

        자투리는 빼고 센다. 방 이탈 계측과 같은 기준을 쓴다 — 기준이 다르면
        '이탈했다' 와 '눌러앉아야 한다' 가 서로 어긋난다.
        """
        seen_ok = self._seen_enough()
        if info is None or seen_ok is None:
            return 0.0
        room, _ox, _oy, res, _W, _H = info
        if seen_ok.shape != room.shape:
            return 0.0
        min_cells = max(1, int(round(self.room_leave_min_area / (res * res))))
        left = actionable_cells(room & ~seen_ok, min_cells)
        return float(max(left, 0)) * res * res

    def _update_commit(self, info):
        """지금 방에 눌러앉을지 갱신하고 결과를 돌려준다.

        방이 바뀌면 체류 시계를 다시 잡는다. 안 그러면 앞 방에서 쓴 시간이
        다음 방의 상한을 깎아 먹는다.
        """
        if self.room_commit_area <= 0.0 or info is None:
            self._commit_room = None
            self._commit_since = None
            return False
        room, ox, oy, res, _W, _H = info
        ys, xs = np.nonzero(room)
        if len(xs) == 0:
            self._commit_room = None
            self._commit_since = None
            return False
        cx = ox + (float(xs.mean()) + 0.5) * res
        cy = oy + (float(ys.mean()) + 0.5) * res

        now = self._now()
        if (self._commit_room is None
                or math.hypot(cx - self._commit_room[0],
                              cy - self._commit_room[1]) > 3.0):
            self._commit_room = (cx, cy)
            self._commit_since = now

        held = now - (self._commit_since or now)
        area = self._room_unseen_area(info)
        commit = room_commit_decision(area, self.room_commit_area,
                                      held, self.room_commit_max_s)
        if commit and not self._commit_logged:
            self._commit_logged = True
            self.get_logger().info(
                f'[방 커밋] 미관측 {area:.1f}m² — 다 볼 때까지 이 방에 머문다')
        elif not commit:
            self._commit_logged = False
        return commit

    def _check_room_leave(self, gx, gy, rx, ry, info):
        """방에 사람이 숨을 만한 미관측을 남기고 나가는지 센다(측정 전용).

        벽 뒤 조난자를 놓치는 경로가 이것이라는 관찰이 있었다. 다만 반대
        방향 고장도 겪었다 — 예전에 '반경 5m 안을 먼저 처리' 로 두었더니
        방 하나를 1.4~4.3m 잔걸음으로 갉아먹으며 나가질 못했다. 그래서
        자투리(기본 0.8m^2 미만)는 세지 않는다. 사람이 숨을 수 있는 크기만
        문제로 본다.

        여기서는 세기만 하고 목표를 바꾸지 않는다. 고치기 전에 이 일이
        실제로 몇 번 일어나는지부터 알아야 한다.
        """
        seen_ok = self._seen_enough()
        if info is None or seen_ok is None:
            return
        room, ox, oy, res, W, H = info
        if seen_ok.shape != room.shape:
            return
        if self._t_start is None:
            self._t_start = self._now()
        elapsed = self._now() - self._t_start

        # 지금 있는 방의 중심(월드 좌표). 방을 알아보는 이름표로 쓴다.
        ys, xs = np.nonzero(room)
        cx = ox + (float(xs.mean()) + 0.5) * res
        cy = oy + (float(ys.mean()) + 0.5) * res

        # 방이 바뀌었나. 바뀌었고 그게 예전에 덜 보고 나온 방이면 왕복이다.
        if self._cur_room is None or math.hypot(
                cx - self._cur_room[0], cy - self._cur_room[1]) > 3.0:
            self._cur_room = (cx, cy)
            for i, (lx, ly, cnt) in enumerate(self._left_rooms):
                if math.hypot(cx - lx, cy - ly) <= 3.0:
                    self._left_rooms[i] = (lx, ly, cnt + 1)
                    self._room_reentries += 1
                    self.get_logger().warn(
                        f'[방 재진입] {elapsed:.0f}s — 덜 보고 나왔던 방으로 '
                        f'되돌아옴 (이 방 {cnt + 1}번째) / 누적 '
                        f'{self._room_reentries}회')
                    break

        if self._in_room(info, gx, gy):
            return          # 목표가 같은 방 안이면 나가는 게 아니다
        min_cells = max(1, int(round(self.room_leave_min_area / (res * res))))
        left = actionable_cells(room & ~seen_ok, min_cells)
        if left <= 0:
            return
        area = float(left) * res * res
        self._room_leaves += 1
        self._room_left_area += area
        if not any(math.hypot(cx - lx, cy - ly) <= 3.0
                   for lx, ly, _ in self._left_rooms):
            self._left_rooms.append((cx, cy, 0))
        # 경과 시간을 같이 남긴다. 수색 초반에는 지도가 통째로 미관측이라
        # 방에 들어가자마자 나와도 '크게 남기고 나감' 으로 잡힌다. 그때
        # 나가는 건 오히려 정상이다 — 아직 못 가본 방이 널렸으니까.
        # 초반과 후반을 갈라 봐야 무엇을 고칠지 정해진다.
        self.get_logger().warn(
            f'[방 이탈] {elapsed:.0f}s — 미관측 {area:.1f}m² 남기고 다른 방으로 '
            f'— 누적 {self._room_leaves}회 / {self._room_left_area:.1f}m² '
            f'/ 재진입 {self._room_reentries}회')

    def _score_cands(self, cands, rx, ry, bounds, room_info=None,
                     restrict=False):
        """주어진 범위 안에서 가장 좋은 후보를 고른다.

        room_info 가 있고 room_bonus 가 0 보다 크면, 지금 있는 방 안의
        후보에 그만큼을 얹는다. 방을 덜 보고 나가는 것을 줄이기 위한 것이다.
        """
        best, best_score, best_kind = None, -1e9, None
        for fx, fy, n, kind in cands:
            # 중복 판정은 반드시 '프론티어 중심' 기준. 당겨진 goal 로 비교하면
            # 같은 프론티어가 매번 새 후보로 통과해 무한 재선택된다.
            if any(math.hypot(fx - vx, fy - vy) < 1.5
                   for vx, vy in self._visited_frontiers):
                continue
            d = math.hypot(fx - rx, fy - ry)
            if d < 0.8:            # 코앞은 의미 없음
                continue
            # 사각지대는 벽 모서리·장애물 뒤라 사방 0.7m 여유 조건에 걸려
            # 후보에서 통째로 빠졌다. 미관측 목표는 여유를 낮춰 접근을 허용한다.
            # (그래도 Nav2 inflation 때문에 못 가면 타임아웃으로 걸러진다)
            clr = (self.goal_clearance if kind == 'frontier'
                   else self.visual_clearance)
            goal = self._pull_back(fx, fy, rx, ry, self.frontier_standoff, clr)
            if goal is None:
                continue           # 접근 가능한 자유공간을 못 찾음 → 건너뜀
            # 당긴 결과가 로봇 코앞이면 도착 판정이 즉시 서서 제자리걸음이 된다
            if math.hypot(goal[0] - rx, goal[1] - ry) < self.frontier_reach:
                continue
            # 동료가 이미 그쪽으로 가고 있으면 넘긴다. 지도를 공유해도 목표를
            # 안 나누면 둘이 같은 구역으로 몰린다(실측: 두 대가 (-0.4,12.1)
            # 과 (0.1,11.9) 를 각각 잡았다).
            if claimed_by_peer(fx, fy, self._peer_goals, self.peer_claim_r):
                continue
            if bounds is not None:
                x0, y0, x1, y1 = bounds
                if not (x0 <= fx <= x1 and y0 <= fy <= y1):
                    # 내 구역 밖이라도 코앞이면 간다. 경계 너머 3m 를
                    # 남겨 두고 건물을 가로지르는 건 낭비다.
                    if not (self.cross_border_r > 0
                            and self.world_bounds is not None
                            and d < self.cross_border_r
                            and self.world_bounds[0] <= fx <= self.world_bounds[2]
                            and self.world_bounds[1] <= fy <= self.world_bounds[3]):
                        continue
            score = goal_score(kind, n, d, self._map_res(),
                               self.frontier_view_r, self.goal_dist_penalty)
            # 지금 있는 방 안이면 보너스. 판정은 프론티어 중심이 아니라
            # 실제로 갈 지점(goal)으로 한다 — 로봇이 가는 곳이 그쪽이다.
            # 눌러앉는 중이면 방 밖 후보는 아예 뺀다. 점수를 얹는 것과
            # 다르다 — 바깥의 더 큰 덩어리가 이기는 일 자체를 없앤다.
            if restrict and room_info is not None:
                if not self._in_room(room_info, goal[0], goal[1]):
                    continue
            if room_info is not None and (self.room_bonus > 0.0
                                          or self.room_far_coef > 0.0):
                if self._in_room(room_info, goal[0], goal[1]):
                    score += self.room_bonus
                    # 같은 방이면 먼 쪽을 먼저 — 안쪽 끝까지 들어갔다 나온다
                    score += far_first_bonus(True, d, self.room_far_coef,
                                             self.room_far_cap)
            if score > best_score:
                best_score, best, best_kind = score, (goal, (fx, fy)), kind
        return best, best_score, best_kind

    def _active_bounds(self):
        """지금 실제로 훑는 범위. 동료를 돕는 중이면 건물 전체다.

        완료 판정이 내 구역만 보면, 돕는 중에 상대 구역의 미탐사를 안 세서
        '다 훑었다' 가 잘못 선다. 목표를 고르는 범위와 세는 범위는 같아야 한다.
        """
        if self._helping and self.world_bounds is not None:
            return self.world_bounds
        return self.explore_bounds

    def _map_res(self):
        """지도 해상도(m/셀). 지도가 아직 없으면 표준값."""
        return self._map_msg.info.resolution if self._map_msg else 0.05

    def _pick_far_goal(self, rx, ry):
        """지역 루프 탈출용 — 거리 벌점 없이 '가장 큰 미관측 덩어리'로 간다.

        _pick_frontier 는 가까운 곳을 선호하도록 점수를 매기므로(그래야 방을
        차례로 훑는다) 후보가 마르면 같은 구역을 맴돈다. 이 함수는 반대로
        거리를 무시하고 크기만 보며, 이미 방문한 곳도 충분히 멀면 허용해
        건물 반대편으로 건너간다.
        """
        cands = list(self._find_visual_frontiers()) + list(self._find_frontiers())
        best, best_score = None, -1e9
        for fx, fy, n in cands:
            d = math.hypot(fx - rx, fy - ry)
            if d < self.far_goal_min_dist:
                continue                       # 지역 루프 탈출이 목적이다
            # 방문 기록은 '가까운 중복'만 막는다. 멀리 있으면 다시 가도 좋다.
            if d < self.far_goal_min_dist * 2 and any(
                    math.hypot(fx - vx, fy - vy) < 1.5
                    for vx, vy in self._visited_frontiers):
                continue
            goal = self._pull_back(fx, fy, rx, ry,
                                   self.frontier_standoff, self.visual_clearance)
            if goal is None:
                continue
            score = n * 1.0 + d * 0.1          # 크고 먼 쪽을 선호
            if score > best_score:
                best_score, best = score, (goal, (fx, fy))
        return best

    # ── FSM ──────────────────────────────────────────────────────────
    def _escape_cmd_tick(self):
        """ESCAPE 중에만 20Hz 로 후진 명령을 낸다."""
        if self.state != ESCAPE:
            return
        t = Twist()
        t.linear.x = -abs(self.escape_speed)
        self.cmd_pub.publish(t)

    def _cmd_watch(self, msg: Twist):
        """Nav2 가 내는 속도 명령의 크기. 박힘 오판을 막는 데 쓴다."""
        self._cmd_mag = abs(msg.linear.x) + abs(msg.angular.z) * 0.3

    def _update_stuck(self, rx, ry, now) -> bool:
        """순찰 중인데 실제로 안 움직이면 '박힘' 으로 본다.

        맵 점유로 판정하려 했으나 쓸 수 없었다. 로봇이 잔해 안에 들어가면
        라이다가 그 안에서 바깥으로 레이를 쏘기 때문에, SLAM 이 로봇이 있는
        셀을 오히려 자유공간으로 지워버린다. 그래서 '움직여야 하는데 안
        움직인다' 는 사실 자체로 감지한다. 원인(잔해 박힘·벽 붙음·경로 실패)을
        가리지 않고 잡히는 장점도 있다.
        """
        ryaw = self._robot_yaw_map()
        if self._stuck_ref is None:
            self._stuck_ref = (rx, ry, ryaw, now)
            return False
        stuck, new_ref = stuck_decision(
            self._stuck_ref, rx, ry, ryaw, now,
            self.stuck_move_eps, self.stuck_turn_eps, self.stuck_confirm_s,
            commanded=self._cmd_mag > self.stuck_cmd_eps)
        if new_ref is not None:
            self._stuck_ref = new_ref
        return stuck

    def _escape_tick(self, rx, ry, now):
        """후진으로 빠져나온다.

        들어올 때 전진했으므로 후진이 들어온 길을 되짚는 가장 안전한 방향이다.
        이 상황에서 Nav2 컨트롤러는 명령을 내지 못하므로 /cmd_vel 충돌은 없다.
        """
        ex, ey = self._escape_from
        moved = math.hypot(rx - ex, ry - ey)
        if moved < self.escape_min_move and now - self._escape_start < self.escape_max_s:
            return   # 실제 후진 명령은 _escape_cmd_tick 이 20Hz 로 낸다
        self.cmd_pub.publish(Twist())          # 정지
        self._stuck_ref = None
        if moved >= self.escape_min_move:
            self.get_logger().info(f'탈출 완료 ({moved:.2f}m 후진) — 순찰 재개')
        else:
            self.get_logger().warn('후진해도 못 빠져나옴 — 다른 목표로 재시도')
        self.state = PATROL
        self._frontier_goal = None
        self._wp_sent_t = None

    def tick(self):
        if not self.map_ready:
            return
        rx, ry = self._robot_pose()

        # 카메라가 지금 보고 있는 부채꼴을 '봤음' 으로 기록.
        # 확인·경보 중(정지 상태)에도 포탑이 돌며 훑으므로 항상 갱신한다.
        self._update_seen(rx, ry)

        # 박힘 감지 — 이동해야 하는 PATROL 상태에서만. INSPECT/FIRE_ALARM 은
        # 일부러 정지해 있는 상태라 제외한다.
        if self.state == PATROL:
            if self._update_stuck(rx, ry, self._now()):
                now = self._now()
                # 연속 탈출만으로 'Nav2 고장'이라 단정하면 안 된다. 잔해가
                # 빽빽한 구역에서는 정상 상태에서도 연속으로 박힌다
                # (실측: 서쪽 잔해구역에서 3회 연속 → 오탐, Nav2 는 전부 active).
                # 실제 생존 여부는 /goal_pose 구독자 수로 직접 본다.
                streak_hit = self._escape_streak >= self.escape_max_streak
                if streak_hit and not self._nav2_alive():
                    # 내비게이션이 정말 죽었다. 계속 후진시키면 로봇만
                    # 엉뚱한 데로 밀려나고 진짜 원인이 가려진다.
                    if not self._nav_down_warned:
                        self._nav_down_warned = True
                        self.get_logger().error(
                            f'탈출 {self._escape_streak}회 연속 + Nav2 가 goal 을 '
                            '받지 않음 — 내비게이션이 죽은 상태다. '
                            '탈출을 중단하고 대기한다(로그 확인 필요).')
                    self._stuck_ref = None
                elif now - self._escape_last_t < self.escape_cooldown:
                    self._stuck_ref = None     # 쿨다운 중 — 다음 기회에
                else:
                    if streak_hit:
                        # Nav2 는 살아 있는데 계속 박힌다 = 잔해가 빽빽한 구역.
                        # 여기서 탈출을 끊으면 로봇이 그대로 멈춰버리므로,
                        # 카운터를 풀어 계속 빠져나오게 하고 목표를 다시 고른다.
                        self.get_logger().warn(
                            f'탈출 {self._escape_streak}회 연속 — 잔해가 빽빽한 '
                            '구역으로 보임(Nav2 는 정상). 탈출을 이어가고 '
                            '목표를 다시 고른다.', throttle_duration_sec=30.0)
                        self._escape_streak = 0
                        # 목표만 비우면 점수식이 같은 곳을 곧바로 다시 고르고,
                        # 같은 자리에서 다시 박힌다(실측: 같은 목표로 10분간
                        # 박힘↔탈출 반복, 목표까지 거리 15~16m 그대로).
                        # 박히게 만든 프론티어를 방문 처리해 다른 곳을 고르게 한다.
                        if self._frontier_src is not None:
                            self._visited_frontiers.append(self._frontier_src)
                        self._frontier_goal = None
                        self._far_lock = False
                    self.get_logger().warn(
                        f'{self.stuck_confirm_s:.0f}초간 못 움직임 ({rx:.1f}, {ry:.1f}) '
                        '— 장애물 박힘으로 보고 후진 탈출')
                    self._escape_start = now
                    self._escape_last_t = now
                    self._escape_streak += 1
                    self._escape_from = (rx, ry)
                    self.state = ESCAPE
                # Nav2 를 먼저 멈춘다. 안 그러면 컨트롤러가 20Hz 로 /cmd_vel 에
                # 계속 명령을 내보내 후진 명령과 경합해 로봇이 거의 안 움직인다.
                self._stop_here()
        elif self.state != ESCAPE:
            self._stuck_ref = None

        if self.state == ESCAPE:
            self._escape_tick(rx, ry, self._now())
            self._publish_markers()
            return

        if self.state == IDLE:
            if self.enabled and self.patrol_mode == 'explore':
                self.state = PATROL
                self.get_logger().info('탐사 순찰 시작 — 미탐사 경계로 진출')
                self._explore_tick(rx, ry)
            elif self.enabled and self.waypoints:
                self.state = PATROL
                self.wp_idx = 0
                self._goto_current_wp('순찰 시작')

        elif self.state == PATROL:
            if not self.enabled:
                self.state = IDLE
            elif self.patrol_mode == 'explore':
                self._explore_tick(rx, ry)
            else:
                wx, wy = self.waypoints[self.wp_idx]
                reached = math.hypot(wx - rx, wy - ry) < self.reach_dist
                timed_out = (self._wp_sent_t is not None
                             and self._now() - self._wp_sent_t > self.wp_timeout)
                if reached:
                    self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
                    self._goto_current_wp('도착 → 다음')
                elif timed_out:
                    self.get_logger().warn(
                        f'WP{self.wp_idx} 도달 실패({self.wp_timeout:.0f}s) — 건너뜀 '
                        '(화재/장애물로 막혔을 수 있음)')
                    self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
                    self._goto_current_wp('건너뜀 → 다음')

        elif self.state == INSPECT:
            # 접근 단계 — 대상 앞 standoff 지점까지 이동
            if not self._approach_arrived and self._approach_goal is not None:
                ax, ay = self._approach_goal
                if math.hypot(ax - rx, ay - ry) < self.approach_reach:
                    self._approach_arrived = True
                    self._stop_here()
                    d = math.hypot(self._inspect_pos[0] - rx,
                                   self._inspect_pos[1] - ry) if self._inspect_pos else 0.0
                    self.get_logger().info(
                        f'접근 완료 (대상까지 {d:.1f}m) — 정지, 조준 스캔')
            # 포탑이 후보를 보도록 조준점 계속 발행 (접근 중에도 미리 겨눔)
            if self._inspect_pos is not None:
                p = Point()
                p.x, p.y, p.z = self._inspect_pos[0], self._inspect_pos[1], 0.5
                self.aim_pub.publish(p)
            # target_manager가 죽거나 응답이 없을 때를 대비한 안전장치
            if (self._inspect_start is not None
                    and self._now() - self._inspect_start > self.inspect_timeout):
                self.get_logger().warn('확인 응답 없음 — 순찰 재개')
                self._inspect_pos = None
                self._approach_goal = None
                self._approach_arrived = False
                self._resume_patrol()

        elif self.state == MANUAL:
            if self._manual_goal is not None:
                mx, my = self._manual_goal
                if math.hypot(mx - rx, my - ry) < self.reach_dist:
                    self.get_logger().info('수동 목표 도착 → 순찰 재개')
                    self._manual_goal = None
                    self._manual_start = None
                    self._resume_patrol()
                elif (self._manual_start is not None
                      and self._now() - self._manual_start > MANUAL_TIMEOUT_S):
                    self.get_logger().warn(
                        f'수동 목표 ({mx:.1f}, {my:.1f}) 를 '
                        f'{MANUAL_TIMEOUT_S:.0f}초 안에 못 감 — 포기하고 순찰 재개')
                    self._manual_goal = None
                    self._manual_start = None
                    self._resume_patrol()

        elif self.state == FIRE_ALARM:
            # 포탑을 화재로 조준 (target_manager 가 /apex_aim_point 소비)
            if self._fire_pos is not None:
                p = Point()
                p.x, p.y, p.z = self._fire_pos[0], self._fire_pos[1], 0.5
                self.aim_pub.publish(p)
            if self._now() - self._alarm_start >= self.alarm_duration:
                self.get_logger().info('경보 종료 → 순찰 재개 (화재 구역 우회)')
                self._fire_pos = None
                self._resume_patrol()

        self._publish_markers()

    def _explore_tick(self, rx, ry):
        """미탐사 경계로 나아가며 맵을 넓힌다."""
        now = self._now()
        goal = self._frontier_goal
        reached  = goal is not None and math.hypot(goal[0]-rx, goal[1]-ry) < self.frontier_reach
        timedout = (self._wp_sent_t is not None
                    and now - self._wp_sent_t > self._explore_budget)
        # 한 번 고른 목표는 도착/시간초과 전까지 붙든다.
        # 안 그러면 frontier_replan(6초)마다 재선정이 돌면서 가까운 후보가
        # 다시 뽑혀, 먼 목표에 닿기도 전에 취소된다. 실측으로 로봇이
        # 19m 떨어진 두 지점을 왕복만 했다(원거리 이탈 125회, 도달 0회).
        #
        # 예전에는 이 붙듦이 '원거리 이탈' 목표에만 걸렸다. 지금은 목표를
        # 전 지도에서 고르므로 어느 목표든 멀 수 있어 전부에 건다.
        # 대가: 가려던 곳이 이동 중에 우연히 관측돼도 일단 간다. 시간초과
        # 상한(explore_goal_timeout_max)이 그 낭비를 막는다.
        if self._far_lock:
            if reached or timedout:
                self._far_lock = False
            elif goal is not None:
                return

        need_new = (goal is None or reached or timedout
                    or now - self._frontier_t > self.frontier_replan)
        if not need_new:
            return

        if goal is not None and reached:
            # 실제로 목표에 도달했다 = 내비게이션이 살아 있다는 증거.
            # 탈출 연속 카운터를 여기서만 푼다(타임아웃은 진행으로 안 친다).
            self._escape_streak = 0
            self._nav_down_warned = False
            self._goals_done += 1
            # 도착만 하고 바로 다음 목표로 뜨면 사각지대를 못 본다.
            # 그 자리에서 포탑이 한 바퀴 훑을 시간을 준다.
            if self._dwell_until is None:
                self._dwell_until = now + self.dwell_s
                self._stop_here()
                return

        # 도착 후 훑는 중 — 포탑 스캔이 돌도록 그대로 둔다
        if self._dwell_until is not None:
            if now < self._dwell_until:
                return
            self._dwell_until = None

        if goal is not None and (reached or timedout):
            # 방문 기록은 '프론티어 중심' 으로 남긴다 (_pick_frontier 의 중복
            # 판정 기준과 같아야 한다). goal 로 남기면 같은 곳을 계속 다시 고른다.
            if self._frontier_src is not None:
                self._visited_frontiers.append(self._frontier_src)
            if timedout:
                self.get_logger().warn(
                    f'프론티어 ({goal[0]:.1f},{goal[1]:.1f}) 도달 실패 — 다른 곳으로')

        picked = self._pick_frontier(rx, ry)
        nxt = picked[0] if picked else None
        self._frontier_t = now
        if nxt is None:
            fr_cells, unseen = self._coverage_left()
            unseen_budget = max(self.done_unseen_area,
                                self._known_free_area() * self.done_unseen_frac)
            phase = ('보충 수색(전원 발견 완료)' if self._all_found_reported
                     else '수색 진행')
            self.get_logger().info(
                f'{phase} — 미탐사 경계 {fr_cells}셀(완료 기준 '
                f'{self.done_frontier_cells}), 미관측 {unseen:.1f}m²(기준 '
                f'{unseen_budget:.1f}), 조난자 {self._victim_count()}'
                + (f'/{self.expected_victims}' if self.expected_victims > 0 else '')
                + '명', throttle_duration_sec=60.0)
            # 판정 규칙은 순수 함수로 분리해 단위 테스트한다(sweep_decision).
            decision = sweep_decision(
                fr_cells, unseen, unseen_budget, self.done_frontier_cells,
                self._goals_done, self.min_goals_for_sweep,
                self._known_free_area(), self.min_area_for_sweep,
                self._victim_count(), self.expected_victims)

            if decision == 'done':
                if not self._explore_done:
                    self._explore_done = True
                    self._sweeps += 1
                    self._report_mission(self._sweeps)
                self._visited_frontiers.clear()
                self._frontier_goal = None
                return

            if decision == 'resweep':
                # 다 훑었는데 인원이 모자란다 = 놓친 것이다. 시야 기록을 지우고
                # 처음부터 다시 훑는다.
                self._sweeps += 1
                self.get_logger().warn(
                    f'전 구역을 훑었으나 조난자 {self._victim_count()}/'
                    f'{self.expected_victims}명만 확인됨 — 놓친 구역이 있다고 보고 '
                    f'{self._sweeps + 1}회차 재수색 시작(시야 기록 초기화)')
                self._seen = None
                self._seen_geom = None
                self._visited_frontiers.clear()
                self._frontier_goal = None
                self._explore_done = False
                return

            # 아직 남았는데 후보만 비었다.
            # 여기서 방문 기록만 지우면 점수식(n*0.5 - d)이 거리에 크게
            # 벌점을 주므로 '가까운 곳'이 곧바로 다시 뽑혀, 로봇이 좁은 구역
            # 몇 곳을 무한 반복한다(실측: 103분 중 60분을 3개 목표에서 왕복,
            # 서쪽 끝 조난자 2명을 끝내 못 찾음).
            # → 먼 미관측 구역으로 한 번 강제로 빠져나간 뒤 재개한다.
            far = self._pick_far_goal(rx, ry)
            if far is not None:
                self._frontier_goal, self._frontier_src = far
                self._send_goal(far[0][0], far[0][1])
                self._wp_sent_t = now
                fd = math.hypot(far[0][0] - rx, far[0][1] - ry)
                self._explore_budget = min(
                    self.explore_tmo_max,
                    self.explore_timeout + fd / max(self.explore_speed, 0.05))
                self._last_goal_kind = '원거리 이탈'
                self._far_lock = True
                self.get_logger().info(
                    f'주변 후보 소진 — 먼 미관측 구역으로 이동 '
                    f'({far[0][0]:.1f}, {far[0][1]:.1f})  {fd:.1f}m  '
                    f'제한 {self._explore_budget:.0f}s')
                return
            self._visited_frontiers.clear()
            self._frontier_goal = None
            return

        self._explore_done = False
        # 같은 목표를 다시 고른 경우엔 goal 재발행/로그를 생략 (Nav2 경로 리셋 방지)
        same = (goal is not None and not reached and not timedout
                and math.hypot(nxt[0] - goal[0], nxt[1] - goal[1]) < 0.5)
        self._frontier_goal = nxt
        self._frontier_src  = picked[1]
        if same:
            return
        self._send_goal(nxt[0], nxt[1])
        self._wp_sent_t = now
        self._far_lock = True           # 도착/시간초과까지 이 목표를 붙든다
        if self.peers:                  # 동료가 이쪽으로 안 오도록 알린다
            c = PoseStamped()
            c.header.frame_id = self.map_frame
            c.header.stamp = self.get_clock().now().to_msg()
            c.pose.position.x, c.pose.position.y = self._frontier_src
            c.pose.orientation.w = 1.0
            self.claim_pub.publish(c)
        # 먼 목표일수록 시간을 더 준다 (직선거리 기준, 경로는 더 길므로 넉넉히)
        gd = math.hypot(nxt[0] - rx, nxt[1] - ry)
        self._explore_budget = min(self.explore_tmo_max,
                                   self.explore_timeout + gd / max(self.explore_speed, 0.05))
        self.get_logger().info(
            f'탐사 목표 → ({nxt[0]:.1f}, {nxt[1]:.1f})  {gd:.1f}m  '
            f'[{getattr(self, "_last_goal_kind", "미탐사 경계")} 기준, '
            f'제한 {self._explore_budget:.0f}s, 누적 방문 {len(self._visited_frontiers)}곳]')

    def _goto_current_wp(self, reason=''):
        wx, wy = self.waypoints[self.wp_idx]
        self._send_goal(wx, wy)
        self._wp_sent_t = self._now()
        self.get_logger().info(f'{reason} WP{self.wp_idx} ({wx:.1f},{wy:.1f})')

    def _resume_patrol(self):
        """조사·경보가 끝나고 순찰로 돌아온다.

        예전에는 여기서 탐사 목표를 통째로 버렸다(`_frontier_goal = None`,
        `_far_lock = False`). 그래서 유령 후보 하나마다 로봇이 어디 가려던
        건지를 잊었다. 유령은 런당 21~24건이고, 실측으로 25.6m 목표를 세 번
        재발행하고도 24초 동안 1.1m 밖에 못 갔다 — 복도에서 멍 때리는 것처럼
        보이던 것의 정체다.

        이제는 가려던 목표를 되살린다. 조사에 쓴 시간만큼 제한시간을 미뤄
        주는 게 중요하다. 안 그러면 조사 시간이 주행 예산을 깎아 목표가
        엉뚱하게 시간초과된다.
        """
        self.state = PATROL
        if self.patrol_mode != 'explore':
            self._goto_current_wp('순찰 재개')
            return

        goal = self._frontier_goal
        if goal is None:
            self._wp_sent_t = None
            self._far_lock = False
            self.get_logger().info('순찰 재개 — 탐사 목표 재선정')
            return

        paused = 0.0
        if self._pause_t0 is not None:
            paused = max(0.0, self._now() - self._pause_t0)
            self._pause_t0 = None
        if self._wp_sent_t is not None:
            self._wp_sent_t += paused       # 멈춰 있던 시간은 주행 예산에서 뺀다
        # 정지 명령으로 Nav2 목표가 덮였으므로 같은 목표를 다시 보낸다
        self._send_goal(goal[0], goal[1])
        self.get_logger().info(
            f'순찰 재개 — 가려던 목표 ({goal[0]:.1f}, {goal[1]:.1f}) 복귀 '
            f'(조사에 {paused:.0f}s 소요, 제한시간 그만큼 연장)')

    # ── 시각화 ───────────────────────────────────────────────────────
    def _publish_markers(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        # 순찰 경로 라인 — 웨이포인트 모드에서만 (탐사 모드는 경로가 미리 없음)
        line = Marker()
        line.header.frame_id = self.map_frame
        line.header.stamp = stamp
        line.ns = 'patrol_route'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.scale.x = 0.06
        line.color.b = 1.0
        line.color.g = 0.6
        line.color.a = 0.6
        line.pose.orientation.w = 1.0
        if self.patrol_mode == 'explore':
            line.action = Marker.DELETE
        else:
            line.action = Marker.ADD
            for (wx, wy) in self.waypoints + [self.waypoints[0]]:
                pt = Point(); pt.x = float(wx); pt.y = float(wy); pt.z = 0.05
                line.points.append(pt)
        ma.markers.append(line)

        # 현재 목표 강조 (탐사 모드는 프론티어 목표)
        cur_goal = None
        if self.state == PATROL:
            if self.patrol_mode == 'explore':
                cur_goal = self._frontier_goal
            elif self.waypoints:
                cur_goal = self.waypoints[self.wp_idx]
        if cur_goal is not None:
            wx, wy = cur_goal
            cur = Marker()
            cur.header.frame_id = self.map_frame
            cur.header.stamp = stamp
            cur.ns = 'patrol_target'
            cur.id = 1
            cur.type = Marker.CYLINDER
            cur.action = Marker.ADD
            cur.pose.position.x = float(wx)
            cur.pose.position.y = float(wy)
            cur.pose.position.z = 0.1
            cur.pose.orientation.w = 1.0
            cur.scale.x = cur.scale.y = 0.5
            cur.scale.z = 0.2
            cur.color.b = 1.0; cur.color.g = 0.8; cur.color.a = 0.8
            ma.markers.append(cur)

        # 화재 경보 배너
        banner = Marker()
        banner.header.frame_id = self.map_frame
        banner.header.stamp = stamp
        banner.ns = 'alarm_banner'
        banner.id = 2
        banner.type = Marker.TEXT_VIEW_FACING
        banner.pose.position.x = self.robot_x
        banner.pose.position.y = self.robot_y
        banner.pose.position.z = 1.8
        banner.pose.orientation.w = 1.0
        banner.scale.z = 0.6
        if self.state == FIRE_ALARM:
            banner.action = Marker.ADD
            banner.color.r = 1.0; banner.color.a = 1.0
            banner.text = '🔥 FIRE ALARM'
        elif self.state == INSPECT:
            banner.action = Marker.ADD
            banner.color.r = 1.0; banner.color.g = 0.9; banner.color.a = 1.0
            banner.text = '👤 확인 중 (정지·조준)'
        elif self.state == ESCAPE:
            banner.action = Marker.ADD
            banner.color.r = 1.0; banner.color.g = 0.5; banner.color.a = 1.0
            banner.text = '⚠ 장애물 탈출 중'
        else:
            banner.action = Marker.DELETE
        ma.markers.append(banner)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
