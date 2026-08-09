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
        # 직선 접근이 막혔을 때 대상 주위를 몇 방향까지 뒤질지
        self.declare_parameter('approach_ring_n', 12)
        # 장애물 탈출
        self.declare_parameter('stuck_confirm_s', 8.0)      # 이만큼 안 움직이면 박힘으로 판단
        self.declare_parameter('stuck_move_eps',  0.15)     # 이 이상 움직이면 정상(m)
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
        self.declare_parameter('cam_fov_rad',   1.089)      # 카메라 수평 FOV
        # 미관측 군집 최소 크기. 카메라 FOV 스윕은 사방에 자잘한 미관측
        # 조각을 항상 남긴다. 작게 잡으면 '주변 미관측' 이 영영 비지 않아
        # 로봇이 제자리에서 조각만 갈아댄다(실측: 목표 35개 중 27개가 주변
        # 미관측, 이동 범위가 56m 건물의 가운데 15m 뿐이었다).
        self.declare_parameter('visual_min_size', 300)      # 셀 수(0.75m^2)
        # 이 반경 안에 안 본 구역이 있으면, 멀리 있는 미탐사 경계보다 먼저 훑는다
        self.declare_parameter('sweep_first_radius', 5.0)
        # 방 안 마무리용 군집 하한(셀). 전역 후보(visual_min_size)보다 작게
        # 잡아 구석의 작은 사각지대까지 목표로 삼는다. 이 값이 크면 계획기가
        # 못 잡는 조각이 남아 완료 판정이 영원히 안 선다.
        self.declare_parameter('visual_min_local', 40)      # 셀 수(0.1m^2)
        # 한 구역을 마무리하는 데 쓸 수 있는 최대 시간(초).
        # 예전의 '연속 3회 제한' 을 대체한다. 횟수로 끊으면 방을 다 봤는지와
        # 무관하게 나가버려, 남은 조각을 지우러 다시 오느라 재수색이 길어졌다.
        self.declare_parameter('room_clear_budget_s', 180.0)
        # 사각지대(벽 모서리·장애물 뒤)는 여유가 안 나오므로 낮게 잡는다
        self.declare_parameter('visual_clearance', 0.45)
        # 목표 도착 후 그 자리에서 포탑이 훑을 시간(초). 바로 다음 목표로
        # 떠나면 방에 들어갔다 사각지대를 못 보고 나온다.
        self.declare_parameter('arrive_dwell_s', 4.0)
        # '수색 완료' 를 인정하기 위한 최소 근거 (기동 직후 오보 방지)
        self.declare_parameter('min_goals_for_sweep', 8)      # 도달한 목표 수
        self.declare_parameter('min_area_for_sweep', 200.0)   # 매핑된 자유공간 m^2
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
        self.approach_ring_n   = int(self.get_parameter('approach_ring_n').value)
        self._explore_budget   = self.explore_timeout   # 현재 goal 의 제한시간(초)
        self.stuck_confirm_s   = float(self.get_parameter('stuck_confirm_s').value)
        self.stuck_move_eps    = float(self.get_parameter('stuck_move_eps').value)
        self.escape_speed      = float(self.get_parameter('escape_speed').value)
        self.escape_max_s      = float(self.get_parameter('escape_max_s').value)
        self.escape_min_move   = float(self.get_parameter('escape_min_move').value)
        self.escape_cooldown   = float(self.get_parameter('escape_cooldown_s').value)
        self.escape_max_streak = int(self.get_parameter('escape_max_streak').value)
        self.cam_range         = float(self.get_parameter('cam_see_range').value)
        self.cam_fov           = float(self.get_parameter('cam_fov_rad').value)
        self.visual_min        = int(self.get_parameter('visual_min_size').value)
        self.sweep_first_r     = float(self.get_parameter('sweep_first_radius').value)
        self.visual_min_local  = int(self.get_parameter('visual_min_local').value)
        self.room_clear_budget = float(self.get_parameter('room_clear_budget_s').value)
        self.visual_clearance  = float(self.get_parameter('visual_clearance').value)
        self.dwell_s           = float(self.get_parameter('arrive_dwell_s').value)
        self.min_goals_for_sweep = int(self.get_parameter('min_goals_for_sweep').value)
        self.min_area_for_sweep  = float(self.get_parameter('min_area_for_sweep').value)
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
        self._local_anchor = None             # 현재 마무리 중인 구역의 기준점
        self._local_since = 0.0               # 그 구역에 들어온 시각(초)
        self._goals_done = 0                  # 실제로 도달한 목표 수
        self._visited_frontiers: list[tuple] = []
        self._explore_done = False
        self._sweeps = 0                      # 건물 전체를 훑은 횟수
        self._victims: dict[int, tuple] = {}  # 확정 조난자 {pid: (x, y, label)}
        self._all_found_reported = False      # '전원 발견' 보고를 이미 냈는지
        self._fires_seen: list[tuple] = []    # 확정 화재 [(x, y)]

        # 시야 커버리지 — SLAM 맵과 같은 격자에 정렬해서 유지한다
        self._seen = None                     # bool 배열 (h, w)
        self._seen_geom = None                # (w, h, res, ox, oy) — 바뀌면 재정렬
        self._turret_yaw = 0.0

        # 조난자 확인(INSPECT) 상태
        self._inspect_pos = None              # 확인 중인 후보 (x,y)
        self._inspect_start = None
        self._state_before_inspect = None
        self._approach_goal = None            # 대상 앞 접근 지점 (x,y)
        self._approach_arrived = False        # 접근 완료 여부

        # 장애물 탈출(ESCAPE) 상태
        self._stuck_ref = None                # (x, y, t) 마지막으로 움직인 기준점
        self._escape_start = None
        self._escape_from = (0.0, 0.0)
        self._escape_last_t = -1e9            # 마지막 탈출 시각
        self._escape_streak = 0               # 연속 탈출 횟수
        self._nav_down_warned = False

        # ── TF ───────────────────────────────────────────────────────
        self._tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=10))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        # ── 구독 ─────────────────────────────────────────────────────
        self.create_subscription(Odometry,     '/odom',        self.odom_cb,   10)
        self.create_subscription(OccupancyGrid, '/map',        self.map_cb,    1)
        self.create_subscription(PoseStamped,   '/goal_pose',  self.goal_echo_cb, 10)
        self.create_subscription(PointStamped,  '/fire_alert', self.fire_cb,   10)
        self.create_subscription(Bool,          '/patrol_enable', self.enable_cb, 10)
        # 조난자 확인 핸드셰이크 (target_manager_node)
        self.create_subscription(PointStamped, '/inspect_request', self.inspect_req_cb,  10)
        self.create_subscription(Bool,         '/inspect_done',    self.inspect_done_cb, 10)
        # 열원 확인 핸드셰이크 (fire_detection_node) — 조난자와 같은 흐름,
        # 다만 불에는 너무 가까이 붙지 않도록 standoff 를 따로 둔다
        self.create_subscription(PointStamped, '/fire_candidate',    self.fire_cand_cb, 10)
        # 카메라가 어디를 보는지 알아야 시야 커버리지를 칠할 수 있다
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        # 수색 결과 요약 보고용 — 확정된 조난자·화재 목록
        self.create_subscription(MarkerArray,  '/patient_markers', self.victims_cb, 10)
        self.create_subscription(PointStamped, '/fire_alert',      self.fire_seen_cb, 10)
        self.create_subscription(Bool,         '/fire_inspect_done', self.inspect_done_cb, 10)

        # ── 발행 ─────────────────────────────────────────────────────
        self.goal_pub   = self.create_publisher(PoseStamped, '/goal_pose',      10)
        self.aim_pub    = self.create_publisher(Point,       '/apex_aim_point', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/patrol_markers', 10)
        # 장애물 탈출용 — 로봇이 장애물 안에 들어가면 Nav2 는 시작 자세가
        # 무효라 경로를 못 낸다. 그때만 직접 후진 명령을 낸다.
        self.cmd_pub    = self.create_publisher(Twist, '/cmd_vel', 10)
        # 한 바퀴 수색 완료 신호
        self.sweep_pub  = self.create_publisher(Bool, '/sweep_complete', 10)
        # 카메라로 실제 훑은 구역 / 아직 못 본 구역을 눈으로 구분하기 위한 격자.
        # SLAM 맵은 '라이다가 지나갔나' 만 보여주므로, 방을 통과만 하고 구석을
        # 안 본 경우가 지도상으로는 멀쩡해 보인다. 로그의 '미관측 279m²' 가
        # 어디를 말하는지 화면에서 볼 수 없었다.
        self.cover_pub  = self.create_publisher(OccupancyGrid, '/coverage_map', 1)

        self.create_timer(0.5, self.tick)     # 2 Hz FSM
        # 탈출 명령은 20Hz 로 낸다. Nav2 컨트롤러도 /cmd_vel 에 20Hz 로 0을
        # 쏘고 있어서, FSM 주기(2Hz)로 보내면 그 사이 0에 묻혀 로봇이 안 움직인다.
        self.create_timer(0.05, self._escape_cmd_tick)
        # 커버리지 격자는 크고 자주 안 바뀌므로 저주기로만 발행
        self.create_timer(2.0, self._publish_coverage)

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
        new = np.zeros((g.info.height, g.info.width), dtype=bool)
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
        n_rays = 21
        step = res * 0.9
        for i in range(n_rays):
            a = cam - half + i * self.cam_fov / (n_rays - 1)
            ca, sa = math.cos(a), math.sin(a)
            d = 0.3
            while d <= self.cam_range:
                ix = int((rx + d * ca - ox) / res)
                iy = int((ry + d * sa - oy) / res)
                if not (0 <= ix < W and 0 <= iy < H):
                    break
                if occ[iy, ix]:
                    break                      # 벽 뒤는 못 본다
                self._seen[iy, ix] = True
                d += step

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
        frontier_cells = int((free & nb).sum())
        if self._seen is None or self._seen.shape != free.shape:
            unseen_area = float(free.sum()) * res * res
        else:
            unseen_area = float((free & ~self._seen).sum()) * res * res
        return (frontier_cells, unseen_area)

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
        if self._seen is not None and self._seen.shape == free.shape:
            out[free & ~self._seen] = 55         # 지나갔지만 눈으로 못 봄
        else:
            out[free] = 55
        m = OccupancyGrid()
        m.header.frame_id = 'map'
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
        target = (arr >= 0) & (arr < 25) & (~self._seen)
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

    def victims_cb(self, msg: MarkerArray):
        """target_manager 가 발행하는 환자 마커에서 확정 목록을 뽑는다."""
        for m in msg.markers:
            if m.ns != 'patient_text' or m.action != Marker.ADD:
                continue
            pid = m.id // 3
            self._victims[pid] = (m.pose.position.x, m.pose.position.y,
                                  m.text.split('\n')[0] if m.text else '')
        # 실종자 수를 채운 순간 바로 보고한다(커버리지를 기다리지 않는다)
        if (self.expected_victims > 0
                and len(self._victims) >= self.expected_victims):
            self._report_all_found()

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
        self.state = FIRE_ALARM
        self.get_logger().warn(
            f'🚨 화재 경보! ({fx:.1f}, {fy:.1f}) — 순찰 정지, 포탑 조준')
        self._stop_here()                       # 현재 위치를 goal 로 → 정지

    # ── 유틸 ─────────────────────────────────────────────────────────
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _robot_pose(self):
        try:
            tf = self._tf_buf.lookup_transform('map', 'base_footprint', Time())
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
                'map', 'base_footprint', Time()).transform.rotation
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
        g.header.frame_id = 'map'
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
        lines = [f'🏁 전원 발견! 조난자 {len(vics)}/{self.expected_victims}명 '
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
                 f'  조난자 {len(vics)}명, 화재 {len(fires)}건']
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
        """
        # 지금 있는 방부터 다 훑고 나간다.
        # 미관측 구역을 '라이다 프론티어가 전부 없어진 뒤' 로 미루면, 방이
        # 여러 개인 건물에서는 라이다 프론티어가 계속 남아 있어서 로봇이
        # 들어간 방의 사각지대를 안 보고 다음 방으로 떠나버린다.
        # → 반경 sweep_first_r 안에 안 본 구역이 있으면 그것부터 처리한다.
        # 방 안을 마무리할 때는 작은 조각까지 목표로 삼는다.
        near = [c for c in self._find_visual_frontiers(self.visual_min_local)
                if math.hypot(c[0] - rx, c[1] - ry) <= self.sweep_first_r]
        now = self._now()

        # '들어간 방은 다 보고 나간다'.
        # 예전에는 연속 3회만 주변을 훑고 무조건 밖으로 나갔다. 그래서 방을
        # 다 못 본 채 떠나고, 남은 조각을 지우러 나중에 다시 오느라 재수색이
        # 길어졌다. 이제는 주변에 안 본 곳이 있으면 계속 훑는다.
        # 다만 한 구역에 영원히 갇히면 안 되므로 시간 예산을 둔다
        # (예전 3회 제한은 이 목적이었으나, 방을 다 봤는지와 무관해서
        #  '덜 보고 나가는' 부작용이 컸다).
        if near:
            anchor_far = (self._local_anchor is None
                          or math.hypot(rx - self._local_anchor[0],
                                        ry - self._local_anchor[1])
                          > self.sweep_first_r * 1.5)
            if anchor_far:                      # 새 구역에 들어왔다
                self._local_anchor = (rx, ry)
                self._local_since = now
            if now - self._local_since <= self.room_clear_budget:
                cands, kind = near, '방 안 미관측(마무리)'
            else:                               # 예산 초과 → 일단 나갔다 온다
                self._local_anchor = None
                cands = self._find_frontiers()
                kind = '미탐사 경계(구역 예산 초과)'
                if not cands:
                    cands = self._find_visual_frontiers()
                    kind = '미관측 구역'
        else:
            self._local_anchor = None
            cands = self._find_frontiers()
            kind = '미탐사 경계'
            if not cands:
                cands = self._find_visual_frontiers()
                kind = '미관측 구역'
        self._last_goal_kind = kind

        best, best_score = None, -1e9
        for fx, fy, n in cands:
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
            clr = (self.visual_clearance if kind != '미탐사 경계'
                   else self.goal_clearance)
            goal = self._pull_back(fx, fy, rx, ry, self.frontier_standoff, clr)
            if goal is None:
                continue           # 접근 가능한 자유공간을 못 찾음 → 건너뜀
            # 당긴 결과가 로봇 코앞이면 도착 판정이 즉시 서서 제자리걸음이 된다
            if math.hypot(goal[0] - rx, goal[1] - ry) < self.frontier_reach:
                continue
            # 크기는 이득, 거리는 비용
            score = n * 0.5 - d
            if score > best_score:
                best_score, best = score, (goal, (fx, fy))
        return best

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

    def _update_stuck(self, rx, ry, now) -> bool:
        """순찰 중인데 실제로 안 움직이면 '박힘' 으로 본다.

        맵 점유로 판정하려 했으나 쓸 수 없었다. 로봇이 잔해 안에 들어가면
        라이다가 그 안에서 바깥으로 레이를 쏘기 때문에, SLAM 이 로봇이 있는
        셀을 오히려 자유공간으로 지워버린다. 그래서 '움직여야 하는데 안
        움직인다' 는 사실 자체로 감지한다. 원인(잔해 박힘·벽 붙음·경로 실패)을
        가리지 않고 잡히는 장점도 있다.
        """
        if self._stuck_ref is None:
            self._stuck_ref = (rx, ry, now)
            return False
        sx, sy, st = self._stuck_ref
        if math.hypot(rx - sx, ry - sy) > self.stuck_move_eps:
            self._stuck_ref = (rx, ry, now)     # 움직였으면 기준 갱신
            return False
        return now - st > self.stuck_confirm_s

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
        # 원거리 이탈 목표는 도착/시간초과 전까지 붙든다.
        # 안 그러면 frontier_replan(6초)마다 재선정이 돌면서 가까운 후보가
        # 다시 뽑혀, 먼 목표에 닿기도 전에 취소된다. 실측으로 로봇이
        # 19m 떨어진 두 지점을 왕복만 했다(원거리 이탈 125회, 도달 0회).
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
            covered = (fr_cells <= self.done_frontier_cells
                       and unseen <= unseen_budget)
            phase = ('보충 수색(전원 발견 완료)' if self._all_found_reported
                     else '수색 진행')
            self.get_logger().info(
                f'{phase} — 미탐사 경계 {fr_cells}셀(완료 기준 '
                f'{self.done_frontier_cells}), 미관측 {unseen:.1f}m²(기준 '
                f'{unseen_budget:.1f}), 조난자 {len(self._victims)}'
                + (f'/{self.expected_victims}' if self.expected_victims > 0 else '')
                + '명', throttle_duration_sec=60.0)
            explored_enough = (self._goals_done >= self.min_goals_for_sweep
                               and self._known_free_area() >= self.min_area_for_sweep)
            # 실종자 수를 아는 경우, 다 찾기 전에는 완료로 보지 않는다.
            all_found = (self.expected_victims <= 0
                         or len(self._victims) >= self.expected_victims)

            if covered and explored_enough and all_found:
                if not self._explore_done:
                    self._explore_done = True
                    self._sweeps += 1
                    self._report_mission(self._sweeps)
                self._visited_frontiers.clear()
                self._frontier_goal = None
                return

            if covered and explored_enough and not all_found:
                # 다 훑었는데 인원이 모자란다 = 놓친 것이다. 시야 기록을 지우고
                # 처음부터 다시 훑는다.
                self._sweeps += 1
                self.get_logger().warn(
                    f'전 구역을 훑었으나 조난자 {len(self._victims)}/'
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
        self.state = PATROL
        if self.patrol_mode == 'explore':
            self._frontier_goal = None      # 다음 tick에서 새 목표 선정
            self._wp_sent_t = None
            self._far_lock = False          # 조사/경보로 끊겼으면 이탈도 무효
            self.get_logger().info('순찰 재개 — 탐사 목표 재선정')
        else:
            self._goto_current_wp('순찰 재개')

    # ── 시각화 ───────────────────────────────────────────────────────
    def _publish_markers(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        # 순찰 경로 라인 — 웨이포인트 모드에서만 (탐사 모드는 경로가 미리 없음)
        line = Marker()
        line.header.frame_id = 'map'
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
            cur.header.frame_id = 'map'
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
        banner.header.frame_id = 'map'
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
