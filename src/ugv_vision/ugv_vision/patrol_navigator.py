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

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Point, PointStamped
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
        self.declare_parameter('inspect_timeout', 12.0)     # 확인 상태 최대 유지(초)

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

        # ── 상태 ─────────────────────────────────────────────────────
        self.state = IDLE
        self.wp_idx = 0
        self.map_ready = False
        self.robot_x = self.robot_y = self.robot_theta = 0.0

        self._just_published = False          # 자기 goal 에코 필터
        self._manual_goal = None              # (x,y)
        self._fire_pos = None                 # 현재 경보 대상 (x,y)
        self._alarm_start = None              # 경보 시작 시각(sec)
        self._wp_sent_t = None                # 현재 WP goal 발행 시각(sec)
        self._alarmed_fires: list[tuple] = [] # 이미 경보한 화재들

        # 탐사(frontier) 상태
        self._map_msg = None                  # 최신 OccupancyGrid
        self._frontier_goal = None            # 현재 향하는 프론티어 (x,y)
        self._frontier_t = 0.0                # 마지막 목표 선정 시각
        self._visited_frontiers: list[tuple] = []
        self._explore_done = False

        # 조난자 확인(INSPECT) 상태
        self._inspect_pos = None              # 확인 중인 후보 (x,y)
        self._inspect_start = None
        self._state_before_inspect = None

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

        # ── 발행 ─────────────────────────────────────────────────────
        self.goal_pub   = self.create_publisher(PoseStamped, '/goal_pose',      10)
        self.aim_pub    = self.create_publisher(Point,       '/apex_aim_point', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/patrol_markers', 10)

        self.create_timer(0.5, self.tick)     # 2 Hz FSM

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
        """조난자 후보 발견 → 즉시 정지하고 확인 상태로."""
        if self.state in (INSPECT, FIRE_ALARM):
            return
        self._inspect_pos = (msg.point.x, msg.point.y)
        self._inspect_start = self._now()
        self._state_before_inspect = self.state
        self.state = INSPECT
        self._stop_here()
        self.get_logger().info(
            f'👤 조난자 후보 ({self._inspect_pos[0]:.1f}, {self._inspect_pos[1]:.1f}) '
            '— 정지, 포탑 조준 확인 중')

    def inspect_done_cb(self, msg: Bool):
        if self.state != INSPECT:
            return
        self.get_logger().info(
            '조난자 확인 완료 — 순찰 재개' if msg.data
            else '확인 실패(놓침) — 순찰 재개')
        self._inspect_pos = None
        self._resume_patrol()

    def enable_cb(self, msg: Bool):
        self.enabled = msg.data
        self.get_logger().info(f'/patrol_enable = {msg.data}')
        if not self.enabled and self.state == PATROL:
            self.state = IDLE

    def goal_echo_cb(self, msg: PoseStamped):
        if self._just_published:
            self._just_published = False       # 자기 에코 → 무시
            return
        # 외부(RViz/CLI) goal → 수동 우선
        self._manual_goal = (msg.pose.position.x, msg.pose.position.y)
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

    def _send_goal(self, x, y, yaw=None):
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
        self._just_published = True
        self.goal_pub.publish(g)

    def _stop_here(self):
        rx, ry = self._robot_pose()
        self._send_goal(rx, ry, yaw=self.robot_theta)

    # ── 프론티어 탐사 ────────────────────────────────────────────────
    def _find_frontiers(self):
        """SLAM 맵에서 '알려진 자유공간 ↔ 미탐사' 경계 셀을 군집화해 반환.

        맵을 미리 알 필요 없이, 지금까지 그린 지도의 가장자리로 나아가며
        스스로 탐사 범위를 넓힌다. 반환: [(x, y, 셀수), ...] (map 좌표)
        """
        grid = self._map_msg
        if grid is None:
            return []
        w, h = grid.info.width, grid.info.height
        res  = grid.info.resolution
        ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
        data = grid.data

        # 자유(0..occ_thresh) 셀 중 상하좌우에 미탐사(-1)가 붙은 셀 = 프론티어
        frontier_cells = []
        for iy in range(1, h - 1):
            row = iy * w
            for ix in range(1, w - 1):
                if not (0 <= data[row + ix] < 25):
                    continue
                if (data[row + ix + 1] == -1 or data[row + ix - 1] == -1
                        or data[row + w + ix] == -1 or data[row - w + ix] == -1):
                    frontier_cells.append((ix, iy))
        if not frontier_cells:
            return []

        # 그리드 인접 군집화 (BFS)
        cellset = set(frontier_cells)
        clusters = []
        while cellset:
            seed = cellset.pop()
            stack, comp = [seed], [seed]
            while stack:
                cx, cy = stack.pop()
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        n = (cx + dx, cy + dy)
                        if n in cellset:
                            cellset.remove(n)
                            stack.append(n)
                            comp.append(n)
            if len(comp) >= self.frontier_min:
                mx = sum(c[0] for c in comp) / len(comp)
                my = sum(c[1] for c in comp) / len(comp)
                clusters.append((ox + (mx + 0.5) * res,
                                 oy + (my + 0.5) * res,
                                 len(comp)))
        return clusters

    def _pick_frontier(self, rx, ry):
        """가까우면서 충분히 큰 프론티어 선택 (이미 시도한 곳은 제외)."""
        best, best_score = None, -1e9
        for fx, fy, n in self._find_frontiers():
            if any(math.hypot(fx - vx, fy - vy) < 1.5
                   for vx, vy in self._visited_frontiers):
                continue
            d = math.hypot(fx - rx, fy - ry)
            if d < 0.8:            # 코앞은 의미 없음
                continue
            # 크기는 이득, 거리는 비용
            score = n * 0.5 - d
            if score > best_score:
                best_score, best = score, (fx, fy)
        return best

    # ── FSM ──────────────────────────────────────────────────────────
    def tick(self):
        if not self.map_ready:
            return
        rx, ry = self._robot_pose()

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
            # 정지 유지 + 포탑이 후보를 보도록 조준점 계속 발행
            if self._inspect_pos is not None:
                p = Point()
                p.x, p.y, p.z = self._inspect_pos[0], self._inspect_pos[1], 0.5
                self.aim_pub.publish(p)
            # target_manager가 죽거나 응답이 없을 때를 대비한 안전장치
            if (self._inspect_start is not None
                    and self._now() - self._inspect_start > self.inspect_timeout):
                self.get_logger().warn('확인 응답 없음 — 순찰 재개')
                self._inspect_pos = None
                self._resume_patrol()

        elif self.state == MANUAL:
            if self._manual_goal is not None:
                mx, my = self._manual_goal
                if math.hypot(mx - rx, my - ry) < self.reach_dist:
                    self.get_logger().info('수동 목표 도착 → 순찰 재개')
                    self._manual_goal = None
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
        reached  = goal is not None and math.hypot(goal[0]-rx, goal[1]-ry) < 1.0
        timedout = (self._wp_sent_t is not None
                    and now - self._wp_sent_t > self.wp_timeout)
        need_new = (goal is None or reached or timedout
                    or now - self._frontier_t > self.frontier_replan)
        if not need_new:
            return

        if goal is not None and (reached or timedout):
            self._visited_frontiers.append(goal)
            if timedout:
                self.get_logger().warn(
                    f'프론티어 ({goal[0]:.1f},{goal[1]:.1f}) 도달 실패 — 다른 곳으로')

        nxt = self._pick_frontier(rx, ry)
        self._frontier_t = now
        if nxt is None:
            if not self._explore_done:
                self._explore_done = True
                self.get_logger().info(
                    '미탐사 경계 없음 — 탐사 완료. 재순찰을 위해 방문 기록 초기화')
            # 다 훑었으면 방문 기록을 비워 재순찰(계속 감시)
            self._visited_frontiers.clear()
            self._frontier_goal = None
            return

        self._explore_done = False
        # 같은 목표를 다시 고른 경우엔 goal 재발행/로그를 생략 (Nav2 경로 리셋 방지)
        same = (goal is not None and not reached and not timedout
                and math.hypot(nxt[0] - goal[0], nxt[1] - goal[1]) < 0.5)
        self._frontier_goal = nxt
        if same:
            return
        self._send_goal(nxt[0], nxt[1])
        self._wp_sent_t = now
        self.get_logger().info(
            f'탐사 목표 → ({nxt[0]:.1f}, {nxt[1]:.1f})  '
            f'[미방문 경계 기준, 누적 방문 {len(self._visited_frontiers)}곳]')

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
