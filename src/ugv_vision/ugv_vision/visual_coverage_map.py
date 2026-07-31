"""
visual_coverage_map.py
======================
라이다 맵(SLAM) vs 카메라 시야 맵 이원화 구조.

핵심 개념:
  - SLAM OccupancyGrid: 벽/장애물 위치 (라이다로 그림)
  - VisualCoverageMap:  카메라 시야가 실제로 훑은 격자 (카메라로 봄)
  - 라이다로 열린 공간이지만 카메라로 못 본 곳 = 미수색 구역 (Unseen Free Space)

격자 값:
  0  = Unseen (자유 공간이지만 카메라로 못 봄)
  1  = Seen   (카메라 FOV가 통과한 셀)
 -1  = Unknown (SLAM도 모르는 구역, 접근 불가)
"""
import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header


def euler_from_quaternion(q):
    t3 = 2.0 * (q.w * q.z + q.x * q.y)
    t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(t3, t4)


class VisualCoverageGrid:
    """
    2D 시야 커버리지 격자.
    TacticalCommanderNode가 인스턴스를 생성해 직접 사용한다.
    """

    def __init__(self, resolution: float = 0.2, size_m: float = 40.0):
        self.resolution = resolution
        self.n = int(size_m / resolution)      # 격자 한 변 셀 수
        self.origin = -size_m / 2.0            # 월드 원점 오프셋 (m)
        # 초기 상태: 전부 Unseen(0)
        self.grid = np.zeros((self.n, self.n), dtype=np.int8)

    # ── 월드 좌표 ↔ 격자 인덱스 ──────────────────────────────────────
    def _w2g(self, wx: float, wy: float):
        gx = int((wx - self.origin) / self.resolution)
        gy = int((wy - self.origin) / self.resolution)
        return gx, gy

    def _in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.n and 0 <= gy < self.n

    # ── 카메라 FOV 부채꼴 투사 (매 프레임 호출) ──────────────────────
    def update_fov(
        self,
        robot_x: float, robot_y: float,
        robot_theta: float, turret_yaw: float,
        fov_rad: float = 1.089,   # 수평 62°
        max_range: float = 4.0,   # 가시거리 (m)
        num_rays: int   = 30,
    ):
        """
        로봇 위치·헤딩·포탑 요 각도로부터 카메라 FOV 부채꼴을 계산하고
        그 내부 격자를 Seen(1) 으로 갱신한다.
        """
        cam_theta = robot_theta + turret_yaw
        half_fov  = fov_rad / 2.0

        for i in range(num_rays):
            ray_angle = cam_theta - half_fov + i * fov_rad / max(num_rays - 1, 1)
            step = self.resolution * 0.8   # 샘플 간격 (격자 해상도보다 촘촘하게)
            d = 0.3
            while d <= max_range:
                wx = robot_x + d * math.cos(ray_angle)
                wy = robot_y + d * math.sin(ray_angle)
                gx, gy = self._w2g(wx, wy)
                if not self._in_bounds(gx, gy):
                    break
                if self.grid[gy, gx] == -1:   # 벽 — 레이 중단
                    break
                self.grid[gy, gx] = 1
                d += step

    def mark_wall(self, wx: float, wy: float):
        """SLAM 맵에서 점유된 셀을 벽(-1)으로 마킹"""
        gx, gy = self._w2g(wx, wy)
        if self._in_bounds(gx, gy):
            self.grid[gy, gx] = -1

    # ── 미수색 프론티어 반환 ──────────────────────────────────────────
    def get_unseen_frontiers(
        self,
        robot_x: float, robot_y: float,
        slam_free: np.ndarray | None = None,
    ) -> list[tuple[float, float]]:
        """
        SLAM 상 자유공간(Free)이면서 아직 Seen(1)이 아닌 셀 좌표 반환.
        slam_free: SLAM OccupancyGrid를 같은 해상도로 리샘플링한 0/1 배열.
                   None이면 VisualCoverageGrid 자체 내부 정보만 사용.
        반환: [(world_x, world_y), ...]
        """
        frontiers = []
        for gy in range(self.n):
            for gx in range(self.n):
                cell = self.grid[gy, gx]
                if cell == 0:    # Unseen
                    if slam_free is None or (
                        0 <= gy < slam_free.shape[0] and
                        0 <= gx < slam_free.shape[1] and
                        slam_free[gy, gx] == 0          # SLAM에서도 자유공간
                    ):
                        wx = self.origin + (gx + 0.5) * self.resolution
                        wy = self.origin + (gy + 0.5) * self.resolution
                        frontiers.append((wx, wy))
        return frontiers

    # ── RViz OccupancyGrid 변환 ───────────────────────────────────────
    def to_occupancy_grid(self, stamp, frame_id: str = 'map') -> OccupancyGrid:
        """
        RViz 시각화 인코딩:
          Seen(1)   →  0  : 탐색 완료 (RViz Map에서 흰색/투명)
          Unseen(0) → -1  : 미탐색 자유공간 (RViz에서 회색 unknown)
          Wall(-1)  → 100 : 벽 (검정)

        RViz 설정 가이드:
          Topic /visual_coverage → Color Scheme: costmap, Alpha 0.45
          → 탐색 완료 구역이 파란색 계열로 구분 표시됨.
          SLAM /map(회색 벽 지도) 위에 /visual_coverage(파란 탐색완료)를
          겹쳐 띄우면 '본 곳/못 본 곳'이 직관적으로 구분된다.
        """
        msg = OccupancyGrid()
        msg.header.stamp    = stamp
        msg.header.frame_id = frame_id
        msg.info.resolution = self.resolution
        msg.info.width      = self.n
        msg.info.height     = self.n
        msg.info.origin.position.x = self.origin
        msg.info.origin.position.y = self.origin
        msg.info.origin.orientation.w = 1.0

        # numpy 배열 → flat list (빠름)
        out = np.full((self.n, self.n), -1, dtype=np.int8)
        out[self.grid == 1]  = 0     # Seen → free (파란색으로 표시될 것)
        out[self.grid == -1] = 100   # Wall
        msg.data = out.flatten().tolist()
        return msg


# ── ROS2 노드 래퍼 ────────────────────────────────────────────────────
class VisualCoverageNode(Node):
    """
    VisualCoverageGrid를 구독-퍼블리시하는 독립 노드.
    TacticalCommanderNode와 별도로 실행하거나 합쳐서 쓸 수 있음.
    """

    def __init__(self):
        super().__init__('visual_coverage_node')

        self.cov_map = VisualCoverageGrid(resolution=0.2, size_m=40.0)

        self.robot_x      = 0.0
        self.robot_y      = 0.0
        self.robot_theta  = 0.0
        self.turret_yaw   = 0.0

        self.create_subscription(Odometry,     '/odom',         self._odom_cb,  10)
        self.create_subscription(JointState,   '/joint_states', self._joint_cb, 10)
        self.create_subscription(OccupancyGrid,'/map',          self._map_cb,   10)

        self.pub_coverage = self.create_publisher(
            OccupancyGrid, '/visual_coverage', 10)

        # 매 0.5초마다 FOV 업데이트 & 퍼블리시
        self.create_timer(0.5, self._update)
        self.get_logger().info('VisualCoverageNode 시작')

    def _odom_cb(self, msg):
        self.robot_x     = msg.pose.pose.position.x
        self.robot_y     = msg.pose.pose.position.y
        self.robot_theta = euler_from_quaternion(msg.pose.pose.orientation)

    def _joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name == 'turret_yaw_joint':
                self.turret_yaw = pos

    def _map_cb(self, msg):
        """SLAM 맵 수신 시 점유 셀을 벽으로 마킹"""
        res  = msg.info.resolution
        ox   = msg.info.origin.position.x
        oy   = msg.info.origin.position.y
        w, h = msg.info.width, msg.info.height
        for gy in range(h):
            for gx in range(w):
                idx = gy * w + gx
                if msg.data[idx] > 50:   # 점유(벽)
                    wx = ox + (gx + 0.5) * res
                    wy = oy + (gy + 0.5) * res
                    self.cov_map.mark_wall(wx, wy)

    def _update(self):
        self.cov_map.update_fov(
            self.robot_x, self.robot_y,
            self.robot_theta, self.turret_yaw,
        )
        grid_msg = self.cov_map.to_occupancy_grid(
            self.get_clock().now().to_msg())
        self.pub_coverage.publish(grid_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VisualCoverageNode())
    rclpy.shutdown()
