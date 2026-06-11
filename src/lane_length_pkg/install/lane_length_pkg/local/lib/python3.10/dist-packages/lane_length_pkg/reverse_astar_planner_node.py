#!/usr/bin/env python3
from __future__ import annotations

from typing import Optional

import heapq
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class ReverseHybridAStarPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('reverse_astar_planner_node')

        self.declare_parameter('plan_hz', 5.0)
        self.declare_parameter('grid_resolution_m', 0.05)
        self.declare_parameter('heading_resolution_deg', 5.0)
        self.declare_parameter('step_m', 0.05)
        self.declare_parameter('max_curvature', 4.0)
        self.declare_parameter('num_steers', 7)
        self.declare_parameter('max_iterations', 8000)
        self.declare_parameter('goal_xy_tol_m', 0.06)
        self.declare_parameter('obstacle_min_radius_m', 0.03)
        self.declare_parameter('obstacle_max_radius_m', 0.20)
        self.declare_parameter('obstacle_inflation_m', 0.02)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('use_map_obstacles', True)
        self.declare_parameter('use_scan_obstacles', False)
        self.declare_parameter('map_occupied_threshold', 50)
        self.declare_parameter('map_collision_radius_m', 0.11)
        self.declare_parameter('map_unknown_is_obstacle', False)
        self.declare_parameter('lidar_range_min_m', 0.10)
        self.declare_parameter('lidar_range_max_m', 3.50)
        self.declare_parameter('scan_yaw_offset_deg', 0.0)
        self.declare_parameter('lidar_front_ignore_deg', 60.0)
        self.declare_parameter('scan_timeout_sec', 0.5)
        self.declare_parameter('scan_cluster_gap_m', 0.10)
        self.declare_parameter('scan_cluster_min_points', 2)
        self.declare_parameter('reverse_obstacle_avoidance', False)

        self.plan_hz = float(self.get_parameter('plan_hz').value)
        self.grid_resolution_m = float(self.get_parameter('grid_resolution_m').value)
        self.heading_resolution_deg = float(self.get_parameter('heading_resolution_deg').value)
        self.step_m = float(self.get_parameter('step_m').value)
        self.max_curvature = float(self.get_parameter('max_curvature').value)
        self.num_steers = max(3, int(self.get_parameter('num_steers').value))
        self.max_iterations = int(self.get_parameter('max_iterations').value)
        self.goal_xy_tol_m = float(self.get_parameter('goal_xy_tol_m').value)
        self.obstacle_min_radius_m = float(self.get_parameter('obstacle_min_radius_m').value)
        self.obstacle_max_radius_m = float(self.get_parameter('obstacle_max_radius_m').value)
        self.obstacle_inflation_m = float(self.get_parameter('obstacle_inflation_m').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.use_map_obstacles = bool(self.get_parameter('use_map_obstacles').value)
        self.use_scan_obstacles = bool(self.get_parameter('use_scan_obstacles').value)
        self.map_occupied_threshold = int(self.get_parameter('map_occupied_threshold').value)
        self.map_collision_radius_m = float(self.get_parameter('map_collision_radius_m').value)
        self.map_unknown_is_obstacle = bool(self.get_parameter('map_unknown_is_obstacle').value)
        self.lidar_range_min_m = float(self.get_parameter('lidar_range_min_m').value)
        self.lidar_range_max_m = float(self.get_parameter('lidar_range_max_m').value)
        self.scan_yaw_offset_rad = math.radians(
            float(self.get_parameter('scan_yaw_offset_deg').value)
        )
        self.lidar_front_ignore_deg = float(self.get_parameter('lidar_front_ignore_deg').value)
        self.scan_timeout_sec = float(self.get_parameter('scan_timeout_sec').value)
        self.scan_cluster_gap_m = float(self.get_parameter('scan_cluster_gap_m').value)
        self.scan_cluster_min_points = max(1, int(self.get_parameter('scan_cluster_min_points').value))
        self.reverse_obstacle_avoidance = bool(
            self.get_parameter('reverse_obstacle_avoidance').value)

        self.goal: dict = {}
        self.pose: Optional[PoseStamped] = None
        self.scan: Optional[LaserScan] = None
        self.scan_stamp: Optional[float] = None
        self.map_msg: Optional[OccupancyGrid] = None
        self.map_occ: Optional[np.ndarray] = None
        self.last_failure_reason = ''
        self.last_map_wait_log_sec = 0.0

        self.create_subscription(String, '/planning/behavior_goal', self.goal_cb, 10)
        self.create_subscription(PoseStamped, '/pose', self.pose_cb, 10)
        if self.use_scan_obstacles:
            self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)
        if self.use_map_obstacles:
            self.create_subscription(OccupancyGrid, self.map_topic, self.map_cb, 1)

        self.path_pub = self.create_publisher(Path, '/planning/path', 10)

        dt = 1.0 / self.plan_hz if self.plan_hz > 0.0 else 0.2
        self.timer = self.create_timer(dt, self.plan_step)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def goal_cb(self, msg: String) -> None:
        try:
            self.goal = json.loads(msg.data)
        except json.JSONDecodeError:
            self.goal = {}

    def pose_cb(self, msg: PoseStamped) -> None:
        self.pose = msg

    def scan_cb(self, msg: LaserScan) -> None:
        self.scan = msg
        self.scan_stamp = self.now_sec()

    # ── 라이다 장애물 추출 (전방 ±ignore_deg 제외) ──────────────────────────
    def map_cb(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.map_occ = np.asarray(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )

    def map_point_free(self, x: float, y: float) -> bool:
        if not self.use_map_obstacles or self.map_msg is None or self.map_occ is None:
            return True

        info = self.map_msg.info
        res = float(info.resolution)
        if res <= 0.0:
            return True

        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        mx = int(math.floor((x - ox) / res))
        my = int(math.floor((y - oy) / res))
        if mx < 0 or my < 0 or mx >= int(info.width) or my >= int(info.height):
            return True

        radius_cells = max(0, int(math.ceil(self.map_collision_radius_m / res)))
        radius_sq = self.map_collision_radius_m * self.map_collision_radius_m
        min_x = max(0, mx - radius_cells)
        max_x = min(int(info.width), mx + radius_cells + 1)
        min_y = max(0, my - radius_cells)
        max_y = min(int(info.height), my + radius_cells + 1)

        for cy in range(min_y, max_y):
            wy = oy + (cy + 0.5) * res
            dy = wy - y
            for cx in range(min_x, max_x):
                wx = ox + (cx + 0.5) * res
                dx = wx - x
                if dx * dx + dy * dy > radius_sq:
                    continue

                occ = int(self.map_occ[cy, cx])
                if occ >= self.map_occupied_threshold:
                    return False
                if self.map_unknown_is_obstacle and occ < 0:
                    return False
        return True

    def obstacle_points(self, px: float, py: float, yaw: float) -> list[tuple[float, float, float]]:
        if self.scan is None or self.scan_stamp is None:
            return []
        if (self.now_sec() - self.scan_stamp) > self.scan_timeout_sec:
            return []

        msg = self.scan
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        angles = (
            msg.angle_min
            + np.arange(ranges.size, dtype=np.float32) * msg.angle_increment
            + self.scan_yaw_offset_rad
        )
        angle_deg = np.degrees(angles)

        ignore = self.lidar_front_ignore_deg
        use_mask = (angle_deg < -ignore) | (angle_deg > ignore)
        valid = (
            use_mask
            & np.isfinite(ranges)
            & (ranges >= self.lidar_range_min_m)
            & (ranges <= min(self.lidar_range_max_m, msg.range_max - 1e-3))
        )
        if not np.any(valid):
            return []

        bxs = ranges[valid] * np.cos(angles[valid])
        bys = ranges[valid] * np.sin(angles[valid])
        c, s = math.cos(yaw), math.sin(yaw)
        wxs = px + c * bxs - s * bys
        wys = py + s * bxs + c * bys

        pts = np.column_stack([wxs, wys])
        dists = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        splits = np.where(dists > self.scan_cluster_gap_m)[0] + 1
        out: list[tuple[float, float, float]] = []
        for group in np.split(pts, splits):
            if len(group) < self.scan_cluster_min_points:
                continue
            cx_g, cy_g = group.mean(axis=0)
            r = float(np.linalg.norm(group - [cx_g, cy_g], axis=1).max())
            r = max(self.obstacle_min_radius_m, min(self.obstacle_max_radius_m, r))
            out.append((float(cx_g), float(cy_g), r + self.obstacle_inflation_m))
        return out

    # ── Hybrid A* ────────────────────────────────────────────────────────────
    def hybrid_astar(
        self,
        sx: float, sy: float, syaw: float,
        gx: float, gy: float,
        obstacles: list[tuple[float, float, float]],
    ) -> Optional[list[tuple[float, float]]]:

        res = max(0.01, self.grid_resolution_m)
        dtheta = math.radians(max(1.0, self.heading_resolution_deg))
        num_theta = max(1, int(round(2.0 * math.pi / dtheta)))
        step = max(0.01, self.step_m)
        curvatures = np.linspace(-self.max_curvature, self.max_curvature, self.num_steers).tolist()

        def discretize(x: float, y: float, yaw: float) -> tuple[int, int, int]:
            ith = int(round(normalize_angle(yaw) / dtheta)) % num_theta
            return int(round(x / res)), int(round(y / res)), ith

        def step_motion(x: float, y: float, yaw: float, kappa: float) -> tuple[float, float, float]:
            # 후진: 헤딩 반대 방향으로 이동, 곡률에 따라 yaw 변화
            if abs(kappa) < 1e-6:
                return x - step * math.cos(yaw), y - step * math.sin(yaw), yaw
            dyaw = -kappa * step
            mid = yaw + dyaw * 0.5
            return (
                x - step * math.cos(mid),
                y - step * math.sin(mid),
                normalize_angle(yaw + dyaw),
            )

        def collision_free(x: float, y: float) -> bool:
            if not self.map_point_free(x, y):
                stats['map_reject'] += 1
                return False
            for ox, oy, r in obstacles:
                if math.hypot(x - ox, y - oy) <= r:
                    stats['obstacle_reject'] += 1
                    return False
            return True

        def h(x: float, y: float) -> float:
            return math.hypot(x - gx, y - gy)

        start_key = discretize(sx, sy, syaw)
        goal_ix = int(round(gx / res))
        goal_iy = int(round(gy / res))
        stats = {'expanded': 0, 'map_reject': 0, 'obstacle_reject': 0}

        if not self.map_point_free(sx, sy):
            self.last_failure_reason = (
                f'start pose collides with map sx={sx:.3f} sy={sy:.3f} '
                f'radius={self.map_collision_radius_m:.2f}m'
            )
            return None
        if not self.map_point_free(gx, gy):
            self.last_failure_reason = (
                f'goal collides with map gx={gx:.3f} gy={gy:.3f} '
                f'radius={self.map_collision_radius_m:.2f}m'
            )
            return None

        # heap: (f, g, continuous_state, discrete_key)
        heap: list[tuple[float, float, tuple, tuple]] = []
        heapq.heappush(heap, (h(sx, sy), 0.0, (sx, sy, syaw), start_key))
        came_from: dict[tuple, tuple[tuple, tuple]] = {}
        g_cost: dict[tuple, float] = {start_key: 0.0}

        for _ in range(self.max_iterations):
            if not heap:
                break
            _, cur_g, (cx, cy, cyaw), cur_key = heapq.heappop(heap)

            if cur_key[:2] == (goal_ix, goal_iy) or math.hypot(cx - gx, cy - gy) <= self.goal_xy_tol_m:
                path: list[tuple[float, float, float]] = [(cx, cy, cyaw)]
                k = cur_key
                while k in came_from:
                    k, state = came_from[k]
                    path.append(state)
                path.reverse()
                return [(x, y) for x, y, _ in path]

            if cur_g > g_cost.get(cur_key, float('inf')):
                continue

            stats['expanded'] += 1
            for kappa in curvatures:
                nx, ny, nyaw = step_motion(cx, cy, cyaw, kappa)
                if not collision_free(nx, ny):
                    continue
                nkey = discretize(nx, ny, nyaw)
                ng = cur_g + step
                if ng < g_cost.get(nkey, float('inf')):
                    came_from[nkey] = (cur_key, (cx, cy, cyaw))
                    g_cost[nkey] = ng
                    heapq.heappush(heap, (ng + h(nx, ny), ng, (nx, ny, nyaw), nkey))

        self.last_failure_reason = (
            f'no path sx=({sx:.3f},{sy:.3f}) gx=({gx:.3f},{gy:.3f}) '
            f'expanded={stats["expanded"]} map_reject={stats["map_reject"]} '
            f'obstacle_reject={stats["obstacle_reject"]} '
            f'map={"yes" if self.map_msg is not None else "no"}'
        )
        return None

    # ── 경로 메시지 생성 ─────────────────────────────────────────────────────
    def build_path(self, points: list[tuple[float, float]], target_theta: float, reverse: bool) -> Path:
        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.pose.header.frame_id if self.pose is not None else 'odom'
        for i, (x, y) in enumerate(points):
            ps = PoseStamped()
            ps.header = out.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            if i + 1 < len(points):
                nx, ny = points[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = target_theta
            if reverse:
                yaw += math.pi
            ps.pose.orientation.z = math.sin(yaw * 0.5)
            ps.pose.orientation.w = math.cos(yaw * 0.5)
            out.poses.append(ps)
        return out

    def path_is_free(self, path: Path, obstacles: list[tuple[float, float, float]]) -> bool:
        check_map = self.use_map_obstacles and self.map_msg is not None and self.map_occ is not None

        for ps in path.poses:
            px = float(ps.pose.position.x)
            py = float(ps.pose.position.y)
            if check_map and not self.map_point_free(px, py):
                return False
            for ox, oy, r in obstacles:
                if math.hypot(px - ox, py - oy) <= r:
                    return False
        return True

    def publish_empty_path(self) -> None:
        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.pose.header.frame_id if self.pose is not None else 'odom'
        self.path_pub.publish(out)

    # ── 메인 루프 ────────────────────────────────────────────────────────────
    def plan_step(self) -> None:
        if self.pose is None:
            return

        goal_type = str(self.goal.get('type', ''))
        if goal_type not in ('reverse_to_pullover', 'side_pull_over', 'reenter_lane'):
            self.publish_empty_path()
            return

        if self.use_map_obstacles and (self.map_msg is None or self.map_occ is None):
            now = self.now_sec()
            if now - self.last_map_wait_log_sec >= 1.0:
                self.get_logger().warn('Waiting for map before Hybrid A* planning')
                self.last_map_wait_log_sec = now
            self.publish_empty_path()
            return

        pose = self.pose.pose
        sx = float(pose.position.x)
        sy = float(pose.position.y)
        yaw = yaw_from_quat(pose.orientation)
        gx = float(self.goal.get('target_x', sx))
        gy = float(self.goal.get('target_y', sy))
        target_theta = float(self.goal.get('target_theta', yaw))
        reverse = bool(self.goal.get('reverse', False))
        obstacles = (
            self.obstacle_points(sx, sy, yaw)
            if self.use_scan_obstacles and (not reverse or self.reverse_obstacle_avoidance)
            else []
        )

        path = self.hybrid_astar(sx, sy, yaw, gx, gy, obstacles)
        if not path:
            self.get_logger().warn(
                f'Hybrid A* failed ({self.last_failure_reason}); '
                'publishing empty path to hold stop'
            )
            self.publish_empty_path()
            return

        if math.hypot(path[-1][0] - gx, path[-1][1] - gy) > 1e-3:
            path.append((gx, gy))
        self.path_pub.publish(self.build_path(path, target_theta, reverse))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReverseHybridAStarPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
