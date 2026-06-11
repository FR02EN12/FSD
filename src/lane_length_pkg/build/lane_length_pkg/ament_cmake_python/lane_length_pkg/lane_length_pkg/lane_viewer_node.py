#!/usr/bin/env python3
import threading
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Float32, String


class ViewerState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.image: Optional[np.ndarray] = None
        self.bev_image: Optional[np.ndarray] = None
        self.centerline = np.empty((0, 2), dtype=np.float32)
        self.left = np.empty((0, 2), dtype=np.float32)
        self.right = np.empty((0, 2), dtype=np.float32)
        self.obstacles = np.empty((0, 3), dtype=np.float32)
        self.scan_points = np.empty((0, 2), dtype=np.float32)
        self.status = 'unknown'
        self.mode = 'unknown'
        self.source = 'none'
        self.width_m = 0.0
        self.safe_stop = False
        self.linear_x = 0.0
        self.angular_z = 0.0


def image_msg_to_bgr(msg: Image) -> Optional[np.ndarray]:
    enc = msg.encoding.lower()
    channels_by_encoding = {
        'bgr8': 3,
        'rgb8': 3,
        'bgra8': 4,
        'rgba8': 4,
        'mono8': 1,
    }
    channels = channels_by_encoding.get(enc)
    if channels is None:
        return None
    try:
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        rows = raw.reshape(msg.height, msg.step)
        arr = rows[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    except ValueError:
        return None
    if enc == 'bgr8':
        return arr.copy()
    if enc == 'rgb8':
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    if enc == 'bgra8':
        return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
    if enc == 'rgba8':
        return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
    if enc == 'mono8':
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    return None


def path_to_xy(msg: Path) -> np.ndarray:
    pts = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]
    return np.asarray(pts, dtype=np.float32) if pts else np.empty((0, 2), dtype=np.float32)


class LaneViewerNode(Node):
    def __init__(self) -> None:
        super().__init__('viewer_node')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('bev_image_topic', '/lane_bev_image')
        self.declare_parameter('centerline_topic', '/fused/centerline_path')
        self.declare_parameter('left_boundary_topic', '/fused/left_boundary_path')
        self.declare_parameter('right_boundary_topic', '/fused/right_boundary_path')
        self.declare_parameter('obstacles_topic', '/fused/obstacles')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('status_topic', '/lane_status')
        self.declare_parameter('source_topic', '/lane_guidance_source')
        self.declare_parameter('mode_topic', '/control_mode')
        self.declare_parameter('safe_stop_topic', '/safe_stop')
        self.declare_parameter('lane_width_topic', '/lane_width_m')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('view_hz', 10.0)
        self.declare_parameter('show_camera', True)
        self.declare_parameter('show_obstacles', False)
        self.declare_parameter('show_lidar_scan_points', True)
        self.declare_parameter('use_actual_bev_background', True)
        self.declare_parameter('map_width_px', 520)
        self.declare_parameter('map_height_px', 520)
        self.declare_parameter('map_x_max_m', 1.60)
        self.declare_parameter('map_y_half_m', 0.80)
        self.declare_parameter('bev_width_px', 120)
        self.declare_parameter('bev_height_px', 220)
        self.declare_parameter('px_per_m', 100.0)
        self.declare_parameter('camera_to_base_m', 0.08)
        self.declare_parameter('bottom_visible_from_camera_m', 0.30)
        self.declare_parameter('lateral_zero_bias_m', 0.0)
        self.declare_parameter('lidar_y_sign_in_actual_bev', -1.0)
        self.declare_parameter('lidar_range_min_m', 0.16)
        self.declare_parameter('lidar_range_max_m', 3.50)
        self.declare_parameter('scan_yaw_offset_rad', 0.0)
        self.declare_parameter('scan_point_radius_px', 1)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.bev_image_topic = str(self.get_parameter('bev_image_topic').value)
        self.centerline_topic = str(self.get_parameter('centerline_topic').value)
        self.left_boundary_topic = str(self.get_parameter('left_boundary_topic').value)
        self.right_boundary_topic = str(self.get_parameter('right_boundary_topic').value)
        self.obstacles_topic = str(self.get_parameter('obstacles_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.source_topic = str(self.get_parameter('source_topic').value)
        self.mode_topic = str(self.get_parameter('mode_topic').value)
        self.safe_stop_topic = str(self.get_parameter('safe_stop_topic').value)
        self.lane_width_topic = str(self.get_parameter('lane_width_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.view_hz = float(self.get_parameter('view_hz').value)
        self.show_camera = bool(self.get_parameter('show_camera').value)
        self.show_obstacles = bool(self.get_parameter('show_obstacles').value)
        self.show_lidar_scan_points = bool(self.get_parameter('show_lidar_scan_points').value)
        self.use_actual_bev_background = bool(self.get_parameter('use_actual_bev_background').value)
        self.map_width_px = int(self.get_parameter('map_width_px').value)
        self.map_height_px = int(self.get_parameter('map_height_px').value)
        self.map_x_max_m = float(self.get_parameter('map_x_max_m').value)
        self.map_y_half_m = float(self.get_parameter('map_y_half_m').value)
        self.bev_width_px = int(self.get_parameter('bev_width_px').value)
        self.bev_height_px = int(self.get_parameter('bev_height_px').value)
        self.px_per_m = float(self.get_parameter('px_per_m').value)
        self.camera_to_base_m = float(self.get_parameter('camera_to_base_m').value)
        self.bottom_visible_from_camera_m = float(self.get_parameter('bottom_visible_from_camera_m').value)
        self.lateral_zero_bias_m = float(self.get_parameter('lateral_zero_bias_m').value)
        self.lidar_y_sign_in_actual_bev = float(self.get_parameter('lidar_y_sign_in_actual_bev').value)
        self.lidar_range_min_m = float(self.get_parameter('lidar_range_min_m').value)
        self.lidar_range_max_m = float(self.get_parameter('lidar_range_max_m').value)
        self.scan_yaw_offset_rad = float(self.get_parameter('scan_yaw_offset_rad').value)
        self.scan_point_radius_px = max(1, int(self.get_parameter('scan_point_radius_px').value))

        self.state = ViewerState()
        self.create_subscription(Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.bev_image_topic, self.bev_image_cb, qos_profile_sensor_data)
        self.create_subscription(Path, self.centerline_topic, self.centerline_cb, 10)
        self.create_subscription(Path, self.left_boundary_topic, self.left_cb, 10)
        self.create_subscription(Path, self.right_boundary_topic, self.right_cb, 10)
        self.create_subscription(PoseArray, self.obstacles_topic, self.obstacles_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(String, self.status_topic, self.status_cb, 10)
        self.create_subscription(String, self.source_topic, self.source_cb, 10)
        self.create_subscription(String, self.mode_topic, self.mode_cb, 10)
        self.create_subscription(Bool, self.safe_stop_topic, self.safe_stop_cb, 10)
        self.create_subscription(Float32, self.lane_width_topic, self.width_cb, 10)
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_cb, 10)

        dt = 1.0 / self.view_hz if self.view_hz > 0.0 else 0.1
        self.timer = self.create_timer(dt, self.draw)
        self.get_logger().info('viewer_node started')

    def image_cb(self, msg: Image) -> None:
        img = image_msg_to_bgr(msg)
        if img is not None:
            with self.state.lock:
                self.state.image = img

    def bev_image_cb(self, msg: Image) -> None:
        img = image_msg_to_bgr(msg)
        if img is not None:
            with self.state.lock:
                self.state.bev_image = img

    def centerline_cb(self, msg: Path) -> None:
        with self.state.lock:
            self.state.centerline = path_to_xy(msg)

    def left_cb(self, msg: Path) -> None:
        with self.state.lock:
            self.state.left = path_to_xy(msg)

    def right_cb(self, msg: Path) -> None:
        with self.state.lock:
            self.state.right = path_to_xy(msg)

    def obstacles_cb(self, msg: PoseArray) -> None:
        pts = [(float(p.position.x), float(p.position.y), float(p.position.z)) for p in msg.poses]
        with self.state.lock:
            self.state.obstacles = np.asarray(pts, dtype=np.float32) if pts else np.empty((0, 3), dtype=np.float32)

    def scan_cb(self, msg: LaserScan) -> None:
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        if ranges.size == 0:
            pts = np.empty((0, 2), dtype=np.float32)
        else:
            angles = msg.angle_min + np.arange(ranges.size, dtype=np.float32) * msg.angle_increment
            valid = (
                np.isfinite(ranges) &
                (ranges >= self.lidar_range_min_m) &
                (ranges <= min(self.lidar_range_max_m, msg.range_max - 1e-3))
            )
            raw_xs = ranges[valid] * np.cos(angles[valid])
            raw_ys = ranges[valid] * np.sin(angles[valid])
            c = np.cos(self.scan_yaw_offset_rad)
            s = np.sin(self.scan_yaw_offset_rad)
            xs = c * raw_xs - s * raw_ys
            ys = s * raw_xs + c * raw_ys
            pts = np.column_stack([xs, ys]).astype(np.float32) if xs.size else np.empty((0, 2), dtype=np.float32)
        with self.state.lock:
            self.state.scan_points = pts

    def status_cb(self, msg: String) -> None:
        with self.state.lock:
            self.state.status = str(msg.data)

    def source_cb(self, msg: String) -> None:
        with self.state.lock:
            self.state.source = str(msg.data)

    def mode_cb(self, msg: String) -> None:
        with self.state.lock:
            self.state.mode = str(msg.data)

    def safe_stop_cb(self, msg: Bool) -> None:
        with self.state.lock:
            self.state.safe_stop = bool(msg.data)

    def width_cb(self, msg: Float32) -> None:
        with self.state.lock:
            self.state.width_m = float(msg.data)

    def cmd_cb(self, msg: Twist) -> None:
        with self.state.lock:
            self.state.linear_x = float(msg.linear.x)
            self.state.angular_z = float(msg.angular.z)

    def map_xy_to_uv(self, x: float, y: float) -> tuple[int, int]:
        u = int(round((self.map_y_half_m - y) / (2.0 * self.map_y_half_m) * (self.map_width_px - 1)))
        v = int(round((self.map_x_max_m - x) / self.map_x_max_m * (self.map_height_px - 1)))
        return u, v

    def bev_xy_to_uv(self, x: float, y: float) -> tuple[int, int]:
        metric_y = y + self.lateral_zero_bias_m
        cam_forward_m = x - self.camera_to_base_m
        bev_u = (self.bev_width_px * 0.5) + metric_y * self.px_per_m
        bev_v = (self.bev_height_px - 1.0) - (
            (cam_forward_m - self.bottom_visible_from_camera_m) * self.px_per_m
        )
        u = int(round(bev_u * (self.map_width_px - 1) / max(self.bev_width_px - 1, 1)))
        v = int(round(bev_v * (self.map_height_px - 1) / max(self.bev_height_px - 1, 1)))
        return u, v

    def draw_path(self, canvas: np.ndarray, pts: np.ndarray, color: tuple[int, int, int], thickness: int) -> None:
        if pts.shape[0] < 2:
            return
        to_uv = self.bev_xy_to_uv if self.using_actual_bev(canvas) else self.map_xy_to_uv
        uv = np.array([to_uv(float(x), float(y)) for x, y in pts], dtype=np.int32)
        cv2.polylines(canvas, [uv.reshape(-1, 1, 2)], False, color, thickness, cv2.LINE_AA)

    def using_actual_bev(self, canvas: np.ndarray) -> bool:
        return bool(getattr(self, '_drawing_actual_bev', False))

    def draw_map(self, snapshot: ViewerState) -> np.ndarray:
        actual_bev = self.use_actual_bev_background and snapshot.bev_image is not None
        self._drawing_actual_bev = actual_bev
        if actual_bev:
            canvas = cv2.resize(snapshot.bev_image, (self.map_width_px, self.map_height_px))
        else:
            canvas = np.full((self.map_height_px, self.map_width_px, 3), 245, dtype=np.uint8)
            for x in np.arange(0.0, self.map_x_max_m + 1e-6, 0.2):
                _, v = self.map_xy_to_uv(float(x), 0.0)
                cv2.line(canvas, (0, v), (self.map_width_px - 1, v), (225, 225, 225), 1)
            for y in np.arange(-self.map_y_half_m, self.map_y_half_m + 1e-6, 0.2):
                u, _ = self.map_xy_to_uv(0.0, float(y))
                cv2.line(canvas, (u, 0), (u, self.map_height_px - 1), (225, 225, 225), 1)

        self.draw_path(canvas, snapshot.left, (255, 120, 0), 2)
        self.draw_path(canvas, snapshot.right, (0, 120, 255), 2)
        self.draw_path(canvas, snapshot.centerline, (0, 180, 0), 3)

        if self.show_lidar_scan_points and snapshot.scan_points.size:
            to_uv = self.bev_xy_to_uv if actual_bev else self.map_xy_to_uv
            for x, y in snapshot.scan_points:
                draw_y = float(y) * self.lidar_y_sign_in_actual_bev if actual_bev else float(y)
                u, v = to_uv(float(x), draw_y)
                if 0 <= u < self.map_width_px and 0 <= v < self.map_height_px:
                    cv2.circle(canvas, (u, v), self.scan_point_radius_px, (0, 0, 255), -1, cv2.LINE_AA)

        if self.show_obstacles:
            for x, y, radius in snapshot.obstacles:
                to_uv = self.bev_xy_to_uv if actual_bev else self.map_xy_to_uv
                draw_y = float(y) * self.lidar_y_sign_in_actual_bev if actual_bev else float(y)
                u, v = to_uv(float(x), draw_y)
                if actual_bev:
                    r_px = max(4, int(round(float(radius) * self.px_per_m * self.map_width_px / max(self.bev_width_px, 1))))
                else:
                    r_px = max(4, int(round(float(radius) / max(2.0 * self.map_y_half_m, 1e-6) * self.map_width_px)))
                cv2.circle(canvas, (u, v), r_px, (0, 0, 220), -1, cv2.LINE_AA)

        to_uv = self.bev_xy_to_uv if actual_bev else self.map_xy_to_uv
        ego = np.array([
            to_uv(0.00, 0.00),
            to_uv(0.10, 0.06),
            to_uv(0.10, -0.06),
        ], dtype=np.int32)
        cv2.fillConvexPoly(canvas, ego, (40, 40, 40), cv2.LINE_AA)

        lines = [
            f'status={snapshot.status} source={snapshot.source}',
            f'mode={snapshot.mode} safe_stop={snapshot.safe_stop}',
            f'width={snapshot.width_m:.3f}m cmd=({snapshot.linear_x:.3f}, {snapshot.angular_z:.3f})',
        ]
        for i, text in enumerate(lines):
            cv2.putText(canvas, text, (12, 24 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 2, cv2.LINE_AA)
        return canvas

    def draw(self) -> None:
        snapshot = ViewerState()
        with self.state.lock:
            snapshot.image = None if self.state.image is None else self.state.image.copy()
            snapshot.bev_image = None if self.state.bev_image is None else self.state.bev_image.copy()
            snapshot.centerline = self.state.centerline.copy()
            snapshot.left = self.state.left.copy()
            snapshot.right = self.state.right.copy()
            snapshot.obstacles = self.state.obstacles.copy()
            snapshot.scan_points = self.state.scan_points.copy()
            snapshot.status = self.state.status
            snapshot.mode = self.state.mode
            snapshot.source = self.state.source
            snapshot.width_m = self.state.width_m
            snapshot.safe_stop = self.state.safe_stop
            snapshot.linear_x = self.state.linear_x
            snapshot.angular_z = self.state.angular_z

        map_img = self.draw_map(snapshot)
        if self.show_camera and snapshot.image is not None:
            cam = cv2.resize(snapshot.image, (self.map_width_px, self.map_height_px))
            view = np.hstack([cam, map_img])
        else:
            view = map_img
        cv2.imshow('lane_viewer', view)
        cv2.waitKey(1)

    def destroy_node(self) -> None:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneViewerNode()
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
