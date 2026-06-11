#!/usr/bin/env python3
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String


def cluster_scan_obstacles(xs: np.ndarray, ys: np.ndarray, gap_m: float, min_points: int, padding_m: float):
    if xs.size == 0:
        return []
    pts = np.column_stack([xs, ys])
    d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    splits = np.where(d > gap_m)[0] + 1
    out = []
    for group in np.split(pts, splits):
        if len(group) < min_points:
            continue
        cx, cy = group.mean(axis=0)
        radius = float(np.linalg.norm(group - [cx, cy], axis=1).max()) + padding_m
        out.append((float(cx), float(cy), radius))
    return out


class LaneIntegrationNode(Node):
    def __init__(self) -> None:
        super().__init__('integration_node')

        self.declare_parameter('centerline_topic', '/lane_centerline_base_path')
        self.declare_parameter('left_boundary_topic', '/lane_left_boundary_base_path')
        self.declare_parameter('right_boundary_topic', '/lane_right_boundary_base_path')
        self.declare_parameter('lane_status_topic', '/lane_status')
        self.declare_parameter('lane_width_topic', '/lane_width_m')
        self.declare_parameter('scan_topic', '/scan')

        self.declare_parameter('lidar_range_min_m', 0.16)
        self.declare_parameter('lidar_range_max_m', 3.50)
        self.declare_parameter('scan_yaw_offset_rad', 0.0)
        self.declare_parameter('obstacle_direction', 'forward')
        self.declare_parameter('scan_ignore_angle_min_deg', -30.0)
        self.declare_parameter('scan_ignore_angle_max_deg', 30.0)
        self.declare_parameter('scan_ignore_range_max_m', 0.35)
        self.declare_parameter('obstacle_x_min_m', 0.05)
        self.declare_parameter('obstacle_x_max_m', 2.50)
        self.declare_parameter('obstacle_y_half_m', 1.20)
        self.declare_parameter('cluster_gap_m', 0.22)
        self.declare_parameter('cluster_min_points', 2)
        self.declare_parameter('cluster_padding_m', 0.04)

        self.centerline_topic = str(self.get_parameter('centerline_topic').value)
        self.left_boundary_topic = str(self.get_parameter('left_boundary_topic').value)
        self.right_boundary_topic = str(self.get_parameter('right_boundary_topic').value)
        self.lane_status_topic = str(self.get_parameter('lane_status_topic').value)
        self.lane_width_topic = str(self.get_parameter('lane_width_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)

        self.lidar_range_min_m = float(self.get_parameter('lidar_range_min_m').value)
        self.lidar_range_max_m = float(self.get_parameter('lidar_range_max_m').value)
        self.scan_yaw_offset_rad = float(self.get_parameter('scan_yaw_offset_rad').value)
        self.obstacle_direction = str(self.get_parameter('obstacle_direction').value).strip().lower()
        self.scan_ignore_angle_min_deg = float(self.get_parameter('scan_ignore_angle_min_deg').value)
        self.scan_ignore_angle_max_deg = float(self.get_parameter('scan_ignore_angle_max_deg').value)
        self.scan_ignore_range_max_m = float(self.get_parameter('scan_ignore_range_max_m').value)
        self.obstacle_x_min_m = float(self.get_parameter('obstacle_x_min_m').value)
        self.obstacle_x_max_m = float(self.get_parameter('obstacle_x_max_m').value)
        self.obstacle_y_half_m = float(self.get_parameter('obstacle_y_half_m').value)
        self.cluster_gap_m = float(self.get_parameter('cluster_gap_m').value)
        self.cluster_min_points = max(1, int(self.get_parameter('cluster_min_points').value))
        self.cluster_padding_m = float(self.get_parameter('cluster_padding_m').value)

        self.latest_centerline: Optional[Path] = None
        self.latest_left: Optional[Path] = None
        self.latest_right: Optional[Path] = None
        self.latest_status: str = 'lost'
        self.latest_width_m: float = 0.0

        self.create_subscription(Path, self.centerline_topic, self.centerline_cb, 10)
        self.create_subscription(Path, self.left_boundary_topic, self.left_cb, 10)
        self.create_subscription(Path, self.right_boundary_topic, self.right_cb, 10)
        self.create_subscription(String, self.lane_status_topic, self.status_cb, 10)
        self.create_subscription(Float32, self.lane_width_topic, self.width_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)

        self.centerline_pub = self.create_publisher(Path, '/fused/centerline_path', 10)
        self.left_pub = self.create_publisher(Path, '/fused/left_boundary_path', 10)
        self.right_pub = self.create_publisher(Path, '/fused/right_boundary_path', 10)
        self.status_pub = self.create_publisher(String, '/fused/lane_status', 10)
        self.width_pub = self.create_publisher(Float32, '/fused/lane_width_m', 10)
        self.obstacles_pub = self.create_publisher(PoseArray, '/fused/obstacles', 10)

        self.get_logger().info('integration_node started | camera lane + lidar obstacles -> /fused/*')

    def centerline_cb(self, msg: Path) -> None:
        self.latest_centerline = msg
        self.centerline_pub.publish(msg)

    def left_cb(self, msg: Path) -> None:
        self.latest_left = msg
        self.left_pub.publish(msg)

    def right_cb(self, msg: Path) -> None:
        self.latest_right = msg
        self.right_pub.publish(msg)

    def status_cb(self, msg: String) -> None:
        self.latest_status = str(msg.data)
        self.status_pub.publish(msg)

    def width_cb(self, msg: Float32) -> None:
        self.latest_width_m = float(msg.data)
        self.width_pub.publish(msg)

    def scan_cb(self, msg: LaserScan) -> None:
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        out = PoseArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'base_link'
        if ranges.size == 0:
            self.obstacles_pub.publish(out)
            return

        angles = msg.angle_min + np.arange(ranges.size, dtype=np.float32) * msg.angle_increment
        valid = (
            np.isfinite(ranges) &
            (ranges >= self.lidar_range_min_m) &
            (ranges <= min(self.lidar_range_max_m, msg.range_max - 1e-3))
        )
        if not np.any(valid):
            self.obstacles_pub.publish(out)
            return

        angle_deg = np.degrees(angles)
        ignore_near_mount = (
            (angle_deg >= self.scan_ignore_angle_min_deg) &
            (angle_deg <= self.scan_ignore_angle_max_deg) &
            (ranges <= self.scan_ignore_range_max_m)
        )
        valid &= ~ignore_near_mount
        if not np.any(valid):
            self.obstacles_pub.publish(out)
            return

        raw_xs = ranges[valid] * np.cos(angles[valid])
        raw_ys = ranges[valid] * np.sin(angles[valid])
        c = np.cos(self.scan_yaw_offset_rad)
        s = np.sin(self.scan_yaw_offset_rad)
        xs = c * raw_xs - s * raw_ys
        ys = s * raw_xs + c * raw_ys
        longitudinal = -xs if self.obstacle_direction == 'reverse' else xs
        roi = (
            (longitudinal >= self.obstacle_x_min_m) &
            (longitudinal <= self.obstacle_x_max_m) &
            (np.abs(ys) <= self.obstacle_y_half_m)
        )
        clusters = cluster_scan_obstacles(
            xs[roi], ys[roi],
            self.cluster_gap_m,
            self.cluster_min_points,
            self.cluster_padding_m,
        )
        for cx, cy, radius in clusters:
            pose = Pose()
            pose.position.x = cx
            pose.position.y = cy
            pose.position.z = radius
            out.poses.append(pose)
        self.obstacles_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneIntegrationNode()
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
