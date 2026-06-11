#!/usr/bin/env python3
from typing import Optional

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PathTrackNode(Node):
    def __init__(self) -> None:
        super().__init__('path_track_node')

        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('lookahead_distance_m', 0.15)
        self.declare_parameter('reverse_lookahead_distance_m', 0.22)
        self.declare_parameter('reverse_speed', 0.020)
        self.declare_parameter('reenter_speed', 0.018)
        self.declare_parameter('max_angular_speed', 0.30)
        self.declare_parameter('reverse_max_angular_speed', 0.15)
        self.declare_parameter('goal_tolerance_m', 0.04)
        self.declare_parameter('min_lookahead_m', 0.08)
        self.declare_parameter('max_lookahead_m', 0.30)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('scan_yaw_offset_deg', 0.0)
        self.declare_parameter('reverse_obstacle_guard', True)
        self.declare_parameter('reverse_obstacle_stop_m', 0.22)
        self.declare_parameter('reverse_obstacle_y_half_m', 0.18)
        self.declare_parameter('reverse_obstacle_min_points', 2)
        self.declare_parameter('scan_timeout_sec', 0.5)

        self.control_hz = float(self.get_parameter('control_hz').value)
        self.lookahead_distance_m = float(self.get_parameter('lookahead_distance_m').value)
        self.reverse_lookahead_distance_m = float(
            self.get_parameter('reverse_lookahead_distance_m').value
        )
        self.reverse_speed = float(self.get_parameter('reverse_speed').value)
        self.reenter_speed = float(self.get_parameter('reenter_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.reverse_max_angular_speed = float(
            self.get_parameter('reverse_max_angular_speed').value
        )
        self.goal_tolerance_m = float(self.get_parameter('goal_tolerance_m').value)
        self.min_lookahead_m = float(self.get_parameter('min_lookahead_m').value)
        self.max_lookahead_m = float(self.get_parameter('max_lookahead_m').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.scan_yaw_offset_rad = math.radians(
            float(self.get_parameter('scan_yaw_offset_deg').value)
        )
        self.reverse_obstacle_guard = bool(self.get_parameter('reverse_obstacle_guard').value)
        self.reverse_obstacle_stop_m = float(self.get_parameter('reverse_obstacle_stop_m').value)
        self.reverse_obstacle_y_half_m = float(self.get_parameter('reverse_obstacle_y_half_m').value)
        self.reverse_obstacle_min_points = max(
            1, int(self.get_parameter('reverse_obstacle_min_points').value)
        )
        self.scan_timeout_sec = float(self.get_parameter('scan_timeout_sec').value)

        self.path: Optional[Path] = None
        self.pose: Optional[PoseStamped] = None
        self.driving_mode: str = ''
        self.prev_driving_mode: str = ''
        self.goal_reached = False
        self.last_scan_stamp: Optional[float] = None
        self.reverse_blocked = False
        self.last_reverse_blocked_log_sec = 0.0
        self.last_stop_reason = ''
        self.last_stop_log_sec = 0.0

        self.create_subscription(Path, '/planning/path', self.path_cb, 10)
        self.create_subscription(PoseStamped, '/pose', self.pose_cb, 10)
        self.create_subscription(String, '/planning/driving_mode', self.mode_cb, 10)
        if self.reverse_obstacle_guard:
            self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_path', 10)
        self.goal_pub = self.create_publisher(String, '/planning/path_goal_reached', 10)

        dt = 1.0 / self.control_hz if self.control_hz > 0.0 else 0.05
        self.timer = self.create_timer(dt, self.control_step)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def path_cb(self, msg: Path) -> None:
        self.path = msg

    def pose_cb(self, msg: PoseStamped) -> None:
        self.pose = msg

    def mode_cb(self, msg: String) -> None:
        mode = msg.data.strip()
        if mode != self.prev_driving_mode:
            self.goal_reached = False
            self.prev_driving_mode = mode
        self.driving_mode = mode

    def scan_cb(self, msg: LaserScan) -> None:
        self.last_scan_stamp = self.now_sec()
        self.reverse_blocked = self.has_rear_obstacle(msg)

    def has_rear_obstacle(self, msg: LaserScan) -> bool:
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        if ranges.size == 0:
            return False

        angles = (
            msg.angle_min
            + np.arange(ranges.size, dtype=np.float32) * msg.angle_increment
            + self.scan_yaw_offset_rad
        )
        valid = np.isfinite(ranges)
        if math.isfinite(msg.range_min) and msg.range_min > 0.0:
            valid &= ranges >= msg.range_min
        if math.isfinite(msg.range_max) and msg.range_max > 0.0:
            valid &= ranges <= msg.range_max

        if not np.any(valid):
            return False

        xs = ranges[valid] * np.cos(angles[valid])
        ys = ranges[valid] * np.sin(angles[valid])
        rear_mask = (
            (xs <= -0.02)
            & ((-xs) <= self.reverse_obstacle_stop_m)
            & (np.abs(ys) <= self.reverse_obstacle_y_half_m)
        )
        return int(np.count_nonzero(rear_mask)) >= self.reverse_obstacle_min_points

    def scan_is_fresh(self) -> bool:
        if self.last_scan_stamp is None:
            return False
        return (self.now_sec() - self.last_scan_stamp) <= self.scan_timeout_sec

    def log_reverse_blocked(self) -> None:
        now = self.now_sec()
        if now - self.last_reverse_blocked_log_sec < 1.0:
            return
        self.last_reverse_blocked_log_sec = now
        self.get_logger().warn(
            f'Reverse obstacle guard stopping: rear obstacle within '
            f'{self.reverse_obstacle_stop_m:.2f}m'
        )

    def log_stop_reason(self, reason: str) -> None:
        now = self.now_sec()
        if reason == self.last_stop_reason and (now - self.last_stop_log_sec) < 1.0:
            return
        self.last_stop_reason = reason
        self.last_stop_log_sec = now
        self.get_logger().warn(f'Path tracker stopping: {reason}')

    def publish_stop(self, reason: str = '') -> None:
        if reason:
            self.log_stop_reason(reason)
        self.cmd_pub.publish(Twist())

    def closest_index(self, rx: float, ry: float) -> int:
        assert self.path is not None
        best_idx = 0
        best_dist = float('inf')
        for i, ps in enumerate(self.path.poses):
            dx = ps.pose.position.x - rx
            dy = ps.pose.position.y - ry
            dist = dx * dx + dy * dy
            if dist < best_dist:
                best_dist = dist
                best_idx = i
        return best_idx

    def lookahead_index(self, closest_idx: int, rx: float, ry: float, lookahead: float) -> int:
        assert self.path is not None
        ld = clamp(lookahead, self.min_lookahead_m, self.max_lookahead_m)
        for i in range(closest_idx, len(self.path.poses)):
            ps = self.path.poses[i]
            if math.hypot(ps.pose.position.x - rx, ps.pose.position.y - ry) >= ld:
                return i
        return len(self.path.poses) - 1

    def control_step(self) -> None:
        mode = self.driving_mode.upper()
        if mode not in ('YIELD_REVERSE', 'YIELD_SIDE', 'REENTER'):
            self.publish_stop()
            return

        if self.path is None:
            self.publish_stop('no path')
            return

        if self.pose is None:
            self.publish_stop('no pose')
            return

        if self.goal_reached:
            self.publish_stop()
            return

        rx = float(self.pose.pose.position.x)
        ry = float(self.pose.pose.position.y)

        if self.path.poses:
            final_pose = self.path.poses[-1].pose
            dist_goal = math.hypot(final_pose.position.x - rx, final_pose.position.y - ry)
            if dist_goal < self.goal_tolerance_m:
                self.goal_reached = True
                msg = String()
                msg.data = self.driving_mode
                self.goal_pub.publish(msg)
                if mode == 'YIELD_REVERSE':
                    self.get_logger().info(
                        f'후진완료: '
                        f'pose=({rx:.3f}, {ry:.3f}) '
                        f'target=({final_pose.position.x:.3f}, {final_pose.position.y:.3f}) '
                        f'dist={dist_goal:.3f}m'
                    )
                self.publish_stop()
                return

        if len(self.path.poses) < 2:
            self.publish_stop(f'path too short poses={len(self.path.poses)}')
            return

        if (
            mode == 'YIELD_REVERSE'
            and self.reverse_obstacle_guard
            and self.scan_is_fresh()
            and self.reverse_blocked
        ):
            self.log_reverse_blocked()
            self.publish_stop()
            return

        ryaw = yaw_from_quat(self.pose.pose.orientation)

        reverse_mode = mode == 'YIELD_REVERSE'
        lookahead = self.reverse_lookahead_distance_m if reverse_mode else self.lookahead_distance_m
        angular_limit = self.reverse_max_angular_speed if reverse_mode else self.max_angular_speed

        closest_idx = self.closest_index(rx, ry)
        la_idx = self.lookahead_index(closest_idx, rx, ry, lookahead)
        la = self.path.poses[la_idx].pose.position

        dx = la.x - rx
        dy = la.y - ry
        local_x = math.cos(ryaw) * dx + math.sin(ryaw) * dy
        local_y = -math.sin(ryaw) * dx + math.cos(ryaw) * dy
        ld_actual = math.hypot(local_x, local_y)
        if ld_actual < 1e-6:
            self.publish_stop('lookahead distance too small')
            return

        if mode == 'YIELD_REVERSE':
            # Reverse pure pursuit with the same lateral sign convention as forward:
            # a target on the robot's right remains negative local_y.
            local_x_r = -local_x
            local_y_r = local_y
            alpha_r = math.atan2(local_y_r, local_x_r)
            kappa_r = 2.0 * math.sin(alpha_r) / ld_actual
            v = -self.reverse_speed
            omega = v * kappa_r
        else:
            alpha = math.atan2(local_y, local_x)
            kappa = 2.0 * math.sin(alpha) / ld_actual
            v = self.reenter_speed
            omega = v * kappa

        tw = Twist()
        tw.linear.x = v
        tw.angular.z = clamp(omega, -angular_limit, angular_limit)
        self.cmd_pub.publish(tw)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
