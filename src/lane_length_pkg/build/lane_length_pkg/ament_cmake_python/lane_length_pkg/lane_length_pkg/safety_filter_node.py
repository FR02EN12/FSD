#!/usr/bin/env python3
from typing import Optional

import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class SafetyFilterNode(Node):
    def __init__(self) -> None:
        super().__init__('safety_filter_node')

        self.declare_parameter('filter_hz', 20.0)
        self.declare_parameter('emergency_hold_sec', 1.0)
        self.declare_parameter('warning_scale', 0.5)
        self.declare_parameter('max_linear_accel', 0.03)
        self.declare_parameter('max_angular_accel', 0.5)
        self.declare_parameter('max_linear_speed', 0.05)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('watchdog_timeout_sec', 0.5)
        self.declare_parameter('led_stop_topic', '/perception/opponent_v2v_sign')
        self.declare_parameter('led_stop_bbox_area_px', 1200)
        self.declare_parameter('led_stop_confirm_sec', 3.0)
        self.declare_parameter('led_stop_hold_sec', 0.5)

        self.filter_hz = float(self.get_parameter('filter_hz').value)
        self.emergency_hold_sec = float(self.get_parameter('emergency_hold_sec').value)
        self.warning_scale = float(self.get_parameter('warning_scale').value)
        self.max_linear_accel = float(self.get_parameter('max_linear_accel').value)
        self.max_angular_accel = float(self.get_parameter('max_angular_accel').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.watchdog_timeout_sec = float(self.get_parameter('watchdog_timeout_sec').value)
        self.led_stop_topic = str(self.get_parameter('led_stop_topic').value)
        self.led_stop_bbox_area_px = int(self.get_parameter('led_stop_bbox_area_px').value)
        self.led_stop_confirm_sec = float(self.get_parameter('led_stop_confirm_sec').value)
        self.led_stop_hold_sec = float(self.get_parameter('led_stop_hold_sec').value)

        self.last_cmd: Optional[Twist] = None
        self.last_cmd_stamp: Optional[float] = None
        self.safety_level = 'CLEAR'
        self.driving_mode = ''
        self.emergency_until: Optional[float] = None
        self.led_stop_until: Optional[float] = None
        self.led_stop_candidate_since: Optional[float] = None
        self.last_stop_reason = ''
        self.last_stop_log_sec = 0.0
        self.prev_linear = 0.0
        self.prev_angular = 0.0
        self.prev_stamp: Optional[float] = None

        self.create_subscription(Twist, '/cmd_vel_selected', self.cmd_cb, 10)
        self.create_subscription(String, '/safety/events', self.safety_cb, 10)
        self.create_subscription(String, self.led_stop_topic, self.led_cb, 10)
        self.create_subscription(String, '/planning/driving_mode', self.mode_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        dt = 1.0 / self.filter_hz if self.filter_hz > 0.0 else 0.05
        self.timer = self.create_timer(dt, self.filter_step)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def cmd_cb(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_stamp = self.now_sec()

    def mode_cb(self, msg: String) -> None:
        self.driving_mode = msg.data.strip().upper()
        if self.driving_mode in ('YIELD_REVERSE', 'YIELD_SIDE', 'REENTER'):
            self.led_stop_candidate_since = None
            self.led_stop_until = None

    def safety_cb(self, msg: String) -> None:
        raw = msg.data
        level = raw
        try:
            data = json.loads(raw)
            level = data.get('severity') or data.get('type') or raw
        except json.JSONDecodeError:
            pass
        level = str(level).strip().upper()

        if level in ('EMERGENCY', 'CRITICAL'):
            self.safety_level = 'EMERGENCY'
            self.emergency_until = self.now_sec() + self.emergency_hold_sec
        elif level == 'WARNING':
            self.safety_level = 'WARNING'
        elif level in ('INFO', 'CLEAR'):
            self.safety_level = 'CLEAR'

    def led_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        try:
            area = int(float(data.get('bbox_area_px', 0)))
        except (TypeError, ValueError):
            area = 0
        detected = bool(data.get('detected', False))
        box_detected = bool(data.get('box_detected', False))
        mask = str(data.get('mask') or data.get('led_mask') or '')
        stop_candidate = (
            detected
            and box_detected
            and mask == '11111'
            and area >= self.led_stop_bbox_area_px
        )

        now = self.now_sec()
        if self.driving_mode in ('YIELD_REVERSE', 'YIELD_SIDE', 'REENTER'):
            self.led_stop_candidate_since = None
            self.led_stop_until = None
            return

        if stop_candidate:
            if self.led_stop_candidate_since is None:
                self.led_stop_candidate_since = now
            if (now - self.led_stop_candidate_since) >= self.led_stop_confirm_sec:
                self.led_stop_until = now + self.led_stop_hold_sec
        else:
            self.led_stop_candidate_since = None

    def log_stop_reason(self, reason: str) -> None:
        now = self.now_sec()
        if reason == self.last_stop_reason and (now - self.last_stop_log_sec) < 1.0:
            return
        self.last_stop_reason = reason
        self.last_stop_log_sec = now
        self.get_logger().warn(f'Safety filter stopping: {reason}')

    def publish_stop(self, reason: str = '') -> None:
        if reason:
            self.log_stop_reason(reason)
        self.cmd_pub.publish(Twist())
        self.prev_linear = 0.0
        self.prev_angular = 0.0

    def filter_step(self) -> None:
        now = self.now_sec()

        if self.emergency_until is not None:
            if now < self.emergency_until:
                self.publish_stop('emergency hold')
                return
            self.emergency_until = None
            if self.safety_level == 'EMERGENCY':
                self.safety_level = 'CLEAR'

        if self.led_stop_until is not None:
            if now < self.led_stop_until:
                self.publish_stop('led 11111 stop hold')
                return
            self.led_stop_until = None

        if self.last_cmd is None or self.last_cmd_stamp is None:
            self.publish_stop('no selected cmd')
            return
        if (now - self.last_cmd_stamp) > self.watchdog_timeout_sec:
            self.publish_stop('selected cmd watchdog timeout')
            return

        target_lin = float(self.last_cmd.linear.x)
        target_ang = float(self.last_cmd.angular.z)

        if self.safety_level == 'WARNING':
            target_lin *= self.warning_scale
            target_ang *= self.warning_scale

        target_lin = clamp(target_lin, -self.max_linear_speed, self.max_linear_speed)
        target_ang = clamp(target_ang, -self.max_angular_speed, self.max_angular_speed)

        if self.prev_stamp is not None:
            dt = now - self.prev_stamp
            if dt > 0.0:
                max_dlin = self.max_linear_accel * dt
                max_dang = self.max_angular_accel * dt
                target_lin = self.prev_linear + clamp(target_lin - self.prev_linear, -max_dlin, max_dlin)
                target_ang = self.prev_angular + clamp(target_ang - self.prev_angular, -max_dang, max_dang)

        self.prev_linear = target_lin
        self.prev_angular = target_ang
        self.prev_stamp = now

        tw = Twist()
        tw.linear.x = target_lin
        tw.angular.z = target_ang
        self.cmd_pub.publish(tw)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyFilterNode()
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
