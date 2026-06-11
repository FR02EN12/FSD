#!/usr/bin/env python3
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class CmdMuxNode(Node):
    def __init__(self) -> None:
        super().__init__('cmd_mux_node')

        self.declare_parameter('mux_hz', 20.0)
        self.declare_parameter('source_timeout_sec', 0.5)
        self.declare_parameter('transition_duration_sec', 0.3)
        self.declare_parameter('approach_speed_scale', 0.7)

        self.mux_hz = float(self.get_parameter('mux_hz').value)
        self.source_timeout_sec = float(self.get_parameter('source_timeout_sec').value)
        self.transition_duration_sec = float(self.get_parameter('transition_duration_sec').value)
        self.approach_speed_scale = float(self.get_parameter('approach_speed_scale').value)

        self.driving_mode = ''
        self.last_cmd_lane: Optional[Twist] = None
        self.last_cmd_lane_stamp: Optional[float] = None
        self.last_cmd_path: Optional[Twist] = None
        self.last_cmd_path_stamp: Optional[float] = None
        self.prev_source: Optional[str] = None
        self.transition_start: Optional[float] = None
        self.prev_output = Twist()
        self.last_published = Twist()

        self.create_subscription(String, '/planning/driving_mode', self.mode_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_lane', self.lane_cmd_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_path', self.path_cmd_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_selected', 10)

        dt = 1.0 / self.mux_hz if self.mux_hz > 0.0 else 0.05
        self.timer = self.create_timer(dt, self.mux_step)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def mode_cb(self, msg: String) -> None:
        self.driving_mode = msg.data.strip()

    def lane_cmd_cb(self, msg: Twist) -> None:
        self.last_cmd_lane = msg
        self.last_cmd_lane_stamp = self.now_sec()

    def path_cmd_cb(self, msg: Twist) -> None:
        self.last_cmd_path = msg
        self.last_cmd_path_stamp = self.now_sec()

    @staticmethod
    def source_for_mode(mode: str) -> str:
        mode = mode.upper()
        if mode in ('LANE_FOLLOW', 'APPROACH_NARROW', 'RIGHT_OFFSET_PASS'):
            return 'lane'
        if mode in ('YIELD_REVERSE', 'YIELD_SIDE', 'REENTER'):
            return 'path'
        return 'zero'

    def get_source_cmd(self, source: str) -> Twist:
        now = self.now_sec()
        if source == 'lane':
            if self.last_cmd_lane is not None and self.last_cmd_lane_stamp is not None:
                if (now - self.last_cmd_lane_stamp) <= self.source_timeout_sec:
                    return self.last_cmd_lane
            return Twist()
        if source == 'path':
            if self.last_cmd_path is not None and self.last_cmd_path_stamp is not None:
                if (now - self.last_cmd_path_stamp) <= self.source_timeout_sec:
                    return self.last_cmd_path
            return Twist()
        return Twist()

    @staticmethod
    def blend_twist(a: Twist, b: Twist, alpha: float) -> Twist:
        out = Twist()
        out.linear.x = a.linear.x * (1.0 - alpha) + b.linear.x * alpha
        out.angular.z = a.angular.z * (1.0 - alpha) + b.angular.z * alpha
        return out

    def mux_step(self) -> None:
        mode = self.driving_mode
        source = self.source_for_mode(mode)
        now = self.now_sec()

        if self.prev_source is not None and source != self.prev_source:
            self.transition_start = now
            self.prev_output = self.last_published

        target = self.get_source_cmd(source)
        if mode.upper() == 'APPROACH_NARROW':
            target.linear.x *= self.approach_speed_scale

        if self.transition_start is not None:
            elapsed = now - self.transition_start
            if elapsed < self.transition_duration_sec and self.transition_duration_sec > 0.0:
                output = self.blend_twist(self.prev_output, target, elapsed / self.transition_duration_sec)
            else:
                output = target
                self.transition_start = None
        else:
            output = target

        self.prev_source = source
        self.last_published = output
        self.cmd_pub.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
