#!/usr/bin/env python3
"""
topic_watchdog: runtime topic-contract verification.

Subscribes to continuous topics in the system contract table and checks that
each arrives within its expected maximum age.  Event-driven topics are kept out
of this node so normal mode changes do not look like communication failures.
No other nodes depend on this; it is purely observational and never alters the
control pipeline.
"""
import json
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Float32, String


# ---------------------------------------------------------------------------
# Contract table: (topic, msg_type, max_age_sec)
# max_age_sec is the longest interval at which a healthy continuous publisher
# should produce messages. Optional/event-driven topics such as /safety/events,
# /planning/path, and /cmd_vel_path are intentionally excluded.
# ---------------------------------------------------------------------------
_CONTRACTS = [
    # Sensor layer
    ('/lane_status',                    String,  0.20),
    ('/lane_error_center_m',            Float32, 0.20),
    ('/lane_error_left_m',              Float32, 0.20),
    ('/fused/lane_width_m',             Float32, 0.20),
    ('/lane_guidance_source',           String,  0.20),
    # Perception layer
    ('/scene/understanding',            String,  0.20),
    ('/perception/opponent_vehicle',    String,  1.00),
    ('/perception/opponent_led_state',  String,  1.00),
    ('/perception/opponent_v2v_sign',   String,  1.00),
    ('/memory/pull_over_candidates',    String,  2.00),
    ('/memory/narrow_passages',         String,  2.00),
    # Localisation
    ('/pose',                           PoseStamped, 0.20),
    # Decision layer
    ('/planning/driving_mode',          String,  0.10),
    ('/planning/deadlock_state',        String,  0.10),
    ('/planning/cooperation_decision',  String,  0.20),
    ('/planning/v2v_led_pattern',       String,  0.20),
    ('/planning/behavior_goal',         String,  0.20),
    # Control layer
    ('/cmd_vel_lane',                   Twist,   0.10),
    ('/cmd_vel_selected',               Twist,   0.10),
    ('/cmd_vel',                        Twist,   0.10),
]

class TopicWatchdogNode(Node):
    def __init__(self) -> None:
        super().__init__('topic_watchdog')

        self.declare_parameter('watchdog_hz', 1.0)
        self.declare_parameter('startup_grace_sec', 3.0)
        self.watchdog_hz = float(self.get_parameter('watchdog_hz').value)
        self.startup_grace_sec = float(
            self.get_parameter('startup_grace_sec').value)
        self._start_stamp = self.get_clock().now().nanoseconds * 1e-9

        # {topic: last_stamp}
        self._last_seen: Dict[str, Optional[float]] = {}
        # {topic: max_age_sec}
        self._max_age: Dict[str, float] = {}

        for (topic, msg_cls, max_age) in _CONTRACTS:
            self._last_seen[topic] = None
            self._max_age[topic] = max_age
            # Closure captures topic name correctly
            self._make_sub(topic, msg_cls)

        self._pub = self.create_publisher(String, '/diagnostics/topic_watchdog', 10)
        dt = 1.0 / max(self.watchdog_hz, 0.1)
        self.create_timer(dt, self._check)
        self.get_logger().info(f'TopicWatchdogNode started, monitoring {len(_CONTRACTS)} topics')

    def _make_sub(self, topic: str, cls: type) -> None:
        def _cb(_msg):
            self._last_seen[topic] = self.get_clock().now().nanoseconds * 1e-9
        self.create_subscription(cls, topic, _cb, 10)

    def _check(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        in_startup_grace = (now - self._start_stamp) < self.startup_grace_sec
        ok: list = []
        stale: list = []

        for topic, last in self._last_seen.items():
            max_age = self._max_age[topic]
            if last is None:
                if in_startup_grace:
                    continue
                age = float('inf')
                stale.append({'topic': topic, 'age_sec': None, 'max_age_sec': max_age})
                self.get_logger().warn(f'[watchdog] {topic}: no message received yet')
            else:
                age = now - last
                if age > max_age:
                    stale.append({'topic': topic, 'age_sec': round(age, 2),
                                  'max_age_sec': max_age})
                    self.get_logger().warn(
                        f'[watchdog] {topic}: stale ({age:.2f} s > {max_age} s)')
                else:
                    ok.append(topic)

        report = {
            'timestamp': round(now, 3),
            'ok_count': len(ok),
            'stale_count': len(stale),
            'stale': stale,
        }
        msg = String()
        msg.data = json.dumps(report)
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TopicWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except BaseException:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except BaseException:
                pass


if __name__ == '__main__':
    main()
