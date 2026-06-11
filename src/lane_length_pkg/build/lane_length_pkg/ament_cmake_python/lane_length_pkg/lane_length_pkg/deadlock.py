#!/usr/bin/env python3
# Performance target T4: deadlock response <= 1.0 s
#   detection_hz raised 10 -> 20 (max detection latency 0.05 s/cycle)
from typing import Optional

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DeadlockDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('deadlock')

        # T4: 20 Hz gives a worst-case per-cycle latency of 0.05 s,
        #     contributing to the overall <= 1.0 s response budget.
        self.declare_parameter('detection_hz', 20.0)
        self.declare_parameter('deadlock_distance_m', 1.5)
        self.declare_parameter('near_threshold_m', 0.8)
        self.declare_parameter('facing_angle_threshold_rad', 0.52)
        self.declare_parameter('distance_change_threshold_m', 0.05)

        self.detection_hz = float(self.get_parameter('detection_hz').value)
        self.deadlock_distance_m = float(self.get_parameter('deadlock_distance_m').value)
        self.near_threshold_m = float(self.get_parameter('near_threshold_m').value)
        self.facing_angle_threshold_rad = float(self.get_parameter('facing_angle_threshold_rad').value)
        self.distance_change_threshold_m = float(self.get_parameter('distance_change_threshold_m').value)

        self.scene_data: dict = {}
        self.last_scene_stamp: Optional[float] = None

        self.deadlock_start_stamp: Optional[float] = None
        self.prev_opponent_distance: Optional[float] = None
        self.prev_distance_stamp: Optional[float] = None

        self.create_subscription(String, '/scene/understanding', self.scene_cb, 10)

        self.deadlock_pub = self.create_publisher(String, '/planning/deadlock_state', 10)

        dt = 1.0 / self.detection_hz if self.detection_hz > 0.0 else 0.1
        self.timer = self.create_timer(dt, self.detect_step)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def scene_cb(self, msg: String) -> None:
        try:
            self.scene_data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid JSON in scene understanding')
        self.last_scene_stamp = self.now_sec()

    def detect_step(self) -> None:
        now = self.now_sec()

        in_narrow = self.scene_data.get('in_narrow_passage',
                    self.scene_data.get('environment', {}).get('in_narrow_passage', False))
        nearest_narrow = self.scene_data.get('nearest_narrow_passage_m',
                         self.scene_data.get('environment', {}).get('nearest_narrow_passage_m', float('inf')))
        opponent = self.scene_data.get('opponent', {})
        opponent_detected = opponent.get('detected', False)
        opponent_angle = opponent.get('angle_rad', float('inf'))
        opponent_distance = opponent.get('distance_m', float('inf'))

        # Condition 1: in or near narrow passage
        cond_narrow = in_narrow or (nearest_narrow < self.near_threshold_m)

        # Condition 2: opponent detected
        cond_opponent = opponent_detected

        # Condition 3: opponent facing us (head-on)
        cond_facing = abs(opponent_angle) < self.facing_angle_threshold_rad

        # Condition 4: opponent within deadlock distance
        cond_close = opponent_distance < self.deadlock_distance_m

        # Condition 5: both vehicles appear stopped (distance not changing)
        cond_stopped = False
        if self.prev_opponent_distance is not None and self.prev_distance_stamp is not None:
            dt = now - self.prev_distance_stamp
            if dt > 0.0:
                rate = abs(opponent_distance - self.prev_opponent_distance) / dt
                cond_stopped = rate < self.distance_change_threshold_m
        else:
            cond_stopped = False

        self.prev_opponent_distance = opponent_distance
        self.prev_distance_stamp = now

        # R1 fix: previous logic required cond_stopped, so the detector only
        # fired AFTER both robots had already halted. Relaxed rule:
        #   required : opponent detected, facing us, within deadlock distance
        #   triggers : at least one of {in narrow passage, opponent stopped}
        # This catches *imminent* deadlocks without losing specificity.
        must_have = cond_opponent and cond_facing and cond_close
        trigger = cond_narrow or cond_stopped
        detected = must_have and trigger

        # Confidence: weighted sum of all conditions (must include opponent)
        confidence = (
            0.2 * float(cond_narrow) +
            0.2 * float(cond_opponent) +
            0.2 * float(cond_facing) +
            0.2 * float(cond_close) +
            0.2 * float(cond_stopped)
        )

        # Track duration
        if detected:
            if self.deadlock_start_stamp is None:
                self.deadlock_start_stamp = now
            duration = now - self.deadlock_start_stamp
        else:
            self.deadlock_start_stamp = None
            duration = 0.0

        result = {
            'detected': detected,
            'confidence': round(confidence, 3),
            'duration_sec': round(duration, 3),
            'opponent_distance_m': round(opponent_distance, 3),
        }

        msg = String()
        msg.data = json.dumps(result)
        self.deadlock_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DeadlockDetectionNode()
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
