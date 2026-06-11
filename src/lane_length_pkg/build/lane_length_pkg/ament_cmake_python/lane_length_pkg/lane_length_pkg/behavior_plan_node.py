#!/usr/bin/env python3
from typing import Optional

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BehaviorPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('behavior_plan_node')

        self.declare_parameter('nominal_speed', 0.028)
        self.declare_parameter('slow_speed', 0.018)
        self.declare_parameter('reverse_speed', 0.020)
        self.declare_parameter('yield_reverse_extra_back_m', 0.0)
        self.declare_parameter('yield_reverse_extra_lateral_m', 0.10)
        self.declare_parameter('publish_hz', 10.0)

        self.nominal_speed = float(self.get_parameter('nominal_speed').value)
        self.slow_speed = float(self.get_parameter('slow_speed').value)
        self.reverse_speed = float(self.get_parameter('reverse_speed').value)
        self.yield_reverse_extra_back_m = max(
            0.0, float(self.get_parameter('yield_reverse_extra_back_m').value))
        self.yield_reverse_extra_lateral_m = max(
            0.0, float(self.get_parameter('yield_reverse_extra_lateral_m').value))
        self.publish_hz = float(self.get_parameter('publish_hz').value)

        self.driving_mode: str = 'LANE_FOLLOW'
        self.scene_data: dict = {}
        self.last_scene_stamp: Optional[float] = None
        self.latched_reverse_goal: Optional[dict] = None

        self.create_subscription(String, '/planning/driving_mode', self.mode_cb, 10)
        self.create_subscription(String, '/scene/understanding', self.scene_cb, 10)

        self.goal_pub = self.create_publisher(String, '/planning/behavior_goal', 10)

        dt = 1.0 / self.publish_hz if self.publish_hz > 0.0 else 0.1
        self.timer = self.create_timer(dt, self.plan_step)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def mode_cb(self, msg: String) -> None:
        mode = msg.data.strip()
        if mode != self.driving_mode and mode != 'YIELD_REVERSE':
            self.latched_reverse_goal = None
        if mode != self.driving_mode and self.driving_mode != 'YIELD_REVERSE':
            self.latched_reverse_goal = None
        self.driving_mode = mode

    def scene_cb(self, msg: String) -> None:
        try:
            self.scene_data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid JSON in scene understanding')
        self.last_scene_stamp = self.now_sec()

    def finite_float(self, data: dict, key: str) -> Optional[float]:
        try:
            value = float(data.get(key))
        except (TypeError, ValueError):
            return None
        return value if math.isfinite(value) else None

    @staticmethod
    def offset_pose_left(
        x: float, y: float, theta: float, offset: float
    ) -> tuple[float, float]:
        return (
            x - offset * math.sin(theta),
            y + offset * math.cos(theta),
        )

    def class_a_left_reverse_goal(self) -> dict:
        class_a_pose = self.scene_data.get('last_class_a_pose')
        if not isinstance(class_a_pose, dict):
            return self.stop_goal('reverse_wait_class_a_pose')

        target_x = self.finite_float(class_a_pose, 'x')
        target_y = self.finite_float(class_a_pose, 'y')
        target_theta = self.finite_float(class_a_pose, 'theta')
        if target_x is None or target_y is None or target_theta is None:
            return self.stop_goal('reverse_wait_class_a_pose')

        lateral_offset = self.yield_reverse_extra_lateral_m
        target_x, target_y = self.offset_pose_left(
            target_x, target_y, target_theta, lateral_offset
        )
        if self.yield_reverse_extra_back_m > 0.0:
            target_x -= self.yield_reverse_extra_back_m * math.cos(target_theta)
            target_y -= self.yield_reverse_extra_back_m * math.sin(target_theta)

        return {
            'type': 'reverse_to_pullover',
            'target_x': target_x,
            'target_y': target_y,
            'target_theta': target_theta,
            'speed_limit': self.reverse_speed,
            'reverse': True,
            'signed_lateral_offset_m': -lateral_offset,
            'lateral_offset_m': lateral_offset,
            'offset_side': 'left',
            'extra_back_m': self.yield_reverse_extra_back_m,
            'extra_lateral_m': self.yield_reverse_extra_lateral_m,
            'fallback': False,
            'width_policy': 'class_a_center_left_fixed_offset',
        }

    def stop_goal(self, goal_type: str = 'stop') -> dict:
        return {
            'type': goal_type,
            'target_x': 0.0,
            'target_y': 0.0,
            'target_theta': 0.0,
            'speed_limit': 0.0,
            'reverse': False,
        }

    def plan_step(self) -> None:
        mode = self.driving_mode

        if mode == 'LANE_FOLLOW':
            goal = {
                'type': 'lane_follow',
                'target_x': 0.0,
                'target_y': 0.0,
                'target_theta': 0.0,
                'speed_limit': self.nominal_speed,
                'reverse': False,
            }
        elif mode == 'APPROACH_NARROW':
            goal = {
                'type': 'lane_follow',
                'target_x': 0.0,
                'target_y': 0.0,
                'target_theta': 0.0,
                'speed_limit': self.slow_speed,
                'reverse': False,
            }
        elif mode in ('DEADLOCK_CHECK', 'NEGOTIATION', 'WAIT'):
            goal = self.stop_goal('stop')
        elif mode == 'YIELD_REVERSE':
            if (
                self.latched_reverse_goal is None or
                self.latched_reverse_goal.get('type') == 'reverse_wait_class_a_pose'
            ):
                self.latched_reverse_goal = self.class_a_left_reverse_goal()
            goal = self.latched_reverse_goal
        elif mode == 'YIELD_SIDE':
            side_target = (self.scene_data.get('side_pull_over') or
                           self.scene_data.get('yield_side_target') or {})
            if side_target:
                goal = {
                    'type': 'side_pull_over',
                    'target_x': side_target.get('x', 0.0),
                    'target_y': side_target.get('y', 0.0),
                    'target_theta': side_target.get('theta', 0.0),
                    'speed_limit': self.slow_speed,
                    'reverse': False,
                }
            else:
                goal = self.stop_goal('yield_wait_for_route')
        elif mode == 'WAIT_FOR_PASS':
            goal = self.stop_goal('wait_for_pass')
        elif mode == 'YIELD_WAIT_CLEAR':
            goal = self.stop_goal('yield_wait_clear')
        elif mode == 'REENTER':
            reentry = self.scene_data.get('lane_reentry_point') or self.scene_data.get('pose', {})
            goal = {
                'type': 'reenter_lane',
                'target_x': reentry.get('x', 0.0),
                'target_y': reentry.get('y', 0.0),
                'target_theta': reentry.get('theta', 0.0),
                'speed_limit': self.slow_speed,
                'reverse': False,
            }
        elif mode == 'EMERGENCY_STOP':
            goal = self.stop_goal('emergency_stop')
        else:
            self.get_logger().warn(f'Unknown driving mode: {mode}')
            goal = self.stop_goal('stop')

        msg = String()
        msg.data = json.dumps(goal)
        self.goal_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BehaviorPlannerNode()
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
