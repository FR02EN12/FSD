#!/usr/bin/env python3
import json
import math
import os
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return float('nan')
    vals = sorted(values)
    idx = (len(vals) - 1) * max(0.0, min(100.0, pct)) / 100.0
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return vals[lo]
    frac = idx - lo
    return vals[lo] * (1.0 - frac) + vals[hi] * frac


class LaneFollowEvalNode(Node):
    def __init__(self) -> None:
        super().__init__('lane_follow_eval')

        self.declare_parameter('duration_sec', 30.0)
        self.declare_parameter('sample_hz', 20.0)
        self.declare_parameter('stale_timeout_sec', 0.5)
        self.declare_parameter('lane_status_topic', '/lane_status')
        self.declare_parameter('center_error_topic', '/lane_error_center_m')
        self.declare_parameter('heading_error_topic', '/lane_heading_error')
        self.declare_parameter('lane_width_topic', '/fused/lane_width_m')
        self.declare_parameter('safe_stop_topic', '/safe_stop')
        self.declare_parameter('decision_status_topic', '/decision_status')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_lane')
        self.declare_parameter('ok_ratio_goal', 0.90)
        self.declare_parameter('valid_error_ratio_goal', 0.90)
        self.declare_parameter('mean_abs_center_error_goal_m', 0.15)
        self.declare_parameter('moving_ratio_goal', 0.70)
        self.declare_parameter('moving_speed_threshold_mps', 0.005)
        self.declare_parameter('narrow_width_threshold_m', 0.19)
        self.declare_parameter('narrow_stop_ratio_goal', 0.90)
        self.declare_parameter('output_dir', '/home/jhp/fsd_ws/eval_logs')
        self.declare_parameter('trial_name', 'lane_follow')

        self.duration_sec = float(self.get_parameter('duration_sec').value)
        self.sample_hz = float(self.get_parameter('sample_hz').value)
        self.stale_timeout_sec = float(self.get_parameter('stale_timeout_sec').value)
        self.ok_ratio_goal = float(self.get_parameter('ok_ratio_goal').value)
        self.valid_error_ratio_goal = float(
            self.get_parameter('valid_error_ratio_goal').value)
        self.mean_abs_center_error_goal_m = float(
            self.get_parameter('mean_abs_center_error_goal_m').value)
        self.moving_ratio_goal = float(self.get_parameter('moving_ratio_goal').value)
        self.moving_speed_threshold_mps = float(
            self.get_parameter('moving_speed_threshold_mps').value)
        self.narrow_width_threshold_m = float(
            self.get_parameter('narrow_width_threshold_m').value)
        self.narrow_stop_ratio_goal = float(
            self.get_parameter('narrow_stop_ratio_goal').value)
        self.output_dir = str(self.get_parameter('output_dir').value)
        self.trial_name = str(self.get_parameter('trial_name').value).strip() or 'lane_follow'

        self.latest_status = ''
        self.latest_status_stamp: Optional[float] = None
        self.latest_center_error: Optional[float] = None
        self.latest_center_stamp: Optional[float] = None
        self.latest_heading_error: Optional[float] = None
        self.latest_heading_stamp: Optional[float] = None
        self.latest_lane_width: Optional[float] = None
        self.latest_width_stamp: Optional[float] = None
        self.latest_safe_stop = False
        self.latest_safe_stop_stamp: Optional[float] = None
        self.latest_decision_status = ''
        self.latest_decision_stamp: Optional[float] = None
        self.latest_cmd: Optional[Twist] = None
        self.latest_cmd_stamp: Optional[float] = None

        self.sample_count = 0
        self.ok_count = 0
        self.valid_error_count = 0
        self.moving_count = 0
        self.valid_width_count = 0
        self.narrow_count = 0
        self.narrow_stop_count = 0
        self.abs_center_errors: list[float] = []
        self.abs_heading_errors: list[float] = []
        self.done = False
        self.start_sec = self.now_sec()
        self.start_wall_time = datetime.now()

        self.create_subscription(
            String,
            str(self.get_parameter('lane_status_topic').value),
            self.status_cb,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter('center_error_topic').value),
            self.center_error_cb,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter('heading_error_topic').value),
            self.heading_error_cb,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter('lane_width_topic').value),
            self.lane_width_cb,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('safe_stop_topic').value),
            self.safe_stop_cb,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('decision_status_topic').value),
            self.decision_status_cb,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('cmd_vel_topic').value),
            self.cmd_cb,
            10,
        )
        self.summary_pub = self.create_publisher(String, '/metrics/lane_follow_eval', 10)

        dt = 1.0 / self.sample_hz if self.sample_hz > 0.0 else 0.05
        self.timer = self.create_timer(dt, self.sample_step)
        self.get_logger().info(
            f'lane_follow_eval started duration={self.duration_sec:.1f}s '
            f'center_goal<={self.mean_abs_center_error_goal_m:.3f}m '
            f'ok_goal>={self.ok_ratio_goal:.2f} '
            f'narrow_width<={self.narrow_width_threshold_m:.3f}m '
            f'output_dir={self.output_dir}'
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def fresh(self, stamp: Optional[float]) -> bool:
        return stamp is not None and (self.now_sec() - stamp) <= self.stale_timeout_sec

    def status_cb(self, msg: String) -> None:
        self.latest_status = msg.data.strip().lower()
        self.latest_status_stamp = self.now_sec()

    def center_error_cb(self, msg: Float32) -> None:
        self.latest_center_error = float(msg.data)
        self.latest_center_stamp = self.now_sec()

    def heading_error_cb(self, msg: Float32) -> None:
        self.latest_heading_error = float(msg.data)
        self.latest_heading_stamp = self.now_sec()

    def lane_width_cb(self, msg: Float32) -> None:
        self.latest_lane_width = float(msg.data)
        self.latest_width_stamp = self.now_sec()

    def safe_stop_cb(self, msg: Bool) -> None:
        self.latest_safe_stop = bool(msg.data)
        self.latest_safe_stop_stamp = self.now_sec()

    def decision_status_cb(self, msg: String) -> None:
        self.latest_decision_status = msg.data.strip().lower()
        self.latest_decision_stamp = self.now_sec()

    def cmd_cb(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.latest_cmd_stamp = self.now_sec()

    def stopped_now(self) -> bool:
        if self.fresh(self.latest_safe_stop_stamp) and self.latest_safe_stop:
            return True
        if (
            self.fresh(self.latest_decision_stamp)
            and self.latest_decision_status.startswith('blocked')
        ):
            return True
        if self.fresh(self.latest_cmd_stamp) and self.latest_cmd is not None:
            return abs(float(self.latest_cmd.linear.x)) < self.moving_speed_threshold_mps
        return False

    def sample_step(self) -> None:
        if self.done:
            return

        now = self.now_sec()
        self.sample_count += 1

        if self.fresh(self.latest_status_stamp) and self.latest_status == 'ok':
            self.ok_count += 1

        if (
            self.fresh(self.latest_center_stamp)
            and self.latest_center_error is not None
            and math.isfinite(self.latest_center_error)
        ):
            self.valid_error_count += 1
            self.abs_center_errors.append(abs(self.latest_center_error))

        if (
            self.fresh(self.latest_heading_stamp)
            and self.latest_heading_error is not None
            and math.isfinite(self.latest_heading_error)
        ):
            self.abs_heading_errors.append(abs(self.latest_heading_error))

        if self.fresh(self.latest_cmd_stamp) and self.latest_cmd is not None:
            if abs(float(self.latest_cmd.linear.x)) >= self.moving_speed_threshold_mps:
                self.moving_count += 1

        if (
            self.fresh(self.latest_width_stamp)
            and self.latest_lane_width is not None
            and math.isfinite(self.latest_lane_width)
            and self.latest_lane_width > 0.0
        ):
            self.valid_width_count += 1
            if self.latest_lane_width <= self.narrow_width_threshold_m:
                self.narrow_count += 1
                if self.stopped_now():
                    self.narrow_stop_count += 1

        if (now - self.start_sec) >= self.duration_sec:
            self.publish_summary(now - self.start_sec)
            self.done = True

    def publish_summary(self, elapsed: float) -> None:
        n = max(1, self.sample_count)
        ok_ratio = self.ok_count / n
        valid_error_ratio = self.valid_error_count / n
        moving_ratio = self.moving_count / n
        valid_width_ratio = self.valid_width_count / n
        narrow_stop_ratio = (
            self.narrow_stop_count / self.narrow_count
            if self.narrow_count > 0 else None
        )
        narrow_stop_ok = (
            narrow_stop_ratio >= self.narrow_stop_ratio_goal
            if narrow_stop_ratio is not None else None
        )

        mean_abs_center = (
            sum(self.abs_center_errors) / len(self.abs_center_errors)
            if self.abs_center_errors else float('nan')
        )
        rmse_center = (
            math.sqrt(sum(v * v for v in self.abs_center_errors) / len(self.abs_center_errors))
            if self.abs_center_errors else float('nan')
        )
        mean_abs_heading = (
            sum(self.abs_heading_errors) / len(self.abs_heading_errors)
            if self.abs_heading_errors else float('nan')
        )
        success = (
            ok_ratio >= self.ok_ratio_goal
            and valid_error_ratio >= self.valid_error_ratio_goal
            and math.isfinite(mean_abs_center)
            and mean_abs_center <= self.mean_abs_center_error_goal_m
            and moving_ratio >= self.moving_ratio_goal
            and (narrow_stop_ok is not False)
        )

        summary = {
            'success': success,
            'trial_name': self.trial_name,
            'started_at': self.start_wall_time.isoformat(timespec='seconds'),
            'finished_at': datetime.now().isoformat(timespec='seconds'),
            'duration_sec': round(elapsed, 3),
            'samples': self.sample_count,
            'ok_ratio': round(ok_ratio, 4),
            'valid_error_ratio': round(valid_error_ratio, 4),
            'valid_width_ratio': round(valid_width_ratio, 4),
            'moving_ratio': round(moving_ratio, 4),
            'narrow_width_threshold_m': round(self.narrow_width_threshold_m, 4),
            'narrow_samples': self.narrow_count,
            'narrow_stop_samples': self.narrow_stop_count,
            'narrow_stop_ratio': round(narrow_stop_ratio, 4)
            if narrow_stop_ratio is not None else None,
            'narrow_stop_ok': narrow_stop_ok,
            'mean_abs_center_error_m': round(mean_abs_center, 4) if math.isfinite(mean_abs_center) else None,
            'rmse_center_error_m': round(rmse_center, 4) if math.isfinite(rmse_center) else None,
            'p95_abs_center_error_m': round(percentile(self.abs_center_errors, 95.0), 4)
            if self.abs_center_errors else None,
            'max_abs_center_error_m': round(max(self.abs_center_errors), 4)
            if self.abs_center_errors else None,
            'mean_abs_heading_error_rad': round(mean_abs_heading, 4)
            if math.isfinite(mean_abs_heading) else None,
            'goals': {
                'ok_ratio': self.ok_ratio_goal,
                'valid_error_ratio': self.valid_error_ratio_goal,
                'mean_abs_center_error_m': self.mean_abs_center_error_goal_m,
                'moving_ratio': self.moving_ratio_goal,
                'narrow_stop_ratio': self.narrow_stop_ratio_goal,
            },
        }
        saved = self.save_summary(summary)
        summary['saved_json'] = saved.get('json')
        summary['saved_jsonl'] = saved.get('jsonl')
        msg = String()
        msg.data = json.dumps(summary, ensure_ascii=False)
        self.summary_pub.publish(msg)
        self.get_logger().info(f'lane_follow_eval summary: {msg.data}')

    def save_summary(self, summary: dict) -> dict:
        output_dir = Path(os.path.expanduser(self.output_dir))
        output_dir.mkdir(parents=True, exist_ok=True)

        stamp = self.start_wall_time.strftime('%Y%m%d_%H%M%S')
        safe_trial = ''.join(
            ch if ch.isalnum() or ch in ('-', '_') else '_'
            for ch in self.trial_name
        )
        json_path = output_dir / f'{safe_trial}_{stamp}.json'
        jsonl_path = output_dir / 'lane_follow_eval.jsonl'

        with json_path.open('w', encoding='utf-8') as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
            f.write('\n')

        with jsonl_path.open('a', encoding='utf-8') as f:
            f.write(json.dumps(summary, ensure_ascii=False))
            f.write('\n')

        return {
            'json': str(json_path),
            'jsonl': str(jsonl_path),
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneFollowEvalNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
