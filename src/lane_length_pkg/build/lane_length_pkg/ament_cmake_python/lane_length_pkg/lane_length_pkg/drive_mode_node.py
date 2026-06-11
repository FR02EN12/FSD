#!/usr/bin/env python3
from typing import Optional

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DrivingModeManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('drive_mode_node')

        self.declare_parameter('state_publish_hz', 20.0)
        self.declare_parameter('deadlock_confirm_sec', 0.30)
        self.declare_parameter('wait_timeout_sec', 3.0)
        self.declare_parameter('yield_accept_wait_sec', 15.0)
        self.declare_parameter('approach_distance_m', 1.0)
        self.declare_parameter('min_dwell_sec', 0.5)
        self.declare_parameter('blocked_reverse_delay_sec', 3.0)
        self.declare_parameter('yield_wait_clear_sec', 3.0)
        self.declare_parameter('led_negotiation_mask', '11111')
        self.declare_parameter('led_negotiation_min_bbox_area_px', 1200)
        self.declare_parameter('led_negotiation_timeout_sec', 1.0)
        self.declare_parameter('led_negotiation_hold_sec', 3.0)
        self.declare_parameter('negotiation_hold_sec', 25.0)
        self.declare_parameter('right_offset_pass_min_sec', 1.5)
        self.declare_parameter('right_offset_pass_timeout_sec', 8.0)

        self.state_publish_hz = float(self.get_parameter('state_publish_hz').value)
        self.deadlock_confirm_sec = float(self.get_parameter('deadlock_confirm_sec').value)
        self.wait_timeout_sec = float(self.get_parameter('wait_timeout_sec').value)
        self.yield_accept_wait_sec = float(self.get_parameter('yield_accept_wait_sec').value)
        self.approach_distance_m = float(self.get_parameter('approach_distance_m').value)
        self.min_dwell_sec = float(self.get_parameter('min_dwell_sec').value)
        self.blocked_reverse_delay_sec = float(
            self.get_parameter('blocked_reverse_delay_sec').value)
        self.yield_wait_clear_sec = float(self.get_parameter('yield_wait_clear_sec').value)
        self.led_negotiation_mask = str(
            self.get_parameter('led_negotiation_mask').value).strip()
        self.led_negotiation_min_bbox_area_px = int(
            self.get_parameter('led_negotiation_min_bbox_area_px').value)
        self.led_negotiation_timeout_sec = float(
            self.get_parameter('led_negotiation_timeout_sec').value)
        self.led_negotiation_hold_sec = float(
            self.get_parameter('led_negotiation_hold_sec').value)
        self.negotiation_hold_sec = float(self.get_parameter('negotiation_hold_sec').value)
        self.right_offset_pass_min_sec = float(
            self.get_parameter('right_offset_pass_min_sec').value)
        self.right_offset_pass_timeout_sec = float(
            self.get_parameter('right_offset_pass_timeout_sec').value)

        self.current_state: str = 'LANE_FOLLOW'
        self.state_enter_stamp: float = self.now_sec()

        self.scene_data: dict = {}
        self.last_scene_stamp: Optional[float] = None
        self.safety_event: Optional[str] = None
        self.cooperation_decision: Optional[str] = None

        self.deadlock_detected: bool = False
        self.deadlock_detect_stamp: Optional[float] = None
        self.wait_enter_stamp: Optional[float] = None
        self.pass_wait_enter_stamp: Optional[float] = None
        self.pass_clear_candidate_stamp: Optional[float] = None
        self.path_goal_reached_mode: Optional[str] = None
        self.width_blocked_stamp: Optional[float] = None
        self.width_blocked_active: bool = False
        self.v2v_sign: dict = {}
        self.last_v2v_stamp: Optional[float] = None
        self.led_negotiation_latch_until: Optional[float] = None
        self.negotiation_hold_until: Optional[float] = None

        self.create_subscription(String, '/scene/understanding', self.scene_cb, 10)
        self.create_subscription(String, '/safety/events', self.safety_cb, 10)
        self.create_subscription(String, '/planning/cooperation_decision', self.coop_cb, 10)
        self.create_subscription(String, '/planning/deadlock_state', self.deadlock_cb, 10)
        self.create_subscription(String, '/planning/path_goal_reached', self.path_goal_cb, 10)
        self.create_subscription(String, '/decision_status', self.decision_status_cb, 10)
        self.create_subscription(String, '/perception/opponent_v2v_sign', self.v2v_cb, 10)

        self.mode_pub = self.create_publisher(String, '/planning/driving_mode', 10)

        dt = 1.0 / self.state_publish_hz if self.state_publish_hz > 0.0 else 0.1
        self.timer = self.create_timer(dt, self.step)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def scene_cb(self, msg: String) -> None:
        try:
            self.scene_data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid JSON in scene understanding')
        self.last_scene_stamp = self.now_sec()

    def coop_cb(self, msg: String) -> None:
        self.cooperation_decision = msg.data.strip()

    def deadlock_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self.deadlock_detected = data.get('detected', False)
        except json.JSONDecodeError:
            pass

    def v2v_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self.v2v_sign = data if isinstance(data, dict) else {}
        except json.JSONDecodeError:
            self.v2v_sign = {'detected': False, 'mask': msg.data.strip()}
        self.last_v2v_stamp = self.now_sec()

    def path_goal_cb(self, msg: String) -> None:
        self.path_goal_reached_mode = msg.data.strip()

    def decision_status_cb(self, msg: String) -> None:
        status = msg.data.strip()
        now = self.now_sec()
        blocked = status.startswith('blocked width=')
        if blocked:
            if self.width_blocked_stamp is None:
                self.width_blocked_stamp = now
            self.width_blocked_active = True
        else:
            self.width_blocked_stamp = None
            self.width_blocked_active = False

    def safety_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self.safety_event = data.get('type', None)
        except json.JSONDecodeError:
            self.safety_event = msg.data.strip()

    def dwell_ok(self) -> bool:
        return (self.now_sec() - self.state_enter_stamp) >= self.min_dwell_sec

    def transition(self, new_state: str) -> None:
        if new_state != self.current_state:
            quiet_states = {'NEGOTIATION', 'YIELD_REVERSE'}
            if self.current_state not in quiet_states and new_state not in quiet_states:
                self.get_logger().info(f'State transition: {self.current_state} -> {new_state}')
            self.current_state = new_state
            self.state_enter_stamp = self.now_sec()
            if new_state == 'NEGOTIATION':
                self.negotiation_hold_until = self.state_enter_stamp + self.negotiation_hold_sec
            else:
                self.negotiation_hold_until = None
            if new_state != 'WAIT_FOR_PASS':
                self.pass_clear_candidate_stamp = None

    def clear_opponent_led_state(self) -> None:
        self.v2v_sign = {}
        self.last_v2v_stamp = None
        self.led_negotiation_latch_until = None

    def led_negotiation_active(self, opponent: dict) -> bool:
        active = self.scene_led_negotiation_active(opponent) or self.direct_led_negotiation_active()
        now = self.now_sec()
        if active:
            self.led_negotiation_latch_until = now + self.led_negotiation_hold_sec
            return True
        return (
            self.led_negotiation_latch_until is not None and
            now < self.led_negotiation_latch_until
        )

    def scene_led_negotiation_active(self, opponent: dict) -> bool:
        if not bool(opponent.get('detected', False)):
            return False
        try:
            bbox_area_px = int(float(opponent.get('bbox_area_px', 0)))
        except (TypeError, ValueError):
            bbox_area_px = 0
        return (
            str(opponent.get('led_mask', '')).strip() == self.led_negotiation_mask and
            bbox_area_px >= self.led_negotiation_min_bbox_area_px
        )

    def direct_led_negotiation_active(self) -> bool:
        if self.last_v2v_stamp is None:
            return False
        if (self.now_sec() - self.last_v2v_stamp) > self.led_negotiation_timeout_sec:
            return False

        detected = bool(self.v2v_sign.get('detected', False))
        box_detected = bool(self.v2v_sign.get('box_detected', False))
        try:
            bbox_area_px = int(float(self.v2v_sign.get('bbox_area_px', 0)))
        except (TypeError, ValueError):
            bbox_area_px = 0

        return (
            detected and
            box_detected and
            str(self.v2v_sign.get('mask', '')).strip() == self.led_negotiation_mask and
            bbox_area_px >= self.led_negotiation_min_bbox_area_px
        )

    def step(self) -> None:
        now = self.now_sec()

        if self.safety_event == 'EMERGENCY':
            self.transition('EMERGENCY_STOP')
            self.safety_event = None
            self._publish()
            return

        if self.current_state == 'EMERGENCY_STOP':
            if self.safety_event == 'CLEAR':
                self.safety_event = None
                self.transition('LANE_FOLLOW')
            self._publish()
            return

        ignore_opponent_led = self.current_state in (
            'WAIT_FOR_PASS',
            'RIGHT_OFFSET_PASS',
            'YIELD_REVERSE',
            'YIELD_SIDE',
            'YIELD_WAIT_CLEAR',
        )
        opponent = {} if ignore_opponent_led else self.scene_data.get('opponent', {})
        opponent_detected = bool(opponent.get('detected', False))
        led_negotiation_active = (
            False if ignore_opponent_led else self.led_negotiation_active(opponent)
        )

        if led_negotiation_active and self.current_state in (
            'LANE_FOLLOW',
            'APPROACH_NARROW',
            'DEADLOCK_CHECK',
            'WAIT',
        ):
            self.transition('NEGOTIATION')
            self._publish()
            return

        if not self.dwell_ok():
            self._publish()
            return

        in_narrow = bool(self.scene_data.get('in_narrow_passage', False))
        nearest_narrow = float(self.scene_data.get('nearest_narrow_passage_m', float('inf')))
        nearest_narrow_known = nearest_narrow >= 0.0
        passed_narrow = bool(self.scene_data.get('passed_narrow_passage', False))
        back_on_lane = bool(self.scene_data.get('back_on_lane', False))
        width_blocked_timed_out = (
            self.width_blocked_active and
            self.width_blocked_stamp is not None and
            (now - self.width_blocked_stamp) >= self.blocked_reverse_delay_sec
        )

        if led_negotiation_active and self.current_state in (
            'LANE_FOLLOW',
            'APPROACH_NARROW',
            'DEADLOCK_CHECK',
            'WAIT',
        ):
            self.transition('NEGOTIATION')

        if self.current_state == 'LANE_FOLLOW':
            if width_blocked_timed_out:
                self.width_blocked_stamp = None
                self.width_blocked_active = False
                self.transition('YIELD_REVERSE')
            elif in_narrow or (nearest_narrow_known and nearest_narrow < self.approach_distance_m):
                self.transition('APPROACH_NARROW')

        elif self.current_state == 'APPROACH_NARROW':
            if width_blocked_timed_out:
                self.width_blocked_stamp = None
                self.width_blocked_active = False
                self.transition('YIELD_REVERSE')
            elif in_narrow and opponent_detected:
                self.transition('DEADLOCK_CHECK')
                self.deadlock_detect_stamp = now
            elif passed_narrow:
                self.transition('LANE_FOLLOW')

        elif self.current_state == 'DEADLOCK_CHECK':
            if not opponent_detected:
                self.transition('LANE_FOLLOW')
            elif self.deadlock_detect_stamp is not None and \
                    (now - self.deadlock_detect_stamp) >= self.deadlock_confirm_sec:
                self.transition('NEGOTIATION')

        elif self.current_state == 'NEGOTIATION':
            negotiation_hold_active = (
                self.negotiation_hold_until is not None and
                now < self.negotiation_hold_until
            )
            if self.cooperation_decision in ('I_YIELD', 'I_YIELD_REVERSE'):
                self.clear_opponent_led_state()
                self.transition('YIELD_REVERSE')
            elif self.cooperation_decision == 'I_YIELD_SIDE':
                self.clear_opponent_led_state()
                self.transition('YIELD_SIDE')
            elif self.cooperation_decision == 'WAIT_RECHECK':
                pass
            elif self.cooperation_decision == 'I_GO':
                self.clear_opponent_led_state()
                self.transition('WAIT_FOR_PASS')
                self.pass_wait_enter_stamp = now
                self.pass_clear_candidate_stamp = None
            elif not negotiation_hold_active and not opponent_detected and not led_negotiation_active and not self.deadlock_detected:
                self.transition('LANE_FOLLOW')

        elif self.current_state == 'YIELD_REVERSE':
            if self.path_goal_reached_mode == 'YIELD_REVERSE':
                self.path_goal_reached_mode = None
                self.transition('YIELD_WAIT_CLEAR')

        elif self.current_state == 'YIELD_SIDE':
            if self.path_goal_reached_mode == 'YIELD_SIDE':
                self.path_goal_reached_mode = None
                self.transition('YIELD_WAIT_CLEAR')

        elif self.current_state == 'YIELD_WAIT_CLEAR':
            if (now - self.state_enter_stamp) >= self.yield_wait_clear_sec:
                self.transition('LANE_FOLLOW')

        elif self.current_state == 'WAIT_FOR_PASS':
            if self.pass_wait_enter_stamp is None:
                self.pass_wait_enter_stamp = now

            if (now - self.pass_wait_enter_stamp) >= self.yield_accept_wait_sec:
                self.pass_wait_enter_stamp = None
                self.pass_clear_candidate_stamp = None
                self.transition('RIGHT_OFFSET_PASS')

        elif self.current_state == 'RIGHT_OFFSET_PASS':
            elapsed = now - self.state_enter_stamp
            pass_min_elapsed = elapsed >= self.right_offset_pass_min_sec
            pass_timeout = elapsed >= self.right_offset_pass_timeout_sec
            if (
                passed_narrow or
                (pass_min_elapsed and not in_narrow and back_on_lane) or
                pass_timeout
            ):
                self.transition('LANE_FOLLOW')

        elif self.current_state == 'WAIT':
            if self.wait_enter_stamp is not None and \
                    (now - self.wait_enter_stamp) >= self.wait_timeout_sec:
                self.transition('NEGOTIATION')

        elif self.current_state == 'REENTER':
            if self.path_goal_reached_mode == 'REENTER' or back_on_lane:
                self.path_goal_reached_mode = None
                self.transition('LANE_FOLLOW')

        self._publish()

    def _publish(self) -> None:
        msg = String()
        msg.data = self.current_state
        self.mode_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DrivingModeManagerNode()
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
