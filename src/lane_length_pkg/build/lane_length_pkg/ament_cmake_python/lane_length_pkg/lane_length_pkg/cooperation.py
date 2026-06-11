#!/usr/bin/env python3
import json
import math
import sys
import threading
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class CooperationDecisionNode(Node):
    RPS_MASK = {
        'ROCK': '11000',
        'SCISSORS': '10100',
        'PAPER': '10010',
    }
    SIGN_TO_RPS = {
        'RPS_ROCK': 'ROCK',
        'RPS_SCISSORS': 'SCISSORS',
        'RPS_PAPER': 'PAPER',
    }
    RPS_BEATS = {
        'ROCK': 'SCISSORS',
        'SCISSORS': 'PAPER',
        'PAPER': 'ROCK',
    }

    def __init__(self) -> None:
        super().__init__('cooperation')

        self.declare_parameter('decision_hz', 10.0)
        self.declare_parameter('max_negotiation_sec', 5.0)
        self.declare_parameter('timeout_default_yield', False)
        self.declare_parameter('protocol_prompt_delay_sec', 1.5)
        self.declare_parameter('decision_hold_sec', 2.0)
        self.declare_parameter('v2v_use_rps', True)
        self.declare_parameter('prompt_protocol_choice', True)
        self.declare_parameter('manual_rps_input', True)
        self.declare_parameter('default_rps_choice', 'ROCK')
        self.declare_parameter('v2v_advertise_sec', 0.8)
        self.declare_parameter('v2v_sign_timeout_sec', 1.0)
        self.declare_parameter('rps_countdown_sec', 3.0)
        self.declare_parameter('led_trigger_enabled', True)
        self.declare_parameter('led_trigger_mask', '11111')
        self.declare_parameter('led_trigger_min_bbox_area_px', 1200)
        self.declare_parameter('road_class_a_min_m', 0.47)
        self.declare_parameter('road_class_b_min_m', 0.19)

        self.decision_hz = float(self.get_parameter('decision_hz').value)
        self.max_negotiation_sec = float(self.get_parameter('max_negotiation_sec').value)
        self.timeout_default_yield = bool(
            self.get_parameter('timeout_default_yield').value)
        self.protocol_prompt_delay_sec = float(
            self.get_parameter('protocol_prompt_delay_sec').value)
        self.decision_hold_sec = float(self.get_parameter('decision_hold_sec').value)
        self.v2v_use_rps = bool(self.get_parameter('v2v_use_rps').value)
        self.prompt_protocol_choice = bool(
            self.get_parameter('prompt_protocol_choice').value)
        self.manual_rps_input = bool(self.get_parameter('manual_rps_input').value)
        self.default_rps_choice = str(
            self.get_parameter('default_rps_choice').value).upper()
        if self.default_rps_choice not in self.RPS_MASK:
            self.default_rps_choice = 'ROCK'
        self.v2v_advertise_sec = float(self.get_parameter('v2v_advertise_sec').value)
        self.v2v_sign_timeout_sec = float(
            self.get_parameter('v2v_sign_timeout_sec').value)
        self.rps_countdown_sec = float(self.get_parameter('rps_countdown_sec').value)
        self.led_trigger_enabled = bool(self.get_parameter('led_trigger_enabled').value)
        self.led_trigger_mask = str(self.get_parameter('led_trigger_mask').value).strip()
        self.led_trigger_min_bbox_area_px = int(
            self.get_parameter('led_trigger_min_bbox_area_px').value)
        self.road_class_a_min_m = float(self.get_parameter('road_class_a_min_m').value)
        self.road_class_b_min_m = float(self.get_parameter('road_class_b_min_m').value)

        self.scene_data: dict = {}
        self.deadlock_state: dict = {}
        self.v2v_sign: dict = {'detected': False, 'sign': 'UNKNOWN', 'mask': '00000'}
        self.driving_mode: str = 'LANE_FOLLOW'
        self.last_scene_stamp: Optional[float] = None
        self.last_deadlock_stamp: Optional[float] = None
        self.last_v2v_stamp: Optional[float] = None

        self.negotiation_start_stamp: Optional[float] = None
        self.current_decision: str = 'WAIT_RECHECK'
        self.current_led_pattern: str = 'MASK:11111'
        self.last_yield_score_detail: dict = {}
        self.decision_stamp: Optional[float] = None
        self.protocol_start_stamp: Optional[float] = None

        self.first_deadlock_stamp: Optional[float] = None
        self.response_time_reported: bool = False

        self.active_protocol: Optional[str] = None
        self.opponent_weight_ready_seen: bool = False
        self.local_rps_choice: Optional[str] = None
        self.rps_countdown_stamp: Optional[float] = None
        self.negotiation_logged: bool = False
        self.active_trigger: Optional[str] = None
        self.negotiation_id: int = 0
        self.protocol_prompt_generation: Optional[int] = None
        self.rps_prompt_generation: Optional[int] = None
        self.negotiation_log_keys: set[str] = set()
        self.last_negotiation_snapshot: Optional[Tuple] = None

        self.create_subscription(String, '/scene/understanding', self.scene_cb, 10)
        self.create_subscription(String, '/planning/deadlock_state', self.deadlock_cb, 10)
        self.create_subscription(String, '/perception/opponent_v2v_sign', self.v2v_cb, 10)
        self.create_subscription(String, '/planning/driving_mode', self.driving_mode_cb, 10)
        self.create_subscription(String, '/planning/negotiation_input', self.negotiation_input_cb, 10)

        self.decision_pub = self.create_publisher(
            String, '/planning/cooperation_decision', 10)
        self.led_pattern_pub = self.create_publisher(
            String, '/planning/v2v_led_pattern', 10)
        self.status_pub = self.create_publisher(
            String, '/planning/v2v_status', 10)
        self.response_time_pub = self.create_publisher(
            Float32, '/metrics/deadlock_response_time_sec', 10)

        dt = 1.0 / self.decision_hz if self.decision_hz > 0.0 else 0.2
        self.timer = self.create_timer(dt, self.decide_step)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def scene_cb(self, msg: String) -> None:
        try:
            self.scene_data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid JSON in scene understanding')
        self.last_scene_stamp = self.now_sec()

    def deadlock_cb(self, msg: String) -> None:
        try:
            self.deadlock_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid JSON in deadlock state')
        self.last_deadlock_stamp = self.now_sec()

    def v2v_cb(self, msg: String) -> None:
        try:
            self.v2v_sign = json.loads(msg.data)
        except json.JSONDecodeError:
            self.v2v_sign = {'detected': False, 'sign': msg.data, 'mask': '00000'}
        if str(self.v2v_sign.get('sign', 'UNKNOWN')) == 'WEIGHT_READY':
            self.opponent_weight_ready_seen = True
        self.last_v2v_stamp = self.now_sec()

    def driving_mode_cb(self, msg: String) -> None:
        self.driving_mode = msg.data.strip()

    def negotiation_input_cb(self, msg: String) -> None:
        answer = msg.data.strip().lower()
        if self.negotiation_start_stamp is None:
            return

        self.get_logger().info(f'협상 입력 수신: {answer}')
        if self.active_protocol is None:
            protocol = self._protocol_from_answer(answer)
            if protocol is not None:
                self._set_active_protocol(protocol, 'local_input')
            else:
                self.get_logger().warn(f'알 수 없는 협상 입력 "{answer}"')
            return

        if self.active_protocol == 'rps' and self.local_rps_choice is None:
            choice = self._rps_choice_from_answer(answer)
            if choice is not None:
                self.local_rps_choice = choice
                self.get_logger().info(f'내 가위바위보 선택: {self.local_rps_choice}')
            else:
                self.get_logger().warn(f'알 수 없는 가위바위보 입력 "{answer}"')

    def decide_step(self) -> None:
        now = self.now_sec()

        reset_modes = {
            'YIELD_REVERSE',
            'YIELD_SIDE',
            'YIELD_WAIT_CLEAR',
            'WAIT_FOR_PASS',
            'RIGHT_OFFSET_PASS',
            'REENTER',
        }
        if self.driving_mode in reset_modes:
            if self.negotiation_start_stamp is not None:
                self._reset_negotiation()
            self.current_decision = 'WAIT_RECHECK'
            self.current_led_pattern = 'MASK:11111'
            self._publish(protocol='idle')
            return

        deadlock_detected = bool(self.deadlock_state.get('detected', False))
        led_triggered = self._led_trigger_active()
        mode_triggered = self.driving_mode == 'NEGOTIATION'
        opponent_present = self._opponent_present()
        negotiation_active = (
            mode_triggered or
            deadlock_detected or
            led_triggered or
            (self.negotiation_start_stamp is not None and opponent_present)
        )

        if not negotiation_active:
            self._reset_negotiation()
            self.current_decision = 'WAIT_RECHECK'
            self.current_led_pattern = 'MASK:11111'
            self._publish(protocol='idle')
            return

        if self.first_deadlock_stamp is None:
            self.first_deadlock_stamp = now
            self.response_time_reported = False

        if self.negotiation_start_stamp is None:
            self.negotiation_id += 1
            self.negotiation_log_keys = set()
            self.last_negotiation_snapshot = None
            self.negotiation_start_stamp = now
            self.decision_stamp = None
            self.protocol_start_stamp = None
            self.active_protocol = None
            if mode_triggered:
                self.active_trigger = 'driving_mode'
            elif led_triggered:
                self.active_trigger = 'led_11111'
            else:
                self.active_trigger = 'deadlock'
            self.opponent_weight_ready_seen = False
            self.local_rps_choice = None
            self.rps_countdown_stamp = now
            self.negotiation_logged = False
            self._log_negotiation_entry()

        if self.decision_stamp is not None:
            self._publish(protocol='held')
            return

        elapsed = now - self.negotiation_start_stamp
        if elapsed >= self.max_negotiation_sec and self.timeout_default_yield:
            if self.decision_stamp is None:
                self.current_decision = 'I_YIELD'
                self.current_led_pattern = 'MASK:11111'
                self.decision_stamp = now
                self.get_logger().info('Negotiation timeout, defaulting to I_YIELD')
            self._publish(protocol='timeout_wait')
            return
        if elapsed >= self.max_negotiation_sec:
            self._log_once(
                'timeout_continue',
                '협상 timeout 지났지만 자동양보 비활성화: 계속 협상 진행')

        if self.active_protocol is None:
            if self._opponent_weight_ready():
                self._set_active_protocol('weight', 'opponent_weight_ready_01010')
            elif elapsed < self.protocol_prompt_delay_sec:
                self._publish(protocol='protocol_prompt_delay')
                return
            else:
                self._start_protocol_selection(self.negotiation_id)
                self._publish(protocol='protocol_prompt')
                return

        if self.active_protocol == 'rps':
            decision, status = self._decide_with_rps(now)
            self.current_decision = decision
            if decision in ('I_GO', 'I_YIELD'):
                self.decision_stamp = now
            self._publish(protocol=status)
            return

        decision, status = self._decide_with_weight()
        self.current_decision = decision
        if decision in ('I_GO', 'I_YIELD'):
            self.decision_stamp = now
        self._publish(protocol=status)

    def _reset_negotiation(self) -> None:
        self.negotiation_start_stamp = None
        self.decision_stamp = None
        self.protocol_start_stamp = None
        self.first_deadlock_stamp = None
        self.response_time_reported = False
        self.active_protocol = None
        self.active_trigger = None
        self.protocol_prompt_generation = None
        self.rps_prompt_generation = None
        self.negotiation_log_keys = set()
        self.last_negotiation_snapshot = None
        self.opponent_weight_ready_seen = False
        self.local_rps_choice = None
        self.rps_countdown_stamp = None
        self.negotiation_logged = False
        self.last_yield_score_detail = {}

    def _log_negotiation_entry(self) -> None:
        if self.negotiation_logged:
            return

        bbox = self.v2v_sign.get('bbox', [])
        bbox_area = self.v2v_sign.get('bbox_area_px', 0)
        led_mask = str(self.v2v_sign.get('mask', '')).strip()
        self.get_logger().info(
            f'협상 모드 진입: trigger={self.active_trigger}, '
            f'led_mask={led_mask}, bbox_area={bbox_area}, bbox={bbox}')
        self.negotiation_logged = True

    def _log_once(self, key: str, message: str) -> None:
        if key in self.negotiation_log_keys:
            return
        self.negotiation_log_keys.add(key)
        self.get_logger().info(message)

    def _set_active_protocol(self, protocol: str, reason: str) -> None:
        previous = self.active_protocol or '-'
        if self.active_protocol == protocol:
            return

        self.active_protocol = protocol
        self.protocol_start_stamp = self.now_sec()
        self.current_decision = 'WAIT_RECHECK'

        if protocol == 'weight':
            self.local_rps_choice = None
            self.rps_prompt_generation = None
            self.opponent_weight_ready_seen = self._opponent_weight_ready()

        self.get_logger().info(
            f'협상 방식 선택: {protocol} '
            f'(이전={previous}, 이유={reason})')

    def _start_protocol_selection(self, generation: int) -> None:
        if not self.prompt_protocol_choice:
            self._set_active_protocol(self._default_protocol(), 'default_parameter')
            return
        if self.protocol_prompt_generation == generation:
            return

        self.protocol_prompt_generation = generation
        self.get_logger().info(
            '[협상 질문] 가위바위보로 결정할까요? '
            'y=가위바위보 / n=양보점수 '
            '(토픽 입력: /planning/negotiation_input)')
        threading.Thread(
            target=self._select_protocol_worker,
            args=(generation,),
            daemon=True,
        ).start()

    def _select_protocol_worker(self, generation: int) -> None:
        protocol = self._select_protocol()
        if (
            generation == self.negotiation_id
            and self.negotiation_start_stamp is not None
            and self.active_protocol is None
        ):
            self._set_active_protocol(protocol, 'console_input')

    def _default_protocol(self) -> str:
        return 'rps' if self.v2v_use_rps else 'weight'

    def _read_console_line(self, prompt: str) -> Optional[str]:
        prompt_text = prompt
        try:
            with open('/dev/tty', 'r+', encoding='utf-8') as tty:
                tty.write(f'{prompt_text}\n입력: ')
                tty.flush()
                line = tty.readline()
        except OSError:
            try:
                print(f'{prompt_text}\n입력: ', end='', flush=True)
                line = sys.stdin.readline()
            except (EOFError, OSError):
                return None

        if line == '':
            return None
        return line.strip().lower()

    def _protocol_from_answer(self, answer: str) -> Optional[str]:
        if answer in ('y', 'yes', '1', 'rps', 'rock', '가위바위보'):
            return 'rps'
        if answer in ('n', 'no', '0', 'weight', 'score', '점수', '양보점수'):
            return 'weight'
        return None

    def _select_protocol(self) -> str:
        default = self._default_protocol()
        if not self.prompt_protocol_choice:
            return default

        answer = self._read_console_line(
            '[협상 모드] 가위바위보로 결정할까요? '
            'y=가위바위보 / n=양보점수: '
        )
        if answer is None:
            self.get_logger().warn(
                f'협상 프로토콜 입력을 받을 수 없어 기본값 사용: {default}')
            return default

        self.get_logger().info(f'협상 콘솔 입력 수신: {answer}')
        protocol = self._protocol_from_answer(answer)
        if protocol is not None:
            return protocol
        self.get_logger().warn(f'알 수 없는 협상 입력 "{answer}", 기본값 사용: {default}')
        return default

    def _start_rps_choice_selection(self, generation: int) -> None:
        if not self.manual_rps_input:
            self.local_rps_choice = self.default_rps_choice
            self.get_logger().info(f'내 가위바위보 선택: {self.local_rps_choice}')
            return
        if self.rps_prompt_generation == generation:
            return

        self.rps_prompt_generation = generation
        self.get_logger().info(
            '[가위바위보 질문] rock/r, scissors/s, paper/p 입력 '
            '(토픽 입력: /planning/negotiation_input)')
        threading.Thread(
            target=self._select_rps_choice_worker,
            args=(generation,),
            daemon=True,
        ).start()

    def _select_rps_choice_worker(self, generation: int) -> None:
        choice = self._prompt_rps_choice()
        if (
            generation == self.negotiation_id
            and self.negotiation_start_stamp is not None
            and self.active_protocol == 'rps'
            and self.local_rps_choice is None
        ):
            self.local_rps_choice = choice
            self.get_logger().info(f'내 가위바위보 선택: {self.local_rps_choice}')

    def _prompt_rps_choice(self) -> str:
        if not self.manual_rps_input:
            return self.default_rps_choice

        raw = self._read_console_line('[가위바위보] rock/r, scissors/s, paper/p 입력: ')
        if raw is None:
            self.get_logger().warn(
                f'가위바위보 입력을 받을 수 없어 기본값 사용: {self.default_rps_choice}')
            return self.default_rps_choice

        choice = self._rps_choice_from_answer(raw)
        if choice is None:
            self.get_logger().warn(
                f'알 수 없는 가위바위보 입력 "{raw}", 기본값 사용: {self.default_rps_choice}')
            return self.default_rps_choice
        return choice

    def _rps_choice_from_answer(self, answer: str) -> Optional[str]:
        mapping = {
            'r': 'ROCK',
            'rock': 'ROCK',
            '바위': 'ROCK',
            's': 'SCISSORS',
            'scissor': 'SCISSORS',
            'scissors': 'SCISSORS',
            '가위': 'SCISSORS',
            'p': 'PAPER',
            'paper': 'PAPER',
            '보': 'PAPER',
        }
        return mapping.get(answer)

    def _decide_with_rps(self, now: float) -> Tuple[str, str]:
        if self.rps_countdown_stamp is None:
            self.rps_countdown_stamp = now

        elapsed = now - self.rps_countdown_stamp
        if self.local_rps_choice is None:
            self.current_led_pattern = 'MASK:10101'
            if elapsed < self.rps_countdown_sec:
                remain = max(0.0, self.rps_countdown_sec - elapsed)
                return 'WAIT_RECHECK', f'rps_countdown remain={remain:.1f}s'
            self._log_once(
                'rps_wait_local_choice',
                '가위바위보 선택 대기: rock/r, scissors/s, paper/p')
            self._start_rps_choice_selection(self.negotiation_id)
            return 'WAIT_RECHECK', 'rps_prompt'

        opponent_sign = str(self.v2v_sign.get('sign', 'UNKNOWN'))
        opponent_mask = str(self.v2v_sign.get('mask', '00000')).strip()
        if opponent_sign == 'WEIGHT_READY' or opponent_mask == '01010':
            self.get_logger().info('상대 01010(양보점수 준비) 감지: weight 방식 수락')
            self._set_active_protocol('weight', 'opponent_weight_ready_01010')
            return self._decide_with_weight()

        opponent_choice = self.SIGN_TO_RPS.get(opponent_sign)
        self.current_led_pattern = f'MASK:{self.RPS_MASK[self.local_rps_choice]}'

        if opponent_choice is None:
            self._log_once(
                f'rps_wait_opponent_{self.local_rps_choice}_{opponent_sign}_{opponent_mask}',
                f'상대 가위바위보 선택 대기: local={self.local_rps_choice}, '
                f'opponent_sign={opponent_sign}, mask={opponent_mask}')
            return 'WAIT_RECHECK', f'rps_wait local={self.local_rps_choice}'

        if opponent_choice == self.local_rps_choice:
            self.get_logger().info(
                f'가위바위보 비김: local={self.local_rps_choice}, opp={opponent_choice}; '
                f'{self.rps_countdown_sec:.1f}초 뒤 재입력')
            self.local_rps_choice = None
            self.rps_countdown_stamp = now
            self.current_led_pattern = 'MASK:10101'
            return 'WAIT_RECHECK', f'rps_tie local={opponent_choice}'

        local_wins = self.RPS_BEATS[self.local_rps_choice] == opponent_choice
        # Project rule: the RPS winner yields; the loser waits, then passes.
        if local_wins:
            return 'I_YIELD', f'rps_win_yield local={self.local_rps_choice} opp={opponent_choice}'
        return 'I_GO', f'rps_lose_go local={self.local_rps_choice} opp={opponent_choice}'

    def _decide_with_weight(self) -> Tuple[str, str]:
        score, value, detail = self._compute_yield_score()
        self.last_yield_score_detail = detail

        if self.negotiation_start_stamp is not None:
            start = self.protocol_start_stamp or self.negotiation_start_stamp
            elapsed = self.now_sec() - start
            if elapsed < self.v2v_advertise_sec:
                self.current_led_pattern = 'MASK:01010'
                return 'WAIT_RECHECK', 'weight_ready'

        self.current_led_pattern = f'MASK:{value:05b}'

        if not self._v2v_fresh():
            return 'WAIT_RECHECK', f'weight_wait_no_sign local_score={score} value={value}'

        opponent_sign = str(self.v2v_sign.get('sign', 'UNKNOWN'))
        opponent_mask = str(self.v2v_sign.get('mask', '00000')).strip()
        opponent_detected = bool(self.v2v_sign.get('detected', False))
        if not opponent_detected:
            return 'WAIT_RECHECK', f'weight_wait_no_opponent local={value:05b}'

        if opponent_sign == 'WEIGHT_READY' or opponent_mask == '01010':
            self.opponent_weight_ready_seen = True
            return 'WAIT_RECHECK', f'weight_wait_opp_ready local={value:05b}'

        if opponent_sign in ('APPROACH', 'RPS_READY') and not self.opponent_weight_ready_seen:
            return 'WAIT_RECHECK', f'weight_wait_opp_sign={opponent_sign}'

        try:
            opponent_value = int(opponent_mask, 2)
        except (TypeError, ValueError):
            return 'WAIT_RECHECK', 'weight_wait_bad_value'

        if value > opponent_value:
            return 'I_YIELD', (
                f'weight_yield local={value:05b}({value}) '
                f'opp={opponent_value:05b}({opponent_value})')
        if value < opponent_value:
            return 'I_GO', (
                f'weight_go local={value:05b}({value}) '
                f'opp={opponent_value:05b}({opponent_value})')
        return 'WAIT_RECHECK', f'weight_tie value={value:05b}({value})'

    def _v2v_fresh(self) -> bool:
        if self.last_v2v_stamp is None:
            return False
        return (self.now_sec() - self.last_v2v_stamp) <= self.v2v_sign_timeout_sec

    def _opponent_weight_ready(self) -> bool:
        if not self._v2v_fresh():
            return False
        opponent_sign = str(self.v2v_sign.get('sign', 'UNKNOWN'))
        opponent_mask = str(self.v2v_sign.get('mask', '00000')).strip()
        return opponent_sign == 'WEIGHT_READY' or opponent_mask == '01010'

    def _opponent_present(self) -> bool:
        if not self._v2v_fresh():
            return False
        detected = bool(self.v2v_sign.get('detected', False))
        box_detected = bool(self.v2v_sign.get('box_detected', False))
        mask = str(self.v2v_sign.get('mask', '00000')).strip()
        sign = str(self.v2v_sign.get('sign', 'UNKNOWN')).strip()
        return detected and box_detected and mask != '00000' and sign != 'UNKNOWN'

    def _led_trigger_active(self) -> bool:
        if not self.led_trigger_enabled or not self._v2v_fresh():
            return False

        mask = str(self.v2v_sign.get('mask', '')).strip()
        detected = bool(self.v2v_sign.get('detected', False))
        box_detected = bool(self.v2v_sign.get('box_detected', False))
        try:
            bbox_area_px = int(float(self.v2v_sign.get('bbox_area_px', 0)))
        except (TypeError, ValueError):
            bbox_area_px = 0

        return (
            detected and
            box_detected and
            mask == self.led_trigger_mask and
            bbox_area_px >= self.led_trigger_min_bbox_area_px
        )

    @staticmethod
    def _clamp01(value: float) -> float:
        return max(0.0, min(1.0, value))

    @staticmethod
    def _to_float(value, fallback: float) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return fallback

    def _range_score(self, value: float, lo: float, hi: float) -> float:
        if not math.isfinite(value) or hi <= lo:
            return 0.0
        return self._clamp01((value - lo) / (hi - lo))

    def _distance_to(self, target: dict) -> float:
        pose = self.scene_data.get('pose', {})
        if not target or not pose:
            return float('inf')
        tx = float(target.get('x', float('nan')))
        ty = float(target.get('y', float('nan')))
        px = float(pose.get('x', float('nan')))
        py = float(pose.get('y', float('nan')))
        if not all(math.isfinite(v) for v in (tx, ty, px, py)):
            return float('inf')
        return math.hypot(tx - px, ty - py)

    def _compute_yield_score(self) -> Tuple[int, int, dict]:
        environment = self.scene_data.get('environment', {})

        road_class = str(
            self.scene_data.get('road_class',
                                environment.get('road_class', 'UNKNOWN'))).upper()
        road_width = self._to_float(
            self.scene_data.get('road_width_m', environment.get('road_width_m', -1.0)),
            -1.0,
        )
        road_width_source = 'scene'
        class_a_pose = self.scene_data.get('last_class_a_pose')
        class_a_width = -1.0
        if isinstance(class_a_pose, dict):
            class_a_width = self._to_float(class_a_pose.get('width', -1.0), -1.0)
        if math.isfinite(class_a_width) and class_a_width >= self.road_class_a_min_m:
            road_class = 'A'
            road_width = class_a_width
            road_width_source = 'last_class_a_pose'

        rear = self.scene_data.get('rear_obstacle', {})
        rear_detected = bool(rear.get('detected', False))
        rear_dist = self._to_float(rear.get('distance_m', 999.0), 999.0)

        vehicle = self.scene_data.get('vehicle_state', {})
        battery_pct = self._to_float(vehicle.get('battery_pct', 80.0), 80.0)
        vehicle_fault = bool(vehicle.get('fault', False)) or not bool(vehicle.get('motion_ok', True))

        if road_class == 'A':
            road_rank = 2
            road_class_score = 1.0
        elif road_class == 'B':
            road_rank = 1
            road_class_score = 0.55
        elif road_class == 'C':
            road_rank = 0
            road_class_score = 0.10
        elif math.isfinite(road_width) and road_width > 0.0:
            if road_width >= self.road_class_a_min_m:
                road_class = 'A'
                road_rank = 2
                road_class_score = 1.0
            elif road_width > self.road_class_b_min_m:
                road_class = 'B'
                road_rank = 1
                road_class_score = 0.55
            else:
                road_class = 'C'
                road_rank = 0
                road_class_score = 0.10
        else:
            road_rank = 1
            road_class_score = 0.35

        rear_clear_rank = 0 if rear_detected else 1
        rear_clearance_score = 1.0 if rear_clear_rank else 0.0

        if vehicle_fault:
            battery_score = 0.0
        elif math.isfinite(battery_pct):
            battery_score = self._clamp01(battery_pct / 100.0)
        else:
            battery_score = 0.5
        battery_bin = max(0, min(4, int(round(battery_score * 4.0))))

        road_value_base = {
            0: 0,    # C: 0-9
            1: 11,   # B: 11-20
            2: 22,   # A: 22-31
        }
        value = road_value_base[road_rank] + (5 * rear_clear_rank) + battery_bin
        score_i = max(0, min(31, int(value)))
        score_percent = max(0, min(100, int(round(value * 100.0 / 31.0))))

        terms = {
            'road_width_class': road_class_score,
            'rear_obstacle_clearance': rear_clearance_score,
            'battery_state': battery_score,
        }
        detail = {
            'score': score_i,
            'score_0_31': score_i,
            'score_binary': f'{score_i:05b}',
            'score_percent': score_percent,
            'value': value,
            'formula': 'lexicographic: road_class_range + 5*rear_clear + battery_bin',
            'priority': [
                'road_width_class',
                'rear_obstacle_clearance',
                'battery_state',
            ],
            'terms': {k: round(v, 3) for k, v in terms.items()},
            'ranks': {
                'road_rank': road_rank,
                'rear_clear_rank': rear_clear_rank,
                'battery_bin': battery_bin,
            },
            'value_ranges': {
                'C': '0-9',
                'B': '11-20',
                'A': '22-31',
            },
            'road_class': road_class,
            'road_width_m': round(road_width, 3) if math.isfinite(road_width) else -1.0,
            'road_width_source': road_width_source,
            'rear_obstacle': {
                'detected': rear_detected,
                'distance_m': round(rear_dist, 3) if math.isfinite(rear_dist) else -1.0,
            },
            'battery_pct': round(battery_pct, 1) if math.isfinite(battery_pct) else -1.0,
        }
        return score_i, value, detail

    @staticmethod
    def _status_phase(protocol: str) -> str:
        if protocol.startswith('rps_countdown'):
            return 'rps_countdown'
        if protocol.startswith('rps_wait'):
            return 'rps_wait_opponent'
        if protocol.startswith('rps_tie'):
            return 'rps_tie'
        if protocol.startswith('rps_win_yield'):
            return 'rps_result'
        if protocol.startswith('rps_lose_go'):
            return 'rps_result'
        if protocol.startswith('weight_wait'):
            return protocol.split()[0]
        if protocol.startswith('weight_yield'):
            return 'weight_result'
        if protocol.startswith('weight_go'):
            return 'weight_result'
        if protocol.startswith('weight_tie'):
            return 'weight_tie'
        return protocol

    def _log_negotiation_snapshot(self, protocol: str) -> None:
        if self.negotiation_start_stamp is None:
            return

        opponent_sign = str(self.v2v_sign.get('sign', 'UNKNOWN'))
        opponent_mask = str(self.v2v_sign.get('mask', '00000')).strip()
        opponent_value = self.v2v_sign.get('value', '-')
        opponent_detected = bool(self.v2v_sign.get('detected', False))
        opponent_rps = self.SIGN_TO_RPS.get(opponent_sign, '-')
        score_binary = self.last_yield_score_detail.get('score_binary', '-')
        score_value = self.last_yield_score_detail.get('score', '-')
        score_road = self.last_yield_score_detail.get('road_class', '-')
        phase = self._status_phase(protocol)

        snapshot = (
            self.negotiation_id,
            phase,
            self.active_protocol or '-',
            self.current_decision,
            self.current_led_pattern,
            self.local_rps_choice or '-',
            score_binary,
            score_value,
            opponent_detected,
            opponent_sign,
            opponent_mask,
            str(opponent_value),
            opponent_rps,
        )
        if snapshot == self.last_negotiation_snapshot:
            return
        self.last_negotiation_snapshot = snapshot

        self.get_logger().info(
            '[협상 상태] '
            f'phase={phase}, 방식={self.active_protocol or "-"}, '
            f'내꺼: led={self.current_led_pattern}, rps={self.local_rps_choice or "-"}, '
            f'score={score_binary}({score_value}), road={score_road}; '
            f'상대꺼: detected={opponent_detected}, sign={opponent_sign}, '
            f'mask={opponent_mask}, value={opponent_value}, rps={opponent_rps}; '
            f'결과={self.current_decision}')

    def _publish(self, protocol: str) -> None:
        self._log_negotiation_snapshot(protocol)

        msg = String()
        msg.data = self.current_decision
        self.decision_pub.publish(msg)

        pattern_msg = String()
        pattern_msg.data = self.current_led_pattern
        self.led_pattern_pub.publish(pattern_msg)

        status_msg = String()
        status_msg.data = json.dumps({
            'protocol': protocol,
            'active_protocol': self.active_protocol,
            'trigger': self.active_trigger,
            'decision': self.current_decision,
            'led_pattern': self.current_led_pattern,
            'opponent_sign': self.v2v_sign,
            'yield_score': self.last_yield_score_detail,
        })
        self.status_pub.publish(status_msg)

        if (not self.response_time_reported
                and self.first_deadlock_stamp is not None
                and self.current_decision in ('I_GO', 'I_YIELD')):
            elapsed = self.now_sec() - self.first_deadlock_stamp
            rt_msg = Float32()
            rt_msg.data = float(elapsed)
            self.response_time_pub.publish(rt_msg)
            self.response_time_reported = True
            self.get_logger().info(
                f'[T4] deadlock response time = {elapsed:.3f} s '
                f'(decision={self.current_decision}, protocol={protocol})')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CooperationDecisionNode()
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
