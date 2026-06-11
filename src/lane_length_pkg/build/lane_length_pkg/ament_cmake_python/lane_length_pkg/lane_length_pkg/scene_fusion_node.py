#!/usr/bin/env python3
from typing import Optional

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, String


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SceneFusionNode(Node):
    def __init__(self) -> None:
        super().__init__('scene_fusion_node')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('pose_topic', '/pose')
        self.declare_parameter('use_pose_topic', False)
        self.declare_parameter('publish_pose', True)
        self.declare_parameter('lane_width_topic', '/fused/lane_width_m')
        self.declare_parameter('lane_status_topic', '/lane_status')
        self.declare_parameter('obstacles_topic', '/fused/obstacles')
        self.declare_parameter('center_error_topic', '/lane_error_center_m')
        self.declare_parameter('memory_pullover_topic', '/memory/pull_over_candidates')
        self.declare_parameter('memory_narrow_topic', '/memory/narrow_passages')
        self.declare_parameter('opponent_topic', '/perception/opponent_vehicle')
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('stale_timeout_sec', 0.6)
        self.declare_parameter('min_valid_road_width_m', 0.05)
        self.declare_parameter('narrow_live_width_threshold_m', 0.40)
        self.declare_parameter('road_class_a_min_m', 0.47)
        self.declare_parameter('road_class_b_min_m', 0.19)
        self.declare_parameter('opponent_detection_distance_m', 0.90)
        self.declare_parameter('opponent_front_x_min_m', 0.08)
        self.declare_parameter('opponent_y_half_m', 0.45)
        self.declare_parameter('opponent_clear_sec', 3.0)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.use_pose_topic = bool(self.get_parameter('use_pose_topic').value)
        self.publish_pose_enabled = bool(self.get_parameter('publish_pose').value)
        self.lane_width_topic = str(self.get_parameter('lane_width_topic').value)
        self.lane_status_topic = str(self.get_parameter('lane_status_topic').value)
        self.obstacles_topic = str(self.get_parameter('obstacles_topic').value)
        self.center_error_topic = str(self.get_parameter('center_error_topic').value)
        self.memory_pullover_topic = str(self.get_parameter('memory_pullover_topic').value)
        self.memory_narrow_topic = str(self.get_parameter('memory_narrow_topic').value)
        self.opponent_topic = str(self.get_parameter('opponent_topic').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.stale_timeout_sec = float(self.get_parameter('stale_timeout_sec').value)
        self.min_valid_road_width_m = float(
            self.get_parameter('min_valid_road_width_m').value)
        self.narrow_live_width_threshold_m = float(
            self.get_parameter('narrow_live_width_threshold_m').value)
        self.road_class_a_min_m = float(self.get_parameter('road_class_a_min_m').value)
        self.road_class_b_min_m = float(self.get_parameter('road_class_b_min_m').value)
        self.opponent_detection_distance_m = float(
            self.get_parameter('opponent_detection_distance_m').value)
        self.opponent_front_x_min_m = float(self.get_parameter('opponent_front_x_min_m').value)
        self.opponent_y_half_m = float(self.get_parameter('opponent_y_half_m').value)
        self.opponent_clear_sec = float(self.get_parameter('opponent_clear_sec').value)

        self.odom_msg: Optional[Odometry] = None
        self.odom_stamp: Optional[float] = None
        self.pose_msg: Optional[PoseStamped] = None
        self.pose_stamp: Optional[float] = None
        self.live_width: Optional[float] = None
        self.live_width_stamp: Optional[float] = None
        self.lane_status: str = 'lost'
        self.lane_status_stamp: Optional[float] = None
        self.obstacles: Optional[PoseArray] = None
        self.obstacles_stamp: Optional[float] = None
        self.center_error_m: float = 0.0
        self.driving_mode: str = ''
        self.prev_driving_mode: str = ''
        self.memory_pullovers: list[dict] = []
        self.memory_narrows: list[dict] = []
        self.memory_pullover_stamp: Optional[float] = None
        self.memory_narrow_stamp: Optional[float] = None
        self.opponent_data: dict = {}
        self.opponent_stamp: Optional[float] = None

        self.was_in_narrow = False
        self.passed_narrow = False
        self.last_opponent_seen: Optional[float] = None
        self.last_logged_road_class: str = ''
        self.last_class_a_pose: Optional[dict] = None

        if self.use_pose_topic:
            self.create_subscription(PoseStamped, self.pose_topic, self.pose_cb, 10)
        else:
            self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.create_subscription(Float32, self.lane_width_topic, self.live_width_cb, 10)
        self.create_subscription(String, self.lane_status_topic, self.status_cb, 10)
        self.create_subscription(PoseArray, self.obstacles_topic, self.obstacles_cb, 10)
        self.create_subscription(Float32, self.center_error_topic, self.center_error_cb, 10)
        self.create_subscription(String, '/planning/driving_mode', self.driving_mode_cb, 10)
        self.create_subscription(String, self.memory_pullover_topic, self.memory_pullover_cb, 10)
        self.create_subscription(String, self.memory_narrow_topic, self.memory_narrow_cb, 10)
        self.create_subscription(String, self.opponent_topic, self.opponent_cb, 10)

        self.scene_pub = self.create_publisher(String, '/scene/understanding', 10)
        self.pose_pub = (
            self.create_publisher(PoseStamped, '/pose', 10)
            if self.publish_pose_enabled else None
        )

        dt = 1.0 / self.publish_hz if self.publish_hz > 0.0 else 0.05
        self.timer = self.create_timer(dt, self.step)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def fresh(self, stamp: Optional[float]) -> bool:
        return stamp is not None and (self.now_sec() - stamp) <= self.stale_timeout_sec

    def odom_cb(self, msg: Odometry) -> None:
        self.odom_msg = msg
        self.odom_stamp = self.now_sec()

    def pose_cb(self, msg: PoseStamped) -> None:
        self.pose_msg = msg
        self.pose_stamp = self.now_sec()

    def live_width_cb(self, msg: Float32) -> None:
        self.live_width = float(msg.data)
        self.live_width_stamp = self.now_sec()

    def status_cb(self, msg: String) -> None:
        self.lane_status = msg.data.strip().lower()
        self.lane_status_stamp = self.now_sec()

    def obstacles_cb(self, msg: PoseArray) -> None:
        self.obstacles = msg
        self.obstacles_stamp = self.now_sec()

    def center_error_cb(self, msg: Float32) -> None:
        v = float(msg.data)
        if math.isfinite(v):
            self.center_error_m = v

    def driving_mode_cb(self, msg: String) -> None:
        mode = msg.data.strip()
        if mode != self.driving_mode:
            self.prev_driving_mode = self.driving_mode
            self.driving_mode = mode
            was_yield_cycle = self.prev_driving_mode in (
                'YIELD_REVERSE',
                'YIELD_SIDE',
                'YIELD_WAIT_CLEAR',
                'REENTER',
            )
            if was_yield_cycle and mode in ('LANE_FOLLOW', 'APPROACH_NARROW'):
                self.last_class_a_pose = None

    def memory_pullover_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.memory_pullovers = data if isinstance(data, list) else []
        self.memory_pullover_stamp = self.now_sec()

    def memory_narrow_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.memory_narrows = data if isinstance(data, list) else []
        self.memory_narrow_stamp = self.now_sec()

    def opponent_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.opponent_data = data if isinstance(data, dict) else {}
        self.opponent_stamp = self.now_sec()

    def resolved_width(self) -> tuple[float, str]:
        if (
            self.fresh(self.live_width_stamp)
            and self.live_width is not None
            and math.isfinite(self.live_width)
            and self.live_width >= self.min_valid_road_width_m
        ):
            return float(self.live_width), 'live'
        return float('inf'), 'unknown'

    def classify_road(self, width: float) -> str:
        if not math.isfinite(width):
            return 'UNKNOWN'
        if width >= self.road_class_a_min_m:
            return 'A'
        if width > self.road_class_b_min_m:
            return 'B'
        return 'C'

    def nearest_pullover(self) -> Optional[dict]:
        if not self.fresh(self.memory_pullover_stamp) or not self.memory_pullovers:
            return None
        return self.memory_pullovers[0]

    def nearest_narrow_distance(self, x: float, y: float) -> float:
        if not self.fresh(self.memory_narrow_stamp) or not self.memory_narrows:
            return -1.0
        best = float('inf')
        for passage in self.memory_narrows:
            try:
                cx = 0.5 * (float(passage['x1']) + float(passage['x2']))
                cy = 0.5 * (float(passage['y1']) + float(passage['y2']))
            except (KeyError, TypeError, ValueError):
                continue
            best = min(best, math.hypot(x - cx, y - cy))
        return best if math.isfinite(best) else -1.0

    def opponent_summary(self) -> dict:
        now = self.now_sec()
        recently_seen = (
            self.last_opponent_seen is not None and
            (now - self.last_opponent_seen) <= self.opponent_clear_sec
        )

        if not self.fresh(self.opponent_stamp):
            return {
                'detected': False,
                'distance_m': -1.0,
                'angle_rad': 0.0,
                'cleared': not recently_seen,
                'bbox': [],
                'bbox_area_px': 0,
                'box_detected': False,
                'led_box_visible': False,
                'led_mask': '00000',
                'v2v_sign': 'UNKNOWN',
            }

        data = self.opponent_data
        box_detected = bool(data.get('box_detected', False))
        slot_locked = bool(data.get('slot_locked', False))
        led_mask = str(data.get('led_mask', data.get('mask', '00000'))).strip()
        v2v_sign = str(data.get('v2v_sign', data.get('sign', 'UNKNOWN'))).strip()
        valid_led_signal = led_mask != '00000' and v2v_sign != 'UNKNOWN'
        current_led_seen = (
            bool(data.get('detected', False))
            and box_detected
            and valid_led_signal
        )
        led_box_visible = current_led_seen or (slot_locked and box_detected and valid_led_signal)
        detected = led_box_visible
        if detected:
            self.last_opponent_seen = now
            recently_seen = True

        try:
            distance_m = float(data.get('distance_m', -1.0))
        except (TypeError, ValueError):
            distance_m = -1.0
        try:
            angle_rad = float(data.get('angle_rad', 0.0))
        except (TypeError, ValueError):
            angle_rad = 0.0
        try:
            bbox_area_px = int(float(data.get('bbox_area_px', 0)))
        except (TypeError, ValueError):
            bbox_area_px = 0
        bbox = data.get('bbox', [])
        if not isinstance(bbox, list):
            bbox = []

        return {
            'detected': detected,
            'distance_m': distance_m,
            'angle_rad': angle_rad,
            'cleared': not recently_seen,
            'bbox': bbox,
            'bbox_area_px': bbox_area_px,
            'box_detected': box_detected,
            'led_box_visible': led_box_visible,
            'led_mask': led_mask,
            'v2v_sign': v2v_sign,
        }

    def pose_from_stamped(self, pose_msg: PoseStamped) -> tuple[float, float, float, str]:
        pose = pose_msg.pose
        theta = yaw_from_quat(pose.orientation)
        return (
            float(pose.position.x),
            float(pose.position.y),
            float(theta),
            pose_msg.header.frame_id or 'map',
        )

    def publish_pose_from_odom(self, pose_msg: Odometry) -> tuple[float, float, float, str]:
        pose = pose_msg.pose.pose
        theta = yaw_from_quat(pose.orientation)

        frame = pose_msg.header.frame_id or 'odom'
        if self.pose_pub is not None:
            out = PoseStamped()
            out.header = pose_msg.header
            out.header.frame_id = frame
            out.pose = pose
            self.pose_pub.publish(out)

        return float(pose.position.x), float(pose.position.y), float(theta), frame

    def step(self) -> None:
        if self.use_pose_topic:
            if self.pose_msg is None:
                return
            x, y, theta, pose_frame = self.pose_from_stamped(self.pose_msg)
        else:
            if self.odom_msg is None:
                return
            x, y, theta, pose_frame = self.publish_pose_from_odom(self.odom_msg)

        width, width_source = self.resolved_width()
        in_narrow = math.isfinite(width) and width < self.narrow_live_width_threshold_m
        mode = self.driving_mode.upper()
        reversing = mode == 'YIELD_REVERSE'
        update_class_a_pose = mode not in (
            'DEADLOCK_CHECK',
            'NEGOTIATION',
            'WAIT',
            'WAIT_FOR_PASS',
            'RIGHT_OFFSET_PASS',
            'YIELD_REVERSE',
            'YIELD_SIDE',
            'YIELD_WAIT_CLEAR',
            'REENTER',
        )
        classify_road = update_class_a_pose
        road_class = self.classify_road(width) if classify_road else 'UNKNOWN'
        if classify_road and road_class != self.last_logged_road_class:
            w_str = f'{width:.3f}m' if math.isfinite(width) else 'unknown'
            self.get_logger().info(f'Road class: {self.last_logged_road_class or "?"} -> {road_class} | width={w_str}')
            self.last_logged_road_class = road_class
        lane_ok = self.fresh(self.lane_status_stamp) and self.lane_status == 'ok'
        if update_class_a_pose and road_class == 'A' and lane_ok:
            # 두 차선 중앙 좌표 계산: center_error_m = 카메라→차선중앙 lateral offset
            # 차선 인식 유효할 때만 저장한다. lane_ok가 아니면 차량 현재 좌표가
            # Class A pose로 저장되어 후진 오프셋이 벽 쪽으로 누적될 수 있다.
            camera_offset_m = 0.01
            corrected_error = self.center_error_m - camera_offset_m
            cx = x + corrected_error * math.sin(theta)
            cy = y - corrected_error * math.cos(theta)
            self.last_class_a_pose = {
                'x': round(cx, 4),
                'y': round(cy, 4),
                'theta': round(theta, 4),
                'width': round(width, 4),
                'stamp': round(self.now_sec(), 3),
                'source': 'lane_center',
            }
            self.get_logger().info(
                f'Class A pose: center_err={self.center_error_m:.3f}m lane_ok={lane_ok} '
                f'→ ({cx:.3f}, {cy:.3f})'
            )
        self.passed_narrow = self.was_in_narrow and not in_narrow
        self.was_in_narrow = in_narrow
        opponent = self.opponent_summary()

        memory_nearest_narrow = self.nearest_narrow_distance(x, y)
        nearest_narrow = 0.0 if in_narrow else memory_nearest_narrow
        pullover = self.nearest_pullover()
        back_on_lane = self.fresh(self.lane_status_stamp) and self.lane_status == 'ok'
        road_width = round(width, 3) if math.isfinite(width) else -1.0

        scene = {
            'pose': {
                'x': round(x, 4),
                'y': round(y, 4),
                'theta': round(theta, 4),
                'frame': pose_frame,
            },
            'environment': {
                'in_narrow_passage': in_narrow,
                'nearest_narrow_passage_m': nearest_narrow,
                'road_width_m': road_width,
                'width_source': width_source,
                'road_class': road_class,
                'pull_over_available': pullover is not None,
                'nearest_pull_over': pullover,
            },
            'in_narrow_passage': in_narrow,
            'nearest_narrow_passage_m': nearest_narrow,
            'passed_narrow_passage': self.passed_narrow,
            'road_width_m': road_width,
            'road_class': road_class,
            'nearest_pullover': pullover,
            'reached_pullover': False,
            'last_class_a_pose': self.last_class_a_pose,
            'lane_reentry_point': {'x': round(x, 4), 'y': round(y, 4), 'theta': round(theta, 4)},
            'back_on_lane': back_on_lane,
            'pass_clear': opponent['cleared'],
            'opponent_cleared_narrow': opponent['cleared'],
            'opponent_led_box_visible': opponent['led_box_visible'],
            'opponent_box_detected': opponent['box_detected'],
            'opponent': {
                'detected': opponent['detected'],
                'distance_m': opponent['distance_m'],
                'angle_rad': opponent['angle_rad'],
                'bbox': opponent['bbox'],
                'bbox_area_px': opponent['bbox_area_px'],
                'box_detected': opponent['box_detected'],
                'led_box_visible': opponent['led_box_visible'],
                'led_mask': opponent['led_mask'],
                'v2v_sign': opponent['v2v_sign'],
            },
        }

        msg = String()
        msg.data = json.dumps(scene)
        self.scene_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SceneFusionNode()
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
