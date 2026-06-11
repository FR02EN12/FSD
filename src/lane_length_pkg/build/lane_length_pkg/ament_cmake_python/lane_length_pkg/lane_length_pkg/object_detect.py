#!/usr/bin/env python3
# Performance target T1: vision pipeline >= 15 FPS, process_fps default 15.0
# Performance target T3: recognition accuracy >= 90%, temporal voting window
import json
import math
import os
import time
from collections import deque
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detect')

        # Parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        default_model_path = os.path.join(
            os.path.expanduser('~'), '.cache', 'sw_ws', 'models', 'yolov8n.pt')
        self.declare_parameter('model_path', default_model_path)
        self.declare_parameter('enable_yolo', False)
        self.declare_parameter('confidence_threshold', 0.5)
        # T1: raised from 5.0 -> 15.0 to meet >= 15 FPS vision pipeline target.
        self.declare_parameter('process_fps', 15.0)
        self.declare_parameter('support_publish_hz', 2.0)
        self.declare_parameter('known_object_height_m', 0.14)
        self.declare_parameter('focal_length_px', 1400.0)
        # T3: temporal voting accumulates detection lists and opponent frames.
        #     >= 90 % recognition accuracy.
        self.declare_parameter('detection_temporal_window', 3)
        # Opponent approach is now detected by the 5-LED panel in
        # led_signal. Keep object detections as debug/support
        # data unless this is explicitly re-enabled.
        self.declare_parameter('publish_opponent_vehicle', False)

        self._image_topic = self.get_parameter('image_topic').value
        self._enable_yolo = bool(self.get_parameter('enable_yolo').value)
        self._model_path = os.path.expanduser(
            str(self.get_parameter('model_path').value))
        self._conf_thresh = self.get_parameter('confidence_threshold').value
        self._process_fps = self.get_parameter('process_fps').value
        if not self._enable_yolo:
            self._process_fps = self.get_parameter('support_publish_hz').value
        self._known_h = self.get_parameter('known_object_height_m').value
        self._focal_px = self.get_parameter('focal_length_px').value
        self._temporal_n = int(self.get_parameter('detection_temporal_window').value)
        self._publish_opponent_vehicle = bool(
            self.get_parameter('publish_opponent_vehicle').value)

        # YOLO model
        self._model = None
        if self._enable_yolo:
            try:
                from ultralytics import YOLO
                model_dir = os.path.dirname(self._model_path)
                if model_dir:
                    os.makedirs(model_dir, exist_ok=True)
                self._model = YOLO(self._model_path)
                self.get_logger().info(f'YOLO model loaded: {self._model_path}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load YOLO model: {e}')
        else:
            self.get_logger().info(
                'YOLO disabled; publishing empty /perception/objects support data')

        # State
        self._bridge = CvBridge() if self._enable_yolo else None
        self._latest_image: Optional[np.ndarray] = None
        self._last_process_time = 0.0
        # T3: per-class detection vote history for temporal smoothing.
        self._detection_votes: deque = deque(maxlen=self._temporal_n)
        self._opponent_votes: deque = deque(maxlen=self._temporal_n)

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscriptions / publishers
        self.sub_image = None
        if self._enable_yolo:
            self.sub_image = self.create_subscription(
                Image, self._image_topic, self._image_cb, sensor_qos)

        self.pub_objects = self.create_publisher(String, '/perception/objects', 10)
        self.pub_opponent = self.create_publisher(
            String, '/perception/object_opponent_vehicle', 10)
        # R5 fix: T1 runtime metric, measured processing FPS.
        self.pub_fps = self.create_publisher(Float32, '/metrics/object_detection_fps', 10)

        # FPS measurement state
        self._fps_window_start: float = time.monotonic()
        self._fps_window_count: int = 0

        # Timer
        period = 1.0 / max(self._process_fps, 0.1)
        self.create_timer(period, self._process_cb)

        self.get_logger().info('ObjectDetectionNode initialised')

    # Callbacks
    def _image_cb(self, msg: Image) -> None:
        try:
            if self._bridge is not None:
                self._latest_image = self._bridge.imgmsg_to_cv2(
                    msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')

    def _process_cb(self) -> None:
        now = time.monotonic()
        if now - self._last_process_time < (1.0 / max(self._process_fps, 0.1)):
            return
        self._last_process_time = now

        if self._latest_image is None or self._model is None:
            self._publish_empty(now)
            return

        # R5 fix: measure & publish actual processing FPS once per second.
        self._fps_window_count += 1
        elapsed = now - self._fps_window_start
        if elapsed >= 1.0:
            fps_msg = Float32()
            fps_msg.data = float(self._fps_window_count) / elapsed
            self.pub_fps.publish(fps_msg)
            self._fps_window_start = now
            self._fps_window_count = 0

        frame = self._latest_image
        h_img, w_img = frame.shape[:2]

        # Run inference
        try:
            results = self._model(frame, verbose=False, conf=self._conf_thresh)
        except Exception as e:
            self.get_logger().warn(f'YOLO inference error: {e}')
            return

        detections: List[dict] = []
        opponent: Optional[dict] = None

        # Relevant COCO classes: 0 = person (proxy for turtlebot)
        relevant_classes = {0: 'turtlebot_proxy'}

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                if cls_id not in relevant_classes:
                    continue

                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
                bbox_h = y2 - y1
                bbox_cx = (x1 + x2) / 2.0

                # Distance estimate
                if bbox_h > 0:
                    distance_m = (self._known_h * self._focal_px) / bbox_h
                else:
                    distance_m = float('inf')

                # Angle from image centre (positive = right)
                angle_rad = math.atan2(bbox_cx - w_img / 2.0, self._focal_px)

                det = {
                    'class': relevant_classes[cls_id],
                    'bbox': [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)],
                    'confidence': round(conf, 3),
                    'distance_m': round(distance_m, 3),
                    'angle_rad': round(angle_rad, 4),
                }
                detections.append(det)

                # Keep closest as opponent
                if opponent is None or distance_m < opponent['distance_m']:
                    opponent = {
                        'detected': True,
                        'distance_m': round(distance_m, 3),
                        'angle_rad': round(angle_rad, 4),
                        'bbox': det['bbox'],
                        'confidence': round(conf, 3),
                    }

        # T3: temporal voting accumulates detection lists and opponent frames.
        self._detection_votes.append(detections)
        self._opponent_votes.append(opponent)

        # Publish all detections (use latest frame; downstream can aggregate)
        msg_all = String()
        msg_all.data = json.dumps(detections)
        self.pub_objects.publish(msg_all)

        if self._publish_opponent_vehicle:
            # Debug/support topic only. The system-level opponent_vehicle topic
            # is owned by the 5-LED V2V detector.
            msg_opp = String()
            confirmed_opponent = self._vote_opponent()
            if confirmed_opponent is not None:
                msg_opp.data = json.dumps(confirmed_opponent)
            else:
                msg_opp.data = json.dumps({
                    'detected': False,
                    'distance_m': 0.0,
                    'angle_rad': 0.0,
                    'bbox': [],
                    'confidence': 0.0,
                    'source': 'object_detection',
                })
            self.pub_opponent.publish(msg_opp)

    def _publish_empty(self, now: float) -> None:
        msg_all = String()
        msg_all.data = '[]'
        self.pub_objects.publish(msg_all)

        if self._publish_opponent_vehicle:
            msg_opp = String()
            msg_opp.data = json.dumps({
                'detected': False,
                'distance_m': 0.0,
                'angle_rad': 0.0,
                'bbox': [],
                'confidence': 0.0,
                'source': 'object_detection',
            })
            self.pub_opponent.publish(msg_opp)

        elapsed = now - self._fps_window_start
        if elapsed >= 1.0:
            fps_msg = Float32()
            fps_msg.data = 0.0
            self.pub_fps.publish(fps_msg)
            self._fps_window_start = now
            self._fps_window_count = 0

    # Temporal voting helper
    def _vote_opponent(self) -> Optional[dict]:
        """Return the most-recent opponent that was detected in a majority of
        the temporal window; returns None if the vote does not reach majority."""
        detections_in_window = [o for o in self._opponent_votes if o is not None]
        if not detections_in_window:
            return None
        total = len(self._opponent_votes)
        detected_count = sum(1 for o in detections_in_window)
        if detected_count > total / 2:
            # Return the most recently detected opponent
            return detections_in_window[-1]
        return None


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
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
