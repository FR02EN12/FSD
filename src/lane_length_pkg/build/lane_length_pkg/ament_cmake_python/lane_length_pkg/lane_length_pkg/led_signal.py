#!/usr/bin/env python3
import json
import math
import os
import time
from collections import Counter, deque
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


class LEDSignalPerceptionNode(Node):
    """Detect the opponent vehicle 5-LED panel and decode V2V patterns."""

    PATTERN_LABELS: Dict[str, str] = {
        '00000': 'normal',
        '11111': 'car detected',
        '10101': 'game',
        '01010': 'no game',
        '11000': 'rock',
        '10100': 'scissor',
        '10010': 'paper',
    }

    SIGN_BY_MASK: Dict[str, str] = {
        '11111': 'APPROACH',
        '10101': 'RPS_READY',
        '01010': 'WEIGHT_READY',
        '11000': 'RPS_ROCK',
        '10100': 'RPS_SCISSORS',
        '10010': 'RPS_PAPER',
    }

    def __init__(self):
        super().__init__('led_signal')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('process_fps', 15.0)
        self.declare_parameter('temporal_window', 5)
        self.declare_parameter('led_count', 5)
        self.declare_parameter('show_debug_view', True)
        self.declare_parameter('debug_window_name', 'led_signal_debug')
        self.declare_parameter('roi_left_ratio', 0.00)
        self.declare_parameter('roi_right_ratio', 1.00)
        self.declare_parameter('roi_top_ratio', 0.00)
        self.declare_parameter('roi_bottom_ratio', 1.00)
        self.declare_parameter('min_detect_bbox_area_px', 1200)
        self.declare_parameter('max_detect_bbox_area_px', 3000)
        self.declare_parameter('bbox_lock_confirm_sec', 0.2)
        self.declare_parameter('track_locked_strip', False)
        self.declare_parameter('freeze_locked_bbox', True)
        self.declare_parameter('wait_pass_redetect_interval_frames', 5)
        self.declare_parameter('wait_pass_redetect_miss_limit', 3)
        self.declare_parameter('enable_image_filter', False)
        self.declare_parameter('image_filter_v4l2_value_mode', False)
        self.declare_parameter('image_filter_brightness', 0.0)
        self.declare_parameter('image_filter_contrast', 1.0)
        self.declare_parameter('image_filter_saturation', 1.0)
        self.declare_parameter('image_filter_gain', 1.0)
        self.declare_parameter('lane_binary_blur_kernel', 5)
        self.declare_parameter('lane_binary_white_l_min', 135)
        self.declare_parameter('lane_binary_white_s_max', 200)
        self.declare_parameter('lane_binary_white_gray_min', 150)

        # Legacy parameters kept so existing launch/config files remain valid.
        self.declare_parameter('brightness_threshold', 180)
        self.declare_parameter('saturation_threshold', 35)
        self.declare_parameter('min_pixel_threshold', 40)
        self.declare_parameter('min_blob_area_px', 8)
        self.declare_parameter('max_blob_area_px', 2500)
        self.declare_parameter('slot_sample_radius_px', 7)
        self.declare_parameter('slot_on_ratio_threshold', 0.08)

        # Parameters copied from the verified OpenCV classifier.
        self.declare_parameter('use_fisheye_undistort', True)
        self.declare_parameter(
            'camera_info_yaml',
            os.path.expanduser('~/fsd_ws/src/perception_py/config/fisheye.yaml'),
        )
        self.declare_parameter('fisheye_calibration_path', '')
        self.declare_parameter('fisheye_balance', 0.0)
        self.declare_parameter('dark_box_thresh', 70)
        self.declare_parameter('min_box_area', 10000)
        self.declare_parameter('min_box_aspect', 1.2)
        self.declare_parameter('max_box_aspect', 6.8)
        self.declare_parameter('min_box_fill_ratio', 0.42)
        self.declare_parameter('led_thresh', 170)
        self.declare_parameter('min_led_area', 5)
        self.declare_parameter('max_led_area', 2500)
        self.declare_parameter('max_led_y_diff', 25)
        self.declare_parameter('slot_tol_pixels', 18)
        self.declare_parameter('slot_tol_ratio', 0.38)
        self.declare_parameter('min_leds_for_box', 2)
        # Orange/yellow LED candidate detector. The final signal is still decoded by ON/OFF pattern,
        # not by LED color. HSV is only used to make weak orange LEDs easier to segment.
        self.declare_parameter('use_hsv_led_detection', True)
        self.declare_parameter('led_h_min', 0)
        self.declare_parameter('led_h_max', 55)
        self.declare_parameter('led_s_min', 35)
        self.declare_parameter('led_v_min', 70)
        self.declare_parameter('led_dilate_iterations', 1)

        self.declare_parameter('opponent_latch_sec', 2.0)
        self.declare_parameter('led_panel_width_m', 0.12)
        self.declare_parameter('focal_length_px', 1400.14208)

        self._image_topic = str(self.get_parameter('image_topic').value)
        self._process_fps = float(self.get_parameter('process_fps').value)
        self._temporal_n = int(self.get_parameter('temporal_window').value)
        self._led_count = int(self.get_parameter('led_count').value)
        self._show_debug_view = bool(self.get_parameter('show_debug_view').value)
        self._debug_window_name = str(self.get_parameter('debug_window_name').value)
        self._roi_left_ratio = float(self.get_parameter('roi_left_ratio').value)
        self._roi_right_ratio = float(self.get_parameter('roi_right_ratio').value)
        self._roi_top_ratio = float(self.get_parameter('roi_top_ratio').value)
        self._roi_bottom_ratio = float(self.get_parameter('roi_bottom_ratio').value)
        self._min_detect_bbox_area_px = int(
            self.get_parameter('min_detect_bbox_area_px').value)
        self._max_detect_bbox_area_px = int(
            self.get_parameter('max_detect_bbox_area_px').value)
        self._bbox_lock_confirm_sec = float(
            self.get_parameter('bbox_lock_confirm_sec').value)
        self._track_locked_strip = bool(
            self.get_parameter('track_locked_strip').value)
        self._freeze_locked_bbox = bool(
            self.get_parameter('freeze_locked_bbox').value)
        self._wait_pass_redetect_interval_frames = max(
            1, int(self.get_parameter('wait_pass_redetect_interval_frames').value))
        self._wait_pass_redetect_miss_limit = max(
            1, int(self.get_parameter('wait_pass_redetect_miss_limit').value))
        self._enable_image_filter = bool(self.get_parameter('enable_image_filter').value)
        self._image_filter_v4l2_value_mode = bool(
            self.get_parameter('image_filter_v4l2_value_mode').value)
        self._image_filter_brightness = float(
            self.get_parameter('image_filter_brightness').value)
        self._image_filter_contrast = float(
            self.get_parameter('image_filter_contrast').value)
        self._image_filter_saturation = float(
            self.get_parameter('image_filter_saturation').value)
        self._image_filter_gain = float(self.get_parameter('image_filter_gain').value)
        self._lane_binary_blur_kernel = int(
            self.get_parameter('lane_binary_blur_kernel').value)
        if self._lane_binary_blur_kernel % 2 == 0:
            self._lane_binary_blur_kernel += 1
        self._lane_binary_white_l_min = int(
            self.get_parameter('lane_binary_white_l_min').value)
        self._lane_binary_white_s_max = int(
            self.get_parameter('lane_binary_white_s_max').value)
        self._lane_binary_white_gray_min = int(
            self.get_parameter('lane_binary_white_gray_min').value)

        self._use_fisheye = bool(self.get_parameter('use_fisheye_undistort').value)
        self._fisheye_balance = float(self.get_parameter('fisheye_balance').value)
        self._dark_box_thresh = int(self.get_parameter('dark_box_thresh').value)
        self._min_box_area = int(self.get_parameter('min_box_area').value)
        self._min_box_aspect = float(self.get_parameter('min_box_aspect').value)
        self._max_box_aspect = float(self.get_parameter('max_box_aspect').value)
        self._min_box_fill_ratio = float(self.get_parameter('min_box_fill_ratio').value)
        self._led_thresh = int(self.get_parameter('led_thresh').value)
        self._min_led_area = int(self.get_parameter('min_led_area').value)
        self._max_led_area = int(self.get_parameter('max_led_area').value)
        self._max_led_y_diff = float(self.get_parameter('max_led_y_diff').value)
        self._slot_tol_pixels = int(self.get_parameter('slot_tol_pixels').value)
        self._slot_tol_ratio = float(self.get_parameter('slot_tol_ratio').value)
        self._min_leds_for_box = int(self.get_parameter('min_leds_for_box').value)
        self._use_hsv_led_detection = bool(
            self.get_parameter('use_hsv_led_detection').value)
        self._led_h_min = int(self.get_parameter('led_h_min').value)
        self._led_h_max = int(self.get_parameter('led_h_max').value)
        self._led_s_min = int(self.get_parameter('led_s_min').value)
        self._led_v_min = int(self.get_parameter('led_v_min').value)
        self._led_dilate_iterations = int(
            self.get_parameter('led_dilate_iterations').value)

        self._opponent_latch = float(self.get_parameter('opponent_latch_sec').value)
        self._panel_width_m = float(self.get_parameter('led_panel_width_m').value)
        self._focal_px = float(self.get_parameter('focal_length_px').value)

        self._bridge = CvBridge()
        self._latest_image: Optional[np.ndarray] = None
        self._mask_votes: deque = deque(maxlen=max(self._temporal_n, 1))
        self._slot_locked = False
        self._fixed_strip_rect: Optional[Tuple[int, int, int, int]] = None
        self._candidate_strip_rect: Optional[Tuple[int, int, int, int]] = None
        self._candidate_since: Optional[float] = None
        self._slot_x_positions: Optional[np.ndarray] = None
        self._slot_y_position: Optional[float] = None
        self._opponent_latch_until = 0.0
        self._last_distance_m = 1.0
        self._last_angle_rad = 0.0
        self._last_bbox: List[float] = []
        self._last_bbox_area_px = 0
        self._last_bbox_right_bottom: Tuple[float, float] = (0.0, 0.0)
        self._driving_mode = ''
        self._wait_pass_frame_count = 0
        self._wait_pass_redetect_misses = 0
        self._wait_pass_forced_unknown = False
        self._camera_calibration = self._load_camera_calibration()
        self._undistort_maps: Dict[Tuple[int, int], Tuple[np.ndarray, np.ndarray]] = {}

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Image, self._image_topic, self._image_cb, sensor_qos)
        self.create_subscription(String, '/planning/driving_mode', self._mode_cb, 10)

        self.pub_led_state = self.create_publisher(
            String, '/perception/opponent_led_state', 10)
        self.pub_v2v_sign = self.create_publisher(
            String, '/perception/opponent_v2v_sign', 10)
        self.pub_opponent = self.create_publisher(
            String, '/perception/opponent_vehicle', 10)

        self._fps_window_start = time.monotonic()
        self._fps_window_count = 0

        period = 1.0 / max(self._process_fps, 0.1)
        self.create_timer(period, self._process_cb)

        self.get_logger().info(
            'LEDSignalPerceptionNode initialised with black-box 5-LED classifier')

    def _image_cb(self, msg: Image) -> None:
        try:
            self._latest_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')

    def _mode_cb(self, msg: String) -> None:
        mode = msg.data.strip().upper()
        if mode != self._driving_mode:
            self._wait_pass_frame_count = 0
            self._wait_pass_redetect_misses = 0
            self._wait_pass_forced_unknown = False
        self._driving_mode = mode

    def _apply_image_filter(self, frame: np.ndarray) -> np.ndarray:
        if not self._enable_image_filter:
            return frame

        contrast = self._image_filter_contrast
        saturation = self._image_filter_saturation
        gain = self._image_filter_gain
        if self._image_filter_v4l2_value_mode:
            contrast = contrast / 32.0
            saturation = saturation / 32.0
            gain = 1.0 + gain / 100.0

        alpha = max(0.0, contrast * gain)
        beta = self._image_filter_brightness
        filtered = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)

        saturation = max(0.0, saturation)
        if abs(saturation - 1.0) > 1e-3:
            hsv = cv2.cvtColor(filtered, cv2.COLOR_BGR2HSV).astype(np.float32)
            hsv[:, :, 1] = np.clip(hsv[:, :, 1] * saturation, 0, 255)
            filtered = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

        return filtered

    def _lane_binary_view(self, frame: np.ndarray) -> np.ndarray:
        blur = cv2.GaussianBlur(
            frame,
            (self._lane_binary_blur_kernel, self._lane_binary_blur_kernel),
            0,
        )
        hls = cv2.cvtColor(blur, cv2.COLOR_BGR2HLS)
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

        white_mask_hls = cv2.inRange(
            hls,
            np.array([0, self._lane_binary_white_l_min, 0], dtype=np.uint8),
            np.array([255, 255, self._lane_binary_white_s_max], dtype=np.uint8),
        )
        white_mask_gray = cv2.inRange(gray, self._lane_binary_white_gray_min, 255)
        color_mask = cv2.bitwise_and(white_mask_hls, white_mask_gray)

        mask = np.zeros_like(color_mask)
        rh, rw = color_mask.shape[:2]
        poly = np.array([[
            (0, rh - 1),
            (int(rw * 0.03), int(rh * 0.03)),
            (int(rw * 0.97), int(rh * 0.03)),
            (rw - 1, rh - 1),
        ]], dtype=np.int32)
        cv2.fillPoly(mask, poly, 255)
        color_mask = cv2.bitwise_and(color_mask, mask)

        kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 11))
        kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        binary = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel_close)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel_open)
        return binary

    def _crop_roi(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        x1 = int(w * min(max(self._roi_left_ratio, 0.0), 1.0))
        x2 = int(w * min(max(self._roi_right_ratio, 0.0), 1.0))
        y1 = int(h * min(max(self._roi_top_ratio, 0.0), 1.0))
        y2 = int(h * min(max(self._roi_bottom_ratio, 0.0), 1.0))
        x1 = max(0, min(w - 1, x1))
        x2 = max(x1 + 1, min(w, x2))
        y1 = max(0, min(h - 1, y1))
        y2 = max(y1 + 1, min(h, y2))
        return frame[y1:y2, x1:x2]

    def _process_cb(self) -> None:
        now = time.monotonic()

        if self._latest_image is None:
            self._publish('00000', 'UNKNOWN', 'no image', 0.0, False, now)
            return

        frame = self._latest_image.copy()
        frame = self._fisheye_undistort(frame)
        frame = self._crop_roi(frame)
        lane_gray_source = frame.copy()
        frame = self._apply_image_filter(frame)

        raw_mask, raw_label, box_detected, led_count, geometry_box = self._read_pattern(frame)
        bbox_area_px = self._bbox_area_px(geometry_box)
        bbox_too_small = bbox_area_px < self._min_detect_bbox_area_px
        bbox_too_large = (
            self._max_detect_bbox_area_px > 0
            and bbox_area_px > self._max_detect_bbox_area_px
        )
        if bbox_too_small or bbox_too_large:
            self._reset_lock_candidate()
            raw_mask = '00000'
            raw_label = (
                f'waiting bbox {self._min_detect_bbox_area_px}-'
                f'{self._max_detect_bbox_area_px}px, area={bbox_area_px}'
            )
            box_detected = False
            led_count = 0
            geometry_box = None

        self._mask_votes.append(raw_mask)
        mask, confidence = self._stable_mask()
        sign = self._classify_mask(mask)
        label = self.PATTERN_LABELS.get(mask, raw_label)

        if box_detected and mask != '00000' and sign != 'UNKNOWN':
            self._opponent_latch_until = now + self._opponent_latch

        if box_detected and geometry_box is not None:
            self._update_geometry(frame, geometry_box)
        else:
            self._clear_geometry()
        self._publish(mask, sign, label, confidence, box_detected, now, led_count)
        self._show_view(
            lane_gray_source,
            frame,
            mask,
            sign,
            label,
            confidence,
            box_detected,
            led_count,
            geometry_box,
        )

    @staticmethod
    def _bbox_area_px(box: Optional[Tuple[int, int, int, int]]) -> int:
        if box is None:
            return 0
        _, _, w, h = box
        return int(max(0, w) * max(0, h))

    def _load_camera_calibration(self) -> Optional[Dict[str, object]]:
        if not self._use_fisheye:
            return None

        yaml_configured = str(self.get_parameter('camera_info_yaml').value).strip()
        npz_configured = str(self.get_parameter('fisheye_calibration_path').value).strip()

        yaml_candidates = [
            yaml_configured,
            os.path.expanduser('~/fsd_ws/src/perception_py/config/fisheye.yaml'),
        ]
        for path in [p for p in yaml_candidates if p]:
            expanded = os.path.abspath(os.path.expanduser(path))
            if not os.path.isfile(expanded):
                continue
            try:
                calibration = self._load_camera_yaml(expanded)
                self.get_logger().info(
                    f'Loaded camera calibration yaml: {expanded} '
                    f'({calibration["model"]})')
                return calibration
            except Exception as exc:
                self.get_logger().warn(
                    f'Failed to load camera calibration yaml {expanded}: {exc}')
                return None

        npz_candidates = [
            npz_configured,
            'fisheye_calibration.npz',
            os.path.expanduser('~/fsd_ws/fisheye_calibration.npz'),
            os.path.expanduser('~/fsd_ws/src/perception_py/config/fisheye_calibration.npz'),
        ]
        for path in [p for p in npz_candidates if p]:
            expanded = os.path.abspath(os.path.expanduser(path))
            if not os.path.isfile(expanded):
                continue
            try:
                calibration = self._load_camera_npz(expanded)
                self.get_logger().info(f'Loaded fisheye npz calibration: {expanded}')
                return calibration
            except Exception as exc:
                self.get_logger().warn(
                    f'Failed to load fisheye npz calibration {expanded}: {exc}')
                return None

        self.get_logger().warn(
            'camera calibration yaml/npz not found; LED classifier will use raw frames')
        return None

    def _load_camera_yaml(self, path: str) -> Dict[str, object]:
        with open(path, 'r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)

        width = int(data['image_width'])
        height = int(data['image_height'])
        k = np.asarray(data['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
        d = np.asarray(data['distortion_coefficients']['data'], dtype=np.float64)
        r = np.asarray(
            data.get('rectification_matrix', {}).get('data', np.eye(3).reshape(-1)),
            dtype=np.float64,
        ).reshape(3, 3)

        projection = data.get('projection_matrix', {})
        p = None
        if projection and 'data' in projection:
            p = np.asarray(projection['data'], dtype=np.float64).reshape(3, 4)

        return {
            'model': str(data.get('distortion_model', 'plumb_bob')),
            'k': k,
            'd': d,
            'r': r,
            'p': p,
            'dim': (width, height),
            'source': 'yaml',
        }

    def _load_camera_npz(self, path: str) -> Dict[str, object]:
        data = np.load(path)
        k = np.asarray(data['K'], dtype=np.float64)
        d = np.asarray(data['D'], dtype=np.float64)
        dim_raw = data['DIM']
        return {
            'model': 'fisheye',
            'k': k,
            'd': d,
            'r': np.eye(3, dtype=np.float64),
            'p': None,
            'dim': (int(dim_raw[0]), int(dim_raw[1])),
            'source': 'npz',
        }

    def _fisheye_undistort(self, frame: np.ndarray) -> np.ndarray:
        if self._camera_calibration is None:
            return frame

        calibration = self._camera_calibration
        k = calibration['k']
        d = calibration['d']
        dim = calibration['dim']
        model = str(calibration['model']).lower()
        h, w = frame.shape[:2]
        key = (w, h)

        if key not in self._undistort_maps:
            scaled_k = self._scale_camera_matrix(k, dim, w, h)
            if model in ('fisheye', 'equidistant'):
                new_k = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                    scaled_k, d, (w, h), np.eye(3), balance=self._fisheye_balance)
                map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                    scaled_k, d, np.eye(3), new_k, (w, h), cv2.CV_16SC2)
            else:
                r = calibration['r']
                p = calibration['p']
                new_k = self._scale_projection_matrix(p, dim, w, h) if p is not None else scaled_k
                map1, map2 = cv2.initUndistortRectifyMap(
                    scaled_k,
                    d,
                    r,
                    new_k,
                    (w, h),
                    cv2.CV_16SC2,
                )
            self._undistort_maps[key] = (map1, map2)

        map1, map2 = self._undistort_maps[key]
        undistorted = cv2.remap(
            frame,
            map1,
            map2,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )

        uh, uw = undistorted.shape[:2]
        crop_x1 = int(uw * 0.12)
        crop_x2 = int(uw * 0.88)
        crop_y1 = int(uh * 0.18)
        crop_y2 = int(uh * 0.82)
        return undistorted[crop_y1:crop_y2, crop_x1:crop_x2]

    @staticmethod
    def _scale_camera_matrix(
        k: np.ndarray,
        dim: Tuple[int, int],
        width: int,
        height: int,
    ) -> np.ndarray:
        scaled = k.copy()
        sx = width / float(dim[0])
        sy = height / float(dim[1])
        scaled[0, 0] *= sx
        scaled[1, 1] *= sy
        scaled[0, 2] *= sx
        scaled[1, 2] *= sy
        return scaled

    @staticmethod
    def _scale_projection_matrix(
        p: np.ndarray,
        dim: Tuple[int, int],
        width: int,
        height: int,
    ) -> np.ndarray:
        scaled = p[:3, :3].copy()
        sx = width / float(dim[0])
        sy = height / float(dim[1])
        scaled[0, 0] *= sx
        scaled[1, 1] *= sy
        scaled[0, 2] *= sx
        scaled[1, 2] *= sy
        return scaled

    def _read_pattern(
        self,
        frame_bgr: np.ndarray,
    ) -> Tuple[str, str, bool, int, Optional[Tuple[int, int, int, int]]]:
        if self._slots_are_valid():
            wait_pass_result = self._wait_pass_redetect_result(frame_bgr)
            if wait_pass_result is not None:
                return wait_pass_result
            return self._read_locked_strip(frame_bgr)

        box, _dark_mask = self._find_black_box(frame_bgr, self._dark_box_thresh)
        if box is None:
            self._reset_lock_candidate()
            return '00000', 'waiting box', False, 0, None

        x, y, w, h = box
        big_roi = frame_bgr[y:y + h, x:x + w].copy()
        if big_roi.size == 0:
            self._reset_lock_candidate()
            return '00000', 'waiting box', False, 0, None

        gray_big = cv2.cvtColor(big_roi, cv2.COLOR_BGR2GRAY)
        gray_big = cv2.GaussianBlur(gray_big, (5, 5), 0)
        big_leds, _led_mask = self._detect_leds_in_box(gray_big, self._led_thresh)
        big_leds = self._refine_led_candidates(big_leds)
        if len(big_leds) != self._led_count:
            self._reset_lock_candidate()
            return '00000', f'waiting all-on, detected {len(big_leds)}', False, len(big_leds), None

        strip_rect = self._make_tight_strip_from_leds(big_roi.shape, big_leds, x, y)
        strip_rect = self._clamp_strip_rect(strip_rect, frame_bgr.shape)
        if strip_rect is None:
            self._reset_lock_candidate()
            return '00000', 'waiting strip ROI', False, len(big_leds), None

        sx1, sy1, sx2, sy2 = strip_rect
        strip_roi = frame_bgr[sy1:sy2, sx1:sx2].copy()
        if strip_roi.size == 0:
            self._reset_lock_candidate()
            return '00000', 'waiting strip ROI', False, len(big_leds), None

        gray_strip = cv2.cvtColor(strip_roi, cv2.COLOR_BGR2GRAY)
        gray_strip = cv2.GaussianBlur(gray_strip, (5, 5), 0)
        strip_leds, _strip_mask = self._detect_leds_in_box(gray_strip, self._led_thresh)
        strip_leds = self._refine_led_candidates(strip_leds)

        if len(strip_leds) != self._led_count:
            self._reset_lock_candidate()
            label = f'waiting clean 5 LEDs in strip, detected {len(strip_leds)}'
            return '00000', label, False, len(strip_leds), None

        lock_ready, stable_sec = self._update_lock_candidate(strip_rect)
        if not lock_ready:
            label = (
                f'waiting stable {stable_sec:.1f}/'
                f'{self._bbox_lock_confirm_sec:.1f}s'
            )
            return '11111', label, True, len(strip_leds), (sx1, sy1, sx2 - sx1, sy2 - sy1)

        self._lock_strip(strip_rect, strip_leds)
        return '11111', 'car detected', True, len(strip_leds), (sx1, sy1, sx2 - sx1, sy2 - sy1)

    def _wait_pass_redetect_result(
        self,
        frame_bgr: np.ndarray,
    ) -> Optional[Tuple[str, str, bool, int, Optional[Tuple[int, int, int, int]]]]:
        if self._driving_mode != 'WAIT_FOR_PASS':
            return None

        self._wait_pass_frame_count += 1
        redetect_now = (
            self._wait_pass_frame_count % self._wait_pass_redetect_interval_frames == 0
        )
        if not redetect_now:
            if self._wait_pass_forced_unknown:
                return '00000', 'wait pass bbox missing', False, 0, None
            return None

        box, _dark_mask = self._find_black_box(frame_bgr, self._dark_box_thresh)
        if box is not None:
            if self._wait_pass_redetect_misses >= self._wait_pass_redetect_miss_limit:
                self.get_logger().info('WAIT_FOR_PASS bbox redetected; canceling UNKNOWN hold')
            self._wait_pass_redetect_misses = 0
            self._wait_pass_forced_unknown = False

            tracked_rect, tracked_leds = self._find_strip_from_black_box(frame_bgr)
            if tracked_rect is not None and len(tracked_leds) == self._led_count:
                self._update_locked_strip(tracked_rect, tracked_leds)
            return None

        self._wait_pass_redetect_misses += 1
        if self._wait_pass_redetect_misses >= self._wait_pass_redetect_miss_limit:
            if not self._wait_pass_forced_unknown:
                self.get_logger().info(
                    f'WAIT_FOR_PASS bbox missing '
                    f'{self._wait_pass_redetect_misses}/'
                    f'{self._wait_pass_redetect_miss_limit}; publishing UNKNOWN'
                )
            self._wait_pass_forced_unknown = True
            self._mask_votes.clear()
            self._opponent_latch_until = 0.0
            return '00000', 'wait pass bbox missing', False, 0, None

        return None

    def _read_locked_strip(
        self,
        frame_bgr: np.ndarray,
    ) -> Tuple[str, str, bool, int, Optional[Tuple[int, int, int, int]]]:
        if self._track_locked_strip and not self._freeze_locked_bbox:
            tracked_rect, tracked_leds = self._find_strip_from_black_box(frame_bgr)
            if tracked_rect is not None and len(tracked_leds) == self._led_count:
                self._update_locked_strip(tracked_rect, tracked_leds)
                sx1, sy1, sx2, sy2 = tracked_rect
                return (
                    '11111',
                    'car detected',
                    True,
                    len(tracked_leds),
                    (sx1, sy1, sx2 - sx1, sy2 - sy1),
                )

        strip_rect = self._clamp_strip_rect(self._fixed_strip_rect, frame_bgr.shape)
        if strip_rect is None:
            self._reset_slot_lock()
            return '00000', 'waiting box', False, 0, None

        x1, y1, x2, y2 = strip_rect
        strip_roi = frame_bgr[y1:y2, x1:x2].copy()
        geometry_box = (x1, y1, x2 - x1, y2 - y1)
        if strip_roi.size == 0:
            return '00000', 'normal', True, 0, geometry_box

        gray_strip = cv2.cvtColor(strip_roi, cv2.COLOR_BGR2GRAY)
        gray_strip = cv2.GaussianBlur(gray_strip, (5, 5), 0)
        strip_leds, _led_mask = self._detect_leds_in_box(gray_strip, self._led_thresh)
        strip_leds = self._refine_led_candidates(strip_leds)

        led_centers_x = sorted([led[0] for led in strip_leds])
        pattern = self._pattern_from_detected_leds(led_centers_x, self._slot_x_positions)
        mask = ''.join(str(bit) for bit in pattern)
        if mask == '00000' and time.monotonic() < self._opponent_latch_until:
            return '11111', 'car detected latch', True, len(strip_leds), geometry_box
        return mask, self.PATTERN_LABELS.get(mask, 'unknown'), True, len(strip_leds), geometry_box

    def _lock_strip(
        self,
        strip_rect: Tuple[int, int, int, int],
        strip_leds: List[Tuple[int, int, int, int, int, int, float]],
    ) -> None:
        self._fixed_strip_rect = strip_rect
        self._slot_x_positions = np.asarray(
            sorted([led[0] for led in strip_leds]), dtype=np.float32)
        self._slot_y_position = float(np.median([led[1] for led in strip_leds]))
        self._slot_locked = True
        self._reset_lock_candidate()
        slots = [round(float(x), 1) for x in np.sort(self._slot_x_positions)]
        self.get_logger().info(
            f'LED bbox locked/frozen: rect={tuple(int(v) for v in strip_rect)}, slots={slots}')

    def _update_locked_strip(
        self,
        strip_rect: Tuple[int, int, int, int],
        strip_leds: List[Tuple[int, int, int, int, int, int, float]],
    ) -> None:
        self._fixed_strip_rect = strip_rect
        self._slot_x_positions = np.asarray(
            sorted([led[0] for led in strip_leds]), dtype=np.float32)
        self._slot_y_position = float(np.median([led[1] for led in strip_leds]))

    def _update_lock_candidate(self, strip_rect: Tuple[int, int, int, int]) -> Tuple[bool, float]:
        now = time.monotonic()
        if (
            self._candidate_strip_rect is None or
            self._rect_iou(self._candidate_strip_rect, strip_rect) < 0.45
        ):
            self._candidate_strip_rect = strip_rect
            self._candidate_since = now
            return self._bbox_lock_confirm_sec <= 0.0, 0.0

        stable_sec = now - (self._candidate_since or now)
        self._candidate_strip_rect = strip_rect
        return stable_sec >= self._bbox_lock_confirm_sec, stable_sec

    def _reset_lock_candidate(self) -> None:
        self._candidate_strip_rect = None
        self._candidate_since = None

    def _reset_slot_lock(self) -> None:
        self._slot_locked = False
        self._fixed_strip_rect = None
        self._slot_x_positions = None
        self._slot_y_position = None
        self._reset_lock_candidate()

    @staticmethod
    def _rect_iou(
        a: Tuple[int, int, int, int],
        b: Tuple[int, int, int, int],
    ) -> float:
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
        ix1 = max(ax1, bx1)
        iy1 = max(ay1, by1)
        ix2 = min(ax2, bx2)
        iy2 = min(ay2, by2)
        inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
        area_a = max(0, ax2 - ax1) * max(0, ay2 - ay1)
        area_b = max(0, bx2 - bx1) * max(0, by2 - by1)
        union = area_a + area_b - inter
        if union <= 0:
            return 0.0
        return float(inter) / float(union)

    def _make_tight_strip_from_leds(
        self,
        big_box_shape: Tuple[int, ...],
        leds: List[Tuple[int, int, int, int, int, int, float]],
        box_x: int,
        box_y: int,
    ) -> Tuple[int, int, int, int]:
        bh_big, bw_big = big_box_shape[:2]

        led_x1 = min(led[2] for led in leds)
        led_y1 = min(led[3] for led in leds)
        led_x2 = max(led[2] + led[4] for led in leds)
        led_y2 = max(led[3] + led[5] for led in leds)

        led_span_x = max(1, led_x2 - led_x1)
        led_span_y = max(1, led_y2 - led_y1)

        pad_x = int(led_span_x * 0.20)
        pad_y = max(4, int(led_span_y * 0.35))

        sx1 = max(0, led_x1 - pad_x)
        sx2 = min(bw_big, led_x2 + pad_x)
        sy1 = max(0, led_y1 - pad_y)
        sy2 = min(bh_big, led_y2 + pad_y)

        return box_x + sx1, box_y + sy1, box_x + sx2, box_y + sy2

    def _clamp_strip_rect(
        self,
        rect: Optional[Tuple[int, int, int, int]],
        frame_shape: Tuple[int, ...],
    ) -> Optional[Tuple[int, int, int, int]]:
        if rect is None:
            return None

        height, width = frame_shape[:2]
        x1, y1, x2, y2 = rect
        x1 = max(0, min(width - 1, int(x1)))
        x2 = max(1, min(width, int(x2)))
        y1 = max(0, min(height - 1, int(y1)))
        y2 = max(1, min(height, int(y2)))
        if x2 <= x1 or y2 <= y1:
            return None
        return x1, y1, x2, y2

    def _slots_are_valid(self) -> bool:
        return (
            self._slot_locked and
            self._fixed_strip_rect is not None and
            self._slot_x_positions is not None
        )

    def _detect_leds_in_roi(
        self,
        roi_bgr: np.ndarray,
    ) -> Tuple[List[Tuple[int, int, int, int, int, int, float]], np.ndarray]:
        """Detect LED blobs from a simple grayscale binary mask."""
        if roi_bgr.size == 0:
            return [], np.zeros((1, 1), dtype=np.uint8)

        gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        _, led_mask = cv2.threshold(gray, self._led_thresh, 255, cv2.THRESH_BINARY)

        kernel = np.ones((3, 3), np.uint8)
        led_mask = cv2.morphologyEx(led_mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(
            led_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        leds: List[Tuple[int, int, int, int, int, int, float]] = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self._min_led_area or area > self._max_led_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w <= 0 or h <= 0:
                continue

            cx = x + w // 2
            cy = y + h // 2
            leds.append((cx, cy, x, y, w, h, area))

        return sorted(leds, key=lambda item: item[0]), led_mask

    def _find_strip_from_black_box(
        self,
        frame_bgr: np.ndarray,
    ) -> Tuple[Optional[Tuple[int, int, int, int]], List[Tuple[int, int, int, int, int, int, float]]]:
        box, _dark_mask = self._find_black_box(frame_bgr, self._dark_box_thresh)
        if box is None:
            return None, []

        x, y, w, h = box
        big_roi = frame_bgr[y:y + h, x:x + w].copy()
        if big_roi.size == 0:
            return None, []

        gray_big = cv2.cvtColor(big_roi, cv2.COLOR_BGR2GRAY)
        gray_big = cv2.GaussianBlur(gray_big, (5, 5), 0)
        big_leds, _led_mask = self._detect_leds_in_box(gray_big, self._led_thresh)
        big_leds = self._refine_led_candidates(big_leds)
        if len(big_leds) != self._led_count:
            return None, big_leds

        strip_rect = self._make_tight_strip_from_leds(big_roi.shape, big_leds, x, y)
        strip_rect = self._clamp_strip_rect(strip_rect, frame_bgr.shape)
        if strip_rect is None:
            return None, big_leds

        sx1, sy1, sx2, sy2 = strip_rect
        strip_roi = frame_bgr[sy1:sy2, sx1:sx2].copy()
        if strip_roi.size == 0:
            return None, big_leds

        gray_strip = cv2.cvtColor(strip_roi, cv2.COLOR_BGR2GRAY)
        gray_strip = cv2.GaussianBlur(gray_strip, (5, 5), 0)
        strip_leds, _strip_mask = self._detect_leds_in_box(gray_strip, self._led_thresh)
        strip_leds = self._refine_led_candidates(strip_leds)
        if len(strip_leds) != self._led_count:
            return None, strip_leds

        return strip_rect, strip_leds

    def _detect_leds_in_box(
        self,
        gray_box: np.ndarray,
        thresh_val: int,
    ) -> Tuple[List[Tuple[int, int, int, int, int, int, float]], np.ndarray]:
        _, led_mask = cv2.threshold(gray_box, thresh_val, 255, cv2.THRESH_BINARY)

        kernel = np.ones((3, 3), np.uint8)
        led_mask = cv2.morphologyEx(led_mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(
            led_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        leds: List[Tuple[int, int, int, int, int, int, float]] = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self._min_led_area or area > self._max_led_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            cx = x + w // 2
            cy = y + h // 2
            leds.append((cx, cy, x, y, w, h, area))

        return sorted(leds, key=lambda item: item[0]), led_mask

    def _refine_led_candidates(
        self,
        box_leds: List[Tuple[int, int, int, int, int, int, float]],
    ) -> List[Tuple[int, int, int, int, int, int, float]]:
        if not box_leds:
            return []

        ys = np.asarray([led[1] for led in box_leds], dtype=np.float32)
        y_med = float(np.median(ys))

        filtered = [led for led in box_leds if abs(led[1] - y_med) <= self._max_led_y_diff]
        filtered = sorted(filtered, key=lambda item: item[0])

        if len(filtered) <= self._led_count:
            return filtered

        best_group = None
        best_score = 1e18
        for idx in range(len(filtered) - self._led_count + 1):
            group = filtered[idx:idx + self._led_count]
            xs = np.asarray([led[0] for led in group], dtype=np.float32)
            diffs = np.diff(xs)
            if np.any(diffs <= 0):
                continue

            score = float(np.std(diffs))
            if score < best_score:
                best_score = score
                best_group = group

        return best_group if best_group is not None else filtered[:self._led_count]

    def _find_black_box(
        self,
        frame_bgr: np.ndarray,
        thresh_val: int,
    ) -> Tuple[Optional[Tuple[int, int, int, int]], np.ndarray]:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        _, dark_mask = cv2.threshold(gray, thresh_val, 255, cv2.THRESH_BINARY_INV)

        kernel_close = np.ones((9, 9), np.uint8)
        kernel_open = np.ones((5, 5), np.uint8)
        dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_CLOSE, kernel_close)
        dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_OPEN, kernel_open)

        contours, _ = cv2.findContours(
            dark_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        height, width = frame_bgr.shape[:2]
        best_box = None
        best_score = -1e18

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self._min_box_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            aspect = w / float(max(h, 1))
            if aspect < self._min_box_aspect or aspect > self._max_box_aspect:
                continue

            rect_area = w * h
            fill_ratio = area / float(max(rect_area, 1))
            if fill_ratio < self._min_box_fill_ratio:
                continue

            roi = frame_bgr[y:y + h, x:x + w]
            if roi.size == 0:
                continue

            gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            gray_roi = cv2.GaussianBlur(gray_roi, (5, 5), 0)
            leds_in_box, _ = self._detect_leds_in_box(gray_roi, self._led_thresh)
            leds_in_box = self._refine_led_candidates(leds_in_box)

            led_count = len(leds_in_box)
            if led_count < self._min_leds_for_box:
                continue

            cx = x + w / 2.0
            cy = y + h / 2.0
            center_penalty = abs(cx - width / 2.0) + abs(cy - height / 2.0)
            score = led_count * 100000.0 + area - 2.0 * center_penalty

            if score > best_score:
                best_score = score
                best_box = (x, y, w, h)

        return best_box, dark_mask

    def _pattern_from_detected_leds(
        self,
        led_centers_x: List[int],
        slot_x_positions: np.ndarray,
    ) -> Tuple[int, int, int, int, int]:
        slots = [0, 0, 0, 0, 0]
        if slot_x_positions is None:
            return tuple(slots)

        slot_xs = np.sort(np.asarray(slot_x_positions, dtype=np.float32))
        led_centers = sorted([float(x) for x in led_centers_x])

        if len(slot_xs) >= 2:
            step = float(np.median(np.diff(slot_xs)))
        else:
            step = 30.0
        tol = max(self._slot_tol_pixels, int(step * self._slot_tol_ratio))

        used_slots = set()
        for lx in led_centers:
            dists = np.abs(slot_xs - lx)
            nearest_slot = int(np.argmin(dists))
            nearest_dist = float(dists[nearest_slot])

            if nearest_dist <= tol and nearest_slot not in used_slots:
                slots[nearest_slot] = 1
                used_slots.add(nearest_slot)

        return tuple(slots)

    def _stable_mask(self) -> Tuple[str, float]:
        if not self._mask_votes:
            return '00000', 0.0
        counts = Counter(self._mask_votes)
        mask, count = counts.most_common(1)[0]
        return mask, float(count) / float(len(self._mask_votes))

    def _classify_mask(self, mask: str) -> str:
        if mask in self.SIGN_BY_MASK:
            return self.SIGN_BY_MASK[mask]
        if '1' in mask:
            return 'YIELD_VALUE'
        return 'UNKNOWN'

    def _update_geometry(
        self,
        frame: np.ndarray,
        box: Optional[Tuple[int, int, int, int]],
    ) -> None:
        if box is None:
            return

        x, y, w, h = box
        frame_h, frame_w = frame.shape[:2]
        self._last_bbox = [
            round(float(x), 1),
            round(float(y), 1),
            round(float(x + w), 1),
            round(float(y + h), 1),
        ]
        self._last_bbox_area_px = int(max(0, w) * max(0, h))
        self._last_bbox_right_bottom = (float(x + w), float(y + h))

        if self._slot_x_positions is not None:
            xs = self._slot_x_positions.astype(np.float32)
            span_px = float(np.max(xs) - np.min(xs))
            center_x = float(x) + float(np.mean(xs))
            if span_px > 1.0:
                self._last_distance_m = (self._panel_width_m * self._focal_px) / span_px
        else:
            center_x = float(x) + w / 2.0

        self._last_angle_rad = math.atan2(center_x - frame_w / 2.0, self._focal_px)

    def _clear_geometry(self) -> None:
        self._last_bbox = []
        self._last_bbox_area_px = 0
        self._last_bbox_right_bottom = (0.0, 0.0)

    def _publish(
        self,
        mask: str,
        sign: str,
        label: str,
        confidence: float,
        box_detected: bool,
        now: float,
        led_count: int = 0,
    ) -> None:
        bbox_valid = box_detected and self._last_bbox_area_px > 0
        valid_signal = mask != '00000' and sign != 'UNKNOWN'
        detected = (bbox_valid and valid_signal) or now < self._opponent_latch_until
        bbox_area_px = self._last_bbox_area_px if bbox_valid else 0
        bbox_right_bottom = self._last_bbox_right_bottom if bbox_valid else (0.0, 0.0)
        value = int(mask, 2) if len(mask) == self._led_count else 0
        legacy = String()
        legacy.data = sign
        self.pub_led_state.publish(legacy)

        sign_msg = String()
        sign_msg.data = json.dumps({
            'detected': detected,
            'sign': sign,
            'label': label,
            'mask': mask,
            'value': value,
            'confidence': round(confidence, 3),
            'angle_rad': round(self._last_angle_rad, 4),
            'bbox': self._last_bbox if bbox_valid else [],
            'bbox_area_px': bbox_area_px,
            'bbox_right_bottom_x': round(bbox_right_bottom[0], 1),
            'bbox_right_bottom_y': round(bbox_right_bottom[1], 1),
            'box_detected': bbox_valid,
            'slot_locked': self._slots_are_valid(),
            'led_count': led_count,
        })
        self.pub_v2v_sign.publish(sign_msg)

        opp_msg = String()
        opp_msg.data = json.dumps({
            'detected': detected,
            'distance_m': round(self._last_distance_m, 3) if detected else 0.0,
            'angle_rad': round(self._last_angle_rad, 4) if detected else 0.0,
            'bbox': self._last_bbox if bbox_valid else [],
            'bbox_area_px': bbox_area_px,
            'bbox_right_bottom': [
                round(bbox_right_bottom[0], 1),
                round(bbox_right_bottom[1], 1),
            ] if bbox_valid else [],
            'confidence': round(confidence, 3) if detected else 0.0,
            'source': 'led_panel',
            'led_mask': mask,
            'led_label': label,
            'v2v_sign': sign,
            'box_detected': bbox_valid,
            'slot_locked': self._slots_are_valid(),
            'led_count': led_count,
        })
        self.pub_opponent.publish(opp_msg)

    def _show_view(
        self,
        lane_gray_source: np.ndarray,
        filtered_frame: np.ndarray,
        mask: str,
        sign: str,
        label: str,
        confidence: float,
        box_detected: bool,
        led_count: int,
        geometry_box: Optional[Tuple[int, int, int, int]],
    ) -> None:
        if not self._show_debug_view:
            return

        _, led_binary = self._detect_leds_in_roi(filtered_frame)
        raw_view = cv2.cvtColor(led_binary, cv2.COLOR_GRAY2BGR)
        view = filtered_frame.copy()
        status_color = (0, 220, 0) if box_detected else (0, 180, 255)

        if geometry_box is not None:
            x, y, w, h = geometry_box
            cv2.rectangle(view, (int(x), int(y)), (int(x + w), int(y + h)), status_color, 2)
            cv2.rectangle(raw_view, (int(x), int(y)), (int(x + w), int(y + h)), status_color, 2)

        strip_rect = self._clamp_strip_rect(self._fixed_strip_rect, view.shape)
        if strip_rect is not None:
            x1, y1, x2, y2 = strip_rect
            cv2.rectangle(view, (x1, y1), (x2, y2), (255, 180, 0), 2)
            cv2.rectangle(raw_view, (x1, y1), (x2, y2), (255, 180, 0), 2)
            if self._slot_x_positions is not None:
                slot_y = int(self._slot_y_position if self._slot_y_position is not None else (y2 - y1) * 0.5)
                for idx, slot_x in enumerate(np.sort(self._slot_x_positions)):
                    bit = mask[idx] if idx < len(mask) else '0'
                    color = (0, 255, 0) if bit == '1' else (80, 80, 255)
                    center = (int(x1 + slot_x), int(y1 + slot_y))
                    cv2.circle(view, center, 8, color, 2)
                    cv2.circle(raw_view, center, 8, color, 2)
                    cv2.putText(
                        view,
                        str(idx + 1),
                        (center[0] - 5, center[1] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.32,
                        color,
                        1,
                        cv2.LINE_AA,
                    )

        lines = [
            f'{mask} {sign} leds={led_count} conf={confidence:.2f}',
            f'area={self._last_bbox_area_px}px',
        ]
        y = 16
        cv2.putText(
            raw_view,
            'LED bin',
            (8, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            view,
            'LED ROI',
            (8, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        y += 18
        for line in lines:
            cv2.putText(
                view,
                line,
                (8, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                view,
                line,
                (8, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                (20, 20, 20),
                1,
                cv2.LINE_AA,
            )
            y += 16

        try:
            if raw_view.shape[:2] != view.shape[:2]:
                raw_view = cv2.resize(raw_view, (view.shape[1], view.shape[0]))
            combined = np.hstack((raw_view, view))
            cv2.imshow(self._debug_window_name, combined)
            cv2.waitKey(1)
        except cv2.error as exc:
            self.get_logger().warn(f'LED debug view disabled: {exc}')
            self._show_debug_view = False


def main(args=None):
    rclpy.init(args=args)
    node = LEDSignalPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if getattr(node, '_show_debug_view', False):
                cv2.destroyWindow(node._debug_window_name)
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
