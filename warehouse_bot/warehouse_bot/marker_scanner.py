#!/usr/bin/env python3
"""Marker scanner - reads the inventory tags on a rack.

Two scan modes:
  aruco     (default) - the robot's camera reads the ArUco tags stuck on each
                        stock bin. A tag must be seen in >= min_detections
                        frames to count, which filters out one-frame glitches.
  rfid_sim            - simulated RFID reader: every tag within rfid_range of
                        the robot is read with probability rfid_read_probability.
                        Useful as a fallback when the camera is unavailable.

Interface
  service   /inventory/scan_zone     warehouse_interfaces/srv/ScanZone
  publishes /inventory/scan_results  warehouse_interfaces/msg/ScanResult
            /inventory/debug_image   sensor_msgs/Image (annotated, during scans)

The service blocks for the scan window while image callbacks keep arriving,
so the node runs on a MultiThreadedExecutor with separate callback groups.
"""

import array
import random
import threading
import time
from collections import Counter

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener

from warehouse_interfaces.msg import ScanResult
from warehouse_interfaces.srv import ScanZone
from warehouse_bot.layout import WarehouseLayout, resolve_layout_path
from warehouse_bot.vision import HAVE_ARUCO, ArucoReader, cv2, image_to_bgr


class MarkerScanner(Node):

    def __init__(self):
        super().__init__('marker_scanner')
        self.declare_parameter('layout_file', '')
        self.declare_parameter('scan_mode', 'aruco')              # aruco | rfid_sim
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('default_duration', 6.0)
        self.declare_parameter('min_detections', 2)
        self.declare_parameter('max_process_rate', 5.0)           # frames/s analysed
        self.declare_parameter('fallback_to_rfid', True)
        self.declare_parameter('rfid_range', 1.6)
        self.declare_parameter('rfid_read_probability', 0.9)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.layout = WarehouseLayout.from_file(resolve_layout_path(p('layout_file')))
        self.scan_mode = str(p('scan_mode')).lower()
        self.image_topic = p('image_topic')
        self.default_duration = float(p('default_duration'))
        self.min_detections = max(1, int(p('min_detections')))
        self.min_period = 1.0 / max(0.5, float(p('max_process_rate')))
        self.fallback_to_rfid = bool(p('fallback_to_rfid'))
        self.rfid_range = float(p('rfid_range'))
        self.rfid_prob = min(1.0, max(0.0, float(p('rfid_read_probability'))))
        self.publish_debug = bool(p('publish_debug_image'))
        self.global_frame = p('global_frame')
        self.robot_frame = p('robot_frame')

        self.reader = None
        if self.scan_mode == 'aruco':
            if HAVE_ARUCO:
                self.reader = ArucoReader(self.layout.aruco_dictionary)
            else:
                self.get_logger().error('OpenCV ArUco module not found - switching to rfid_sim. '
                                        'Install it with: sudo apt install python3-opencv')
                self.scan_mode = 'rfid_sim'
        elif self.scan_mode != 'rfid_sim':
            raise ValueError(f'Unknown scan_mode "{self.scan_mode}" (use aruco or rfid_sim)')

        # Shared scan state (guarded by _lock)
        self._lock = threading.Lock()
        self._scanning = False
        self._counts = Counter()
        self._frames = 0
        self._active_zone = ''
        self._last_processed = 0.0
        self._images_received = 0

        self._service_group = MutuallyExclusiveCallbackGroup()
        self._sensor_group = MutuallyExclusiveCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.result_pub = self.create_publisher(ScanResult, '/inventory/scan_results', 10)
        self.debug_pub = self.create_publisher(Image, '/inventory/debug_image', 2)
        self.image_sub = None
        if self.reader is not None:
            # Subscribe once the camera publisher exists, matching its QoS
            self._sub_timer = self.create_timer(1.0, self._try_subscribe,
                                                callback_group=self._sensor_group)
        self.create_service(ScanZone, '/inventory/scan_zone', self.handle_scan,
                            callback_group=self._service_group)
        self.get_logger().info(f'Marker scanner ready (mode: {self.scan_mode}, '
                               f'dictionary: {self.layout.aruco_dictionary})')

    # ------------------------------------------------------------------ #
    #  Camera handling
    # ------------------------------------------------------------------ #
    def _try_subscribe(self) -> None:
        if self.image_sub is not None:
            return
        infos = self.get_publishers_info_by_topic(self.image_topic)
        if not infos:
            self.get_logger().info(f'Waiting for a camera publisher on {self.image_topic} ...',
                                   throttle_duration_sec=10.0)
            return
        reliability = ReliabilityPolicy.RELIABLE
        if any(i.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT for i in infos):
            reliability = ReliabilityPolicy.BEST_EFFORT
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=reliability)
        self.image_sub = self.create_subscription(Image, self.image_topic, self._on_image, qos,
                                                  callback_group=self._sensor_group)
        self._sub_timer.cancel()
        self.get_logger().info(f'Subscribed to {self.image_topic} ({reliability.name})')

    def _on_image(self, msg: Image) -> None:
        self._images_received += 1
        now = time.monotonic()
        with self._lock:
            if not self._scanning or now - self._last_processed < self.min_period:
                return
            self._last_processed = now
            zone = self._active_zone
        try:
            bgr = image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=10.0)
            return
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, id_list, mirrored = self.reader.detect(gray)
        with self._lock:
            if not self._scanning:
                return
            self._frames += 1
            self._counts.update(set(id_list))
        if self.publish_debug:
            self._publish_debug(bgr, corners, ids, id_list, mirrored, zone, msg.header)

    def _publish_debug(self, bgr, corners, ids, id_list, mirrored, zone, header) -> None:
        img = ArucoReader.draw(bgr, corners, ids)
        if img.shape[1] > 640:                           # keep the debug stream light
            scale = 640.0 / img.shape[1]
            img = cv2.resize(img, (640, int(img.shape[0] * scale)))
        label = f'Zone {zone} | tags: {sorted(set(id_list)) or "-"}'
        if mirrored:
            label += ' (mirrored)'
        cv2.putText(img, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
        cv2.putText(img, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        out = Image()
        out.header = header
        out.height, out.width = int(img.shape[0]), int(img.shape[1])
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = out.width * 3
        out.data = array.array('B', img.tobytes())
        self.debug_pub.publish(out)

    # ------------------------------------------------------------------ #
    #  Simulated RFID
    # ------------------------------------------------------------------ #
    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, Time())
        except TransformException:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def _rfid_sample(self, counts: Counter) -> bool:
        pose = self._robot_xy()
        if pose is None:
            return False
        rx, ry = pose
        for m in self.layout.placed_markers:
            if (m.x - rx) ** 2 + (m.y - ry) ** 2 <= self.rfid_range ** 2 \
                    and random.random() < self.rfid_prob:
                counts[m.marker_id] += 1
        return True

    # ------------------------------------------------------------------ #
    #  Service
    # ------------------------------------------------------------------ #
    def handle_scan(self, request, response):
        zone_id = request.zone_id
        if zone_id not in self.layout.zones:
            response.success = False
            response.message = f'Unknown zone "{zone_id}"'
            return response
        duration = float(request.duration) if request.duration > 0 else self.default_duration
        mode = self.scan_mode
        self.get_logger().info(f'Scanning zone {zone_id} for {duration:.1f} s ({mode})')

        with self._lock:
            self._counts = Counter()
            self._frames = 0
            self._active_zone = zone_id
            self._scanning = (mode == 'aruco')

        rfid_counts = Counter()
        rfid_samples = 0
        start = time.monotonic()
        while time.monotonic() - start < duration:
            if mode == 'rfid_sim':
                rfid_samples += int(self._rfid_sample(rfid_counts))
                time.sleep(0.25)
            else:
                time.sleep(0.05)

        with self._lock:
            self._scanning = False
            counts, frames = Counter(self._counts), self._frames

        note = ''
        if mode == 'rfid_sim':
            counts, frames = rfid_counts, rfid_samples
            min_hits = 1
        else:
            min_hits = self.min_detections
            if frames == 0:
                note = f'no camera frames received on {self.image_topic}'
                if self.fallback_to_rfid:
                    self.get_logger().warn(f'{note} - falling back to simulated RFID')
                    mode, counts, frames, min_hits = 'rfid_sim', Counter(), 0, 1
                    for _ in range(8):
                        frames += int(self._rfid_sample(counts))
                        time.sleep(0.1)
                    note += ' (used rfid_sim fallback)'

        detected = sorted(mid for mid, c in counts.items() if c >= min_hits)
        elapsed = time.monotonic() - start

        result = ScanResult()
        result.header.stamp = self.get_clock().now().to_msg()
        result.header.frame_id = self.global_frame
        result.zone_id = zone_id
        result.scan_mode = mode
        result.detected_ids = detected
        result.frames_processed = int(frames)
        result.duration = float(elapsed)
        self.result_pub.publish(result)

        response.success = frames > 0
        response.detected_ids = detected
        response.message = (f'{len(detected)} tag(s) from {frames} {mode} sample(s)'
                            + (f'; {note}' if note else ''))
        self.get_logger().info(f'Zone {zone_id}: tags {detected} '
                               f'({frames} samples, {elapsed:.1f} s, {mode})')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MarkerScanner()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
