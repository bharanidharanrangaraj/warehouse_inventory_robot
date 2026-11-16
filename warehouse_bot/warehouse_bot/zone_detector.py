#!/usr/bin/env python3
"""Zone detector - answers "which inventory zone is the robot in right now?"

The node looks up the robot pose (map -> base_footprint) from TF, tests it
against the oriented zone rectangles defined in the layout file and publishes:

  /warehouse/current_zone  warehouse_interfaces/ZoneStatus   every 1/rate s
  /warehouse/zone_events   warehouse_interfaces/ZoneStatus   on ENTER / EXIT

Parameters
  layout_file   path to warehouse_layout.yaml ('' = installed default)
  global_frame  map frame (default 'map')
  robot_frame   robot base frame (default 'base_footprint')
  rate          update rate in Hz (default 5.0)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from warehouse_interfaces.msg import ZoneStatus
from warehouse_bot.layout import WarehouseLayout, quaternion_to_yaw, resolve_layout_path


class ZoneDetector(Node):

    def __init__(self):
        super().__init__('zone_detector')
        self.declare_parameter('layout_file', '')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('rate', 5.0)

        path = resolve_layout_path(self.get_parameter('layout_file').value)
        self.layout = WarehouseLayout.from_file(path)
        self.global_frame = self.get_parameter('global_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        rate = max(0.5, float(self.get_parameter('rate').value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.status_pub = self.create_publisher(ZoneStatus, '/warehouse/current_zone', 10)
        # Transient local so late subscribers still get the recent ENTER/EXIT history
        events_qos = QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.event_pub = self.create_publisher(ZoneStatus, '/warehouse/zone_events', events_qos)

        self.current_zone_id = ''
        self.create_timer(1.0 / rate, self.update)
        self.get_logger().info(
            f'Loaded {len(self.layout.zones)} zones from {path}: '
            + ', '.join(f'{z.id} ({z.name})' for z in self.layout.zones.values()))

    # ------------------------------------------------------------------ #
    def find_zone(self, x: float, y: float):
        for zone in self.layout.zones.values():
            if zone.contains(x, y):
                return zone
        return None

    def make_msg(self, zone, event: str, x: float, y: float, yaw: float) -> ZoneStatus:
        msg = ZoneStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame
        msg.zone_id = zone.id if zone else ''
        msg.zone_name = zone.name if zone else ''
        msg.zone_type = zone.kind if zone else ''
        msg.event = event
        msg.robot_x, msg.robot_y, msg.robot_yaw = float(x), float(y), float(yaw)
        return msg

    def update(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, Time())
        except TransformException as exc:
            self.get_logger().warn(
                f'Waiting for {self.global_frame} -> {self.robot_frame} transform ({exc})',
                throttle_duration_sec=10.0)
            return

        t, q = tf.transform.translation, tf.transform.rotation
        x, y = t.x, t.y
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        zone = self.find_zone(x, y)
        zone_id = zone.id if zone else ''

        if zone_id != self.current_zone_id:
            if self.current_zone_id:
                old = self.layout.zones[self.current_zone_id]
                self.event_pub.publish(self.make_msg(old, ZoneStatus.EVENT_EXIT, x, y, yaw))
                self.get_logger().info(f'EXIT  zone {old.id} ({old.name})')
            if zone:
                self.event_pub.publish(self.make_msg(zone, ZoneStatus.EVENT_ENTER, x, y, yaw))
                self.get_logger().info(
                    f'ENTER zone {zone.id} ({zone.name}) at ({x:.2f}, {y:.2f})')
            self.current_zone_id = zone_id

        event = ZoneStatus.EVENT_INSIDE if zone else ZoneStatus.EVENT_NONE
        self.status_pub.publish(self.make_msg(zone, event, x, y, yaw))


def main(args=None):
    rclpy.init(args=args)
    node = ZoneDetector()
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
