#!/usr/bin/env python3
"""Inventory manager - compares what the robot saw with the stock records.

Subscribes
  /inventory/scan_results   warehouse_interfaces/ScanResult   (from marker_scanner)
  /warehouse/current_zone   warehouse_interfaces/ZoneStatus   (from zone_detector)
  /warehouse/events         std_msgs/String (JSON)            (from mission_manager)
Publishes
  /inventory/report         warehouse_interfaces/InventoryReport  one per rack + "ALL"
  /inventory/zone_markers   visualization_msgs/MarkerArray        zone colours in RViz
Writes
  <results_dir>/inventory_<timestamp>.json  after every change

RViz colours: grey = not scanned, green = OK, orange = needs attention
(misplaced/unknown/relocated), red = something is missing.
"""

import json
import os
from datetime import datetime

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from warehouse_interfaces.msg import InventoryItem, InventoryReport, ScanResult, ZoneStatus
from warehouse_bot import inventory_logic as logic
from warehouse_bot.layout import WarehouseLayout, resolve_layout_path, yaw_to_quaternion

STATE_COLORS = {
    logic.ZONE_PENDING: (0.6, 0.6, 0.6),
    logic.ZONE_OK: (0.10, 0.80, 0.25),
    logic.ZONE_ATTENTION: (1.00, 0.60, 0.00),
    logic.ZONE_MISSING: (0.90, 0.10, 0.10),
    'DROP': (0.15, 0.45, 0.95),
    'DOCK': (0.00, 0.80, 0.85),
}


def rgba(rgb, a=1.0) -> ColorRGBA:
    return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=float(a))


class InventoryManager(Node):

    def __init__(self):
        super().__init__('inventory_manager')
        self.declare_parameter('layout_file', '')
        self.declare_parameter('results_dir', '~/warehouse_results')
        self.declare_parameter('frame_id', 'map')

        self.layout = WarehouseLayout.from_file(
            resolve_layout_path(self.get_parameter('layout_file').value))
        self.frame_id = self.get_parameter('frame_id').value
        results_dir = os.path.expanduser(self.get_parameter('results_dir').value)
        os.makedirs(results_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.report_path = os.path.join(results_dir, f'inventory_{stamp}.json')

        self.scans = {}            # zone_id -> set of detected marker ids
        self.relocated = set()     # marker ids taken to the drop station
        self.current_zone = ''
        self.states = logic.evaluate(self.layout.catalog, self.scans, self.relocated)

        self.report_pub = self.create_publisher(InventoryReport, '/inventory/report', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/inventory/zone_markers', 10)
        self.create_subscription(ScanResult, '/inventory/scan_results', self.on_scan, 10)
        self.create_subscription(ZoneStatus, '/warehouse/current_zone', self.on_zone, 10)
        self.create_subscription(String, '/warehouse/events', self.on_event, 10)
        self.create_timer(1.0, self.publish_markers)
        self.create_timer(2.0, self.publish_reports)

        self.get_logger().info(f'Inventory manager ready: {len(self.layout.catalog)} catalogue '
                               f'items in {len(self.layout.rack_zones())} rack zones. '
                               f'Report file: {self.report_path}')

    # ------------------------------------------------------------------ #
    #  Inputs
    # ------------------------------------------------------------------ #
    def on_scan(self, msg: ScanResult) -> None:
        if msg.zone_id not in self.layout.zones:
            self.get_logger().warn(f'Scan result for unknown zone "{msg.zone_id}" ignored')
            return
        self.scans[msg.zone_id] = set(int(i) for i in msg.detected_ids)
        self.refresh()
        counts = logic.zone_counts(self.states, msg.zone_id)
        state = logic.zone_state(self.states, msg.zone_id, True)
        self.get_logger().info(
            f'Zone {msg.zone_id} -> {state}: {counts["ok"]}/{counts["expected"]} OK, '
            f'{counts["missing"]} missing, {counts["misplaced"]} misplaced, '
            f'{counts["unknown"]} unknown')
        for s in logic.zone_items(self.states, msg.zone_id):
            if s.status != logic.STATUS_OK:
                self.get_logger().info(
                    f'   tag {s.marker_id:>2} {s.sku:<14} {s.status:<9} '
                    f'(expected {s.expected_zone or "-"}, found {s.found_zone or "-"})')

    def on_zone(self, msg: ZoneStatus) -> None:
        self.current_zone = msg.zone_id

    def on_event(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if event.get('event') == 'DROP' and 'marker_id' in event:
            self.relocated.add(int(event['marker_id']))
            self.get_logger().info(f'Tag {event["marker_id"]} relocated to the drop station')
            self.refresh()

    # ------------------------------------------------------------------ #
    #  Outputs
    # ------------------------------------------------------------------ #
    def refresh(self) -> None:
        self.states = logic.evaluate(self.layout.catalog, self.scans, self.relocated)
        self.publish_reports()
        self.publish_markers()
        self.write_report()

    def _report(self, zone_id: str, items, counts: dict, state: str) -> InventoryReport:
        msg = InventoryReport()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.zone_id = zone_id
        msg.zone_state = state
        for s in items:
            msg.items.append(InventoryItem(
                marker_id=int(s.marker_id), sku=s.sku, name=s.name, quantity=int(s.quantity),
                expected_zone=s.expected_zone, found_zone=s.found_zone, status=s.status))
        msg.expected_count = counts['expected']
        msg.ok_count = counts['ok']
        msg.missing_count = counts['missing']
        msg.misplaced_count = counts['misplaced']
        msg.unknown_count = counts['unknown']
        msg.relocated_count = counts['relocated']
        msg.accuracy = float(counts['accuracy'])
        return msg

    def publish_reports(self) -> None:
        for zone in self.layout.rack_zones():
            items = logic.zone_items(self.states, zone.id)
            counts = logic.zone_counts(self.states, zone.id)
            state = logic.zone_state(self.states, zone.id, zone.id in self.scans)
            self.report_pub.publish(self._report(zone.id, items, counts, state))
        self.report_pub.publish(self._report('ALL', self.states,
                                             logic.zone_counts(self.states), ''))

    def publish_markers(self) -> None:
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for idx, zone in enumerate(self.layout.zones.values()):
            if zone.kind == 'RACK':
                state = logic.zone_state(self.states, zone.id, zone.id in self.scans)
                c = logic.zone_counts(self.states, zone.id)
                text = f'{zone.id} - {zone.name}\n{c["ok"]}/{c["expected"]} OK'
                extras = [f'{c[k]} {k}' for k in ('missing', 'misplaced', 'unknown', 'relocated')
                          if c[k]]
                if state == logic.ZONE_PENDING:
                    text += '\n(not scanned)'
                elif extras:
                    text += '\n' + ', '.join(extras)
                color = STATE_COLORS[state]
            else:
                color = STATE_COLORS[zone.kind]
                text = zone.name
            active = zone.id == self.current_zone
            qz, qw = yaw_to_quaternion(zone.area.yaw)[2:]

            fill = self._marker('zone_fill', idx, Marker.CUBE, stamp)
            fill.pose.position.x, fill.pose.position.y, fill.pose.position.z = \
                zone.area.x, zone.area.y, 0.01
            fill.pose.orientation.z, fill.pose.orientation.w = qz, qw
            fill.scale.x, fill.scale.y, fill.scale.z = zone.area.size_x, zone.area.size_y, 0.02
            fill.color = rgba(color, 0.45 if active else 0.30)
            arr.markers.append(fill)

            outline = self._marker('zone_outline', idx, Marker.LINE_STRIP, stamp)
            outline.pose.orientation.w = 1.0
            corners = zone.area.corners()
            outline.points = [Point(x=cx, y=cy, z=0.03) for cx, cy in corners + corners[:1]]
            outline.scale.x = 0.06 if active else 0.025
            outline.color = rgba((1.0, 1.0, 1.0) if active else color, 1.0)
            arr.markers.append(outline)

            label = self._marker('zone_label', idx, Marker.TEXT_VIEW_FACING, stamp)
            label.pose.position.x, label.pose.position.y, label.pose.position.z = \
                zone.area.x, zone.area.y, 0.8
            label.pose.orientation.w = 1.0
            label.scale.z = 0.18
            label.color = rgba((1.0, 1.0, 1.0), 1.0)
            label.text = text
            arr.markers.append(label)

            goal = self._marker('scan_pose', idx, Marker.ARROW, stamp)
            gqz, gqw = yaw_to_quaternion(zone.goal.yaw)[2:]
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = \
                zone.goal.x, zone.goal.y, 0.05
            goal.pose.orientation.z, goal.pose.orientation.w = gqz, gqw
            goal.scale.x, goal.scale.y, goal.scale.z = 0.35, 0.06, 0.06
            goal.color = rgba(color, 0.9)
            arr.markers.append(goal)
        self.marker_pub.publish(arr)

    def _marker(self, ns: str, mid: int, mtype: int, stamp) -> Marker:
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        return m

    def write_report(self) -> None:
        zones = {}
        for zone in self.layout.rack_zones():
            zones[zone.id] = {
                'name': zone.name,
                'state': logic.zone_state(self.states, zone.id, zone.id in self.scans),
                'detected_ids': sorted(self.scans.get(zone.id, [])),
                **logic.zone_counts(self.states, zone.id),
            }
        data = {
            'warehouse': self.layout.name,
            'updated': datetime.now().isoformat(timespec='seconds'),
            'summary': logic.zone_counts(self.states),
            'zones': zones,
            'items': [s.to_dict() for s in self.states],
        }
        try:
            with open(self.report_path, 'w') as f:
                json.dump(data, f, indent=2)
        except OSError as exc:
            self.get_logger().warn(f'Could not write {self.report_path}: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = InventoryManager()
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
