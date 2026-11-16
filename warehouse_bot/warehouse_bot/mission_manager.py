#!/usr/bin/env python3
"""Mission manager - runs a complete inventory audit with Nav2.

Mission
  1. Localise at the charging dock and wait for Nav2.
  2. AUDIT: for every rack zone in zone_order
       - navigate to the zone's scan pose (Nav2 plans around obstacles)
       - confirm the zone detector agrees we are inside the zone
       - sweep-scan: call /inventory/scan_zone and, while it listens, rotate
         +a, -2a, +a so tags at the rack ends also enter the camera view
  3. RELOCATE: for every MISPLACED / UNKNOWN tag, go back to where it was
     found ("PICK"), then deliver it to the pick/drop station ("DROP").
  4. Return to the dock and write a JSON results file with navigation
     accuracy, zone recognition and tag recall/precision per zone.

Uses nav2_simple_commander.BasicNavigator for all motion.
Run with use_sim_time:=true (mission.launch.py does this for you).
"""

import json
import math
import os
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from std_msgs.msg import String

from warehouse_interfaces.msg import InventoryReport, ZoneStatus
from warehouse_interfaces.srv import ScanZone
from warehouse_bot.inventory_logic import FLAGGED_STATUSES
from warehouse_bot.layout import (WarehouseLayout, normalize_angle, resolve_layout_path,
                                  yaw_to_quaternion)


class MissionManager(Node):

    def __init__(self, navigator: BasicNavigator):
        super().__init__('mission_manager')
        self.nav = navigator

        self.declare_parameter('layout_file', '')
        self.layout = WarehouseLayout.from_file(
            resolve_layout_path(self.get_parameter('layout_file').value))
        m = self.layout.mission
        default_order = ','.join(m.get('zone_order', [z.id for z in self.layout.rack_zones()]))

        self.declare_parameter('zone_order', default_order)
        self.declare_parameter('scan_duration', float(m.get('scan_duration', 6.0)))
        self.declare_parameter('sweep_angle', float(m.get('sweep_angle', 0.35)))
        self.declare_parameter('relocate_flagged', bool(m.get('relocate_flagged', True)))
        self.declare_parameter('return_to_dock', bool(m.get('return_to_dock', True)))
        self.declare_parameter('nav_timeout', float(m.get('nav_timeout', 180.0)))
        self.declare_parameter('set_initial_pose', True)
        self.declare_parameter('results_dir', '~/warehouse_results')

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        order = str(p('zone_order')).strip() or default_order
        self.zone_order = [z.strip() for z in order.split(',') if z.strip()]
        bad = [z for z in self.zone_order
               if z not in self.layout.zones or self.layout.zones[z].kind != 'RACK']
        if bad:
            raise ValueError(f'zone_order contains unknown rack zones: {bad}')
        self.scan_duration = float(p('scan_duration'))
        self.sweep_angle = max(0.0, float(p('sweep_angle')))
        self.relocate_flagged = bool(p('relocate_flagged'))
        self.return_to_dock = bool(p('return_to_dock'))
        self.nav_timeout = float(p('nav_timeout'))
        self.set_initial_pose = bool(p('set_initial_pose'))
        self.results_dir = os.path.expanduser(p('results_dir'))

        self.scan_client = self.create_client(ScanZone, '/inventory/scan_zone')
        self.event_pub = self.create_publisher(String, '/warehouse/events', 10)
        self.create_subscription(ZoneStatus, '/warehouse/current_zone', self._on_zone, 10)
        self.create_subscription(InventoryReport, '/inventory/report', self._on_report, 10)

        self.latest_zone = None
        self.latest_summary = None
        self.distance_travelled = 0.0
        self._last_xy = None
        self.audit = []        # one record per audited zone
        self.actions = []      # relocation / docking records
        self.started = None

    # ------------------------------------------------------------------ #
    #  Callbacks and helpers
    # ------------------------------------------------------------------ #
    def _on_zone(self, msg: ZoneStatus) -> None:
        self.latest_zone = msg
        xy = (msg.robot_x, msg.robot_y)
        if self._last_xy is not None:
            step = math.dist(xy, self._last_xy)
            if step < 1.0:                       # ignore localisation jumps
                self.distance_travelled += step
        self._last_xy = xy

    def _on_report(self, msg: InventoryReport) -> None:
        if msg.zone_id == 'ALL':
            self.latest_summary = msg

    def _pose(self, goal) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(goal.x), float(goal.y)
        _, _, qz, qw = yaw_to_quaternion(goal.yaw)
        pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
        return pose

    def _spin_self(self, seconds: float) -> None:
        """Process this node's subscriptions for a while."""
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def _event(self, event: str, **data) -> None:
        payload = {'event': event, 'time': datetime.now().isoformat(timespec='seconds'), **data}
        self.event_pub.publish(String(data=json.dumps(payload)))

    def _wait_for_task(self, timeout: float):
        start = time.monotonic()
        while not self.nav.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.02)
            if time.monotonic() - start > timeout:
                self.get_logger().warn(f'Task exceeded {timeout:.0f} s - cancelling')
                self.nav.cancelTask()
                return TaskResult.CANCELED, time.monotonic() - start
        return self.nav.getResult(), time.monotonic() - start

    # ------------------------------------------------------------------ #
    #  Motion primitives
    # ------------------------------------------------------------------ #
    def _navigate(self, label: str, goal) -> dict:
        self.get_logger().info(f'-> {label}: ({goal.x:.2f}, {goal.y:.2f}, '
                               f'{math.degrees(goal.yaw):.0f} deg)')
        record = {'target': label,
                  'goal': {'x': round(goal.x, 3), 'y': round(goal.y, 3),
                           'yaw_deg': round(math.degrees(goal.yaw), 1)}}
        if not self.nav.goToPose(self._pose(goal)):
            record.update(nav_result='REJECTED', nav_time_s=0.0)
            return record
        result, elapsed = self._wait_for_task(self.nav_timeout)
        self._spin_self(0.6)                     # let the zone status catch up
        record['nav_result'] = getattr(result, 'name', str(result))
        record['nav_time_s'] = round(elapsed, 2)
        z = self.latest_zone
        if z is not None:
            record['final_pose'] = {'x': round(z.robot_x, 3), 'y': round(z.robot_y, 3),
                                    'yaw_deg': round(math.degrees(z.robot_yaw), 1)}
            record['position_error_m'] = round(math.hypot(z.robot_x - goal.x,
                                                          z.robot_y - goal.y), 3)
            record['yaw_error_deg'] = round(
                abs(math.degrees(normalize_angle(z.robot_yaw - goal.yaw))), 2)
        return record

    def _rotate(self, angle: float) -> bool:
        if not self.nav.spin(spin_dist=angle, time_allowance=10):
            return False
        result, _ = self._wait_for_task(15.0)
        return result == TaskResult.SUCCEEDED

    def _sweep_scan(self, zone_id: str):
        request = ScanZone.Request()
        request.zone_id = zone_id
        request.duration = float(self.scan_duration)
        future = self.scan_client.call_async(request)
        if self.sweep_angle > 0.0:
            a = self.sweep_angle
            for angle in (a, -2.0 * a, a):       # ends facing the rack again
                if not self._rotate(angle):
                    self.get_logger().warn('Sweep rotation did not complete')
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.scan_duration + 15.0)
        return future.result() if future.done() else None

    # ------------------------------------------------------------------ #
    #  Mission phases
    # ------------------------------------------------------------------ #
    def _audit_zone(self, zone) -> None:
        self.get_logger().info(f'===== Audit zone {zone.id}: {zone.name} =====')
        record = {'zone': zone.id, 'name': zone.name}
        record.update(self._navigate(zone.id, zone.goal))
        truth = self.layout.markers_in_zone(zone.id)
        record['truth_ids'] = truth

        if record['nav_result'] != 'SUCCEEDED':
            self.get_logger().error(f'Could not reach zone {zone.id} ({record["nav_result"]})')
            record.update(zone_recognized=False, scan_ok=False, detected_ids=[])
            self.audit.append(record)
            return

        recognised = self.latest_zone is not None and self.latest_zone.zone_id == zone.id
        record['zone_recognized'] = recognised
        if not recognised:
            seen = self.latest_zone.zone_id if self.latest_zone else 'none'
            self.get_logger().warn(f'Zone detector reports "{seen}", expected {zone.id}')

        self._event('SCAN_START', zone=zone.id)
        response = self._sweep_scan(zone.id)
        detected = sorted(int(i) for i in response.detected_ids) if response else []
        record['scan_ok'] = bool(response and response.success)
        record['scan_message'] = response.message if response else 'no response from scanner'
        record['detected_ids'] = detected
        hits = len(set(detected) & set(truth))
        record['tag_recall'] = round(hits / len(truth), 3) if truth else 1.0
        record['tag_precision'] = round(hits / len(detected), 3) if detected else \
            (1.0 if not truth else 0.0)
        self.get_logger().info(f'Zone {zone.id}: detected {detected}, ground truth {truth}')
        self.audit.append(record)

    def _relocate(self, item) -> None:
        source = self.layout.zones.get(item.found_zone)
        if source is None:
            return
        self.get_logger().info(f'===== Relocate tag {item.marker_id} ({item.status}) '
                               f'from zone {source.id} =====')
        rec = {'action': 'RELOCATE', 'marker_id': int(item.marker_id), 'status': item.status,
               'from_zone': source.id}
        pick = self._navigate(f'PICK@{source.id}', source.goal)
        rec['pick'] = pick
        if pick.get('nav_result') != 'SUCCEEDED':
            rec['completed'] = False
            self.actions.append(rec)
            return
        self._event('PICK', marker_id=int(item.marker_id), sku=item.sku, zone=source.id)
        self._spin_self(2.0)                     # simulated gripper time
        drop_zone = self.layout.zones['DROP']
        drop = self._navigate('DROP', drop_zone.goal)
        rec['drop'] = drop
        rec['completed'] = drop.get('nav_result') == 'SUCCEEDED'
        if rec['completed']:
            self._event('DROP', marker_id=int(item.marker_id), sku=item.sku, zone='DROP')
            self._spin_self(2.0)
        self.actions.append(rec)

    def run(self) -> bool:
        self.started = time.monotonic()
        dock = self.layout.dock
        if self.set_initial_pose:
            self.get_logger().info('Setting the initial pose at the charging dock')
            self.nav.setInitialPose(self._pose(dock))
        self.get_logger().info('Waiting for Nav2 to become active ...')
        self.nav.waitUntilNav2Active()

        if not self.scan_client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error('/inventory/scan_zone is not available - '
                                    'start inventory.launch.py (or bringup.launch.py) first')
            return False

        self.get_logger().info(f'Mission start - zone order: {self.zone_order}')
        self._event('MISSION_START', zones=self.zone_order)

        for zone_id in self.zone_order:                         # phase 1: audit
            self._audit_zone(self.layout.zones[zone_id])

        self._spin_self(2.5)                                    # wait for the summary
        flagged = []
        if self.latest_summary is not None:
            flagged = [i for i in self.latest_summary.items if i.status in FLAGGED_STATUSES]
        if flagged:
            self.get_logger().info('Flagged tags: ' + ', '.join(
                f'{i.marker_id} ({i.status} in {i.found_zone})' for i in flagged))
        if self.relocate_flagged:                               # phase 2: relocate
            for item in flagged:
                self._relocate(item)

        if self.return_to_dock:                                 # phase 3: dock
            self.actions.append({'action': 'DOCK', **self._navigate('DOCK', dock)})

        self._event('MISSION_COMPLETE')
        self._spin_self(2.5)
        return True

    # ------------------------------------------------------------------ #
    #  Results
    # ------------------------------------------------------------------ #
    def save_results(self) -> None:
        if not self.audit and not self.actions:
            return
        nav_records = self.audit + [a.get('pick', {}) for a in self.actions] + \
            [a.get('drop', {}) for a in self.actions] + \
            [a for a in self.actions if a.get('action') == 'DOCK']
        nav_records = [r for r in nav_records if 'nav_result' in r]
        ok = [r for r in nav_records if r['nav_result'] == 'SUCCEEDED']
        pos = [r['position_error_m'] for r in ok if 'position_error_m' in r]
        yaw = [r['yaw_error_deg'] for r in ok if 'yaw_error_deg' in r]
        truth = sum(len(r['truth_ids']) for r in self.audit)
        found = sum(len(set(r['detected_ids']) & set(r['truth_ids'])) for r in self.audit)
        detected = sum(len(r['detected_ids']) for r in self.audit)
        summary = {
            'navigation_goals': len(nav_records),
            'navigation_success_rate': round(len(ok) / len(nav_records), 3) if nav_records else 0,
            'mean_position_error_m': round(sum(pos) / len(pos), 3) if pos else None,
            'max_position_error_m': max(pos) if pos else None,
            'mean_yaw_error_deg': round(sum(yaw) / len(yaw), 2) if yaw else None,
            'zone_recognition_rate': round(
                sum(1 for r in self.audit if r.get('zone_recognized')) / len(self.audit), 3)
            if self.audit else 0,
            'tag_recall': round(found / truth, 3) if truth else None,
            'tag_precision': round(found / detected, 3) if detected else None,
            'distance_travelled_m': round(self.distance_travelled, 2),
            'mission_time_s': round(time.monotonic() - self.started, 1) if self.started else None,
        }
        if self.latest_summary is not None:
            s = self.latest_summary
            summary['inventory'] = {
                'expected': s.expected_count, 'ok': s.ok_count, 'missing': s.missing_count,
                'misplaced': s.misplaced_count, 'unknown': s.unknown_count,
                'relocated': s.relocated_count, 'record_accuracy': round(s.accuracy, 3),
                'items': [{'marker_id': i.marker_id, 'sku': i.sku, 'status': i.status,
                           'expected_zone': i.expected_zone, 'found_zone': i.found_zone}
                          for i in s.items]}
        data = {'finished': datetime.now().isoformat(timespec='seconds'),
                'zone_order': self.zone_order, 'scan_duration_s': self.scan_duration,
                'sweep_angle_rad': self.sweep_angle, 'summary': summary,
                'audit': self.audit, 'actions': self.actions}
        os.makedirs(self.results_dir, exist_ok=True)
        path = os.path.join(self.results_dir,
                            f'mission_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json')
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)

        log = self.get_logger()
        log.info('================ MISSION SUMMARY ================')
        for key, value in summary.items():
            if key != 'inventory':
                log.info(f'{key:>26}: {value}')
        for r in self.audit:
            log.info(f'  zone {r["zone"]}: nav={r.get("nav_result")} '
                     f'err={r.get("position_error_m")} m/{r.get("yaw_error_deg")} deg '
                     f'zone_ok={r.get("zone_recognized")} tags={r.get("detected_ids")}')
        log.info(f'Results written to {path}')


def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()
    manager = MissionManager(navigator)
    try:
        manager.run()
    except KeyboardInterrupt:
        manager.get_logger().info('Mission interrupted')
        try:
            navigator.cancelTask()
        except Exception:  # noqa: BLE001 - best effort during shutdown
            pass
    finally:
        manager.save_results()
        manager.destroy_node()
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
