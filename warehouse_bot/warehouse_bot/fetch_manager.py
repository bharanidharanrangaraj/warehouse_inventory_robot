#!/usr/bin/env python3
"""Fetch manager - retrieval mode: brings the component a user orders.

A user picks a component (capacitor, resistor, transformer or diode) on the
order web page, or calls the service directly:

    ros2 service call /warehouse/request_item warehouse_interfaces/srv/RequestItem "{product: diode}"

Orders wait in a FIFO queue. For each order the rover
  1. drives to the product's rack with Nav2 (TO_RACK)
  2. reads the rack's ArUco bin labels with its camera to make sure it is at the
     right rack (CONFIRMING; one retry with a sweep)
  3. creeps in to the arm's working distance, and item_handler's arm picks one
     unit off the shelf into the cargo tray; then it backs out (PICKING)
  4. drives to the user counter (TO_USER), creeps in, and the arm sets the unit
     down on the counter (HANDING_OVER)
When no orders are left it returns to the charging dock (a new order interrupts
that). Every finished order is logged to <results_dir>/fetch_<timestamp>.json.
The short creep in/out is driven directly on /cmd_vel with odometry, because Nav2
keeps the robot's footprint out of the inflated zone right next to the shelves.

Services   /warehouse/request_item  warehouse_interfaces/srv/RequestItem
Publishes  /warehouse/fetch_status  warehouse_interfaces/FetchStatus (2 Hz + on change)
           /warehouse/events        std_msgs/String (JSON)
Subscribes /warehouse/current_zone  warehouse_interfaces/ZoneStatus
           /warehouse/stock         warehouse_interfaces/StockReport
Calls      /inventory/scan_zone, /warehouse/pick_item, /warehouse/place_item

Threading: this node is spun by a MultiThreadedExecutor in a background thread
(so orders are accepted while the robot drives); the main thread runs the
deliveries with nav2_simple_commander's BasicNavigator.
"""

import json
import math
import os
import threading
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from warehouse_interfaces.msg import FetchStatus, OrderRecord, StockReport, ZoneStatus
from warehouse_interfaces.srv import PickItem, RequestItem, ScanZone
from warehouse_bot import fetch_logic as fl
from warehouse_bot.layout import (WarehouseLayout, normalize_angle, resolve_layout_path,
                                  yaw_to_quaternion)

STOCK_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class FetchManager(Node):

    def __init__(self):
        super().__init__('fetch_manager')
        self.declare_parameter('layout_file', '')
        self.layout = WarehouseLayout.from_file(
            resolve_layout_path(self.get_parameter('layout_file').value))
        f, m = self.layout.fetch, self.layout.mission

        defaults = {
            'confirm_with_camera': bool(f.get('confirm_with_camera', True)),
            'require_confirmation': bool(f.get('require_confirmation', True)),
            'confirm_duration': float(f.get('confirm_duration', 3.0)),
            'sweep_angle': float(m.get('sweep_angle', 0.35)),
            'return_to_dock_when_idle': bool(f.get('return_to_dock_when_idle', True)),
            'idle_return_delay': float(f.get('idle_return_delay', 8.0)),
            'max_queue': int(f.get('max_queue', 8)),
            'nav_timeout': float(m.get('nav_timeout', 180.0)),
            'set_initial_pose': True,
            'results_dir': '~/warehouse_results',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.confirm_with_camera = bool(p('confirm_with_camera'))
        self.require_confirmation = bool(p('require_confirmation'))
        self.confirm_duration = float(p('confirm_duration'))
        self.sweep_angle = max(0.0, float(p('sweep_angle')))
        self.return_to_dock = bool(p('return_to_dock_when_idle'))
        self.idle_return_delay = max(0.0, float(p('idle_return_delay')))
        self.nav_timeout = float(p('nav_timeout'))
        self.set_initial_pose = bool(p('set_initial_pose'))
        results_dir = os.path.expanduser(p('results_dir'))
        os.makedirs(results_dir, exist_ok=True)
        self.results_path = os.path.join(
            results_dir, f'fetch_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json')

        self.lock = threading.RLock()
        self.book = fl.OrderBook(self.layout.products, max_queue=int(p('max_queue')))
        self.stock = {pid: prod.units for pid, prod in self.layout.products.items()}
        self.robot_state = FetchStatus.ROBOT_STARTING
        self.status_message = 'Starting up - waiting for Nav2'
        self.latest_zone = None
        self.carrying = None          # item still on the robot after a failed delivery
        self.records = []             # one per finished order (results file)
        self.nav = None
        self.started = time.time()

        cb = ReentrantCallbackGroup()
        self.create_service(RequestItem, '/warehouse/request_item', self._on_request,
                            callback_group=cb)
        self.status_pub = self.create_publisher(FetchStatus, '/warehouse/fetch_status', 10)
        self.event_pub = self.create_publisher(String, '/warehouse/events', 10)
        self.create_subscription(ZoneStatus, '/warehouse/current_zone', self._on_zone, 10,
                                 callback_group=cb)
        self.create_subscription(StockReport, '/warehouse/stock', self._on_stock, STOCK_QOS,
                                 callback_group=cb)
        self.create_subscription(Odometry, '/odom', self._on_odom, 20, callback_group=cb)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = None
        self.scan_client = self.create_client(ScanZone, '/inventory/scan_zone', callback_group=cb)
        self.pick_client = self.create_client(PickItem, '/warehouse/pick_item', callback_group=cb)
        self.place_client = self.create_client(Trigger, '/warehouse/place_item', callback_group=cb)
        self.create_timer(0.5, self.publish_status, callback_group=cb)

        self.get_logger().info(
            'Fetch manager ready. Products: '
            + ', '.join(f'{p.name} (rack {p.rack_id})' for p in self.layout.products.values()))

    # ------------------------------------------------------------------ #
    #  ROS callbacks (background executor thread)
    # ------------------------------------------------------------------ #
    def _on_zone(self, msg: ZoneStatus) -> None:
        self.latest_zone = msg

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.odom = (p.x, p.y)

    def _on_stock(self, msg: StockReport) -> None:
        with self.lock:
            for s in msg.products:
                self.stock[s.product] = int(s.available)

    def _on_request(self, request, response):
        with self.lock:
            pid = self.book.match_product(request.product)
            shelf = self.stock.get(pid, 0) if pid else 0
            accepted, order, message = self.book.request(request.product, shelf)
        response.accepted = accepted
        response.order_id = order.order_id if order else ''
        response.message = message
        if accepted:
            self.get_logger().info(f'New order {message}')
            self._event('ORDER', order_id=order.order_id, product=order.product,
                        rack=order.rack_id)
        else:
            self.get_logger().warn(f'Order for "{request.product}" refused: {message}')
        self.publish_status()
        return response

    def _record(self, order: fl.Order, now: float) -> OrderRecord:
        return OrderRecord(order_id=order.order_id, product=order.product,
                           product_name=order.product_name, rack_id=order.rack_id,
                           state=order.state, message=order.message,
                           requested_at=float(order.requested_at),
                           duration=float(order.duration(now)))

    def publish_status(self) -> None:
        now = time.time()
        msg = FetchStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            msg.robot_state = self.robot_state
            msg.message = self.status_message
            if self.book.current is not None:
                msg.has_current = True
                msg.current = self._record(self.book.current, now)
            msg.queue = [self._record(o, now) for o in self.book.queue]
            msg.history = [self._record(o, now) for o in self.book.history[:10]]
        self.status_pub.publish(msg)

    def _event(self, event: str, **data) -> None:
        payload = {'event': event, 'time': datetime.now().isoformat(timespec='seconds'), **data}
        self.event_pub.publish(String(data=json.dumps(payload)))

    # ------------------------------------------------------------------ #
    #  Helpers used by the worker (main thread)
    # ------------------------------------------------------------------ #
    def _set_state(self, state: str, message: str) -> None:
        with self.lock:
            self.robot_state, self.status_message = state, message
        self.get_logger().info(message)
        self.publish_status()

    def _step(self, state: str, message: str) -> None:
        with self.lock:
            self.book.advance(state, message)
            order_id = self.book.current.order_id
            self.robot_state = FetchStatus.ROBOT_BUSY
            self.status_message = f'{order_id}: {message}'
        self.get_logger().info(f'[{order_id}] {state}: {message}')
        self.publish_status()

    def _call(self, client, request, timeout: float):
        """Blocking service call from the worker thread (the executor completes it)."""
        if not client.wait_for_service(timeout_sec=3.0):
            return None
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            return None
        return future.result()

    def _pose(self, goal) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(goal.x), float(goal.y)
        _, _, pose.pose.orientation.z, pose.pose.orientation.w = yaw_to_quaternion(goal.yaw)
        return pose

    def _navigate(self, label: str, goal, interrupt=None) -> dict:
        self.get_logger().info(f'-> {label}: ({goal.x:.2f}, {goal.y:.2f}, '
                               f'{math.degrees(goal.yaw):.0f} deg)')
        record = {'target': label, 'goal': {'x': round(goal.x, 3), 'y': round(goal.y, 3),
                                            'yaw_deg': round(math.degrees(goal.yaw), 1)}}
        start = time.monotonic()
        if not self.nav.goToPose(self._pose(goal)):
            record.update(nav_result='REJECTED', nav_time_s=0.0)
            return record
        result = None
        while not self.nav.isTaskComplete():          # spins the navigator for 0.1 s
            if interrupt is not None and interrupt():
                self.nav.cancelTask()
                result = 'INTERRUPTED'
                break
            if time.monotonic() - start > self.nav_timeout:
                self.get_logger().warn(f'{label}: no arrival after {self.nav_timeout:.0f} s')
                self.nav.cancelTask()
                result = 'TIMEOUT'
                break
        if result is None:
            r = self.nav.getResult()
            result = getattr(r, 'name', str(r))
        time.sleep(0.5)                               # let the zone status catch up
        record['nav_result'] = result
        record['nav_time_s'] = round(time.monotonic() - start, 2)
        z = self.latest_zone
        if z is not None and result == 'SUCCEEDED':
            record['position_error_m'] = round(math.hypot(z.robot_x - goal.x,
                                                          z.robot_y - goal.y), 3)
            record['yaw_error_deg'] = round(
                abs(math.degrees(normalize_angle(z.robot_yaw - goal.yaw))), 2)
        return record

    def _stop(self) -> None:
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)

    def _approach(self, target, speed: float = 0.08, max_dist: float = 0.6) -> dict:
        """Square up to target's heading, then drive straight until the robot centre is
        level with target (signed distance along the heading). Returns the distance driven,
        so _retreat can undo it."""
        time.sleep(0.8)                       # let Nav2's velocity smoother go quiet
        z = self.latest_zone
        if z is None or self.odom is None:
            return {'driven_m': 0.0, 'note': 'no pose'}
        # 1. rotate to the target heading
        start = time.monotonic()
        while time.monotonic() - start < 8.0:
            z = self.latest_zone
            err = normalize_angle(target.yaw - z.robot_yaw)
            if abs(err) < 0.03:
                break
            t = Twist()
            t.angular.z = max(-0.4, min(0.4, 1.5 * err))
            if abs(t.angular.z) < 0.12:
                t.angular.z = math.copysign(0.12, err)
            self.cmd_pub.publish(t)
            time.sleep(0.05)
        self._stop()
        time.sleep(0.3)
        # 2. drive straight
        z = self.latest_zone
        dist = ((target.x - z.robot_x) * math.cos(z.robot_yaw)
                + (target.y - z.robot_y) * math.sin(z.robot_yaw))
        dist = max(-max_dist, min(max_dist, dist))
        driven = self._drive_straight(dist, target.yaw, speed)
        return {'driven_m': round(driven, 3),
                'final_error_m': round(abs(dist) - abs(driven), 3)}

    def _drive_straight(self, dist: float, heading: float, speed: float = 0.08) -> float:
        if abs(dist) < 0.01 or self.odom is None:
            return 0.0
        x0, y0 = self.odom
        start = time.monotonic()
        travelled = 0.0
        while travelled < abs(dist) - 0.005 and time.monotonic() - start < abs(dist) / speed + 6.0:
            left = abs(dist) - travelled
            t = Twist()
            t.linear.x = math.copysign(max(0.03, min(speed, 0.8 * left + 0.02)), dist)
            z = self.latest_zone
            if z is not None:
                t.angular.z = max(-0.25, min(0.25, 1.2 * normalize_angle(heading - z.robot_yaw)))
            self.cmd_pub.publish(t)
            time.sleep(0.05)
            travelled = math.dist(self.odom, (x0, y0))
        self._stop()
        return math.copysign(travelled, dist)

    def _retreat(self, approach: dict) -> None:
        driven = approach.get('driven_m', 0.0) if approach else 0.0
        if abs(driven) > 0.01:
            z = self.latest_zone
            self._drive_straight(-driven, z.robot_yaw if z else 0.0)

    def _rotate(self, angle: float) -> None:
        if not self.nav.spin(spin_dist=angle, time_allowance=10):
            return
        start = time.monotonic()
        while not self.nav.isTaskComplete() and time.monotonic() - start < 15.0:
            pass

    def _scan(self, rack_id: str, sweep: bool):
        future = self.scan_client.call_async(
            ScanZone.Request(zone_id=rack_id, duration=float(self.confirm_duration)))
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if sweep and self.sweep_angle > 0.0:
            for angle in (self.sweep_angle, -2.0 * self.sweep_angle, self.sweep_angle):
                self._rotate(angle)
        done.wait(self.confirm_duration + 15.0)
        return future.result() if future.done() else None

    def _confirm(self, rack_id: str):
        """True if the camera sees at least one bin label that belongs to this rack."""
        expected = {mid for mid, item in self.layout.catalog.items() if item.zone == rack_id}
        if not self.scan_client.wait_for_service(timeout_sec=3.0):
            return True, [], 'scanner not running - confirmation skipped'
        seen = []
        for sweep in (False, True):
            response = self._scan(rack_id, sweep)
            seen = sorted(int(i) for i in response.detected_ids) if response else []
            if set(seen) & expected:
                return True, seen, f'saw rack {rack_id} labels {sorted(set(seen) & expected)}'
            if not sweep:
                self.get_logger().warn(f'No rack {rack_id} label in view (saw {seen}) - '
                                       'sweeping and trying again')
        return False, seen, f'no rack {rack_id} label seen (saw {seen or "nothing"})'

    # ------------------------------------------------------------------ #
    #  Delivery sequence
    # ------------------------------------------------------------------ #
    def _finish(self, rec: dict, success: bool, message: str) -> bool:
        with self.lock:
            order = self.book.finish(success, message)
            self.status_message = f'{order.order_id}: {message}'
        rec.update(result=order.state, message=message,
                   total_time_s=round(order.duration(time.time()), 1))
        self.records.append(rec)
        self.write_results()
        log = self.get_logger().info if success else self.get_logger().error
        log(f'[{order.order_id}] {order.state}: {message} '
            f'({rec["total_time_s"]:.0f} s)')
        self._event('ORDER_DONE', order_id=order.order_id, result=order.state, message=message)
        self.publish_status()
        return success

    def _hand_over(self) -> dict:
        """At the counter: creep in, let the arm put the unit down, back out."""
        rec = {}
        if self.carrying:
            rec['approach'] = approach = self._approach(self.layout.counter_pick_pose())
            place = self._call(self.place_client, Trigger.Request(), timeout=120.0)
            rec['place'] = place.message if place is not None else 'no answer from item_handler'
            if place is not None and not place.success:
                self.get_logger().warn(f'Place failed: {place.message}')
            self._retreat(approach)
        self.carrying = None
        return rec

    def _deliver_leftover(self) -> None:
        """An earlier delivery failed with the item still on board: drop it off first."""
        if not self.carrying:
            return
        self._set_state(FetchStatus.ROBOT_BUSY,
                        'Delivering an item still on the robot from an earlier order')
        nav = self._navigate('USER COUNTER', self.layout.zones['DROP'].goal)
        if nav['nav_result'] == 'SUCCEEDED':
            self._hand_over()

    def _execute(self, order: fl.Order) -> bool:
        name, rack = order.product_name, self.layout.zones[order.rack_id]
        rec = {'order_id': order.order_id, 'product': order.product, 'rack_id': rack.id,
               'requested_at': datetime.fromtimestamp(order.requested_at).isoformat(
                   timespec='seconds')}
        self._deliver_leftover()

        # 1. drive to the rack
        self._step(fl.TO_RACK, f'Driving to rack {rack.id} ({rack.name})')
        rec['to_rack'] = nav = self._navigate(f'RACK {rack.id}', rack.goal)
        if nav['nav_result'] != 'SUCCEEDED':
            return self._finish(rec, False, f'Could not reach rack {rack.id} ({nav["nav_result"]})')
        z = self.latest_zone
        rec['zone_recognized'] = bool(z is not None and z.zone_id == rack.id)

        # 2. make sure it is the right rack
        if self.confirm_with_camera:
            self._step(fl.CONFIRMING, f'Checking the bin labels on rack {rack.id} with the camera')
            ok, seen, note = self._confirm(rack.id)
            rec['confirm'] = {'ok': ok, 'seen_ids': seen, 'note': note}
            self.get_logger().info(f'[{order.order_id}] camera check: {note}')
            if not ok and self.require_confirmation:
                return self._finish(rec, False, f'Camera check failed: {note}')

        # 3. creep in, pick one unit off the shelf with the arm, back out
        self._step(fl.PICKING, f'Moving in to rack {rack.id} and picking a {name} with the arm')
        rec['approach'] = approach = self._approach(self.layout.rack_pick_pose(rack.id))
        pick = self._call(self.pick_client, PickItem.Request(rack_id=rack.id), timeout=120.0)
        self._retreat(approach)
        if pick is None:
            self.get_logger().warn('item_handler is not running - continuing without the arm')
            rec['item'] = None
        elif not pick.success:
            return self._finish(rec, False, f'Pick failed: {pick.message}')
        else:
            rec['item'] = pick.item_name
            self.carrying = pick.item_name
        self._event('FETCH_PICK', order_id=order.order_id, product=order.product,
                    rack=rack.id, item=rec['item'])

        # 4. bring it to the user
        self._step(fl.TO_USER, f'Bringing the {name} to the user counter')
        drop = self.layout.zones['DROP']
        for attempt in (1, 2):
            rec['to_user'] = nav = self._navigate('USER COUNTER', drop.goal)
            if nav['nav_result'] == 'SUCCEEDED':
                break
            self.get_logger().warn(f'Counter not reached ({nav["nav_result"]}), attempt {attempt}')
        if nav['nav_result'] != 'SUCCEEDED':
            return self._finish(rec, False, 'Could not reach the user counter - '
                                            'the item is still on the robot')
        self._step(fl.HANDING_OVER, f'Putting the {name} on the counter with the arm')
        rec['hand_over'] = self._hand_over()
        self._event('FETCH_DELIVER', order_id=order.order_id, product=order.product,
                    item=rec['item'])
        return self._finish(rec, True, f'Delivered - your {name} is on the user counter')

    def _orders_waiting(self) -> bool:
        with self.lock:
            return bool(self.book.queue)

    def run(self, navigator: BasicNavigator) -> None:
        self.nav = navigator
        if self.set_initial_pose:
            self.nav.setInitialPose(self._pose(self.layout.dock))
        self.nav.waitUntilNav2Active()
        self._set_state(FetchStatus.ROBOT_IDLE, 'Ready - waiting for orders')
        at_dock = self.set_initial_pose
        idle_since = time.monotonic()

        while rclpy.ok():
            with self.lock:
                order = self.book.start_next()
            if order is not None:
                at_dock = False
                self._execute(order)
                idle_since = time.monotonic()
                if not self._orders_waiting():
                    self._set_state(FetchStatus.ROBOT_IDLE, 'Ready - waiting for orders')
                continue
            if (self.return_to_dock and not at_dock
                    and time.monotonic() - idle_since > self.idle_return_delay):
                self._set_state(FetchStatus.ROBOT_RETURNING,
                                'No more orders - returning to the charging dock')
                nav = self._navigate('DOCK', self.layout.dock, interrupt=self._orders_waiting)
                at_dock = nav['nav_result'] != 'INTERRUPTED'   # don't retry a failed dock
                self._set_state(FetchStatus.ROBOT_IDLE,
                                'Ready at the charging dock' if nav['nav_result'] == 'SUCCEEDED'
                                else 'Ready - waiting for orders')
                idle_since = time.monotonic()
                continue
            time.sleep(0.1)

    # ------------------------------------------------------------------ #
    def write_results(self) -> None:
        if not self.records:
            return
        done = [r for r in self.records if r.get('result') == fl.DELIVERED]
        legs = [leg for r in self.records for leg in (r.get('to_rack'), r.get('to_user')) if leg]
        ok_legs = [leg for leg in legs if leg.get('nav_result') == 'SUCCEEDED']
        errs = [leg['position_error_m'] for leg in ok_legs if 'position_error_m' in leg]
        summary = {
            'orders': len(self.records),
            'delivered': len(done),
            'failed': len(self.records) - len(done),
            'mean_delivery_time_s': round(sum(r['total_time_s'] for r in done) / len(done), 1)
            if done else None,
            'navigation_success_rate': round(len(ok_legs) / len(legs), 3) if legs else None,
            'mean_position_error_m': round(sum(errs) / len(errs), 3) if errs else None,
            'camera_confirmations': sum(1 for r in self.records if r.get('confirm', {}).get('ok')),
        }
        data = {'started': datetime.fromtimestamp(self.started).isoformat(timespec='seconds'),
                'updated': datetime.now().isoformat(timespec='seconds'),
                'summary': summary, 'orders': self.records}
        try:
            with open(self.results_path, 'w') as f:
                json.dump(data, f, indent=2)
        except OSError as exc:
            self.get_logger().warn(f'Could not write {self.results_path}: {exc}')


def main(args=None):
    rclpy.init(args=args)
    manager = FetchManager()
    navigator = BasicNavigator()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(manager)

    def spin():
        try:
            executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass

    threading.Thread(target=spin, daemon=True).start()
    try:
        manager.run(navigator)
    except (KeyboardInterrupt, ExternalShutdownException):
        try:
            navigator.cancelTask()
        except Exception:  # noqa: BLE001 - best effort during shutdown
            pass
    finally:
        manager.write_results()
        executor.shutdown()
        manager.destroy_node()
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
