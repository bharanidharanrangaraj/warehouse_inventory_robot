#!/usr/bin/env python3
"""Item handler - drives the rover's arm and gripper to pick and place components.

Pick (robot parked close to a rack, facing it):
    open gripper -> reach over the unit -> lower to it -> close the fingers ->
    lift -> back out of the shelf -> swing round -> set it down in the cargo tray ->
    open -> stow the arm
Place (robot parked at the user counter):
    take the unit out of the tray -> swing to the front -> lower it onto a free spot
    on the counter -> release -> stow

Joint angles come from arm_kinematics (closed-form IK) using the rover's true pose
from Gazebo; motions run through ros2_control's arm_controller / gripper_controller
(FollowJointTrajectory). Gazebo Classic cannot hold a part reliably by finger
friction, so while the fingers are closed the part is attached to the wrist link
(and, once released in the tray, to the rover) by updating its pose through
/gazebo/set_entity_state - the usual way pick-and-place is simulated.

Services
  /warehouse/pick_item   warehouse_interfaces/srv/PickItem  shelf -> cargo tray (rack_id)
  /warehouse/place_item  std_srvs/srv/Trigger              cargo tray -> user counter
  /warehouse/restock     std_srvs/srv/Trigger              every unit back on its shelf
Publishes
  /warehouse/stock       warehouse_interfaces/StockReport  units per shelf (latched, 1 Hz)
Uses
  /arm_controller/follow_joint_trajectory, /gripper_controller/follow_joint_trajectory,
  /joint_states, /gazebo/get_entity_state, /gazebo/set_entity_state

Parameters
  layout_file   path to warehouse_layout.yaml ('' = installed default)
  robot_entity  Gazebo model name of the rover (default 'rover')
  arm_speed     rad/s used to time arm motions (default 0.8)
  carry_rate    Hz at which a held part follows the gripper / tray (default 20)
"""

import math
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from warehouse_interfaces.msg import ProductStock, StockReport
from warehouse_interfaces.srv import PickItem
from warehouse_bot import arm_kinematics as ak
from warehouse_bot.layout import (WarehouseLayout, quaternion_to_yaw, resolve_layout_path,
                                  yaw_to_quaternion)

ON_SHELF = 'ON_SHELF'
IN_GRIPPER = 'IN_GRIPPER'
ON_TRAY = 'ON_TRAY'
DELIVERED = 'DELIVERED'

STOCK_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
APPROACH = 0.10      # pre-grasp: this far back from the part along the reach direction
CLEAR = 0.13         # height above the tray / counter spot before going down


# --------------------------------------------------------------------------- #
#  Small rigid-transform helpers (position, quaternion x y z w)
# --------------------------------------------------------------------------- #
def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw,
                     aw * bw - ax * bx - ay * by - az * bz])


def qconj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def qrot(q, v):
    return qmul(qmul(q, np.array([v[0], v[1], v[2], 0.0])), qconj(q))[:3]


def compose(a, b):
    return a[0] + qrot(a[1], b[0]), qmul(a[1], b[1])


def inverse(a):
    qi = qconj(a[1])
    return -qrot(qi, a[0]), qi


def from_pose(p: Pose):
    return (np.array([p.position.x, p.position.y, p.position.z]),
            np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))


def yaw_tf(x, y, z, yaw):
    return np.array([x, y, z]), np.array(yaw_to_quaternion(yaw))


class ItemHandler(Node):

    def __init__(self):
        super().__init__('item_handler')
        self.declare_parameter('layout_file', '')
        self.declare_parameter('robot_entity', 'rover')
        self.declare_parameter('arm_speed', 0.8)
        self.declare_parameter('carry_rate', 20.0)

        self.layout = WarehouseLayout.from_file(
            resolve_layout_path(self.get_parameter('layout_file').value))
        self.robot = str(self.get_parameter('robot_entity').value)
        self.wrist = f'{self.robot}::wrist_link'
        self.arm_speed = max(0.2, float(self.get_parameter('arm_speed').value))
        carry_rate = max(5.0, float(self.get_parameter('carry_rate').value))
        self.counter_z = self.layout.drop_station.size_z

        self.items = {h.name: {'home': h, 'state': ON_SHELF, 'slot': None,
                               'shape': self.layout.products[h.product_id].shape}
                      for h in self.layout.item_homes}
        self.attached = {}                 # item -> (reference frame, relative transform)
        self.lock = threading.RLock()
        self.op_lock = threading.Lock()    # one pick / place / restock at a time
        self.joints = dict(zip(ak.JOINTS, ak.STOW))
        self._carry_futures = {}
        self._synced = False

        cb = ReentrantCallbackGroup()
        self.set_cli = self.create_client(SetEntityState, '/gazebo/set_entity_state', callback_group=cb)
        self.get_cli = self.create_client(GetEntityState, '/gazebo/get_entity_state', callback_group=cb)
        self.arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory',
                                callback_group=cb)
        self.grip = ActionClient(self, FollowJointTrajectory,
                                 '/gripper_controller/follow_joint_trajectory', callback_group=cb)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 20, callback_group=cb)

        self.create_service(PickItem, '/warehouse/pick_item', self.on_pick, callback_group=cb)
        self.create_service(Trigger, '/warehouse/place_item', self.on_place, callback_group=cb)
        self.create_service(Trigger, '/warehouse/restock', self.on_restock, callback_group=cb)

        self.stock_pub = self.create_publisher(StockReport, '/warehouse/stock', STOCK_QOS)
        self.create_timer(1.0 / carry_rate, self._carry_tick,
                          callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(1.0, self._publish_stock, callback_group=MutuallyExclusiveCallbackGroup())
        self._sync_timer = self.create_timer(2.0, self._sync_once,
                                             callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(
            f'Item handler ready: {len(self.items)} units on {len(self.layout.products)} racks, '
            f'{len(self.layout.counter_slots)} delivery spots on the counter')

    # ------------------------------------------------------------------ #
    #  ROS helpers
    # ------------------------------------------------------------------ #
    def _on_joints(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name in self.joints:
                self.joints[name] = pos

    def _wait(self, future, timeout: float):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        return future.result() if done.wait(timeout) else None

    def _call(self, client, request, timeout: float = 2.0):
        if not client.service_is_ready() and not client.wait_for_service(timeout_sec=timeout):
            return None
        return self._wait(client.call_async(request), timeout)

    def _entity(self, name: str, frame: str = 'world'):
        res = self._call(self.get_cli, GetEntityState.Request(name=name, reference_frame=frame))
        if res is None or not res.success:
            return None
        return from_pose(res.state.pose)

    def _set(self, name: str, tf, frame: str = 'world') -> bool:
        s = EntityState(name=name, reference_frame=frame)
        p, q = tf
        s.pose.position.x, s.pose.position.y, s.pose.position.z = (float(v) for v in p)
        (s.pose.orientation.x, s.pose.orientation.y,
         s.pose.orientation.z, s.pose.orientation.w) = (float(v) for v in q)
        res = self._call(self.set_cli, SetEntityState.Request(state=s), timeout=1.0)
        return bool(res and res.success)

    def _robot_pose(self):
        tf = self._entity(self.robot)
        if tf is None:
            return None
        p, q = tf
        return float(p[0]), float(p[1]), quaternion_to_yaw(*q)

    def _trajectory(self, client, names, positions, secs: float) -> bool:
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Arm controller is not available')
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        point = JointTrajectoryPoint(positions=[float(v) for v in positions])
        point.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1.0) * 1e9))
        goal.trajectory.points = [point]
        handle = self._wait(client.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            return False
        result = self._wait(handle.get_result_async(), secs + 15.0)
        return result is not None and result.result.error_code == 0

    def _arm_to(self, q, secs=None) -> bool:
        delta = max(abs(a - self.joints[j]) for j, a in zip(ak.JOINTS, q))
        t = secs if secs is not None else max(0.7, delta / self.arm_speed)
        return self._trajectory(self.arm, ak.JOINTS, q, t)

    def _reach(self, point, hint=0.0, secs=None) -> bool:
        q = ak.solve(*point, turret_hint=hint)
        if q is None:
            self.get_logger().warn(f'Out of reach: {tuple(round(v, 3) for v in point)}')
            return False
        return self._arm_to(q, secs)

    def _gripper(self, opening: float, secs: float = 0.7) -> bool:
        o = min(ak.FINGER_MAX, max(0.0, opening))
        return self._trajectory(self.grip, ak.GRIPPER_JOINTS, (o, o), secs)

    # ------------------------------------------------------------------ #
    #  Attachments (the "grasp")
    # ------------------------------------------------------------------ #
    def _attach(self, name: str, frame: str) -> bool:
        ref = self._entity(frame)
        obj = self._entity(name)
        if ref is None or obj is None:
            return False
        with self.lock:
            self.attached[name] = (frame, compose(inverse(ref), obj))
        return True

    def _attach_on_tray(self, name: str, robot) -> None:
        """Release into the tray: sit the part upright on the tray floor, ride on the rover."""
        obj = self._entity(name)
        yaw = quaternion_to_yaw(*obj[1]) - robot[2] if obj is not None else 0.0
        with self.lock:
            self.attached[name] = (self.robot, yaw_tf(ak.TRAY[0], ak.TRAY[1], ak.TRAY[2] - 0.0, yaw))

    def _detach(self, name: str) -> None:
        with self.lock:
            self.attached.pop(name, None)

    def _carry_tick(self) -> None:
        with self.lock:
            attached = dict(self.attached)
        if not attached or not self.set_cli.service_is_ready():
            return
        for name, (frame, (p, q)) in attached.items():
            fut = self._carry_futures.get(name)
            if fut is not None and not fut.done():
                continue
            s = EntityState(name=name, reference_frame=frame)
            s.pose.position.x, s.pose.position.y, s.pose.position.z = (float(v) for v in p)
            (s.pose.orientation.x, s.pose.orientation.y,
             s.pose.orientation.z, s.pose.orientation.w) = (float(v) for v in q)
            self._carry_futures[name] = self.set_cli.call_async(SetEntityState.Request(state=s))

    # ------------------------------------------------------------------ #
    #  Stock
    # ------------------------------------------------------------------ #
    def _held(self):
        with self.lock:
            return [n for n, it in self.items.items() if it['state'] in (IN_GRIPPER, ON_TRAY)]

    def _publish_stock(self) -> None:
        msg = StockReport()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            for p in self.layout.products.values():
                units = [it for it in self.items.values() if it['home'].product_id == p.id]
                msg.products.append(ProductStock(
                    product=p.id, product_name=p.name, rack_id=p.rack_id,
                    available=sum(1 for it in units if it['state'] == ON_SHELF),
                    capacity=len(units)))
            held = [n for n, it in self.items.items() if it['state'] in (IN_GRIPPER, ON_TRAY)]
            msg.carried_item = held[0] if held else ''
            msg.delivered_count = sum(1 for it in self.items.values() if it['state'] == DELIVERED)
        self.stock_pub.publish(msg)

    def _sync_once(self) -> None:
        """Put every unit on its shelf once Gazebo is up (also fixes a node restart)."""
        if self._synced:
            return
        if not self.set_cli.service_is_ready():
            self.get_logger().info('Waiting for /gazebo/set_entity_state ...', throttle_duration_sec=15.0)
            return
        with self.op_lock:
            ok = all(self._set(n, yaw_tf(it['home'].x, it['home'].y, it['home'].z, it['home'].yaw))
                     for n, it in self.items.items())
        if ok:
            self._synced = True
            self._sync_timer.cancel()
            self.get_logger().info('All units are on their shelves')
            self._publish_stock()

    # ------------------------------------------------------------------ #
    #  Motions
    # ------------------------------------------------------------------ #
    def _grasp_size(self, name):
        return ak.GRASP.get(self.items[name]['shape'], ak.GRASP['box'])

    def _tray_points(self, height: float):
        tx, ty, tz = ak.TRAY
        return (tx, ty, tz + height + 0.004), (tx, ty, tz + height + CLEAR)

    def _turret_back(self) -> float:
        return math.pi if self.joints['arm_turret'] >= 0 else -math.pi

    def _stow(self) -> bool:
        return self._arm_to(ak.STOW)

    def _fail(self, response, message: str):
        self.get_logger().error(message)
        response.message = message
        self._stow()
        return response

    def on_pick(self, request, response):
        response.success, response.item_name = False, ''
        if not self.op_lock.acquire(blocking=False):
            response.message = 'The arm is busy'
            return response
        try:
            rack_id = request.rack_id.strip().upper()
            if self._held():
                response.message = f'Already holding {self._held()[0]}'
                return response
            robot = self._robot_pose()
            if robot is None:
                response.message = 'Gazebo entity-state service is not available'
                return response
            with self.lock:
                candidates = [n for n, it in self.items.items()
                              if it['home'].rack_id == rack_id and it['state'] == ON_SHELF]
            if not candidates:
                response.message = f'Rack {rack_id} is empty'
                return response
            # nearest unit the arm can actually reach from where the rover stopped
            best = None
            for n in sorted(candidates, key=lambda n: math.hypot(self.items[n]['home'].x - robot[0],
                                                                 self.items[n]['home'].y - robot[1])):
                h = self.items[n]['home']
                g = ak.to_robot_frame(robot, (h.x, h.y, h.z + self._grasp_size(n)[0]))
                if ak.reachable(g, APPROACH):
                    best = (n, g)
                    break
            if best is None:
                response.message = (f'No unit on rack {rack_id} is within the arm\'s reach from '
                                    f'({robot[0]:.2f}, {robot[1]:.2f})')
                return response
            name, g = best
            height, width = self._grasp_size(name)
            sx, sy, _ = ak.SHOULDER
            r = math.hypot(g[0] - sx, g[1] - sy)
            ux, uy = (g[0] - sx) / r, (g[1] - sy) / r
            pre = (g[0] - ux * APPROACH, g[1] - uy * APPROACH, g[2])
            self.get_logger().info(f'Picking {name} from rack {rack_id}')

            self._gripper(width / 2 + 0.02)
            ok = (self._reach((pre[0], pre[1], pre[2] + 0.08))
                  and self._reach(pre)
                  and self._reach(g, secs=1.4))
            if not ok:
                return self._fail(response, f'Arm could not reach {name}')
            self._gripper(width / 2 - 0.003, secs=0.8)
            if not self._attach(name, self.wrist):
                return self._fail(response, 'Could not read the gripper pose from Gazebo')
            with self.lock:
                self.items[name]['state'] = IN_GRIPPER
            self._publish_stock()
            self._reach((g[0], g[1], g[2] + 0.04), secs=0.8)
            self._reach((pre[0], pre[1], pre[2] + 0.06), secs=1.0)
            place, above = self._tray_points(height)
            back = self._turret_back()
            ok = self._reach(above, hint=back) and self._reach(place, hint=back, secs=1.2)
            self._gripper(width / 2 + 0.02, secs=0.6)
            self._attach_on_tray(name, robot)
            with self.lock:
                self.items[name]['state'] = ON_TRAY
            self._reach(above, hint=back, secs=0.8)
            self._stow()
            self._publish_stock()
            response.success, response.item_name = True, name
            response.message = f'{name} picked from rack {rack_id} and stowed in the cargo tray'
            self.get_logger().info(response.message)
            return response
        finally:
            self.op_lock.release()

    def on_place(self, request, response):
        response.success = False
        if not self.op_lock.acquire(blocking=False):
            response.message = 'The arm is busy'
            return response
        try:
            with self.lock:
                tray = [n for n, it in self.items.items() if it['state'] == ON_TRAY]
                used = {it['slot'] for it in self.items.values() if it['slot'] is not None}
            if not tray:
                response.message = 'Nothing in the cargo tray'
                return response
            name = tray[0]
            robot = self._robot_pose()
            if robot is None:
                response.message = 'Gazebo entity-state service is not available'
                return response
            height, width = self._grasp_size(name)
            spot = None
            for i, s in enumerate(self.layout.counter_slots):
                if i in used:
                    continue
                g = ak.to_robot_frame(robot, (s.x, s.y, self.counter_z + height + 0.004))
                if ak.reachable(g, APPROACH):
                    spot = (i, s, g)
                    break
            if spot is None:
                response.message = ('No free spot on the user counter within reach - '
                                    'restock to clear the counter')
                return response
            idx, slot, g = spot
            self.get_logger().info(f'Placing {name} on counter spot {idx + 1}')

            back = self._turret_back()
            place, above = self._tray_points(height)
            self._gripper(width / 2 + 0.02, secs=0.6)
            ok = self._reach(above, hint=back) and self._reach(place, hint=back, secs=1.2)
            if not ok:
                return self._fail(response, 'Arm could not reach the cargo tray')
            self._gripper(width / 2 - 0.003, secs=0.8)
            if not self._attach(name, self.wrist):
                return self._fail(response, 'Could not read the gripper pose from Gazebo')
            with self.lock:
                self.items[name]['state'] = IN_GRIPPER
            self._reach(above, hint=back, secs=1.0)
            sx, sy, _ = ak.SHOULDER
            r = math.hypot(g[0] - sx, g[1] - sy)
            ux, uy = (g[0] - sx) / r, (g[1] - sy) / r
            ok = (self._reach((g[0] - ux * 0.05, g[1] - uy * 0.05, g[2] + CLEAR))
                  and self._reach(g, secs=1.4))
            if not ok:
                return self._fail(response, 'Arm could not reach the counter spot')
            self._gripper(width / 2 + 0.02, secs=0.6)
            self._detach(name)
            obj = self._entity(name)
            yaw = quaternion_to_yaw(*obj[1]) if obj is not None else slot.yaw
            self._set(name, yaw_tf(slot.x, slot.y, self.counter_z, yaw))   # settle on the top
            with self.lock:
                self.items[name]['state'] = DELIVERED
                self.items[name]['slot'] = idx
            self._reach((g[0] - ux * 0.06, g[1] - uy * 0.06, g[2] + 0.10), secs=0.9)
            self._stow()
            self._publish_stock()
            response.success = True
            response.message = f'{name} is on the user counter (spot {idx + 1})'
            self.get_logger().info(response.message)
            return response
        finally:
            self.op_lock.release()

    def on_restock(self, request, response):
        response.success = False
        if not self.op_lock.acquire(blocking=False):
            response.message = 'The arm is busy'
            return response
        try:
            if self._held():
                response.message = (f'The rover is carrying {self._held()[0]} - '
                                    'wait until it is delivered')
                return response
            with self.lock:
                moved = [n for n, it in self.items.items() if it['state'] != ON_SHELF]
            for name in moved:
                h = self.items[name]['home']
                self._set(name, yaw_tf(h.x, h.y, h.z, h.yaw))
                with self.lock:
                    self.items[name]['state'] = ON_SHELF
                    self.items[name]['slot'] = None
            self._publish_stock()
            response.success = True
            response.message = f'Restocked {len(moved)} unit(s); every shelf is full again'
            self.get_logger().info(response.message)
            return response
        finally:
            self.op_lock.release()


def main(args=None):
    rclpy.init(args=args)
    node = ItemHandler()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
