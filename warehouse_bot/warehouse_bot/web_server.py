#!/usr/bin/env python3
"""Web server - the order page, the external interface for retrieval mode.

Serves web/index.html plus a small JSON API and talks to the robot over ROS:

  GET  /              the order page
  GET  /api/layout    warehouse geometry for the page's live map
  GET  /api/status    robot state, current order, queue, history, shelf stock, robot pose
  GET  /api/camera/overview.jpg   live 3D view of the warehouse (fixed overview camera)
  GET  /api/camera/robot.jpg      what the rover's front camera sees
  POST /api/order     {"product": "diode"}   -> /warehouse/request_item
  POST /api/restock   {}                     -> /warehouse/restock

Subscribes /warehouse/fetch_status, /warehouse/stock, /warehouse/current_zone,
/warehouse/overview/image_raw, /camera/image_raw.

Parameters
  layout_file  path to warehouse_layout.yaml ('' = installed default)
  host         address to listen on (default 0.0.0.0)
  port         TCP port (default 8088)  -> open http://localhost:8088
  web_dir      folder with index.html ('' = installed share/warehouse_bot/web)

On a Windows + WSL PC whose firewall blocks traffic between Windows and WSL,
tools/web_relay_windows.py makes the page reachable from a Windows browser (see README).
"""

import json
import mimetypes
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from warehouse_interfaces.msg import FetchStatus, StockReport, ZoneStatus
from warehouse_interfaces.srv import RequestItem
from warehouse_bot.layout import PACKAGE_NAME, WarehouseLayout, resolve_layout_path

STOCK_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
FRESH_S = {'fetch_manager': 3.0, 'item_handler': 5.0, 'localisation': 3.0}


def order_dict(o) -> dict:
    return {'order_id': o.order_id, 'product': o.product, 'product_name': o.product_name,
            'rack_id': o.rack_id, 'state': o.state, 'message': o.message,
            'requested_at': o.requested_at, 'duration': round(o.duration, 1)}


class WebBridge(Node):
    """ROS side of the web server: caches the latest robot data, calls services."""

    def __init__(self):
        super().__init__('web_server')
        self.declare_parameter('layout_file', '')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8088)
        self.declare_parameter('web_dir', '')

        self.layout = WarehouseLayout.from_file(
            resolve_layout_path(self.get_parameter('layout_file').value))
        self.layout_json = json.dumps(self.layout.to_web()).encode()
        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        web_dir = str(self.get_parameter('web_dir').value)
        if not web_dir:
            from ament_index_python.packages import get_package_share_directory
            web_dir = os.path.join(get_package_share_directory(PACKAGE_NAME), 'web')
        # abspath, not realpath: with colcon --symlink-install the installed files are
        # symlinks into the source tree, and resolving them would fail the folder check
        self.web_dir = os.path.abspath(os.path.expanduser(web_dir))

        self.lock = threading.Lock()
        self.fetch, self.stock, self.zone = None, None, None
        self.seen = {'fetch_manager': 0.0, 'item_handler': 0.0, 'localisation': 0.0}

        cb = ReentrantCallbackGroup()
        self.create_subscription(FetchStatus, '/warehouse/fetch_status', self._on_fetch, 10,
                                 callback_group=cb)
        self.create_subscription(StockReport, '/warehouse/stock', self._on_stock, STOCK_QOS,
                                 callback_group=cb)
        self.create_subscription(ZoneStatus, '/warehouse/current_zone', self._on_zone, 10,
                                 callback_group=cb)
        self.request_client = self.create_client(RequestItem, '/warehouse/request_item',
                                                 callback_group=cb)
        self.restock_client = self.create_client(Trigger, '/warehouse/restock', callback_group=cb)
        # live views: keep the newest frame, encode to JPEG only when someone asks
        self.frames = {'overview': None, 'robot': None}
        self.jpeg_cache = {}
        self.create_subscription(Image, '/warehouse/overview/image_raw',
                                 lambda m: self._on_frame('overview', m), qos_profile_sensor_data,
                                 callback_group=cb)
        self.create_subscription(Image, '/camera/image_raw', lambda m: self._on_frame('robot', m),
                                 qos_profile_sensor_data, callback_group=cb)

    def _on_frame(self, key, msg):
        self.frames[key] = msg

    def jpeg(self, key: str):
        msg = self.frames.get(key)
        if msg is None:
            return None
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        cached = self.jpeg_cache.get(key)
        if cached and cached[0] == stamp:
            return cached[1]
        try:
            import cv2
            from warehouse_bot.vision import image_to_bgr
            img = image_to_bgr(msg)
            if img.shape[1] > 960:
                img = cv2.resize(img, (960, int(img.shape[0] * 960 / img.shape[1])),
                                 interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        except Exception as exc:  # noqa: BLE001 - never break the page over one frame
            self.get_logger().warn(f'Could not encode {key} frame: {exc}', throttle_duration_sec=30.0)
            return None
        if not ok:
            return None
        data = buf.tobytes()
        self.jpeg_cache[key] = (stamp, data)
        return data

    # ------------------------------------------------------------------ #
    def _on_fetch(self, msg):
        with self.lock:
            self.fetch, self.seen['fetch_manager'] = msg, time.monotonic()

    def _on_stock(self, msg):
        with self.lock:
            self.stock, self.seen['item_handler'] = msg, time.monotonic()

    def _on_zone(self, msg):
        with self.lock:
            self.zone, self.seen['localisation'] = msg, time.monotonic()

    def status(self) -> dict:
        now = time.monotonic()
        with self.lock:
            fetch, stock, zone = self.fetch, self.stock, self.zone
            online = {k: (now - t) < FRESH_S[k] for k, t in self.seen.items()}
        robot = {'state': 'OFFLINE', 'message': 'Waiting for the robot software to start',
                 'x': None, 'y': None, 'yaw': None, 'zone': '', 'zone_name': ''}
        if fetch is not None and online['fetch_manager']:
            robot.update(state=fetch.robot_state, message=fetch.message)
        if zone is not None:
            robot.update(x=round(zone.robot_x, 3), y=round(zone.robot_y, 3),
                         yaw=round(zone.robot_yaw, 3), zone=zone.zone_id, zone_name=zone.zone_name)
        if stock is not None:
            shelves = [{'product': s.product, 'name': s.product_name, 'rack_id': s.rack_id,
                        'available': s.available, 'capacity': s.capacity} for s in stock.products]
        else:
            shelves = [{'product': p.id, 'name': p.name, 'rack_id': p.rack_id,
                        'available': None, 'capacity': p.units}
                       for p in self.layout.products.values()]
        return {
            'online': online,
            'robot': robot,
            'current': order_dict(fetch.current) if fetch is not None and fetch.has_current
            else None,
            'queue': [order_dict(o) for o in fetch.queue] if fetch is not None else [],
            'history': [order_dict(o) for o in fetch.history] if fetch is not None else [],
            'stock': shelves,
            'carried_item': stock.carried_item if stock is not None else '',
            'delivered_count': stock.delivered_count if stock is not None else 0,
        }

    def call(self, client, request, timeout: float = 8.0):
        """Service call from an HTTP thread; the executor thread completes the future."""
        if not client.wait_for_service(timeout_sec=1.0):
            return None
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        return future.result() if done.wait(timeout) else None


def make_handler(bridge: WebBridge):

    class Handler(BaseHTTPRequestHandler):
        server_version = 'WarehouseOrderStation/1.0'

        def log_message(self, fmt, *args):          # keep the launch console quiet
            pass

        def _send(self, code: int, body: bytes, ctype: str = 'application/json') -> None:
            try:
                self.send_response(code)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)
            except (ConnectionError, OSError):
                pass        # client went away before the answer was ready

        def _json(self, code: int, data) -> None:
            self._send(code, json.dumps(data).encode())

        def _body(self) -> dict:
            length = int(self.headers.get('Content-Length') or 0)
            if not length:
                return {}
            try:
                return json.loads(self.rfile.read(length) or b'{}')
            except ValueError:
                return {}

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path == '/api/status':
                return self._json(200, bridge.status())
            if path == '/api/layout':
                return self._send(200, bridge.layout_json)
            if path.startswith('/api/camera/') and path.endswith('.jpg'):
                data = bridge.jpeg(path[len('/api/camera/'):-len('.jpg')])
                if data is None:
                    return self._json(503, {'error': 'no camera frame yet'})
                return self._send(200, data, 'image/jpeg')
            name = 'index.html' if path in ('/', '') else path.lstrip('/')
            full = os.path.normpath(os.path.join(bridge.web_dir, name))   # strips ../
            if not full.startswith(bridge.web_dir + os.sep) or not os.path.isfile(full):
                return self._json(404, {'error': 'not found'})
            with open(full, 'rb') as f:
                data = f.read()
            ctype = mimetypes.guess_type(full)[0] or 'application/octet-stream'
            if ctype.startswith('text/') or ctype.endswith('javascript'):
                ctype += '; charset=utf-8'
            self._send(200, data, ctype)

        def do_POST(self):
            path = self.path.split('?', 1)[0]
            body = self._body()
            if path == '/api/order':
                product = str(body.get('product', '')).strip()
                if not product:
                    return self._json(400, {'accepted': False, 'message': 'No product given'})
                res = bridge.call(bridge.request_client, RequestItem.Request(product=product))
                if res is None:
                    return self._json(503, {'accepted': False, 'message':
                                            'The robot is not ready yet - try again in a moment'})
                return self._json(200, {'accepted': res.accepted, 'order_id': res.order_id,
                                        'message': res.message})
            if path == '/api/restock':
                res = bridge.call(bridge.restock_client, Trigger.Request())
                if res is None:
                    return self._json(503, {'success': False,
                                            'message': 'item_handler is not running'})
                return self._json(200, {'success': res.success, 'message': res.message})
            return self._json(404, {'error': 'not found'})

    return Handler


def main(args=None):
    rclpy.init(args=args)
    bridge = WebBridge()
    try:
        server = ThreadingHTTPServer((bridge.host, bridge.port), make_handler(bridge))
    except OSError as exc:
        bridge.get_logger().error(f'Cannot listen on {bridge.host}:{bridge.port}: {exc}')
        bridge.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)
    server.daemon_threads = True
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(bridge)

    def spin():
        # Ctrl+C / SIGTERM shut the ROS context down, which ends spin(); then stop the
        # HTTP loop too, otherwise the process would keep serving without ROS.
        try:
            executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            server.shutdown()

    threading.Thread(target=spin, daemon=True).start()
    shown = 'localhost' if bridge.host in ('0.0.0.0', '') else bridge.host
    bridge.get_logger().info(f'Order page: http://{shown}:{bridge.port}  '
                             f'(serving {bridge.web_dir})')
    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        executor.shutdown()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
