"""Warehouse layout model shared by every node and by the world generator.

config/warehouse_layout.yaml is the single source of truth for the project:
the Gazebo world, the Nav2 map, the zone rectangles, the scan poses, the
orderable products with their shelf positions and the expected inventory are
all derived from it here, so they can never drift apart.

This module only needs the Python standard library and PyYAML, so it can be
unit-tested and used by the generator without a ROS installation.
"""

import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import yaml

PACKAGE_NAME = 'warehouse_bot'


# --------------------------------------------------------------------------- #
#  Paths
# --------------------------------------------------------------------------- #
def default_layout_path() -> str:
    """Layout file installed with the package (share/warehouse_bot/config)."""
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory(PACKAGE_NAME),
                        'config', 'warehouse_layout.yaml')


def resolve_layout_path(path: str) -> str:
    """Use the given path, or the installed default when it is empty."""
    return os.path.expanduser(path) if path else default_layout_path()


# --------------------------------------------------------------------------- #
#  Small geometry helpers
# --------------------------------------------------------------------------- #
def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """(x, y, z, w) quaternion for a rotation of `yaw` about the z axis."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def transform_point(ox: float, oy: float, oyaw: float,
                    lx: float, ly: float) -> Tuple[float, float]:
    """Express a point given in a local frame (ox, oy, oyaw) in the world frame."""
    c, s = math.cos(oyaw), math.sin(oyaw)
    return ox + c * lx - s * ly, oy + s * lx + c * ly


# --------------------------------------------------------------------------- #
#  Data classes
# --------------------------------------------------------------------------- #
@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class Rect:
    """Oriented rectangle: centre, heading and size along its own axes."""

    x: float
    y: float
    yaw: float
    size_x: float
    size_y: float

    def corners(self) -> List[Tuple[float, float]]:
        hx, hy = self.size_x / 2.0, self.size_y / 2.0
        return [transform_point(self.x, self.y, self.yaw, lx, ly)
                for lx, ly in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))]

    def contains(self, px: float, py: float, margin: float = 0.0) -> bool:
        dx, dy = px - self.x, py - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        lx = c * dx + s * dy          # rotate the point into the rect frame
        ly = -s * dx + c * dy
        return (abs(lx) <= self.size_x / 2.0 + margin
                and abs(ly) <= self.size_y / 2.0 + margin)


@dataclass
class Zone:
    """An area the robot can be 'in', plus the pose it drives to for that zone."""

    id: str
    name: str
    kind: str          # 'RACK', 'DROP' or 'DOCK'
    area: Rect
    goal: Pose2D

    def contains(self, x: float, y: float) -> bool:
        return self.area.contains(x, y)


PRODUCT_SHAPES = ('capacitor', 'resistor', 'transformer', 'diode', 'box')


@dataclass
class Product:
    """Something a user can order in retrieval mode: one product type per rack."""

    id: str            # e.g. 'diode'
    name: str          # e.g. 'Diode'
    rack_id: str       # e.g. 'D'
    rack_name: str     # e.g. 'Diodes'
    units: int         # pickable models on a fully stocked shelf
    shape: str         # one of PRODUCT_SHAPES (how it looks in Gazebo)


@dataclass
class Rack:
    id: str
    name: str
    x: float
    y: float
    yaw: float
    length: float
    depth: float
    height: float
    face: int
    color: Tuple[float, float, float]
    slots: List[dict] = field(default_factory=list)
    product: 'Product' = None


@dataclass
class ItemHome:
    """A pickable component model and the shelf position it is stocked at."""

    name: str          # Gazebo model name, e.g. 'item_D_0'
    product_id: str
    rack_id: str
    index: int
    x: float
    y: float
    z: float
    yaw: float


@dataclass
class Box:
    """Generic static box: obstacle or station."""

    name: str
    x: float
    y: float
    yaw: float
    size_x: float
    size_y: float
    size_z: float
    color: Tuple[float, float, float]


@dataclass
class PlacedMarker:
    """A tag that physically exists in the simulated world (ground truth)."""

    marker_id: int
    zone_id: str
    x: float
    y: float
    z: float
    yaw: float


@dataclass
class CatalogItem:
    """A line in the stock records (what SHOULD be in the warehouse)."""

    marker_id: int
    sku: str
    name: str
    zone: str
    quantity: int


# --------------------------------------------------------------------------- #
#  Layout
# --------------------------------------------------------------------------- #
def _pose(d: dict) -> Tuple[float, float, float]:
    return float(d['x']), float(d['y']), float(d.get('yaw', 0.0))


def _color(value, default=(0.6, 0.6, 0.6)) -> Tuple[float, float, float]:
    if not value:
        return default
    return tuple(float(v) for v in value[:3])


class WarehouseLayout:
    """Parsed layout with every derived quantity the nodes need."""

    TAG_OFFSET = 0.004   # tags sit 4 mm in front of the rack face
    SHELF_TOP_Z = 0.325  # top of the bottom shelf board (see generate_world.rack_model)
    ITEM_INSET = 0.10    # stocked components sit this far behind the rack's front edge
    COUNTER_ROWS = (0.10, 0.25)    # delivery rows, measured from the counter's front edge
    COUNTER_PITCH = 0.20           # spacing of delivery spots along a row

    def __init__(self, data: dict):
        self.raw = data
        wh = data['warehouse']
        room = wh['room']
        self.name = str(wh.get('name', 'Warehouse'))
        self.room_min_x = float(room['min_x'])
        self.room_max_x = float(room['max_x'])
        self.room_min_y = float(room['min_y'])
        self.room_max_y = float(room['max_y'])
        self.wall_thickness = float(wh.get('wall_thickness', 0.15))
        self.wall_height = float(wh.get('wall_height', 1.0))
        self.map_resolution = float(wh.get('map_resolution', 0.05))

        scan = data.get('scan', {})
        self.standoff = float(scan.get('standoff', 0.8))
        self.zone_depth = float(scan.get('zone_depth', 1.3))
        self.zone_margin = float(scan.get('zone_margin', 0.2))
        self.marker_size = float(scan.get('marker_size', 0.18))
        self.marker_height = float(scan.get('marker_height', 0.18))
        self.aruco_dictionary = str(scan.get('aruco_dictionary', 'DICT_4X4_50'))

        self.dock = Pose2D(*_pose(data['dock']))
        # charging station right behind the docked robot (it is seen by the LiDAR -> in the map)
        sx, sy = transform_point(self.dock.x, self.dock.y, self.dock.yaw, -0.50, 0.0)
        self.dock_station = Box('charging_station', sx, sy, self.dock.yaw, 0.14, 0.60, 0.42,
                                (0.16, 0.17, 0.19))

        self.racks: List[Rack] = []
        self.products: Dict[str, Product] = {}      # product id -> product, in rack order
        for r in data['racks']:
            x, y, yaw = _pose(r['pose'])
            size = r.get('size', {})
            rack = Rack(
                id=str(r['id']), name=str(r.get('name', r['id'])),
                x=x, y=y, yaw=yaw,
                length=float(size.get('length', 1.2)),
                depth=float(size.get('depth', 0.5)),
                height=float(size.get('height', 1.2)),
                face=1 if int(r.get('face', -1)) > 0 else -1,
                color=_color(r.get('color')),
                slots=list(r.get('slots', [])))
            p = r.get('product')
            if p:
                pid = str(p['id']).strip().lower()
                if pid in self.products:
                    raise ValueError(f'Product id "{pid}" is used by more than one rack')
                rack.product = Product(pid, str(p.get('name', pid.title())), rack.id, rack.name,
                                       int(p.get('units', 3)), str(p.get('shape', 'box')))
                self.products[pid] = rack.product
            self.racks.append(rack)

        ds = data['drop_station']
        x, y, yaw = _pose(ds['pose'])
        size = ds.get('size', {})
        self.drop_station = Box(str(ds.get('name', 'Drop station')), x, y, yaw,
                                float(size.get('x', 0.5)), float(size.get('y', 0.8)),
                                float(size.get('z', 0.45)),
                                _color(ds.get('color'), (0.15, 0.45, 0.85)))
        self.drop_standoff = float(ds.get('standoff', 0.55))

        self.obstacles: List[Box] = []
        for o in data.get('obstacles', []):
            x, y, yaw = _pose(o['pose'])
            size = o['size']
            self.obstacles.append(Box(str(o['name']), x, y, yaw,
                                      float(size['x']), float(size['y']), float(size['z']),
                                      _color(o.get('color'))))

        self.catalog: Dict[int, CatalogItem] = {}
        for item in data.get('catalog', []):
            mid = int(item['marker_id'])
            self.catalog[mid] = CatalogItem(mid, str(item['sku']), str(item['name']),
                                            str(item['zone']), int(item.get('quantity', 1)))

        self.mission: dict = dict(data.get('mission', {}))
        self.fetch: dict = dict(data.get('fetch', {}))
        # robot centre -> rack face / counter edge while the arm works
        self.pick_standoff = float(self.fetch.get('pick_standoff', 0.50))
        self.counter_standoff = float(self.fetch.get('counter_standoff', 0.45))

        # ---- derived zones (insertion order = racks, drop station, dock) ----
        self.zones: Dict[str, Zone] = {}
        for rack in self.racks:
            self.zones[rack.id] = self._rack_zone(rack)
        self.zones['DROP'] = self._drop_zone()
        self.zones['DOCK'] = Zone('DOCK', 'Charging dock', 'DOCK',
                                  Rect(self.dock.x, self.dock.y, self.dock.yaw, 1.0, 1.0),
                                  self.dock)

        # ---- ground-truth tag placement ----
        self.placed_markers: List[PlacedMarker] = []
        for rack in self.racks:
            for slot in rack.slots:
                mx, my = transform_point(rack.x, rack.y, rack.yaw, float(slot['offset']),
                                         rack.face * (rack.depth / 2.0 + self.TAG_OFFSET))
                self.placed_markers.append(PlacedMarker(int(slot['marker_id']), rack.id,
                                                        mx, my, self.marker_height, rack.yaw))

        # ---- pickable component models (retrieval mode) ----
        self.item_homes: List[ItemHome] = []
        for rack in self.racks:
            if rack.product is None:
                continue
            n = rack.product.units
            for k in range(n):
                lx = -rack.length / 2.0 + rack.length / max(n, 1) * (k + 0.5)
                ly = rack.face * (rack.depth / 2.0 - self.ITEM_INSET)
                ix, iy = transform_point(rack.x, rack.y, rack.yaw, lx, ly)
                # long axis points out of the shelf, so the gripper closes across the part
                self.item_homes.append(ItemHome(f'item_{rack.id}_{k}', rack.product.id, rack.id, k,
                                                ix, iy, self.SHELF_TOP_Z,
                                                normalize_angle(rack.yaw + math.pi / 2.0)))
        self.counter_slots = self._counter_slots()
        self._validate()

    # ------------------------------------------------------------------ #
    @classmethod
    def from_file(cls, path: str) -> 'WarehouseLayout':
        with open(os.path.expanduser(path), 'r') as f:
            return cls(yaml.safe_load(f))

    def _rack_zone(self, rack: Rack) -> Zone:
        f = rack.face
        cx, cy = transform_point(rack.x, rack.y, rack.yaw,
                                 0.0, f * (rack.depth / 2.0 + self.zone_depth / 2.0))
        area = Rect(cx, cy, rack.yaw, rack.length + 2.0 * self.zone_margin, self.zone_depth)
        gx, gy = transform_point(rack.x, rack.y, rack.yaw,
                                 0.0, f * (rack.depth / 2.0 + self.standoff))
        # face the rack: rack on local -y side -> look along +y, and vice versa
        goal = Pose2D(gx, gy, normalize_angle(rack.yaw - f * math.pi / 2.0))
        return Zone(rack.id, rack.name, 'RACK', area, goal)

    def _drop_zone(self) -> Zone:
        t = self.drop_station
        cx, cy = transform_point(t.x, t.y, t.yaw, -(t.size_x / 2.0 + self.zone_depth / 2.0), 0.0)
        area = Rect(cx, cy, t.yaw, self.zone_depth, t.size_y + 2.0 * self.zone_margin)
        gx, gy = transform_point(t.x, t.y, t.yaw, -(t.size_x / 2.0 + self.drop_standoff), 0.0)
        return Zone('DROP', t.name, 'DROP', area, Pose2D(gx, gy, t.yaw))

    # ------------------------------------------------------------------ #
    #  Where the arm works
    # ------------------------------------------------------------------ #
    def rack_pick_pose(self, rack_id: str) -> Pose2D:
        """Robot pose in front of a rack's centre, close enough for the arm."""
        rack = next(r for r in self.racks if r.id == rack_id)
        gx, gy = transform_point(rack.x, rack.y, rack.yaw, 0.0,
                                 rack.face * (rack.depth / 2.0 + self.pick_standoff))
        return Pose2D(gx, gy, normalize_angle(rack.yaw - rack.face * math.pi / 2.0))

    def counter_pick_pose(self) -> Pose2D:
        t = self.drop_station
        gx, gy = transform_point(t.x, t.y, t.yaw, -(t.size_x / 2.0 + self.counter_standoff), 0.0)
        return Pose2D(gx, gy, t.yaw)

    def grasp_heights(self) -> List[float]:
        from warehouse_bot import arm_kinematics as ak
        return sorted({ak.GRASP.get(p.shape, ak.GRASP['box'])[0] for p in self.products.values()})

    def _counter_slots(self) -> List[Pose2D]:
        """Delivery spots on the user counter that the arm reaches from the counter pick
        pose for every product, nearest row first, centre outwards."""
        from warehouse_bot import arm_kinematics as ak
        t = self.drop_station
        robot = self.counter_pick_pose()
        half = t.size_y / 2.0 - 0.07
        n = int(half // self.COUNTER_PITCH)
        offsets = sorted((k * self.COUNTER_PITCH for k in range(-n, n + 1)), key=abs)
        slots = []
        for depth in self.COUNTER_ROWS:
            if depth > t.size_x - 0.05:
                continue
            for ly in offsets:
                sx, sy = transform_point(t.x, t.y, t.yaw, -t.size_x / 2.0 + depth, ly)
                local = ak.to_robot_frame((robot.x, robot.y, robot.yaw), (sx, sy, 0.0))
                if all(ak.reachable((local[0], local[1], t.size_z + h + 0.005))
                       for h in self.grasp_heights()):
                    slots.append(Pose2D(sx, sy, t.yaw))
        return slots

    def _validate(self) -> None:
        rack_ids = {r.id for r in self.racks}
        if len(rack_ids) != len(self.racks):
            raise ValueError('Rack ids must be unique')
        for p in self.products.values():
            if p.shape not in PRODUCT_SHAPES:
                raise ValueError(f'Product "{p.id}" has unknown shape "{p.shape}" '
                                 f'(use one of {", ".join(PRODUCT_SHAPES)})')
            if not 0 <= p.units <= 8:
                raise ValueError(f'Product "{p.id}" units must be between 0 and 8')
        # the arm must reach every stocked unit from its rack's pick pose, and the tray
        from warehouse_bot import arm_kinematics as ak
        for item in self.item_homes:
            robot = self.rack_pick_pose(item.rack_id)
            h = ak.GRASP.get(self.products[item.product_id].shape, ak.GRASP['box'])[0]
            local = ak.to_robot_frame((robot.x, robot.y, robot.yaw), (item.x, item.y, item.z + h))
            if not ak.reachable(local):
                raise ValueError(f'The arm cannot reach {item.name} from the rack {item.rack_id} '
                                 f'pick pose - move it, or change fetch.pick_standoff')
        if self.item_homes and not self.counter_slots:
            raise ValueError('No delivery spot on the user counter is within reach of the arm')
        for item in self.catalog.values():
            if item.zone not in rack_ids:
                raise ValueError(f'Catalogue item {item.marker_id} refers to unknown rack '
                                 f'"{item.zone}"')
        placed = [m.marker_id for m in self.placed_markers]
        if len(placed) != len(set(placed)):
            raise ValueError('Each marker id may only be placed once in the world')
        for zone in self.zones.values():
            if not self.room_contains(zone.goal.x, zone.goal.y):
                raise ValueError(f'Goal of zone {zone.id} is outside the room')
        zones = list(self.zones.values())
        for i, a in enumerate(zones):
            for b in zones[i + 1:]:
                if any(b.area.contains(*c) for c in a.area.corners()) or \
                   any(a.area.contains(*c) for c in b.area.corners()):
                    raise ValueError(f'Zones {a.id} and {b.id} overlap')

    # ------------------------------------------------------------------ #
    def room_contains(self, x: float, y: float) -> bool:
        return self.room_min_x < x < self.room_max_x and self.room_min_y < y < self.room_max_y

    def rack_zones(self) -> List[Zone]:
        return [z for z in self.zones.values() if z.kind == 'RACK']

    def markers_in_zone(self, zone_id: str) -> List[int]:
        return sorted(m.marker_id for m in self.placed_markers if m.zone_id == zone_id)

    def wall_rects(self) -> List[Rect]:
        t = self.wall_thickness
        w = self.room_max_x - self.room_min_x
        h = self.room_max_y - self.room_min_y
        cx = (self.room_min_x + self.room_max_x) / 2.0
        cy = (self.room_min_y + self.room_max_y) / 2.0
        return [
            Rect(cx, self.room_max_y + t / 2.0, 0.0, w + 2.0 * t, t),   # north
            Rect(cx, self.room_min_y - t / 2.0, 0.0, w + 2.0 * t, t),   # south
            Rect(self.room_max_x + t / 2.0, cy, 0.0, t, h),             # east
            Rect(self.room_min_x - t / 2.0, cy, 0.0, t, h),             # west
        ]

    def occupancy_rects(self) -> List[Rect]:
        """Footprints of everything the LiDAR can see (used to draw the map)."""
        rects = self.wall_rects()
        rects += [Rect(r.x, r.y, r.yaw, r.length, r.depth) for r in self.racks]
        t = self.drop_station
        rects.append(Rect(t.x, t.y, t.yaw, t.size_x, t.size_y))
        rects += [Rect(o.x, o.y, o.yaw, o.size_x, o.size_y) for o in self.obstacles]
        d = self.dock_station
        rects.append(Rect(d.x, d.y, d.yaw, d.size_x, d.size_y))
        return rects

    def items_on_rack(self, rack_id: str) -> List[ItemHome]:
        return [i for i in self.item_homes if i.rack_id == rack_id]

    def to_web(self) -> dict:
        """Plain-JSON description of the warehouse for the order web page's live map."""
        def corners(rect: Rect):
            return [[round(x, 3), round(y, 3)] for x, y in rect.corners()]

        racks = {r.id: r for r in self.racks}
        t = self.drop_station
        return {
            'name': self.name,
            'room': {'min_x': self.room_min_x, 'max_x': self.room_max_x,
                     'min_y': self.room_min_y, 'max_y': self.room_max_y},
            'racks': [{'id': r.id, 'name': r.name, 'color': list(r.color),
                       'corners': corners(Rect(r.x, r.y, r.yaw, r.length, r.depth)),
                       'product': r.product.id if r.product else None} for r in self.racks],
            'zones': [{'id': z.id, 'name': z.name, 'kind': z.kind, 'corners': corners(z.area),
                       'goal': {'x': round(z.goal.x, 3), 'y': round(z.goal.y, 3),
                                'yaw': round(z.goal.yaw, 3)}} for z in self.zones.values()],
            'counter': {'name': t.name, 'corners': corners(Rect(t.x, t.y, t.yaw,
                                                                  t.size_x, t.size_y))},
            'dock': {'x': self.dock.x, 'y': self.dock.y, 'yaw': self.dock.yaw},
            'obstacles': [{'name': o.name, 'corners': corners(Rect(o.x, o.y, o.yaw,
                                                                    o.size_x, o.size_y))}
                          for o in self.obstacles],
            'products': [{'id': p.id, 'name': p.name, 'rack_id': p.rack_id,
                          'rack_name': p.rack_name, 'units': p.units, 'shape': p.shape,
                          'color': list(racks[p.rack_id].color)}
                         for p in self.products.values()],
        }
