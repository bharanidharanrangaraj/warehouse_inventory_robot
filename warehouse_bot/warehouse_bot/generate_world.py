#!/usr/bin/env python3
"""Generate the Gazebo world, the Nav2 map, textures and tag models from the layout.

    ros2 run warehouse_bot generate_world --package-dir ~/warehouse_ws/src/warehouse_bot
    # or, without ROS, from the package folder:
    python3 warehouse_bot/generate_world.py --package-dir .

Outputs (inside --package-dir):
    worlds/warehouse.world            Gazebo Classic world: pallet racks with signs,
                                      pickable components, user counter, charging
                                      station, textured floor, lights, overview camera
    maps/warehouse_map.pgm + .yaml    ground-truth occupancy map for Nav2
    models/aruco_marker_<id>/         one textured tag model per marker id
    models/warehouse_textures/        floor texture and sign boards (OGRE materials)
    models/warehouse_crate/           crate you can drop in the robot's path

The robot itself is described in rover_description.py and spawned by world.launch.py.
Because world and map come from the same geometry they always match, which
makes AMCL localisation reliable. Rebuild the package after regenerating.
"""

import argparse
import math
import os
import sys

import numpy as np

try:
    from warehouse_bot.layout import WarehouseLayout
except ImportError:  # running as a plain script from the source tree
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from warehouse_bot.layout import WarehouseLayout

TAG_TEXTURE_PX = 512       # texture size (tag 336 px + 88 px white quiet zone each side)
TAG_INNER_PX = 336
TEX = 'warehouse_textures'

# Palette
RACK_BLUE = (0.10, 0.30, 0.66)
BEAM_ORANGE = (0.96, 0.47, 0.08)
GALV = (0.74, 0.76, 0.79)
DARK = (0.20, 0.21, 0.23)
WALL = (0.88, 0.88, 0.85)
SAFETY_YELLOW = (0.98, 0.80, 0.08)
CARDBOARD = (0.70, 0.53, 0.33)

# Fixed overview camera (also the Gazebo GUI start view): whole room, from the south
OVERVIEW_POSE = (0.0, -6.5, 7.0, 0.0, 0.80, math.pi / 2)


# --------------------------------------------------------------------------- #
#  SDF helpers
# --------------------------------------------------------------------------- #
def fmt(*values) -> str:
    return ' '.join(f'{v:.4f}' for v in values)


def material(rgb, specular=0.05, emissive=None) -> str:
    r, g, b = rgb
    return (f'<material><ambient>{r * 0.7:.3f} {g * 0.7:.3f} {b * 0.7:.3f} 1</ambient>'
            f'<diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>'
            f'<specular>{specular:.2f} {specular:.2f} {specular:.2f} 1</specular>'
            + (f'<emissive>{fmt(*emissive)} 1</emissive>' if emissive else '') + '</material>')


def script_material(name, uri=f'model://{TEX}/materials/scripts',
                    tex_uri=f'model://{TEX}/materials/textures') -> str:
    return f'<material><script><uri>{uri}</uri><uri>{tex_uri}</uri><name>{name}</name></script></material>'


GAZEBO_MEDIA = 'file://media/materials/scripts/gazebo.material'


def gazebo_material(name) -> str:
    return f'<material><script><uri>{GAZEBO_MEDIA}</uri><name>{name}</name></script></material>'


def visual(name, pose, geometry, rgb=None, mat=None, shadows=True, specular=0.05, emissive=None) -> str:
    m = mat if mat is not None else material(rgb, specular, emissive)
    return (f'<visual name="{name}"><pose>{fmt(*pose)}</pose>'
            f'<cast_shadows>{"true" if shadows else "false"}</cast_shadows>'
            f'<geometry>{geometry}</geometry>{m}</visual>')


def box_visual(name, pose, size, rgb=None, **kw) -> str:
    return visual(name, pose, box(*size), rgb, **kw)


def box_collision(name, pose, size) -> str:
    return (f'<collision name="{name}"><pose>{fmt(*pose)}</pose>'
            f'<geometry><box><size>{fmt(*size)}</size></box></geometry></collision>')


def static_model(name, x, y, yaw, body) -> str:
    return (f'\n    <model name="{name}"><static>true</static>'
            f'<pose>{fmt(x, y, 0, 0, 0, yaw)}</pose>'
            f'<link name="link">{body}</link></model>')


def cylinder(radius, length) -> str:
    return f'<cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder>'


def box(sx, sy, sz) -> str:
    return f'<box><size>{fmt(sx, sy, sz)}</size></box>'


def sign_board(name, pose, width, height, sign) -> str:
    """Thin board with a printed sign texture (readable from the side it faces, +x of
    `pose`'s yaw) plus a dark frame."""
    x, y, z, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    face = visual(f'{name}_face', (x, y, z, 0, 0, yaw), box(0.006, width, height),
                  mat=script_material(f'Warehouse/Sign_{sign}'), shadows=False)
    bx, by = x - 0.006 * c, y - 0.006 * s
    frame = box_visual(f'{name}_frame', (bx, by, z, 0, 0, yaw), (0.008, width + 0.03, height + 0.03),
                       DARK)
    return face + frame


# --------------------------------------------------------------------------- #
#  Electronic components (visual only; z = 0 is the bottom of the part)
# --------------------------------------------------------------------------- #
HALF_PI = math.pi / 2
LEAD = (0.72, 0.72, 0.75)


def _axial(prefix, s, oz, body_rgb, body_r, body_len, bands):
    """Axial part lying along x (resistor, diode): body, coloured bands, two leads."""
    z = oz + body_r * s + 0.002
    parts = [visual(f'{prefix}body', (0, 0, z, 0, HALF_PI, 0),
                    cylinder(body_r * s, body_len * s), body_rgb, specular=0.3)]
    for i, (x, width, rgb) in enumerate(bands):
        parts.append(visual(f'{prefix}band{i}', (x * s, 0, z, 0, HALF_PI, 0),
                            cylinder((body_r + 0.0012) * s, width * s), rgb, specular=0.3))
    lead_len = 0.04
    for i, side in enumerate((-1, 1)):
        x = side * (body_len / 2 + lead_len / 2) * s
        parts.append(visual(f'{prefix}lead{i}', (x, 0, z, 0, HALF_PI, 0),
                            cylinder(0.004 * s, lead_len * s), LEAD, specular=0.6))
    return parts


def component_visuals(shape, s=1.0, prefix='', color=(0.6, 0.6, 0.6), oz=0.0):
    """Visuals for one component, scaled by s (1.0 = the pickable unit, ~0.15 m),
    standing on the plane z = oz of the link they are added to."""
    if shape == 'capacitor':          # blue electrolytic can with a polarity stripe
        return [visual(f'{prefix}can', (0, 0, oz + 0.065 * s, 0, 0, 0),
                       cylinder(0.045 * s, 0.13 * s), (0.12, 0.28, 0.70), specular=0.5),
                visual(f'{prefix}top', (0, 0, oz + 0.133 * s, 0, 0, 0),
                       cylinder(0.042 * s, 0.006 * s), (0.80, 0.80, 0.84), specular=0.8),
                visual(f'{prefix}stripe', (0.0425 * s, 0, oz + 0.066 * s, 0, 0, 0),
                       box(0.006 * s, 0.028 * s, 0.124 * s), (0.88, 0.88, 0.95))]
    if shape == 'resistor':           # 1 kOhm: brown-black-red, gold tolerance band
        return _axial(prefix, s, oz, (0.82, 0.72, 0.52), 0.03, 0.09,
                      [(-0.028, 0.009, (0.45, 0.25, 0.10)), (-0.010, 0.009, (0.05, 0.05, 0.05)),
                       (0.008, 0.009, (0.80, 0.10, 0.10)), (0.032, 0.009, (0.85, 0.68, 0.20))])
    if shape == 'diode':              # black body, silver cathode band
        return _axial(prefix, s, oz, (0.08, 0.08, 0.09), 0.028, 0.09,
                      [(0.030, 0.014, (0.85, 0.85, 0.88))])
    if shape == 'transformer':        # laminated core with a copper winding
        return [visual(f'{prefix}base', (0, 0, oz + 0.005 * s, 0, 0, 0),
                       box(0.15 * s, 0.10 * s, 0.01 * s), (0.20, 0.20, 0.22)),
                visual(f'{prefix}core', (0, 0, oz + 0.06 * s, 0, 0, 0),
                       box(0.12 * s, 0.085 * s, 0.10 * s), (0.38, 0.38, 0.42), specular=0.4),
                visual(f'{prefix}coil', (0, 0, oz + 0.06 * s, 0, 0, 0),
                       box(0.065 * s, 0.095 * s, 0.075 * s), (0.78, 0.42, 0.12), specular=0.6)]
    return [visual(f'{prefix}box', (0, 0, oz + 0.06 * s, 0, 0, 0),
                   box(0.12 * s, 0.12 * s, 0.12 * s), color)]


def item_model(item, shape, color) -> str:
    """A pickable unit. Kinematic and gravity-free with no collision: it never pushes the
    robot or blocks the LiDAR, and item_handler moves it with /gazebo/set_entity_state
    while the gripper holds it. (Static models cannot be used: Gazebo does not send pose
    updates of static models to the GUI.)"""
    body = ''.join(component_visuals(shape, 1.0, color=color))
    return (f'\n    <model name="{item.name}"><static>false</static>'
            f'<pose>{fmt(item.x, item.y, item.z, 0, 0, item.yaw)}</pose>'
            f'<link name="link"><gravity>false</gravity><kinematic>true</kinematic>'
            f'<inertial><mass>0.1</mass><inertia><ixx>1e-4</ixx><iyy>1e-4</iyy><izz>1e-4</izz>'
            f'<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>{body}</link></model>')


# --------------------------------------------------------------------------- #
#  World elements
# --------------------------------------------------------------------------- #
def floor_model(layout) -> str:
    """Polished concrete slab covering the room (visual; the ground plane collides)."""
    w = layout.room_max_x - layout.room_min_x
    h = layout.room_max_y - layout.room_min_y
    cx = (layout.room_min_x + layout.room_max_x) / 2
    cy = (layout.room_min_y + layout.room_max_y) / 2
    return static_model('warehouse_floor', 0, 0, 0, visual(
        'slab', (cx, cy, -0.0045, 0, 0, 0), box(w, h, 0.01), mat=script_material('Warehouse/Concrete'),
        shadows=False))


def walls_model(layout) -> str:
    hgt = layout.wall_height
    body = ''
    cx = (layout.room_min_x + layout.room_max_x) / 2
    cy = (layout.room_min_y + layout.room_max_y) / 2
    for i, r in enumerate(layout.wall_rects()):
        pose = (r.x, r.y, hgt / 2, 0, 0, r.yaw)
        size = (r.size_x, r.size_y, hgt)
        body += box_collision(f'wall_{i}', pose, size)
        body += box_visual(f'wall_{i}', pose, size, WALL)
        # yellow safety band + dark top trim on the room side
        inward = (cx - r.x, cy - r.y)
        n = math.hypot(*inward)
        ox, oy = (inward[0] / n, inward[1] / n)
        horiz = r.size_x > r.size_y
        t = (r.size_y if horiz else r.size_x) / 2 + 0.004
        bx, by = r.x + ox * t if not horiz else r.x, r.y + oy * t if horiz else r.y
        band = (r.size_x - 0.3, 0.006, 0.10) if horiz else (0.006, r.size_y - 0.02, 0.10)
        body += box_visual(f'band_{i}', (bx, by, 0.07, 0, 0, 0), band, SAFETY_YELLOW, shadows=False)
        # cap sits slightly above the wall top (no coplanar faces -> no flicker)
        trim = (r.size_x + 0.02, r.size_y + 0.02, 0.03)
        body += box_visual(f'trim_{i}', (r.x, r.y, hgt + 0.005, 0, 0, 0), trim, DARK)
    return static_model('warehouse_walls', 0, 0, 0, body)


def rack_model(rack) -> str:
    """Pallet racking: blue upright frames with bracing, orange beams, wire decks,
    storage totes, a plinth carrying the ArUco bin labels, and printed signs."""
    L, D, H = rack.length, rack.depth, rack.height
    f = rack.face
    parts = [box_collision('body', (0, 0, H / 2, 0, 0, 0), (L, D, H))]   # = map footprint
    parts.append(box_visual('plinth', (0, 0, 0.15, 0, 0, 0), (L - 0.01, D - 0.01, 0.30), DARK))
    # back mesh panel
    parts.append(box_visual('back_mesh', (0, -f * (D / 2 - 0.012), 0.30 + (H - 0.30) / 2, 0, 0, 0),
                            (L - 0.08, 0.008, H - 0.32), (0.56, 0.58, 0.61), specular=0.3))
    # uprights
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        parts.append(box_visual(f'upright_{i}', (sx * (L / 2 - 0.03), sy * (D / 2 - 0.03), H / 2, 0, 0, 0),
                                (0.06, 0.06, H), RACK_BLUE, specular=0.3))
    # end-frame bracing (zig-zag on both ends)
    dy = D - 0.06
    for e, sx in enumerate((1, -1)):
        x = sx * (L / 2 - 0.03)
        zs = [0.32, 0.62, 0.92, H - 0.06]
        for k in range(len(zs) - 1):
            z0, z1 = zs[k], zs[k + 1]
            ang = math.atan2(z1 - z0, dy) * (1 if k % 2 == 0 else -1)
            length = math.hypot(dy, z1 - z0)
            parts.append(box_visual(f'brace_{e}_{k}', (x, 0, (z0 + z1) / 2, ang, 0, 0),
                                    (0.022, length, 0.022), RACK_BLUE))
        parts.append(box_visual(f'tie_{e}', (x, 0, H - 0.05, 0, 0, 0), (0.022, dy, 0.03), RACK_BLUE))
    # beams + wire decks
    for i, z in enumerate((0.31, 0.66, H - 0.02)):
        parts.append(box_visual(f'deck_{i}', (0, 0, z, 0, 0, 0), (L - 0.07, D - 0.07, 0.02), GALV,
                                specular=0.4))
        for side in (1, -1):
            parts.append(box_visual(f'beam_{i}_{side}', (0, side * (D / 2 - 0.03), z - 0.035, 0, 0, 0),
                                    (L - 0.06, 0.045, 0.07), BEAM_ORANGE, specular=0.35))
    # storage totes on the middle shelf, in the rack colour
    n = 3
    tote = rack.color
    rim = tuple(min(1.0, 0.25 + v) for v in tote)
    for j in range(n):
        x = -L / 2 + L / n * (j + 0.5)
        parts.append(box_visual(f'tote_{j}', (x, 0, 0.675 + 0.08, 0, 0, 0),
                                (L / n * 0.84, D * 0.74, 0.16), tote, specular=0.25))
        parts.append(box_visual(f'tote_rim_{j}', (x, 0, 0.675 + 0.165, 0, 0, 0),
                                (L / n * 0.88, D * 0.78, 0.014), rim))
    # empty bottom shelf without a product? fill it with cartons
    if rack.product is None:
        for j in range(n):
            x = -L / 2 + L / n * (j + 0.5)
            parts.append(box_visual(f'carton_{j}', (x, 0, 0.325 + 0.11, 0, 0, 0),
                                    (L / n * 0.8, D * 0.7, 0.22), CARDBOARD))
    if rack.product is not None:
        # big copy of the product on top of the rack, so the racks are easy to tell apart
        parts += component_visuals(rack.product.shape, 2.4, prefix='icon_', color=rack.color, oz=H)
    # signs on the aisle side and on the back
    zs = 1.00
    parts.append(sign_board('sign_front', (0, f * (D / 2 + 0.016), zs, f * math.pi / 2), 0.78, 0.15,
                            rack.id))
    parts.append(sign_board('sign_back', (0, -f * (D / 2 + 0.016), zs, -f * math.pi / 2), 0.78, 0.15,
                            rack.id))
    return static_model(f'rack_{rack.id}', rack.x, rack.y, rack.yaw, ''.join(parts))


def counter_model(layout) -> str:
    """User counter: grey cabinet, wooden top, accent stripe and a sign."""
    t = layout.drop_station
    sx, sy, sz = t.size_x, t.size_y, t.size_z
    parts = [box_collision('body', (0, 0, sz / 2, 0, 0, 0), (sx, sy, sz))]
    parts.append(box_visual('plinth', (0, 0, 0.03, 0, 0, 0), (sx - 0.04, sy - 0.04, 0.06), DARK))
    parts.append(box_visual('cabinet', (0, 0, 0.06 + (sz - 0.09) / 2, 0, 0, 0),
                            (sx - 0.01, sy - 0.01, sz - 0.09), (0.86, 0.87, 0.89), specular=0.2))
    parts.append(box_visual('stripe', (-sx / 2 - 0.002, 0, sz * 0.55, 0, 0, 0), (0.006, sy - 0.04, 0.05),
                            t.color, specular=0.3))
    parts.append(visual('top', (0, 0, sz - 0.015, 0, 0, 0), box(sx + 0.03, sy + 0.04, 0.03),
                        mat=gazebo_material('Gazebo/Wood')))
    # sign on a post at the back, facing the robot (-x)
    parts.append(box_visual('post', (sx / 2 - 0.03, 0, sz + 0.22, 0, 0, 0), (0.03, 0.03, 0.44), DARK))
    parts.append(sign_board('sign', (sx / 2 - 0.05, 0, sz + 0.36, math.pi), 0.62, 0.15, 'Counter'))
    return static_model('user_counter', t.x, t.y, t.yaw, ''.join(parts))


def charging_station_model(layout) -> str:
    d = layout.dock_station
    parts = [box_collision('body', (0, 0, d.size_z / 2, 0, 0, 0), (d.size_x, d.size_y, d.size_z))]
    parts.append(box_visual('body', (0, 0, d.size_z / 2, 0, 0, 0), (d.size_x, d.size_y, d.size_z),
                            d.color, specular=0.3))
    parts.append(box_visual('front', (d.size_x / 2 + 0.002, 0, d.size_z / 2, 0, 0, 0),
                            (0.004, d.size_y - 0.06, d.size_z - 0.08), (0.30, 0.32, 0.35)))
    parts.append(box_visual('contacts', (d.size_x / 2 + 0.006, 0, 0.12, 0, 0, 0), (0.006, 0.26, 0.035),
                            (0.2, 1.0, 0.4), emissive=(0.1, 0.8, 0.3), shadows=False))
    parts.append(box_visual('lamp', (0, 0, d.size_z + 0.01, 0, 0, 0), (0.06, 0.2, 0.02),
                            (0.2, 1.0, 0.4), emissive=(0.1, 0.8, 0.3)))
    parts.append(sign_board('sign', (0.02, 0, d.size_z + 0.16, 0.0), 0.56, 0.14, 'Dock'))
    return static_model('charging_station', d.x, d.y, d.yaw, ''.join(parts))


def obstacle_model(o) -> str:
    sx, sy, sz = o.size_x, o.size_y, o.size_z
    parts = [box_collision('body', (0, 0, sz / 2, 0, 0, 0), (sx, sy, sz))]
    if 'pallet' in o.name:
        parts.append(visual('pallet', (0, 0, 0.06, 0, 0, 0), box(sx, sy, 0.12),
                            mat=gazebo_material('Gazebo/WoodPallet')))
        h = sz - 0.12
        for i, (px, py) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
            bx, by = px * sx / 4, py * sy / 4
            bh = h if i % 3 == 0 else h * 0.82
            parts.append(box_visual(f'carton_{i}', (bx, by, 0.12 + bh / 2, 0, 0, 0),
                                    (sx / 2 - 0.02, sy / 2 - 0.02, bh), CARDBOARD))
            parts.append(box_visual(f'tape_{i}', (bx, by, 0.12 + bh + 0.001, 0, 0, 0),
                                    (sx / 2 - 0.02, 0.05, 0.002), (0.82, 0.70, 0.50), shadows=False))
    elif 'pillar' in o.name:
        parts.append(box_visual('body', (0, 0, sz / 2, 0, 0, 0), (sx, sy, sz), SAFETY_YELLOW, specular=0.2))
        for k, z in enumerate((0.15, 0.45, 0.75)):
            parts.append(box_visual(f'band_{k}', (0, 0, z, 0, 0, 0), (sx + 0.006, sy + 0.006, 0.08), DARK))
    else:
        parts.append(box_visual('body', (0, 0, sz / 2, 0, 0, 0), (sx, sy, sz), o.color))
    return static_model(o.name, o.x, o.y, o.yaw, ''.join(parts))


def floor_markings_model(layout) -> str:
    """Visual-only floor paint: zone pads with borders, aisle lines (no collision)."""
    parts = []
    racks = {r.id: r for r in layout.racks}
    for i, zone in enumerate(layout.zones.values()):
        a = zone.area
        if zone.kind == 'RACK':
            c = racks[zone.id].color
        elif zone.kind == 'DROP':
            c = (0.15, 0.45, 0.85)
        else:
            c = (0.10, 0.70, 0.45)
        pale = tuple(0.62 + 0.38 * v for v in c)
        parts.append(box_visual(f'zone_{i}', (a.x, a.y, 0.0008, 0, 0, a.yaw),
                                (a.size_x, a.size_y, 0.0016), pale, shadows=False))
        cs, sn = math.cos(a.yaw), math.sin(a.yaw)
        for k, (lx, ly, bx, by) in enumerate(((a.size_x / 2, 0, 0.05, a.size_y),
                                              (-a.size_x / 2, 0, 0.05, a.size_y),
                                              (0, a.size_y / 2, a.size_x, 0.05),
                                              (0, -a.size_y / 2, a.size_x, 0.05))):
            parts.append(box_visual(f'zone_{i}_edge_{k}', (a.x + cs * lx - sn * ly, a.y + sn * lx + cs * ly,
                                                           0.0012, 0, 0, a.yaw),
                                    (bx, by, 0.0024), c, shadows=False))
    # yellow lines along the main east-west aisle
    rack_zones = layout.rack_zones()
    north = [z for z in rack_zones if z.area.y > 0]
    south = [z for z in rack_zones if z.area.y <= 0]
    span = layout.room_max_x - layout.room_min_x - 1.0
    cx = (layout.room_min_x + layout.room_max_x) / 2
    if north:
        y = min(z.area.y - z.area.size_y / 2 for z in north) - 0.10
        parts.append(box_visual('aisle_line_n', (cx, y, 0.0012, 0, 0, 0), (span, 0.07, 0.0024),
                                SAFETY_YELLOW, shadows=False))
    if south:
        y = max(z.area.y + z.area.size_y / 2 for z in south) + 0.10
        parts.append(box_visual('aisle_line_s', (cx, y, 0.0012, 0, 0, 0), (span, 0.07, 0.0024),
                                SAFETY_YELLOW, shadows=False))
    return static_model('floor_markings', 0, 0, 0, ''.join(parts))


def overview_camera_model() -> str:
    """Fixed camera over the south wall; publishes /warehouse/overview/image_raw (1 Hz)
    for the order page's live view."""
    return (f'\n    <model name="overview_camera"><static>true</static>'
            f'<pose>{fmt(*OVERVIEW_POSE)}</pose><link name="link">'
            '<sensor name="overview" type="camera"><always_on>true</always_on>'
            '<update_rate>1</update_rate><camera name="overview"><horizontal_fov>1.30</horizontal_fov>'
            '<image><width>1280</width><height>720</height><format>R8G8B8</format></image>'
            '<clip><near>0.1</near><far>40</far></clip></camera>'
            '<plugin name="overview_camera" filename="libgazebo_ros_camera.so">'
            '<ros><namespace>/warehouse</namespace></ros><camera_name>overview</camera_name>'
            '<frame_name>overview_camera</frame_name></plugin></sensor></link></model>')


def build_world(layout) -> str:
    models = [floor_model(layout), walls_model(layout), floor_markings_model(layout)]
    models += [rack_model(r) for r in layout.racks]
    models.append(counter_model(layout))
    models.append(charging_station_model(layout))
    models += [obstacle_model(o) for o in layout.obstacles]
    models.append(overview_camera_model())
    tags = ''.join(
        f'\n    <include><uri>model://aruco_marker_{m.marker_id}</uri>'
        f'<name>tag_{m.zone_id}_{m.marker_id}</name>'
        f'<pose>{fmt(m.x, m.y, m.z, 0, 0, m.yaw)}</pose></include>'
        for m in layout.placed_markers)
    racks = {r.id: r for r in layout.racks}
    tags += ''.join(item_model(i, layout.products[i.product_id].shape, racks[i.rack_id].color)
                    for i in layout.item_homes)
    size = 2 * max(layout.room_max_x - layout.room_min_x,
                   layout.room_max_y - layout.room_min_y)
    return f'''<?xml version="1.0"?>
<!-- GENERATED by warehouse_bot/generate_world.py from config/warehouse_layout.yaml.
     Edit the layout file and regenerate instead of editing this file by hand. -->
<sdf version="1.6">
  <world name="warehouse">
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <!-- /gazebo/get_entity_state + /gazebo/set_entity_state: item_handler uses them to
         keep a gripped component in the rover's gripper / cargo tray -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>1.0</update_rate>
    </plugin>
    <scene>
      <ambient>0.50 0.50 0.52 1</ambient>
      <background>0.72 0.78 0.85 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
      <origin_visual>false</origin_visual>
    </scene>
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>{fmt(*OVERVIEW_POSE)}</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.80 0.79 0.76 1</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
      <direction>-0.45 0.35 -0.82</direction>
    </light>
    <light name="fill" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.32 0.33 0.36 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>0.4 -0.5 -0.75</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{size:.1f} {size:.1f}</size></plane></geometry>
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <pose>0 0 -0.012 0 0 0</pose>
          <geometry><plane><normal>0 0 1</normal><size>{size:.1f} {size:.1f}</size></plane></geometry>
          {material((0.50, 0.51, 0.50))}
        </visual>
      </link>
    </model>{''.join(models)}{tags}
  </world>
</sdf>
'''


# --------------------------------------------------------------------------- #
#  Textures (OGRE material scripts + PNGs)
# --------------------------------------------------------------------------- #
SIGN_W, SIGN_H = 1024, 200


def concrete_image(px=512, seed=7) -> np.ndarray:
    """One 1 m x 1 m slab of light polished concrete with a joint line on its edges."""
    import cv2
    rng = np.random.default_rng(seed)
    low = cv2.resize(rng.normal(0, 1, (8, 8)).astype(np.float32), (px, px), interpolation=cv2.INTER_CUBIC)
    mid = cv2.resize(rng.normal(0, 1, (48, 48)).astype(np.float32), (px, px), interpolation=cv2.INTER_CUBIC)
    grain = rng.normal(0, 1, (px, px)).astype(np.float32)
    v = 168 + 7 * low + 4 * mid + 3.5 * grain
    v[:2, :] -= 30
    v[:, :2] -= 30
    v = np.clip(v, 0, 255)
    img = np.stack([v - 2, v, v + 3], axis=-1)              # slightly warm grey (BGR)
    return np.clip(img, 0, 255).astype(np.uint8)


def sign_image(badge: str, title: str, subtitle: str, rgb) -> np.ndarray:
    """White sign: coloured badge with a letter on the left, title + subtitle on the right."""
    import cv2
    img = np.full((SIGN_H, SIGN_W, 3), 250, np.uint8)
    bgr = tuple(int(255 * c) for c in reversed(rgb))
    cv2.rectangle(img, (0, 0), (SIGN_H, SIGN_H), bgr, -1)
    font = cv2.FONT_HERSHEY_DUPLEX
    (tw, th), _ = cv2.getTextSize(badge, font, 4.2, 9)
    cv2.putText(img, badge, ((SIGN_H - tw) // 2, (SIGN_H + th) // 2), font, 4.2, (255, 255, 255), 9,
                cv2.LINE_AA)
    scale = 2.5
    while cv2.getTextSize(title, font, scale, 5)[0][0] > SIGN_W - SIGN_H - 60 and scale > 1.0:
        scale -= 0.1
    cv2.putText(img, title, (SIGN_H + 34, 112), font, scale, (25, 28, 33), 5, cv2.LINE_AA)
    cv2.putText(img, subtitle, (SIGN_H + 38, 165), cv2.FONT_HERSHEY_SIMPLEX, 1.25, (90, 95, 105), 2,
                cv2.LINE_AA)
    cv2.rectangle(img, (0, 0), (SIGN_W - 1, SIGN_H - 1), (40, 40, 40), 6)
    return img


def write_textures(layout, models_dir: str) -> None:
    import cv2
    root = os.path.join(models_dir, TEX)
    os.makedirs(os.path.join(root, 'materials', 'scripts'), exist_ok=True)
    os.makedirs(os.path.join(root, 'materials', 'textures'), exist_ok=True)
    tex = os.path.join(root, 'materials', 'textures')
    w = layout.room_max_x - layout.room_min_x
    h = layout.room_max_y - layout.room_min_y
    cv2.imwrite(os.path.join(tex, 'concrete.png'), concrete_image())

    signs = {}
    for r in layout.racks:
        sub = (f'{r.product.name}s  -  {r.product.units} units on the bottom shelf'
               if r.product else 'storage rack')
        signs[r.id] = sign_image(r.id, r.name.upper(), sub, r.color)
    signs['Counter'] = sign_image('U', 'USER COUNTER', 'pick up your components here', (0.15, 0.45, 0.85))
    signs['Dock'] = sign_image('C', 'CHARGING DOCK', 'robot parking', (0.10, 0.62, 0.36))
    for key, img in signs.items():
        cv2.imwrite(os.path.join(tex, f'sign_{key}.png'), img)

    scripts = [f'''material Warehouse/Concrete
{{
  technique
  {{
    pass
    {{
      ambient 0.85 0.85 0.85 1
      diffuse 0.95 0.95 0.95 1
      specular 0.12 0.12 0.12 1 24
      texture_unit
      {{
        texture concrete.png
        filtering anisotropic
        max_anisotropy 8
        scale {1.0 / w:.5f} {1.0 / h:.5f}
      }}
    }}
  }}
}}
''']
    for key in signs:
        scripts.append(f'''material Warehouse/Sign_{key}
{{
  technique
  {{
    pass
    {{
      lighting off
      texture_unit
      {{
        texture sign_{key}.png
        filtering anisotropic
        max_anisotropy 8
      }}
    }}
  }}
}}
''')
    with open(os.path.join(root, 'materials', 'scripts', 'warehouse.material'), 'w') as f:
        f.write('\n'.join(scripts))
    with open(os.path.join(root, 'model.config'), 'w') as f:
        f.write(f'''<?xml version="1.0"?>
<model>
  <name>{TEX}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author><name>warehouse_bot generator</name><email>none@example.com</email></author>
  <description>Floor texture and sign boards for the warehouse world (materials only).</description>
</model>
''')
    with open(os.path.join(root, 'model.sdf'), 'w') as f:
        f.write(f'<?xml version="1.0"?>\n<sdf version="1.6"><model name="{TEX}"><static>true</static>'
                '<link name="link"/></model></sdf>\n')


# --------------------------------------------------------------------------- #
#  Tag + crate models
# --------------------------------------------------------------------------- #
def tag_image(marker_id: int, dictionary: str) -> np.ndarray:
    import cv2
    aruco = cv2.aruco
    d = aruco.getPredefinedDictionary(getattr(aruco, dictionary))
    if hasattr(aruco, 'generateImageMarker'):
        img = aruco.generateImageMarker(d, marker_id, TAG_INNER_PX)
    else:
        img = aruco.drawMarker(d, marker_id, TAG_INNER_PX)
    pad = (TAG_TEXTURE_PX - TAG_INNER_PX) // 2
    return cv2.copyMakeBorder(img, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)


def write_tag_model(models_dir: str, marker_id: int, dictionary: str, size: float) -> None:
    import cv2
    name = f'aruco_marker_{marker_id}'
    root = os.path.join(models_dir, name)
    os.makedirs(os.path.join(root, 'materials', 'scripts'), exist_ok=True)
    os.makedirs(os.path.join(root, 'materials', 'textures'), exist_ok=True)
    cv2.imwrite(os.path.join(root, 'materials', 'textures', f'{name}.png'),
                tag_image(marker_id, dictionary))
    with open(os.path.join(root, 'materials', 'scripts', f'{name}.material'), 'w') as f:
        f.write(f'''material ArucoMarker{marker_id}
{{
  technique
  {{
    pass
    {{
      lighting off
      texture_unit
      {{
        texture {name}.png
        filtering none
      }}
    }}
  }}
}}
''')
    with open(os.path.join(root, 'model.config'), 'w') as f:
        f.write(f'''<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author><name>warehouse_bot generator</name><email>none@example.com</email></author>
  <description>ArUco {dictionary} tag id {marker_id} used as an inventory bin label.</description>
</model>
''')
    with open(os.path.join(root, 'model.sdf'), 'w') as f:
        f.write(f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="tag">
        <cast_shadows>false</cast_shadows>
        <geometry><box><size>{size:.3f} 0.002 {size:.3f}</size></box></geometry>
        <material>
          <script>
            <uri>model://{name}/materials/scripts</uri>
            <uri>model://{name}/materials/textures</uri>
            <name>ArucoMarker{marker_id}</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
''')


def write_crate_model(models_dir: str) -> None:
    root = os.path.join(models_dir, 'warehouse_crate')
    os.makedirs(root, exist_ok=True)
    with open(os.path.join(root, 'model.config'), 'w') as f:
        f.write('''<?xml version="1.0"?>
<model>
  <name>warehouse_crate</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author><name>warehouse_bot generator</name><email>none@example.com</email></author>
  <description>Static 0.4 m crate for dynamic obstacle-avoidance tests.</description>
</model>
''')
    body = (box_collision('body', (0, 0, 0.2, 0, 0, 0), (0.4, 0.4, 0.4))
            + box_visual('body', (0, 0, 0.2, 0, 0, 0), (0.4, 0.4, 0.4), (0.85, 0.30, 0.15)))
    with open(os.path.join(root, 'model.sdf'), 'w') as f:
        f.write('<?xml version="1.0"?>\n<sdf version="1.6">\n  <model name="warehouse_crate">'
                f'<static>true</static><link name="link">{body}</link></model>\n</sdf>\n')


# --------------------------------------------------------------------------- #
#  Map
# --------------------------------------------------------------------------- #
def build_map(layout):
    """Rasterise every obstacle footprint into a trinary occupancy grid."""
    res = layout.map_resolution
    margin = 0.5
    ox, oy = layout.room_min_x - margin, layout.room_min_y - margin
    width = int(round((layout.room_max_x - layout.room_min_x + 2 * margin) / res))
    height = int(round((layout.room_max_y - layout.room_min_y + 2 * margin) / res))
    xs = ox + (np.arange(width) + 0.5) * res
    ys = oy + (np.arange(height) + 0.5) * res
    X, Y = np.meshgrid(xs, ys)                     # row 0 = lowest y
    grid = np.full((height, width), 205, dtype=np.uint8)               # unknown
    inside = ((X > layout.room_min_x) & (X < layout.room_max_x)
              & (Y > layout.room_min_y) & (Y < layout.room_max_y))
    grid[inside] = 254                                                # free
    for r in layout.occupancy_rects():
        c, s = math.cos(r.yaw), math.sin(r.yaw)
        dx, dy = X - r.x, Y - r.y
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        occupied = ((np.abs(lx) <= r.size_x / 2 + res / 2)
                    & (np.abs(ly) <= r.size_y / 2 + res / 2))
        grid[occupied] = 0                                            # occupied
    return np.flipud(grid), (ox, oy)               # PGM row 0 = highest y


def write_map(layout, maps_dir: str, name: str = 'warehouse_map') -> None:
    os.makedirs(maps_dir, exist_ok=True)
    grid, (ox, oy) = build_map(layout)
    with open(os.path.join(maps_dir, f'{name}.pgm'), 'wb') as f:
        f.write(f'P5\n'
                f'{grid.shape[1]} {grid.shape[0]}\n255\n'.encode())
        f.write(grid.tobytes())
    with open(os.path.join(maps_dir, f'{name}.yaml'), 'w') as f:
        f.write(f'''# Generated from config/warehouse_layout.yaml (ground-truth map)
image: {name}.pgm
mode: trinary
resolution: {layout.map_resolution}
origin: [{ox:.3f}, {oy:.3f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.15
''')


# --------------------------------------------------------------------------- #
def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--package-dir', default='.',
                        help='warehouse_bot source folder (default: current directory)')
    parser.add_argument('--layout', default=None,
                        help='layout file (default: <package-dir>/config/warehouse_layout.yaml)')
    parser.add_argument('--skip-tags', action='store_true',
                        help='do not regenerate tag models and textures (no OpenCV needed)')
    args = parser.parse_args(argv)

    pkg = os.path.abspath(os.path.expanduser(args.package_dir))
    layout_path = args.layout or os.path.join(pkg, 'config', 'warehouse_layout.yaml')
    layout = WarehouseLayout.from_file(layout_path)

    os.makedirs(os.path.join(pkg, 'worlds'), exist_ok=True)
    with open(os.path.join(pkg, 'worlds', 'warehouse.world'), 'w') as f:
        f.write(build_world(layout))
    write_map(layout, os.path.join(pkg, 'maps'))

    models_dir = os.path.join(pkg, 'models')
    write_crate_model(models_dir)
    if not args.skip_tags:
        ids = sorted({m.marker_id for m in layout.placed_markers} | set(layout.catalog))
        for mid in ids:
            write_tag_model(models_dir, mid, layout.aruco_dictionary, layout.marker_size)
        write_textures(layout, models_dir)
        print(f'Tag models written for ids {ids}; textures in models/{TEX}')

    print(f'World : {os.path.join(pkg, "worlds", "warehouse.world")}')
    print(f'Map   : {os.path.join(pkg, "maps", "warehouse_map.yaml")}')
    for z in layout.zones.values():
        print(f'Zone {z.id:<5} {z.name:<24} goal=({z.goal.x:6.2f}, {z.goal.y:6.2f}, '
              f'{math.degrees(z.goal.yaw):6.1f} deg)')
    print(f'Counter delivery spots reachable by the arm: {len(layout.counter_slots)}')
    print('Now rebuild:  colcon build --packages-select warehouse_bot')


if __name__ == '__main__':
    main()
