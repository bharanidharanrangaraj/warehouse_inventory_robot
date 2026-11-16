"""The warehouse rover: a 4-wheel skid-steer base with a 4-joint arm and a gripper.

One Python description produces both robot files, so they can never disagree:

    build_urdf()   -> URDF for robot_state_publisher, RViz and ros2_control
    build_sdf(...) -> SDF for Gazebo Classic, with per-part materials (Gazebo drops
                      URDF colours) and the drive, LiDAR, camera, joint-state and
                      ros2_control plugins

    python3 -m warehouse_bot.rover_description --urdf    # print the URDF
    python3 -m warehouse_bot.rover_description --sdf     # print the SDF

Frames: base_footprint on the floor under the rover centre (x forward, y left,
z up); base_link at wheel-axle height. Arm geometry comes from arm_kinematics.
"""

import argparse
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from warehouse_bot import arm_kinematics as ak

# --------------------------------------------------------------------------- #
#  Dimensions (metres)
# --------------------------------------------------------------------------- #
WHEEL_R = 0.085
WHEEL_W = 0.06
WHEEL_X = 0.17                   # front/rear axle position
WHEEL_Y = 0.215                  # wheel centre line, left/right
BASE_Z = WHEEL_R                 # base_link above base_footprint
CHASSIS = (0.52, 0.34, 0.13)     # body box
CHASSIS_Z = 0.03                 # body centre above base_link
DECK_TOP = CHASSIS_Z + CHASSIS[2] / 2 + 0.012     # top of the deck plate (base_link frame)
LIDAR_XYZ = (0.01, 0.0, DECK_TOP + 0.155)          # scan plane, base_link frame
CAMERA_XYZ = (0.275, 0.0, 0.12)                    # optical centre, base_link frame
TURRET_XYZ = (ak.SHOULDER[0], 0.0, DECK_TOP + 0.03)  # turret yaw joint, base_link frame
SHOULDER_UP = ak.SHOULDER[2] - BASE_Z - TURRET_XYZ[2]  # shoulder above the turret joint
FINGER_BASE = 0.075              # finger joints sit this far along the wrist link
FOOTPRINT = (0.31, 0.25)         # half length / half width incl. wheels and bumpers

# Colours (r, g, b)
GRAPHITE = (0.15, 0.16, 0.18)
DECK = (0.78, 0.79, 0.81)
ORANGE = (0.97, 0.45, 0.05)
RUBBER = (0.06, 0.06, 0.07)
ALU = (0.70, 0.72, 0.75)
DARK = (0.09, 0.09, 0.10)
LENS = (0.03, 0.05, 0.09)
HUB = (0.55, 0.57, 0.60)

assert abs(BASE_Z + DECK_TOP - (ak.TRAY[2] - 0.008)) < 1e-6, 'tray floor must sit on the deck'


# --------------------------------------------------------------------------- #
#  Model
# --------------------------------------------------------------------------- #
@dataclass
class Visual:
    name: str
    shape: str                   # box | cylinder | sphere
    size: Tuple[float, ...]      # box (x, y, z) | cylinder (radius, length) | sphere (radius,)
    xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    color: Tuple[float, float, float] = GRAPHITE
    emissive: Optional[Tuple[float, float, float]] = None
    shiny: float = 0.15


@dataclass
class Link:
    name: str
    mass: float = 0.0            # 0 = no inertial (frame only)
    inertia: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    visuals: List[Visual] = field(default_factory=list)
    collision: Optional[Visual] = None
    gravity: bool = True
    friction: Optional[float] = None


@dataclass
class Joint:
    name: str
    kind: str                    # fixed | continuous | revolute | prismatic
    parent: str
    child: str
    xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    axis: Tuple[float, float, float] = (0.0, 0.0, 1.0)
    limits: Optional[Tuple[float, float]] = None
    effort: float = 50.0
    velocity: float = 2.0


def box_inertia(m, x, y, z):
    return (m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12)


def cyl_inertia_y(m, r, h):
    """Cylinder whose axis is the link's y axis (wheels)."""
    side = m * (3 * r * r + h * h) / 12
    return (side, m * r * r / 2, side)


HALF_PI = math.pi / 2
Y_AXIS_CYL = (HALF_PI, 0.0, 0.0)     # rotate a z-axis cylinder onto y


def build_model() -> Tuple[List[Link], List[Joint]]:
    links: List[Link] = []
    joints: List[Joint] = []
    cx, cy, cz = CHASSIS
    top = DECK_TOP

    # ---- body -------------------------------------------------------------
    base = Link('base_link', 14.0, box_inertia(14.0, cx, cy, cz))
    base.collision = Visual('body', 'box', CHASSIS, (0, 0, CHASSIS_Z))
    v = base.visuals
    v.append(Visual('chassis', 'box', CHASSIS, (0, 0, CHASSIS_Z), color=GRAPHITE))
    v.append(Visual('deck', 'box', (cx - 0.02, cy - 0.02, 0.012), (0, 0, top - 0.006), color=DECK))
    for side in (1, -1):
        v.append(Visual(f'stripe_{side}', 'box', (cx - 0.10, 0.004, 0.028),
                        (0, side * (cy / 2 + 0.001), CHASSIS_Z + 0.012), color=ORANGE, shiny=0.4))
        v.append(Visual(f'skirt_{side}', 'box', (cx - 0.02, 0.006, 0.02),
                        (0, side * (cy / 2 + 0.002), CHASSIS_Z - cz / 2 + 0.01), color=DARK))
    for end, name in ((1, 'front'), (-1, 'rear')):
        v.append(Visual(f'bumper_{name}', 'box', (0.035, cy + 0.02, 0.07),
                        (end * (cx / 2 + 0.0175), 0, 0.0), color=RUBBER))
        light = (1.0, 0.97, 0.85) if end > 0 else (1.0, 0.08, 0.04)
        for side in (1, -1):
            v.append(Visual(f'light_{name}_{side}', 'box', (0.006, 0.055, 0.02),
                            (end * (cx / 2 + 0.036), side * 0.11, 0.018),
                            color=light, emissive=light))
    # LiDAR mast + puck + status beacon
    lx, ly, lz = LIDAR_XYZ
    v.append(Visual('lidar_mast', 'cylinder', (0.022, lz - top - 0.02), (lx, ly, (top + lz - 0.02) / 2),
                    color=DARK))
    v.append(Visual('lidar_body', 'cylinder', (0.045, 0.045), (lx, ly, lz - 0.0), color=DARK, shiny=0.5))
    v.append(Visual('lidar_window', 'cylinder', (0.046, 0.014), (lx, ly, lz + 0.004), color=LENS, shiny=0.9))
    v.append(Visual('beacon', 'cylinder', (0.014, 0.018), (lx, ly, lz + 0.032),
                    color=(0.2, 1.0, 0.35), emissive=(0.1, 0.9, 0.3)))
    # front camera on a small bracket
    camx, camy, camz = CAMERA_XYZ
    v.append(Visual('camera_bracket', 'box', (0.03, 0.04, camz - top), (camx - 0.02, 0, (camz + top) / 2 - 0.01),
                    color=DARK))
    v.append(Visual('camera_body', 'box', (0.03, 0.10, 0.032), (camx - 0.012, camy, camz), color=DARK))
    for side in (1, -1):
        v.append(Visual(f'camera_lens_{side}', 'cylinder', (0.011, 0.006),
                        (camx + 0.005, side * 0.03, camz), (0, HALF_PI, 0), color=LENS, shiny=0.9))
    # cargo tray on the rear deck
    tx = ak.TRAY[0]
    v.append(Visual('tray_floor', 'box', (0.20, 0.20, 0.008), (tx, 0, top + 0.004), color=DARK))
    for i, (dx, dy, sx, sy) in enumerate(((0.10, 0, 0.008, 0.20), (-0.10, 0, 0.008, 0.20),
                                          (0, 0.10, 0.20, 0.008), (0, -0.10, 0.20, 0.008))):
        v.append(Visual(f'tray_wall_{i}', 'box', (sx, sy, 0.035), (tx + dx, dy, top + 0.0175),
                        color=ORANGE, shiny=0.4))
    # arm mounting disc
    v.append(Visual('arm_mount', 'cylinder', (0.075, 0.03), (TURRET_XYZ[0], 0, top + 0.015), color=DARK))
    links.append(base)
    joints.append(Joint('base_joint', 'fixed', 'base_footprint', 'base_link', (0, 0, BASE_Z)))

    # ---- wheels -------------------------------------------------------------
    for fx, fname in ((1, 'f'), (-1, 'r')):
        for sy, sname in ((1, 'l'), (-1, 'r')):
            name = f'wheel_{fname}{sname}'
            w = Link(f'{name}_link', 0.8, cyl_inertia_y(0.8, WHEEL_R, WHEEL_W), friction=0.9)
            w.collision = Visual('tyre', 'cylinder', (WHEEL_R, WHEEL_W), rpy=Y_AXIS_CYL)
            w.visuals += [
                Visual('tyre', 'cylinder', (WHEEL_R, WHEEL_W), rpy=Y_AXIS_CYL, color=RUBBER, shiny=0.05),
                Visual('tread_in', 'cylinder', (WHEEL_R - 0.006, WHEEL_W + 0.002), rpy=Y_AXIS_CYL,
                       color=(0.12, 0.12, 0.13)),
                Visual('hub', 'cylinder', (0.048, WHEEL_W + 0.004), rpy=Y_AXIS_CYL, color=HUB, shiny=0.6),
                Visual('cap', 'cylinder', (0.02, WHEEL_W + 0.008), rpy=Y_AXIS_CYL, color=ORANGE, shiny=0.4),
            ]
            for k in range(5):                       # spokes, so you can see it turn
                a = k * 2 * math.pi / 5
                w.visuals.append(Visual(f'spoke_{k}', 'box', (0.07, WHEEL_W + 0.006, 0.012),
                                        (0.0, 0.0, 0.0), (0, a, 0), color=DARK))
            links.append(w)
            joints.append(Joint(f'{name}_joint', 'continuous', 'base_link', f'{name}_link',
                                (fx * WHEEL_X, sy * WHEEL_Y, 0.0), axis=(0, 1, 0), effort=30, velocity=20))

    # ---- sensor frames ------------------------------------------------------
    links.append(Link('laser_link'))
    joints.append(Joint('laser_joint', 'fixed', 'base_link', 'laser_link', LIDAR_XYZ))
    links.append(Link('camera_link'))
    joints.append(Joint('camera_joint', 'fixed', 'base_link', 'camera_link', CAMERA_XYZ))
    links.append(Link('camera_optical_link'))
    joints.append(Joint('camera_optical_joint', 'fixed', 'camera_link', 'camera_optical_link',
                        rpy=(-HALF_PI, 0.0, -HALF_PI)))

    # ---- arm ----------------------------------------------------------------
    lim = ak.LIMITS
    turret = Link('turret_link', 0.4, (0.002, 0.002, 0.002), gravity=False)
    turret.visuals += [
        Visual('housing', 'cylinder', (0.065, 0.07), (0, 0, 0.035), color=ORANGE, shiny=0.45),
        Visual('ring', 'cylinder', (0.068, 0.012), (0, 0, 0.006), color=DARK),
        Visual('cheek_l', 'box', (0.07, 0.014, 0.10), (0, 0.048, SHOULDER_UP - 0.01), color=DARK),
        Visual('cheek_r', 'box', (0.07, 0.014, 0.10), (0, -0.048, SHOULDER_UP - 0.01), color=DARK),
    ]
    links.append(turret)
    joints.append(Joint('arm_turret', 'revolute', 'base_link', 'turret_link', TURRET_XYZ,
                        axis=(0, 0, 1), limits=lim['arm_turret'], velocity=1.5))

    upper = Link('upper_arm_link', 0.4, (0.0005, 0.003, 0.003), gravity=False)
    upper.visuals += [
        Visual('shoulder_motor', 'cylinder', (0.042, 0.082), rpy=Y_AXIS_CYL, color=DARK, shiny=0.5),
        Visual('shoulder_cap', 'cylinder', (0.025, 0.086), rpy=Y_AXIS_CYL, color=ALU, shiny=0.7),
        Visual('beam', 'box', (ak.L1 - 0.05, 0.056, 0.06), (ak.L1 / 2, 0, 0), color=ORANGE, shiny=0.45),
        Visual('beam_rib', 'box', (ak.L1 - 0.09, 0.058, 0.012), (ak.L1 / 2, 0, 0), color=DARK),
    ]
    links.append(upper)
    joints.append(Joint('arm_shoulder', 'revolute', 'turret_link', 'upper_arm_link',
                        (0, 0, SHOULDER_UP), axis=(0, 1, 0), limits=lim['arm_shoulder'], velocity=1.5))

    fore = Link('forearm_link', 0.3, (0.0004, 0.002, 0.002), gravity=False)
    fore.visuals += [
        Visual('elbow_motor', 'cylinder', (0.036, 0.074), rpy=Y_AXIS_CYL, color=DARK, shiny=0.5),
        Visual('elbow_cap', 'cylinder', (0.02, 0.078), rpy=Y_AXIS_CYL, color=ALU, shiny=0.7),
        Visual('beam', 'box', (ak.L2 - 0.04, 0.046, 0.05), (ak.L2 / 2, 0, 0), color=ORANGE, shiny=0.45),
        Visual('beam_rib', 'box', (ak.L2 - 0.08, 0.048, 0.01), (ak.L2 / 2, 0, 0), color=DARK),
    ]
    links.append(fore)
    joints.append(Joint('arm_elbow', 'revolute', 'upper_arm_link', 'forearm_link', (ak.L1, 0, 0),
                        axis=(0, 1, 0), limits=lim['arm_elbow'], velocity=1.5))

    wrist = Link('wrist_link', 0.2, (0.0002, 0.0002, 0.0002), gravity=False)
    wrist.visuals += [
        Visual('wrist_motor', 'cylinder', (0.03, 0.064), rpy=Y_AXIS_CYL, color=DARK, shiny=0.5),
        Visual('wrist_cap', 'cylinder', (0.016, 0.068), rpy=Y_AXIS_CYL, color=ALU, shiny=0.7),
        Visual('wrist_block', 'box', (0.05, 0.05, 0.05), (0.03, 0, 0), color=GRAPHITE),
        Visual('palm', 'box', (0.024, 0.15, 0.045), (FINGER_BASE - 0.012, 0, 0), color=ALU, shiny=0.6),
        Visual('rail', 'box', (0.008, 0.15, 0.012), (FINGER_BASE + 0.002, 0, 0.018), color=DARK),
    ]
    links.append(wrist)
    joints.append(Joint('arm_wrist', 'revolute', 'forearm_link', 'wrist_link', (ak.L2, 0, 0),
                        axis=(0, 1, 0), limits=lim['arm_wrist'], velocity=2.0))

    for side, name in ((1, 'left'), (-1, 'right')):
        f = Link(f'gripper_{name}_link', 0.05, (0.00002, 0.00002, 0.00002), gravity=False)
        f.visuals += [
            Visual('finger', 'box', (0.07, 0.012, 0.035), (0.035, side * 0.006, 0), color=ALU, shiny=0.6),
            Visual('pad', 'box', (0.05, 0.004, 0.03), (0.045, -side * 0.001, 0), color=RUBBER),
        ]
        links.append(f)
        joints.append(Joint(f'gripper_{name}_joint', 'prismatic', 'wrist_link', f'gripper_{name}_link',
                            (FINGER_BASE, 0, 0), axis=(0, side, 0), limits=(0.0, ak.FINGER_MAX),
                            effort=20, velocity=0.2))
    return links, joints


# --------------------------------------------------------------------------- #
#  Emitters
# --------------------------------------------------------------------------- #
def _f(*vals) -> str:
    return ' '.join(f'{v:.5g}' for v in vals)


def _geom_urdf(vis: Visual) -> str:
    if vis.shape == 'box':
        return f'<box size="{_f(*vis.size)}"/>'
    if vis.shape == 'cylinder':
        return f'<cylinder radius="{vis.size[0]:.5g}" length="{vis.size[1]:.5g}"/>'
    return f'<sphere radius="{vis.size[0]:.5g}"/>'


def _geom_sdf(vis: Visual) -> str:
    if vis.shape == 'box':
        return f'<box><size>{_f(*vis.size)}</size></box>'
    if vis.shape == 'cylinder':
        return (f'<cylinder><radius>{vis.size[0]:.5g}</radius>'
                f'<length>{vis.size[1]:.5g}</length></cylinder>')
    return f'<sphere><radius>{vis.size[0]:.5g}</radius></sphere>'


def _initial_positions():
    return dict(zip(ak.JOINTS, ak.STOW))


def build_urdf() -> str:
    links, joints = build_model()
    out = ['<?xml version="1.0"?>',
           '<!-- GENERATED by warehouse_bot/rover_description.py - edit that file instead -->',
           '<robot name="warehouse_rover">', '  <link name="base_footprint"/>']
    for ln in links:
        out.append(f'  <link name="{ln.name}">')
        for vis in ln.visuals:
            r, g, b = vis.color
            out.append(f'    <visual name="{vis.name}"><origin xyz="{_f(*vis.xyz)}" rpy="{_f(*vis.rpy)}"/>'
                       f'<geometry>{_geom_urdf(vis)}</geometry>'
                       f'<material name="{ln.name}_{vis.name}"><color rgba="{_f(r, g, b)} 1"/></material>'
                       f'</visual>')
        if ln.collision is not None:
            c = ln.collision
            out.append(f'    <collision><origin xyz="{_f(*c.xyz)}" rpy="{_f(*c.rpy)}"/>'
                       f'<geometry>{_geom_urdf(c)}</geometry></collision>')
        if ln.mass > 0:
            ixx, iyy, izz = ln.inertia
            out.append(f'    <inertial><mass value="{ln.mass:.5g}"/>'
                       f'<inertia ixx="{ixx:.6g}" iyy="{iyy:.6g}" izz="{izz:.6g}" ixy="0" ixz="0" iyz="0"/>'
                       f'</inertial>')
        out.append('  </link>')
    for j in joints:
        out.append(f'  <joint name="{j.name}" type="{j.kind}"><parent link="{j.parent}"/>'
                   f'<child link="{j.child}"/><origin xyz="{_f(*j.xyz)}" rpy="{_f(*j.rpy)}"/>'
                   + (f'<axis xyz="{_f(*j.axis)}"/>' if j.kind != 'fixed' else '')
                   + (f'<limit lower="{j.limits[0]:.5g}" upper="{j.limits[1]:.5g}" effort="{j.effort:g}" '
                      f'velocity="{j.velocity:g}"/>' if j.limits else '')
                   + '</joint>')
    # ros2_control: arm + gripper (the wheels are driven by gazebo_ros_diff_drive)
    init = _initial_positions()
    out.append('  <ros2_control name="RoverArm" type="system">')
    out.append('    <hardware><plugin>gazebo_ros2_control/GazeboSystem</plugin></hardware>')
    for name in ak.JOINTS + ak.GRIPPER_JOINTS:
        out.append(f'    <joint name="{name}"><command_interface name="position"/>'
                   f'<state_interface name="position"><param name="initial_value">'
                   f'{init.get(name, 0.03):.5f}</param></state_interface>'
                   f'<state_interface name="velocity"/></joint>')
    out.append('  </ros2_control>')
    out.append('</robot>')
    return '\n'.join(out) + '\n'


def _material_sdf(vis: Visual) -> str:
    r, g, b = vis.color
    s = vis.shiny
    em = vis.emissive
    return (f'<material><ambient>{_f(r * 0.6, g * 0.6, b * 0.6)} 1</ambient>'
            f'<diffuse>{_f(r, g, b)} 1</diffuse><specular>{_f(s, s, s)} 1</specular>'
            + (f'<emissive>{_f(*em)} 1</emissive>' if em else '') + '</material>')


def _link_pose(joints: List[Joint]):
    """Model-frame position of every link frame (all joint rotations are zero here,
    and only the camera optical frame has an rpy, which carries no visuals)."""
    pos = {'base_footprint': (0.0, 0.0, 0.0)}
    pending = list(joints)
    while pending:
        for j in list(pending):
            if j.parent in pos:
                px, py, pz = pos[j.parent]
                pos[j.child] = (px + j.xyz[0], py + j.xyz[1], pz + j.xyz[2])
                pending.remove(j)
    return pos


def build_sdf(controllers_yaml: str = '', with_control: bool = True) -> str:
    links, joints = build_model()
    pos = _link_pose(joints)
    body = []
    for ln in links:
        if ln.mass <= 0:
            continue                                   # pure TF frames: not needed in Gazebo
        x, y, z = pos[ln.name]
        parts = [f'<link name="{ln.name}"><pose>{_f(x, y, z)} 0 0 0</pose>']
        if not ln.gravity:
            parts.append('<gravity>false</gravity>')
        ixx, iyy, izz = ln.inertia
        parts.append(f'<inertial><mass>{ln.mass:.5g}</mass><inertia><ixx>{ixx:.6g}</ixx><iyy>{iyy:.6g}</iyy>'
                     f'<izz>{izz:.6g}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>')
        for vis in ln.visuals:
            parts.append(f'<visual name="{vis.name}"><pose>{_f(*vis.xyz)} {_f(*vis.rpy)}</pose>'
                         f'<geometry>{_geom_sdf(vis)}</geometry>{_material_sdf(vis)}</visual>')
        if ln.collision is not None:
            c = ln.collision
            surface = ''
            if ln.friction is not None:
                mu = ln.friction
                surface = (f'<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction>'
                           '<contact><ode><kp>1e6</kp><kd>100</kd><max_vel>0.1</max_vel>'
                           '<min_depth>0.001</min_depth></ode></contact></surface>')
            parts.append(f'<collision name="{c.name}"><pose>{_f(*c.xyz)} {_f(*c.rpy)}</pose>'
                         f'<geometry>{_geom_sdf(c)}</geometry>{surface}</collision>')
        if ln.name == 'base_link':
            parts.append(_sensors_sdf())
        parts.append('</link>')
        body.append(''.join(parts))
    for j in joints:
        if j.kind == 'fixed':
            continue                                   # sensor frames only
        kind = 'revolute' if j.kind == 'continuous' else j.kind
        limit = ''
        if j.limits:
            limit = (f'<limit><lower>{j.limits[0]:.5g}</lower><upper>{j.limits[1]:.5g}</upper>'
                     f'<effort>{j.effort:g}</effort><velocity>{j.velocity:g}</velocity></limit>')
        damping = '' if j.kind == 'continuous' else '<dynamics><damping>2.0</damping></dynamics>'
        body.append(f'<joint name="{j.name}" type="{kind}"><parent>{j.parent}</parent>'
                    f'<child>{j.child}</child><axis><xyz>{_f(*j.axis)}</xyz>{limit}{damping}</axis></joint>')
    body.append(_drive_plugins_sdf())
    if with_control:
        body.append('<plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">'
                    f'<parameters>{controllers_yaml}</parameters>'
                    '<robot_param>robot_description</robot_param>'
                    '<robot_param_node>robot_state_publisher</robot_param_node></plugin>')
    return ('<?xml version="1.0"?>\n<!-- GENERATED by warehouse_bot/rover_description.py -->\n'
            '<sdf version="1.6"><model name="warehouse_rover">'
            + '\n'.join(body) + '</model></sdf>\n')


def _sensors_sdf() -> str:
    lx, ly, lz = LIDAR_XYZ
    cx, cy, cz = CAMERA_XYZ
    return (
        f'<sensor name="lidar" type="ray"><pose>{_f(lx, ly, lz)} 0 0 0</pose>'
        '<always_on>true</always_on><visualize>false</visualize><update_rate>10</update_rate>'
        '<ray><scan><horizontal><samples>360</samples><resolution>1</resolution>'
        '<min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal></scan>'
        '<range><min>0.12</min><max>8.0</max><resolution>0.01</resolution></range>'
        '<noise><type>gaussian</type><mean>0</mean><stddev>0.008</stddev></noise></ray>'
        '<plugin name="lidar_driver" filename="libgazebo_ros_ray_sensor.so">'
        '<ros><remapping>~/out:=scan</remapping></ros>'
        '<output_type>sensor_msgs/LaserScan</output_type><frame_name>laser_link</frame_name>'
        '</plugin></sensor>'
        f'<sensor name="front_camera" type="camera"><pose>{_f(cx, cy, cz)} 0 0 0</pose>'
        '<always_on>true</always_on><update_rate>10</update_rate>'
        '<camera name="front"><horizontal_fov>1.2</horizontal_fov>'
        '<image><width>640</width><height>480</height><format>R8G8B8</format></image>'
        '<clip><near>0.03</near><far>15</far></clip></camera>'
        '<plugin name="camera_driver" filename="libgazebo_ros_camera.so">'
        '<ros><namespace>/</namespace></ros><camera_name>camera</camera_name>'
        '<frame_name>camera_optical_link</frame_name></plugin></sensor>')


def _drive_plugins_sdf() -> str:
    sep = 2 * WHEEL_Y
    dia = 2 * WHEEL_R
    return (
        '<plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">'
        '<update_rate>30</update_rate><num_wheel_pairs>2</num_wheel_pairs>'
        '<left_joint>wheel_fl_joint</left_joint><right_joint>wheel_fr_joint</right_joint>'
        '<left_joint>wheel_rl_joint</left_joint><right_joint>wheel_rr_joint</right_joint>'
        f'<wheel_separation>{sep:.4f}</wheel_separation><wheel_separation>{sep:.4f}</wheel_separation>'
        f'<wheel_diameter>{dia:.4f}</wheel_diameter><wheel_diameter>{dia:.4f}</wheel_diameter>'
        '<max_wheel_torque>30</max_wheel_torque><max_wheel_acceleration>3.0</max_wheel_acceleration>'
        '<publish_odom>true</publish_odom><publish_odom_tf>true</publish_odom_tf>'
        '<publish_wheel_tf>false</publish_wheel_tf><odometry_source>1</odometry_source>'
        '<odometry_frame>odom</odometry_frame><robot_base_frame>base_footprint</robot_base_frame>'
        '</plugin>'
        '<plugin name="wheel_states" filename="libgazebo_ros_joint_state_publisher.so">'
        '<update_rate>10</update_rate><joint_name>wheel_fl_joint</joint_name>'
        '<joint_name>wheel_fr_joint</joint_name><joint_name>wheel_rl_joint</joint_name>'
        '<joint_name>wheel_rr_joint</joint_name></plugin>')


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--urdf', action='store_true')
    ap.add_argument('--sdf', action='store_true')
    ap.add_argument('--controllers', default='rover_controllers.yaml')
    a = ap.parse_args(argv)
    print(build_sdf(a.controllers) if a.sdf else build_urdf())


if __name__ == '__main__':
    main()
