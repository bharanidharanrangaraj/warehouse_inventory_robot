"""Geometry and inverse kinematics of the warehouse rover's arm - pure Python.

The rover (urdf/warehouse_rover.urdf.xacro) carries a 4-joint arm on the front of
its deck plus a two-finger gripper:

    arm_turret    yaw about z         (turns the whole arm)
    arm_shoulder  pitch about y   \
    arm_elbow     pitch about y    >  planar 3-link chain in the turret plane
    arm_wrist     pitch about y   /
    gripper_left_joint / gripper_right_joint   prismatic fingers (opening per finger)

All pitch joints rotate about +y, so a positive angle tips the link DOWN. With
every joint at 0 the arm points straight forward, horizontally.

All numbers below must match the xacro file (same names there). Coordinates are
in the robot's base_footprint frame: x forward, y left, z up, origin on the floor
under the middle of the rover.
"""

import math
from typing import Dict, Optional, Sequence, Tuple

# ---- arm geometry (keep in sync with urdf/warehouse_rover.urdf.xacro) ----
SHOULDER = (0.13, 0.0, 0.302)    # shoulder pitch axis (on the turret yaw axis)
L1 = 0.30                        # shoulder -> elbow
L2 = 0.27                        # elbow -> wrist
L3 = 0.125                       # wrist axis -> grasp point between the fingertips
FINGER_MAX = 0.06                # max opening of each finger from the centre line

JOINTS = ('arm_turret', 'arm_shoulder', 'arm_elbow', 'arm_wrist')
GRIPPER_JOINTS = ('gripper_left_joint', 'gripper_right_joint')
LIMITS = {                       # radians
    'arm_turret': (-3.3, 3.3),
    'arm_shoulder': (-2.0, 0.6),
    'arm_elbow': (-2.0, 2.9),
    'arm_wrist': (-2.4, 2.4),
}

# Folded "crane" pose for driving: upper arm straight up, forearm pointing back
# over the deck, gripper hanging down above the cargo tray (out of the camera view).
STOW = (0.0, -math.pi / 2, -math.pi / 2, -math.pi / 2)

# Cargo tray on the rear deck: centre of its floor (top surface)
TRAY = (-0.19, 0.0, 0.200)

# Per component shape: (grasp height above the part's base, width across the fingers)
GRASP = {
    'capacitor': (0.068, 0.090),
    'resistor': (0.032, 0.062),
    'diode': (0.030, 0.057),
    'transformer': (0.060, 0.095),
    'box': (0.060, 0.120),
}


def solve(x: float, y: float, z: float, pitch: float = 0.0,
          turret_hint: float = 0.0) -> Optional[Tuple[float, float, float, float]]:
    """Joint angles that put the grasp point at (x, y, z) with the gripper tilted
    `pitch` rad from horizontal (0 = level, negative = pointing down).
    Elbow-up solution. Returns None if the point is out of reach or off-limits.
    turret_hint picks the yaw closest to it (e.g. +pi vs -pi when reaching back)."""
    sx, sy, sz = SHOULDER
    dx, dy = x - sx, y - sy
    r = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx) if r > 1e-6 else turret_hint
    # choose the equivalent yaw (yaw, yaw +/- 2pi) closest to the hint
    yaw = min((yaw - 2 * math.pi, yaw, yaw + 2 * math.pi), key=lambda a: abs(a - turret_hint))
    rw = r - L3 * math.cos(pitch)              # wrist position in the arm plane
    zw = (z - sz) - L3 * math.sin(pitch)
    d = (rw * rw + zw * zw - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    if abs(d) > 1.0:
        return None
    bend = math.acos(d)                        # elbow bend, >= 0
    a1 = math.atan2(zw, rw) + math.atan2(L2 * math.sin(bend), L1 + L2 * math.cos(bend))
    a2 = a1 - bend                             # forearm elevation
    q = (yaw, -a1, bend, a2 - pitch)
    if not within_limits(q):
        return None
    return q


def within_limits(q: Sequence[float]) -> bool:
    return all(LIMITS[j][0] - 1e-9 <= v <= LIMITS[j][1] + 1e-9 for j, v in zip(JOINTS, q))


def forward(q: Sequence[float]) -> Tuple[float, float, float, float]:
    """Grasp point (x, y, z) and gripper pitch for joint angles q (inverse of solve)."""
    yaw, sh, el, wr = q
    a1 = -sh
    a2 = a1 - el
    p = a2 - wr
    r = L1 * math.cos(a1) + L2 * math.cos(a2) + L3 * math.cos(p)
    z = L1 * math.sin(a1) + L2 * math.sin(a2) + L3 * math.sin(p)
    sx, sy, sz = SHOULDER
    return sx + r * math.cos(yaw), sy + r * math.sin(yaw), sz + z, p


def to_robot_frame(robot_xy_yaw: Tuple[float, float, float],
                   point: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """World point -> base_footprint frame of a robot at (x, y, yaw)."""
    rx, ry, ryaw = robot_xy_yaw
    dx, dy = point[0] - rx, point[1] - ry
    c, s = math.cos(ryaw), math.sin(ryaw)
    return c * dx + s * dy, -s * dx + c * dy, point[2]


def reachable(point: Tuple[float, float, float], approach: float = 0.10,
              lift: float = 0.06) -> bool:
    """True if a level grasp at `point` (robot frame) works, including the
    pre-grasp (backed off by `approach` along the reach direction) and the lift."""
    x, y, z = point
    sx, sy, _ = SHOULDER
    r = math.hypot(x - sx, y - sy)
    ux, uy = ((x - sx) / r, (y - sy) / r) if r > 1e-6 else (1.0, 0.0)
    for px, py, pz in ((x, y, z), (x - ux * approach, y - uy * approach, z),
                       (x, y, z + lift)):
        if solve(px, py, pz) is None:
            return False
    return True


def joint_dict(q: Sequence[float]) -> Dict[str, float]:
    return dict(zip(JOINTS, q))
