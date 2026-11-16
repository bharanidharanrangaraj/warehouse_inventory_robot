"""Unit tests for the rover's arm kinematics and its robot description (no ROS needed).

Run:  python3 -m pytest test/
"""
import math
import xml.etree.ElementTree as ET

import pytest

from warehouse_bot import arm_kinematics as ak
from warehouse_bot import rover_description as rover


@pytest.mark.parametrize('point,pitch', [
    ((0.60, 0.0, 0.39), 0.0),          # straight into the shelf
    ((0.55, 0.40, 0.36), 0.0),         # unit at the right end of the shelf
    ((0.55, -0.40, 0.36), 0.0),
    ((0.45, 0.0, 0.70), -0.5),         # high, gripper tilted down
    ((-0.19, 0.0, 0.27), 0.0),         # cargo tray (reaching backwards)
])
def test_inverse_then_forward_returns_the_target(point, pitch):
    hint = math.pi if point[0] < ak.SHOULDER[0] else 0.0
    q = ak.solve(*point, pitch=pitch, turret_hint=hint)
    assert q is not None, point
    x, y, z, p = ak.forward(q)
    assert (x, y, z) == pytest.approx(point, abs=1e-6)
    assert math.cos(p - pitch) == pytest.approx(1.0, abs=1e-6)
    assert ak.within_limits(q)


def test_out_of_reach_returns_none():
    assert ak.solve(1.5, 0.0, 0.4) is None              # beyond L1 + L2 + L3
    assert ak.solve(ak.SHOULDER[0], 0.0, ak.SHOULDER[2] + 2.0) is None


def test_turret_hint_picks_the_nearest_yaw():
    back = (-0.19, 0.0, 0.27)
    assert ak.solve(*back, turret_hint=math.pi)[0] == pytest.approx(math.pi)
    assert ak.solve(*back, turret_hint=-math.pi)[0] == pytest.approx(-math.pi)


def test_stow_pose_hangs_the_gripper_over_the_tray():
    x, y, z, p = ak.forward(ak.STOW)
    assert abs(x - ak.TRAY[0]) < 0.08 and abs(y) < 1e-9
    assert z > ak.TRAY[2] + 0.2                       # well above the tray and the LiDAR
    assert math.sin(p) == pytest.approx(-1.0)          # pointing straight down


def test_tray_is_reachable_for_every_part():
    for shape, (h, w) in ak.GRASP.items():
        assert w <= 2 * ak.FINGER_MAX, shape
        tx, ty, tz = ak.TRAY
        assert ak.solve(tx, ty, tz + h + 0.004, turret_hint=math.pi) is not None, shape
        assert ak.solve(tx, ty, tz + h + 0.13, turret_hint=math.pi) is not None, shape


def test_robot_files_agree():
    urdf = ET.fromstring(rover.build_urdf().split('?>', 1)[1])
    sdf = ET.fromstring(rover.build_sdf('/tmp/c.yaml').split('-->', 1)[1])
    urdf_joints = {j.get('name'): j.get('type') for j in urdf.findall('joint')}
    sdf_joints = {j.get('name') for j in sdf.find('model').findall('joint')}
    moving = {n for n, t in urdf_joints.items() if t != 'fixed'}
    assert moving == sdf_joints
    assert set(ak.JOINTS + ak.GRIPPER_JOINTS) <= moving
    controlled = {j.get('name') for j in urdf.find('ros2_control').findall('joint')}
    assert controlled == set(ak.JOINTS + ak.GRIPPER_JOINTS)
    # the shoulder in the URDF chain sits where the kinematics expects it
    chain = {j.get('name'): [float(v) for v in j.find('origin').get('xyz').split()]
             for j in urdf.findall('joint')}
    z = chain['base_joint'][2] + chain['arm_turret'][2] + chain['arm_shoulder'][2]
    assert (chain['arm_turret'][0], z) == pytest.approx((ak.SHOULDER[0], ak.SHOULDER[2]))
    for name, lo_hi in ak.LIMITS.items():
        lim = urdf.find(f"joint[@name='{name}']/limit")
        assert (float(lim.get('lower')), float(lim.get('upper'))) == pytest.approx(lo_hi)
