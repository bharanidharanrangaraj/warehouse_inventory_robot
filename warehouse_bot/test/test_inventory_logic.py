"""Unit tests for the layout model and the inventory reconciliation rules.

Run without ROS:   python3 -m pytest test/
Run with colcon:   colcon test --packages-select warehouse_bot && colcon test-result --verbose
"""
import math
import os

import pytest

from warehouse_bot import inventory_logic as logic
from warehouse_bot.layout import Rect, WarehouseLayout

LAYOUT = os.path.join(os.path.dirname(__file__), '..', 'config', 'warehouse_layout.yaml')


@pytest.fixture(scope='module')
def layout():
    return WarehouseLayout.from_file(LAYOUT)


def test_zones_and_goals(layout):
    assert list(layout.zones) == ['A', 'B', 'C', 'D', 'DROP', 'DOCK']
    for zone in layout.zones.values():
        assert zone.contains(zone.goal.x, zone.goal.y), f'goal of {zone.id} outside its zone'
    # scan poses face their rack
    assert math.isclose(layout.zones['A'].goal.yaw, math.pi / 2)
    assert math.isclose(layout.zones['C'].goal.yaw, -math.pi / 2)


def test_scan_poses_are_collision_free(layout):
    robot_radius = 0.22
    for zone in layout.zones.values():
        for rect in layout.occupancy_rects():
            assert not rect.contains(zone.goal.x, zone.goal.y, margin=robot_radius), \
                f'scan pose of {zone.id} too close to an obstacle'


def test_rect_contains_rotated():
    r = Rect(0, 0, math.pi / 4, 2.0, 0.2)
    assert r.contains(0.5, 0.5)
    assert not r.contains(0.5, -0.5)


def test_full_audit(layout):
    scans = {z: layout.markers_in_zone(z) for z in ('A', 'B', 'C', 'D')}
    states = logic.evaluate(layout.catalog, scans)
    by_id = {s.marker_id: s for s in states}
    assert by_id[0].status == logic.STATUS_OK
    assert by_id[5].status == logic.STATUS_MISSING
    assert by_id[11].status == logic.STATUS_MISPLACED and by_id[11].found_zone == 'C'
    assert by_id[20].status == logic.STATUS_UNKNOWN and by_id[20].found_zone == 'D'
    assert logic.zone_state(states, 'A', True) == logic.ZONE_OK
    assert logic.zone_state(states, 'B', True) == logic.ZONE_MISSING
    assert logic.zone_state(states, 'C', True) == logic.ZONE_ATTENTION
    assert logic.zone_state(states, 'D', True) == logic.ZONE_ATTENTION
    total = logic.zone_counts(states)
    assert (total['expected'], total['ok'], total['missing']) == (11, 9, 1)
    assert {s.marker_id for s in logic.flagged_items(states)} == {11, 20}


def test_partial_audit_is_pending(layout):
    states = logic.evaluate(layout.catalog, {'C': layout.markers_in_zone('C')})
    by_id = {s.marker_id: s for s in states}
    assert by_id[0].status == logic.STATUS_PENDING
    assert by_id[11].status == logic.STATUS_MISPLACED      # known as soon as C is scanned
    assert logic.zone_state(states, 'A', False) == logic.ZONE_PENDING


def test_relocation_clears_foreign_items(layout):
    scans = {z: layout.markers_in_zone(z) for z in ('A', 'B', 'C', 'D')}
    states = logic.evaluate(layout.catalog, scans, relocated={11, 20})
    by_id = {s.marker_id: s for s in states}
    assert by_id[11].status == logic.STATUS_RELOCATED and by_id[11].found_zone == 'DROP'
    assert logic.zone_state(states, 'C', True) == logic.ZONE_OK
    assert logic.zone_state(states, 'D', True) == logic.ZONE_ATTENTION   # awaiting restock
