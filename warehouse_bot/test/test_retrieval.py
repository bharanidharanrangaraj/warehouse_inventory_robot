"""Unit tests for retrieval mode: the product layout and the order book.

Run without ROS:   python3 -m pytest test/
Run with colcon:   colcon test --packages-select warehouse_bot && colcon test-result --verbose
"""
import os

import pytest

from warehouse_bot import fetch_logic as fl
from warehouse_bot.layout import Rect, WarehouseLayout

LAYOUT = os.path.join(os.path.dirname(__file__), '..', 'config', 'warehouse_layout.yaml')


@pytest.fixture(scope='module')
def layout():
    return WarehouseLayout.from_file(LAYOUT)


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def book(layout):
    return fl.OrderBook(layout.products, max_queue=3, clock=FakeClock())


# --------------------------------------------------------------------------- #
#  Layout
# --------------------------------------------------------------------------- #
def test_four_products_one_per_rack(layout):
    assert list(layout.products) == ['capacitor', 'resistor', 'transformer', 'diode']
    assert [p.rack_id for p in layout.products.values()] == ['A', 'B', 'C', 'D']
    assert [r.name for r in layout.racks] == ['Capacitors', 'Resistors', 'Transformers', 'Diodes']


def test_items_sit_on_their_rack_shelf(layout):
    racks = {r.id: r for r in layout.racks}
    assert len(layout.item_homes) == sum(p.units for p in layout.products.values())
    for item in layout.item_homes:
        r = racks[item.rack_id]
        assert Rect(r.x, r.y, r.yaw, r.length, r.depth).contains(item.x, item.y), item.name
        assert item.z == pytest.approx(layout.SHELF_TOP_Z)
        assert item.product_id == r.product.id
    names = [i.name for i in layout.item_homes]
    assert len(names) == len(set(names))


def test_counter_spots_are_on_the_counter_and_reachable(layout):
    from warehouse_bot import arm_kinematics as ak
    t = layout.drop_station
    top = Rect(t.x, t.y, t.yaw, t.size_x, t.size_y)
    assert len(layout.counter_slots) >= 6
    robot = layout.counter_pick_pose()
    for s in layout.counter_slots:
        assert top.contains(s.x, s.y)
        local = ak.to_robot_frame((robot.x, robot.y, robot.yaw), (s.x, s.y, 0.0))
        for h in layout.grasp_heights():
            assert ak.reachable((local[0], local[1], t.size_z + h + 0.004))
    spots = {(round(s.x, 3), round(s.y, 3)) for s in layout.counter_slots}
    assert len(spots) == len(layout.counter_slots)


def test_every_shelf_unit_is_reachable_from_its_pick_pose(layout):
    from warehouse_bot import arm_kinematics as ak
    for item in layout.item_homes:
        robot = layout.rack_pick_pose(item.rack_id)
        h = ak.GRASP[layout.products[item.product_id].shape][0]
        local = ak.to_robot_frame((robot.x, robot.y, robot.yaw), (item.x, item.y, item.z + h))
        assert ak.reachable(local), item.name
        # the pick pose is free of obstacles for the rover's footprint
        for rect in layout.occupancy_rects():
            assert not rect.contains(robot.x, robot.y, margin=0.31)


def test_web_summary_is_plain_json(layout):
    import json
    data = layout.to_web()
    json.dumps(data)                                    # must not raise
    assert [p['id'] for p in data['products']] == list(layout.products)
    assert {z['id'] for z in data['zones']} >= {'A', 'B', 'C', 'D', 'DROP', 'DOCK'}


def test_unknown_shape_is_rejected(layout):
    import copy
    data = copy.deepcopy(layout.raw)
    data['racks'][0]['product']['shape'] = 'banana'
    with pytest.raises(ValueError, match='unknown shape'):
        WarehouseLayout(data)


# --------------------------------------------------------------------------- #
#  Order book
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize('text', ['diode', 'Diode', 'DIODES', ' diodes ', 'D', 'rack D'])
def test_product_aliases(book, text):
    assert book.match_product(text) == 'diode'


def test_unknown_product_rejected(book):
    ok, order, msg = book.request('banana', shelf_units=3)
    assert not ok and order is None
    assert 'Capacitor' in msg and 'Diode' in msg


def test_orders_reserve_shelf_stock(book):
    for _ in range(2):
        assert book.request('resistor', shelf_units=2)[0]
    ok, _, msg = book.request('resistor', shelf_units=2)
    assert not ok and 'out of stock' in msg
    assert book.request('capacitor', shelf_units=2)[0]      # other products unaffected


def test_queue_limit(book):
    for _ in range(3):
        assert book.request('capacitor', shelf_units=9)[0]
    ok, _, msg = book.request('capacitor', shelf_units=9)
    assert not ok and 'queue is full' in msg


def test_fifo_and_lifecycle(book):
    _, first, _ = book.request('diode', 3)
    _, second, _ = book.request('capacitor', 3)
    assert book.start_next() is first
    assert book.start_next() is None                       # robot busy
    assert first.state == fl.TO_RACK and second.message == 'Next in line'
    book.clock.t += 5
    for state in (fl.CONFIRMING, fl.PICKING, fl.TO_USER, fl.HANDING_OVER):
        book.advance(state, state.lower())
    book.clock.t += 60
    done = book.finish(True, 'Delivered')
    assert done is first and done.state == fl.DELIVERED
    assert done.duration(book.clock.t) == pytest.approx(65.0)
    assert book.history[0] is first and book.current is None
    assert book.start_next() is second


def test_picked_unit_is_no_longer_reserved(book):
    book.request('transformer', 1)
    book.start_next()
    assert book.reserved('transformer') == 1
    book.advance(fl.PICKING)
    assert book.reserved('transformer') == 1               # still on the shelf
    book.advance(fl.TO_USER)
    assert book.reserved('transformer') == 0               # it is on the robot now


def test_failed_order_frees_its_reservation(book):
    book.request('diode', 1)
    book.start_next()
    book.finish(False, 'Could not reach rack D')
    assert book.history[0].state == fl.FAILED
    assert book.reserved('diode') == 0
    assert book.request('diode', 1)[0]


def test_advance_rejects_final_states(book):
    book.request('diode', 1)
    book.start_next()
    with pytest.raises(ValueError):
        book.advance(fl.DELIVERED)
