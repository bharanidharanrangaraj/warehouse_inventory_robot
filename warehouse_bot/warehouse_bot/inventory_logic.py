"""Inventory reconciliation: compare what the robot saw with the stock records.

Kept free of ROS imports so the rules can be unit-tested on their own
(see test/test_inventory_logic.py).

Item status rules
-----------------
RELOCATED  the robot carried the item to the pick/drop station
OK         tag seen in the zone the catalogue expects
MISPLACED  tag seen, but in a different zone
MISSING    expected zone has been scanned and the tag was not seen anywhere
PENDING    expected zone not scanned yet
UNKNOWN    tag seen that is not in the catalogue at all

Zone state rules (rack zones only)
----------------------------------
PENDING    zone not scanned yet
MISSING    at least one item that belongs here is MISSING
ATTENTION  a foreign/unknown tag is here, or an item that belongs here is elsewhere
OK         everything that belongs here is here and nothing else is
"""

from dataclasses import dataclass, asdict
from typing import Dict, Iterable, List, Optional

STATUS_OK = 'OK'
STATUS_MISSING = 'MISSING'
STATUS_MISPLACED = 'MISPLACED'
STATUS_UNKNOWN = 'UNKNOWN'
STATUS_PENDING = 'PENDING'
STATUS_RELOCATED = 'RELOCATED'

ZONE_PENDING = 'PENDING'
ZONE_OK = 'OK'
ZONE_ATTENTION = 'ATTENTION'
ZONE_MISSING = 'MISSING'

FLAGGED_STATUSES = (STATUS_MISPLACED, STATUS_UNKNOWN)


@dataclass
class ItemState:
    marker_id: int
    sku: str
    name: str
    quantity: int
    expected_zone: str
    found_zone: str
    status: str

    def to_dict(self) -> dict:
        return asdict(self)


def evaluate(catalog: Dict[int, object],
             scans: Dict[str, Iterable[int]],
             relocated: Iterable[int] = (),
             drop_zone_id: str = 'DROP') -> List[ItemState]:
    """Return the status of every catalogue item plus any unknown tags.

    catalog   marker_id -> CatalogItem (needs .sku .name .zone .quantity)
    scans     zone_id -> ids detected the last time that zone was scanned
    relocated marker ids the robot has carried to the drop station
    """
    relocated = set(int(m) for m in relocated)
    found_in: Dict[int, List[str]] = {}
    for zone_id, ids in scans.items():
        for mid in sorted(set(int(i) for i in ids)):
            found_in.setdefault(mid, []).append(zone_id)

    states: List[ItemState] = []
    for mid in sorted(catalog):
        item = catalog[mid]
        zones = found_in.get(mid, [])
        if mid in relocated:
            status, found = STATUS_RELOCATED, drop_zone_id
        elif item.zone in zones:
            status, found = STATUS_OK, item.zone
        elif zones:
            status, found = STATUS_MISPLACED, zones[0]
        elif item.zone in scans:
            status, found = STATUS_MISSING, ''
        else:
            status, found = STATUS_PENDING, ''
        states.append(ItemState(mid, item.sku, item.name, item.quantity,
                                item.zone, found, status))

    for mid in sorted(found_in):
        if mid in catalog:
            continue
        if mid in relocated:
            states.append(ItemState(mid, 'UNREGISTERED', f'Unknown tag {mid}', 0, '',
                                    drop_zone_id, STATUS_RELOCATED))
        else:
            states.append(ItemState(mid, 'UNREGISTERED', f'Unknown tag {mid}', 0, '',
                                    found_in[mid][0], STATUS_UNKNOWN))
    return states


def zone_items(states: List[ItemState], zone_id: str) -> List[ItemState]:
    """Items that belong in the zone, plus anything that was found there."""
    return [s for s in states if s.expected_zone == zone_id or s.found_zone == zone_id]


def zone_counts(states: List[ItemState], zone_id: Optional[str] = None) -> dict:
    """Counters for one zone, or for the whole warehouse when zone_id is None."""
    if zone_id is None:
        selected = states
        expected = [s for s in states if s.expected_zone]
    else:
        selected = zone_items(states, zone_id)
        expected = [s for s in selected if s.expected_zone == zone_id]
    ok = sum(1 for s in expected if s.status == STATUS_OK)
    return {
        'expected': len(expected),
        'ok': ok,
        'missing': sum(1 for s in expected if s.status == STATUS_MISSING),
        'misplaced': sum(1 for s in selected if s.status == STATUS_MISPLACED),
        'unknown': sum(1 for s in selected if s.status == STATUS_UNKNOWN),
        'relocated': sum(1 for s in selected if s.status == STATUS_RELOCATED),
        'accuracy': (ok / len(expected)) if expected else 0.0,
    }


def zone_state(states: List[ItemState], zone_id: str, scanned: bool) -> str:
    if not scanned:
        return ZONE_PENDING
    selected = zone_items(states, zone_id)
    if any(s.expected_zone == zone_id and s.status == STATUS_MISSING for s in selected):
        return ZONE_MISSING
    if any(s.status in (STATUS_MISPLACED, STATUS_UNKNOWN, STATUS_RELOCATED) for s in selected):
        return ZONE_ATTENTION
    return ZONE_OK


def flagged_items(states: List[ItemState]) -> List[ItemState]:
    """Items the robot should carry to the drop station."""
    return [s for s in states if s.status in FLAGGED_STATUSES]
