"""Order book for retrieval mode - pure Python, no ROS, so it can be unit-tested.

A user orders one unit of a product ("diode"). The order waits in a FIFO queue
until the robot is free, then moves through the delivery steps:

    QUEUED -> TO_RACK -> CONFIRMING -> PICKING -> TO_USER -> HANDING_OVER -> DELIVERED
                                                         (or FAILED at any step)

Stock is reserved when an order is accepted: a unit counts as taken from the
moment it is ordered until it leaves the shelf, so the robot is never sent to an
empty rack because two people ordered the last unit at the same time.
"""

import re
import time
from dataclasses import asdict, dataclass
from typing import Callable, Dict, List, Optional, Tuple

QUEUED = 'QUEUED'
TO_RACK = 'TO_RACK'
CONFIRMING = 'CONFIRMING'
PICKING = 'PICKING'
TO_USER = 'TO_USER'
HANDING_OVER = 'HANDING_OVER'
DELIVERED = 'DELIVERED'
FAILED = 'FAILED'

STEPS = (QUEUED, TO_RACK, CONFIRMING, PICKING, TO_USER, HANDING_OVER, DELIVERED)
ACTIVE_STATES = (TO_RACK, CONFIRMING, PICKING, TO_USER, HANDING_OVER)
FINAL_STATES = (DELIVERED, FAILED)
# The ordered unit is still on the shelf in these states, so it is "reserved"
RESERVING_STATES = (QUEUED, TO_RACK, CONFIRMING, PICKING)


def normalise(text: str) -> str:
    """'  Diodes ' -> 'diodes'; drops everything except letters and digits."""
    return re.sub(r'[^a-z0-9]', '', str(text).lower())


@dataclass
class Order:
    order_id: str
    product: str
    product_name: str
    rack_id: str
    state: str = QUEUED
    message: str = ''
    requested_at: float = 0.0
    started_at: float = 0.0
    finished_at: float = 0.0

    @property
    def finished(self) -> bool:
        return self.state in FINAL_STATES

    def duration(self, now: float) -> float:
        if not self.started_at:
            return 0.0
        return (self.finished_at or now) - self.started_at

    def to_dict(self, now: Optional[float] = None) -> dict:
        d = asdict(self)
        d['duration'] = round(self.duration(time.time() if now is None else now), 1)
        return d


class OrderBook:
    """FIFO order queue plus the one order the robot is working on.

    products: mapping product id -> object with .id, .name, .rack_id, .rack_name
              (warehouse_bot.layout.Product fits).
    """

    def __init__(self, products: Dict[str, object], max_queue: int = 8,
                 history_size: int = 20, clock: Callable[[], float] = time.time):
        self.products = dict(products)
        self.max_queue = max(1, int(max_queue))
        self.history_size = max(1, int(history_size))
        self.clock = clock
        self.queue: List[Order] = []
        self.current: Optional[Order] = None
        self.history: List[Order] = []          # newest first
        self._counter = 0
        self._aliases: Dict[str, str] = {}
        for p in self.products.values():
            for alias in (p.id, p.name, p.name + 's', p.rack_id, p.rack_name,
                          f'rack{p.rack_id}'):
                self._aliases.setdefault(normalise(alias), p.id)

    # ------------------------------------------------------------------ #
    def match_product(self, text: str) -> Optional[str]:
        """Product id for 'diode', 'Diodes', 'D', 'rack D' ... or None."""
        return self._aliases.get(normalise(text))

    def reserved(self, product_id: str) -> int:
        """Units of a product promised to orders but still on the shelf."""
        orders = self.queue + ([self.current] if self.current else [])
        return sum(1 for o in orders if o.product == product_id and o.state in RESERVING_STATES)

    def request(self, text: str, shelf_units: int) -> Tuple[bool, Optional[Order], str]:
        """Try to place an order. shelf_units = units physically on the rack now."""
        pid = self.match_product(text)
        if pid is None:
            names = ', '.join(p.name for p in self.products.values())
            return False, None, f'Unknown product "{text}". Choose one of: {names}.'
        product = self.products[pid]
        if len(self.queue) >= self.max_queue:
            return False, None, f'The queue is full ({self.max_queue} orders). Try again soon.'
        free = int(shelf_units) - self.reserved(pid)
        if free <= 0:
            return False, None, (f'{product.name} is out of stock on rack {product.rack_id} '
                                 f'({product.rack_name}).')
        self._counter += 1
        order = Order(order_id=f'ORD-{self._counter:03d}', product=pid,
                      product_name=product.name, rack_id=product.rack_id,
                      requested_at=self.clock())
        ahead = len(self.queue) + (1 if self.current else 0)
        order.message = 'Waiting for the robot' if ahead else 'Next in line'
        self.queue.append(order)
        where = 'The robot is on its way.' if not ahead else f'{ahead} order(s) ahead of you.'
        return True, order, f'{order.order_id}: one {product.name} from rack {product.rack_id}. {where}'

    def start_next(self) -> Optional[Order]:
        """Hand the oldest waiting order to the robot (None if busy or nothing waits)."""
        if self.current is not None or not self.queue:
            return None
        self.current = self.queue.pop(0)
        self.current.started_at = self.clock()
        self.advance(TO_RACK, f'Driving to rack {self.current.rack_id}')
        for i, o in enumerate(self.queue):
            o.message = 'Next in line' if i == 0 else f'{i} order(s) ahead'
        return self.current

    def advance(self, state: str, message: str = '') -> None:
        if self.current is None:
            raise RuntimeError('No order in progress')
        if state not in ACTIVE_STATES:
            raise ValueError(f'{state} is not an in-progress state')
        self.current.state = state
        self.current.message = message

    def finish(self, success: bool, message: str = '') -> Order:
        if self.current is None:
            raise RuntimeError('No order in progress')
        order = self.current
        order.state = DELIVERED if success else FAILED
        order.message = message
        order.finished_at = self.clock()
        self.history.insert(0, order)
        del self.history[self.history_size:]
        self.current = None
        return order
