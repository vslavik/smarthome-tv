
from collections.abc import Callable
import heapq
from itertools import count
import threading
import time


class ScheduledQueue:
    """
    Thread-safe priority queue, ordered by readiness time, that supports both
    immediate order-preserving delivery and delayed execution.

    Items are ordered by monotonic ready time, with insertion order preserved for
    items scheduled for the same time.
    """

    def __init__(self, *, clock: Callable[[], float] = time.monotonic) -> None:
        """
        Initialize the queue using the given monotonic clock function.
        """
        self._clock = clock
        self._condition = threading.Condition()
        self._counter = count()
        self._items: list[tuple[float, int, object]] = []

    def __len__(self) -> int:
        with self._condition:
            return len(self._items)

    def empty(self) -> bool:
        with self._condition:
            return not self._items

    def put(self, item: object, *, delay: float = 0.0) -> None:
        """
        Enqueue an item to become available after the given delay in seconds,
        or immediately if no delay is specified.
        """
        ready_at = self._clock() + max(delay, 0.0)
        with self._condition:
            heapq.heappush(self._items, (ready_at, next(self._counter), item))
            self._condition.notify()

    def get(self, *, timeout: float = 0.0) -> object | None:
        """
        Return the next due item, or None if no item becomes due before timeout.
        Blocks for the duration of the timeout or until an item becomes due,
        whichever is sooner.
        """
        deadline = self._clock() + max(timeout, 0.0)

        with self._condition:
            while True:
                now = self._clock()
                wait_until = deadline

                if self._items:
                    first_time = self._items[0][0]
                    if first_time <= now:
                        _ready_at, _counter, item = heapq.heappop(self._items)
                        return item
                    wait_until = min(wait_until, first_time)

                if wait_until <= now:
                    return None

                wait_for = wait_until - now
                self._condition.wait(wait_for)
