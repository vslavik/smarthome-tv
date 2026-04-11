#!/usr/bin/env python3

import argparse
import logging
import signal
import sys
import time

from androidtv_monitor import AndroidTVMonitor


logger = logging.getLogger(__name__)


DEFAULT_PORT = 5555
DEFAULT_POLL_INTERVAL = 2.0
DEFAULT_RECONNECT_INTERVAL = 5.0

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--adbkey", required=True)
    parser.add_argument("--poll-interval", type=float, default=DEFAULT_POLL_INTERVAL)
    parser.add_argument("--reconnect-interval", type=float, default=DEFAULT_RECONNECT_INTERVAL)
    parser.add_argument("--debug", action="store_true")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    monitor = AndroidTVMonitor(host=args.host, port=args.port, adbkey=args.adbkey)
    stopped = False
    last_result = None

    def stop(*_args: object) -> None:
        nonlocal stopped
        stopped = True
        logger.info("stopping monitor")

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    next_probe_at = 0.0
    try:
        while not stopped:
            now = time.monotonic()
            if now < next_probe_at:
                time.sleep(min(0.2, next_probe_at - now))
                continue

            result = monitor.poll()
            if last_result is None or result.change_key() != last_result.change_key():
                logger.info("Status change: %s", result)
                last_result = result

            interval = args.reconnect_interval if result.error else args.poll_interval
            next_probe_at = time.monotonic() + interval
    finally:
        monitor.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
