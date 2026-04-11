#!/usr/bin/env python3

import argparse
import json
import logging
import signal
import sys
import time

from androidtv_monitor import AndroidTVMonitor, AndroidTVState


logger = logging.getLogger(__name__)


DEFAULT_PORT = 5555
DEFAULT_POLL_INTERVAL = 2.0
DEFAULT_RECONNECT_INTERVAL = 5.0


def result_to_dict(result: AndroidTVState) -> dict[str, object]:
    return {
        "state": result.state,
        "current_app": result.current_app,
        "media_session_state": result.media_session_state,
        "audio_state": result.audio_state,
        "error": result.error,
    }


def format_result(result: AndroidTVState) -> str:
    parts = [
        f"state={result.state or 'unknown'}",
    ]

    if result.current_app:
        parts.append(f"app={result.current_app}")
    if result.media_session_state is not None:
        parts.append(f"media_session={result.media_session_state}")
    if result.audio_state:
        parts.append(f"audio={result.audio_state}")
    if result.error:
        parts.append(f"error={result.error}")

    return " ".join(parts)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--adbkey", required=True)
    parser.add_argument("--poll-interval", type=float, default=DEFAULT_POLL_INTERVAL)
    parser.add_argument("--reconnect-interval", type=float, default=DEFAULT_RECONNECT_INTERVAL)
    parser.add_argument("--oneshot", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--print-every-poll", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    monitor = AndroidTVMonitor(host=args.host, port=args.port, adbkey=args.adbkey)
    stopped = False
    last_change_key: tuple[object, ...] | None = None

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
            if args.print_every_poll or result.change_key() != last_change_key:
                if args.json:
                    print(json.dumps(result_to_dict(result), sort_keys=True), flush=True)
                else:
                    print(format_result(result), flush=True)
                last_change_key = result.change_key()

            if args.oneshot:
                break

            interval = args.reconnect_interval if result.error else args.poll_interval
            next_probe_at = time.monotonic() + interval
    finally:
        monitor.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
