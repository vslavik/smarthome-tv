#!/usr/bin/env python3

import argparse
import json
import logging
import os
from queue import Empty, SimpleQueue
import signal
import sys
import time
from dataclasses import dataclass

import paho.mqtt.client as mqtt

from ps5_probe import probe


MQTT_PORT = 1883

MQTT_KEEPALIVE = 30
POLL_INTERVAL = 1.0
PROBE_TIMEOUT = 0.5
IDLE_SLEEP = 0.2
# PS5 goes into non-responding state during on<->standby transition, so we wait a bit before declaring it off
OFF_DELAY = 15.0

MQTT_STATE_TOPIC     = "tvaux/internal/ps5/ddp"
MQTT_COMMAND_TOPIC   = "tvaux/internal/ps5/command"


logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class MonitorCommand:
    action: str
    host: str | None = None


@dataclass
class BridgeState:
    paused: bool = False
    published_state: str | None = None
    missing_since: float | None = None
    next_probe_at: float = 0.0
    stopped: bool = False


class Ps5MqttBridge:
    def __init__(self, *, ps5_host: str | None, mqtt_host: str):
        self.ps5_host = ps5_host
        self.state = BridgeState(paused=ps5_host is None)
        self.commands: SimpleQueue[MonitorCommand] = SimpleQueue()
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(mqtt_host, MQTT_PORT, keepalive=MQTT_KEEPALIVE)

    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        client.subscribe(MQTT_COMMAND_TOPIC, qos=1)

    def on_message(self, client, userdata, msg):
        try:
            raw = json.loads(msg.payload)
        except json.JSONDecodeError:
            logger.warning("ignoring invalid command payload on %s: %r", msg.topic, msg.payload)
            return
        if not isinstance(raw, dict):
            logger.warning("ignoring non-object command payload on %s: %r", msg.topic, msg.payload)
            return

        action = raw.get("action")
        host = raw.get("host")
        if not isinstance(action, str):
            logger.warning("ignoring command without string action on %s: %r", msg.topic, msg.payload)
            return
        if host is not None and not isinstance(host, str):
            logger.warning("ignoring command with non-string host on %s: %r", msg.topic, msg.payload)
            return

        command = MonitorCommand(action=action, host=host)
        self.commands.put(command)

    def start_monitoring(self, host: str) -> None:
        host = host.strip()
        if not host:
            logger.warning("ignoring monitor request with empty host")
            return

        previous_host = self.ps5_host
        host_changed = host != previous_host
        was_paused = self.state.paused

        if not was_paused and not host_changed:
            logger.info("PS5 monitoring already active for host %s", host)
            return

        self.ps5_host = host
        self.state.paused = False
        self.state.missing_since = None
        self.state.next_probe_at = 0.0

        if was_paused or host_changed:
            self.state.published_state = None

        if host_changed and previous_host is not None:
            logger.info("switching PS5 monitoring host from %s to %s", previous_host, host)
        logger.info("starting PS5 monitoring for host %s", host)

    def pause_monitoring(self) -> None:
        if self.state.paused:
            logger.info("PS5 monitoring already paused (host=%s)", self.ps5_host or "none")
            return

        self.state.paused = True
        self.state.missing_since = None
        self.state.next_probe_at = 0.0
        logger.info("pausing PS5 monitoring for host %s", self.ps5_host or "none")

    def handle_command(self, command: MonitorCommand) -> None:
        if command.action == "pause":
            self.pause_monitoring()
            return

        if command.action == "monitor":
            target_host = self.ps5_host if command.host is None else command.host
            if target_host is None:
                logger.warning("ignoring monitor command without a host; no previous host configured")
                return
            self.start_monitoring(target_host)
            return

        logger.warning("ignoring unknown command %r", command)

    def publish_state(self, state, details):
        previous = self.state.published_state
        self.client.publish(MQTT_STATE_TOPIC, json.dumps(details), qos=1, retain=True)
        self.state.published_state = state
        logger.info("state %s -> %s", previous, state)

    def handle_probe_result(self, result):
        now = time.monotonic()

        if result["code"] is None:
            if self.state.missing_since is None:
                self.state.missing_since = now
            if self.state.published_state != "off" and now - self.state.missing_since >= OFF_DELAY:
                self.publish_state("off", result)
            return

        self.state.missing_since = None
        if result["state"] != self.state.published_state:
            self.publish_state(result["state"], result)

    def run(self) -> int:
        self.client.loop_start()
        signal.signal(signal.SIGINT, self.stop)
        signal.signal(signal.SIGTERM, self.stop)
        if self.ps5_host is None:
            logger.info("starting PS5 bridge in paused state with no host configured")
        else:
            logger.info("starting PS5 bridge for host %s", self.ps5_host)

        try:
            while not self.state.stopped:
                try:
                    while True:
                        command = self.commands.get_nowait()
                        self.handle_command(command)
                except Empty:
                    pass

                if self.state.paused:
                    time.sleep(IDLE_SLEEP)
                    continue

                now = time.monotonic()
                if now < self.state.next_probe_at:
                    time.sleep(min(IDLE_SLEEP, self.state.next_probe_at - now))
                    continue

                if self.ps5_host is None:
                    logger.warning("PS5 monitoring is active but no host is configured")
                    self.pause_monitoring()
                    time.sleep(IDLE_SLEEP)
                    continue

                result = probe(self.ps5_host, timeout=PROBE_TIMEOUT, tries=1)
                self.state.next_probe_at = time.monotonic() + POLL_INTERVAL
                self.handle_probe_result(result)
        finally:
            self.client.loop_stop()
            self.client.disconnect()

        return 0

    def stop(self, *_args):
        self.state.stopped = True
        logger.info("stopping PS5 bridge (host=%s)", self.ps5_host or "none")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default=os.getenv("PS5_HOST"), help="IP address or hostname of the PS5")
    parser.add_argument("--mqtt-host", default=os.getenv("MQTT_HOST"), help="IP address or hostname of the MQTT broker")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    host = args.host or None

    if not args.mqtt_host:
        raise SystemExit("--mqtt-host or MQTT_HOST is required")

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    bridge = Ps5MqttBridge(
        ps5_host=host,
        mqtt_host=args.mqtt_host,
    )
    return bridge.run()


if __name__ == "__main__":
    sys.exit(main())
