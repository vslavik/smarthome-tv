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


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
)


@dataclass
class BridgeState:
    paused: bool = False
    published_state: str | None = None
    missing_since: float | None = None
    next_probe_at: float = 0.0
    stopped: bool = False


class Ps5MqttBridge:
    def __init__(self, *, ps5_host: str, mqtt_host: str):
        self.ps5_host = ps5_host
        self.state = BridgeState()
        self.commands = SimpleQueue()
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(mqtt_host, MQTT_PORT, keepalive=MQTT_KEEPALIVE)

    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        client.subscribe(MQTT_COMMAND_TOPIC, qos=1)

    def on_message(self, client, userdata, msg):
        command = msg.payload.decode("utf-8", "replace").strip().lower()
        self.commands.put(command)

    def handle_command(self, command):
        if command == "stop":
            self.state.paused = True
            self.state.missing_since = None
            logging.info("received command stop")
        elif command == "start":
            self.state.paused = False
            self.state.missing_since = None
            self.state.next_probe_at = 0.0
            logging.info("received command start")
        else:
            logging.warning("ignoring unknown command %r", command)

    def publish_state(self, state, details):
        previous = self.state.published_state
        self.client.publish(MQTT_STATE_TOPIC, json.dumps(details), qos=1, retain=True)
        self.state.published_state = state
        logging.info("state %s -> %s", previous, state)

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
        logging.info("starting bridge for PS5 %s", self.ps5_host)

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

                result = probe(self.ps5_host, timeout=PROBE_TIMEOUT, tries=1)
                self.state.next_probe_at = time.monotonic() + POLL_INTERVAL
                self.handle_probe_result(result)
        finally:
            self.client.loop_stop()
            self.client.disconnect()

        return 0

    def stop(self, *_args):
        self.state.stopped = True
        logging.info("stopping bridge")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default=os.getenv("PS5_HOST"), help="IP address or hostname of the PS5")
    parser.add_argument("--mqtt-host", default=os.getenv("MQTT_HOST"), help="IP address or hostname of the MQTT broker")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")

    args = parser.parse_args()

    if not args.host:
        raise SystemExit("--host or PS5_HOST is required")
    if not args.mqtt_host:
        raise SystemExit("--mqtt-host or MQTT_HOST is required")

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    bridge = Ps5MqttBridge(
        ps5_host=args.host,
        mqtt_host=args.mqtt_host,
    )
    return bridge.run()


if __name__ == "__main__":
    sys.exit(main())
