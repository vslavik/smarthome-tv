#!/usr/bin/env python3

import argparse
import logging
import os
from queue import Empty, SimpleQueue
import signal
import sys
import time
from typing import Literal

import msgspec
import paho.mqtt.client as mqtt

from androidtv_monitor import AndroidTVMonitor


logger = logging.getLogger(__name__)


MQTT_PORT = 1883
MQTT_KEEPALIVE = 30
MQTT_STATE_TOPIC = "tvaux/internal/androidtv/state"
MQTT_COMMAND_TOPIC = "tvaux/internal/androidtv/command"

DEFAULT_PORT = 5555
DEFAULT_POLL_INTERVAL = 2.0
IDLE_SLEEP = 0.2


class MonitorCommand(msgspec.Struct):
    action: Literal["monitor", "pause"]
    host: str | None = None


class AndroidTVMqttBridge:
    def __init__(
        self,
        *,
        host: str | None,
        port: int,
        adbkey: str,
        mqtt_host: str,
        poll_interval: float,
    ) -> None:
        self.host = host
        self.port = port
        self.adbkey = adbkey
        self.monitor: AndroidTVMonitor | None = None
        self.poll_interval = poll_interval
        self.commands: SimpleQueue[MonitorCommand] = SimpleQueue()
        self.encoder = msgspec.json.Encoder()
        self.paused = True
        self.stopped = False
        self.next_poll_at = 0.0
        self.last_state = None

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(mqtt_host, MQTT_PORT, keepalive=MQTT_KEEPALIVE)

        if host is not None:
            self.start_monitoring(host)

    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        client.subscribe(MQTT_COMMAND_TOPIC, qos=1)

    def on_message(self, client, userdata, msg):
        try:
            command = msgspec.json.decode(msg.payload, type=MonitorCommand)
        except msgspec.DecodeError:
            logger.warning("ignoring invalid command payload on %s: %r", msg.topic, msg.payload)
            return
        self.commands.put(command)

    def stop(self, *_args: object) -> None:
        self.stopped = True
        logger.info("stopping Android TV bridge (host=%s)", self.host or "none")

    def start_monitoring(self, host: str) -> None:
        host = host.strip()
        if not host:
            logger.warning("ignoring monitor request with empty host")
            return

        previous_host = self.host
        host_changed = host != previous_host
        was_paused = self.paused

        if not was_paused and not host_changed:
            logger.info("Android TV monitoring already active for host %s", host)
            return

        if host_changed and self.monitor is not None:
            logger.info("switching Android TV monitoring host from %s to %s", previous_host, host)
            self.monitor.close()
            self.monitor = None

        if self.monitor is None:
            self.monitor = AndroidTVMonitor(host=host, port=self.port, adbkey=self.adbkey)

        self.host = host
        self.paused = False
        self.next_poll_at = 0.0

        if was_paused or host_changed:
            self.last_state = None

        if host_changed and previous_host is not None:
            logger.info("starting Android TV monitoring for host %s", host)
        elif was_paused:
            logger.info("starting Android TV monitoring for host %s", host)

    def pause_monitoring(self) -> None:
        if self.paused:
            logger.info("Android TV monitoring already paused (host=%s)", self.host or "none")
            return

        self.paused = True
        self.next_poll_at = 0.0
        self.last_state = None
        if self.monitor is not None:
            self.monitor.close()
            self.monitor = None
        logger.info("pausing Android TV monitoring for host %s", self.host or "none")

    def handle_command(self, command: MonitorCommand) -> None:
        if command.action == "pause":
            self.pause_monitoring()
            return

        if command.action == "monitor":
            target_host = self.host if command.host is None else command.host
            if target_host is None:
                logger.warning("ignoring monitor command without a host; no previous host configured")
                return
            self.start_monitoring(target_host)
            return

    def publish_state(self, state) -> None:
        self.client.publish(
            MQTT_STATE_TOPIC,
            self.encoder.encode(state),
            qos=1,
            retain=True,
        )

    def run(self) -> int:
        self.client.loop_start()
        signal.signal(signal.SIGINT, self.stop)
        signal.signal(signal.SIGTERM, self.stop)
        if self.paused:
            logger.info("starting Android TV bridge in paused state with no host configured")
        else:
            logger.info("starting Android TV bridge for host %s", self.host)

        try:
            while not self.stopped:
                try:
                    while True:
                        self.handle_command(self.commands.get_nowait())
                except Empty:
                    pass

                if self.paused:
                    time.sleep(IDLE_SLEEP)
                    continue

                if self.monitor is None:
                    logger.warning("Android TV monitoring is active but no monitor is configured")
                    self.pause_monitoring()
                    time.sleep(IDLE_SLEEP)
                    continue

                now = time.monotonic()
                if now < self.next_poll_at:
                    time.sleep(min(IDLE_SLEEP, self.next_poll_at - now))
                    continue

                state = self.monitor.poll()
                self.next_poll_at = time.monotonic() + self.poll_interval

                if self.last_state is None or state.change_key() != self.last_state.change_key():
                    logger.info("%s", state)
                    self.publish_state(state)
                    self.last_state = state
        finally:
            self.client.loop_stop()
            self.client.disconnect()
            if self.monitor is not None:
                self.monitor.close()

        return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default=os.getenv("ANDROIDTV_HOST"))
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--adbkey", required=True)
    parser.add_argument("--mqtt-host", default=os.getenv("MQTT_HOST"))
    parser.add_argument("--poll-interval", type=float, default=DEFAULT_POLL_INTERVAL)
    parser.add_argument("--debug", action="store_true")
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

    bridge = AndroidTVMqttBridge(
        host=host,
        port=args.port,
        adbkey=args.adbkey,
        mqtt_host=args.mqtt_host,
        poll_interval=args.poll_interval,
    )
    return bridge.run()


if __name__ == "__main__":
    sys.exit(main())
