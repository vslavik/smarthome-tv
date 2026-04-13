#!/usr/bin/env python3

import argparse
import logging
import os
from queue import Empty, SimpleQueue
import signal
import sys
import time

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


class AndroidTVMqttBridge:
    def __init__(
        self,
        *,
        host: str,
        port: int,
        adbkey: str,
        mqtt_host: str,
        poll_interval: float,
    ) -> None:
        self.monitor = AndroidTVMonitor(host=host, port=port, adbkey=adbkey)
        self.poll_interval = poll_interval
        self.commands: SimpleQueue[str] = SimpleQueue()
        self.encoder = msgspec.json.Encoder()
        self.paused = False
        self.stopped = False
        self.next_poll_at = 0.0
        self.last_state = None

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(mqtt_host, MQTT_PORT, keepalive=MQTT_KEEPALIVE)

    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        client.subscribe(MQTT_COMMAND_TOPIC, qos=1)

    def on_message(self, client, userdata, msg):
        command = msg.payload.decode("utf-8", "replace").strip().lower()
        self.commands.put(command)

    def stop(self, *_args: object) -> None:
        self.stopped = True
        logger.info("stopping bridge")

    def handle_command(self, command: str) -> None:
        if command == "stop":
            self.paused = True
            logger.info("received command stop")
            return

        if command == "start":
            self.paused = False
            self.next_poll_at = 0.0
            logger.info("received command start")
            return

        logger.warning("ignoring unknown command %r", command)

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

    if not args.host:
        raise SystemExit("--host or ANDROIDTV_HOST is required")
    if not args.mqtt_host:
        raise SystemExit("--mqtt-host or MQTT_HOST is required")

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    bridge = AndroidTVMqttBridge(
        host=args.host,
        port=args.port,
        adbkey=args.adbkey,
        mqtt_host=args.mqtt_host,
        poll_interval=args.poll_interval,
    )
    return bridge.run()


if __name__ == "__main__":
    sys.exit(main())
