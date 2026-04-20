#!/usr/bin/env python3

import argparse
import asyncio
import json
import logging
import os
from queue import Empty, SimpleQueue
import signal
import sys
from dataclasses import dataclass

import paho.mqtt.client as mqtt

from streaming_estimator import StreamingEstimator
from streaming_snmp import DEFAULT_COMMUNITY, SnmpClient


MQTT_PORT = 1883
MQTT_KEEPALIVE = 30
IDLE_SLEEP = 0.2
POLL_INTERVAL = 5.0

MQTT_STATE_TOPIC = "tvaux/internal/streaming"
MQTT_COMMAND_TOPIC = "tvaux/internal/streaming/command"

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class MonitorCommand:
    action: str


@dataclass
class BridgeState:
    paused: bool = False
    next_poll_at: float = 0.0
    stopped: bool = False


class StreamingMqttBridge:
    def __init__(
        self,
        *,
        switch_host: str,
        monitored_port: int,
        mqtt_host: str,
        snmp_community: str,
    ) -> None:
        self.switch_host = switch_host
        self.monitored_port = monitored_port
        self.interval = POLL_INTERVAL
        self.snmp_community = snmp_community
        self.state = BridgeState()
        self.commands: SimpleQueue[MonitorCommand] = SimpleQueue()
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(mqtt_host, MQTT_PORT, keepalive=MQTT_KEEPALIVE)
        self.estimator = StreamingEstimator()
        self.snmp_client: SnmpClient | None = None
        self.interface_index: int | None = None
        self.previous_sample = None

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
        if not isinstance(action, str):
            logger.warning("ignoring command without string action on %s: %r", msg.topic, msg.payload)
            return

        self.commands.put(MonitorCommand(action=action))

    def start_monitoring(self) -> None:
        if not self.state.paused:
            logger.info("streaming monitoring already active for %s port %s", self.switch_host, self.monitored_port)
            return

        self.state.paused = False
        self.state.next_poll_at = 0.0
        self.reset_monitoring_state()
        logger.info("starting streaming monitoring for %s port %s", self.switch_host, self.monitored_port)

    def pause_monitoring(self) -> None:
        if self.state.paused:
            logger.info("streaming monitoring already stopped")
            return

        self.state.paused = True
        self.state.next_poll_at = 0.0
        self.reset_monitoring_state()
        self.publish_bitrate(0.0)
        logger.info("stopping streaming monitoring")

    def reset_monitoring_state(self) -> None:
        self.estimator.reset()
        self.snmp_client = None
        self.interface_index = None
        self.previous_sample = None

    def handle_command(self, command: MonitorCommand) -> None:
        if command.action == "monitor":
            self.start_monitoring()
            return
        if command.action == "pause":
            self.pause_monitoring()
            return
        logger.warning("ignoring unknown command %r", command)

    def publish_bitrate(self, bitrate_mbps: float) -> None:
        payload = json.dumps({"bitrate": round(bitrate_mbps)})
        self.client.publish(MQTT_STATE_TOPIC, payload, qos=0, retain=False)

    async def run_async(self) -> int:
        self.client.loop_start()
        signal.signal(signal.SIGINT, self.stop)
        signal.signal(signal.SIGTERM, self.stop)
        logger.info("starting streaming bridge for %s port %s", self.switch_host, self.monitored_port)

        try:
            while not self.state.stopped:
                try:
                    while True:
                        command = self.commands.get_nowait()
                        self.handle_command(command)
                except Empty:
                    pass

                if self.state.paused:
                    await asyncio.sleep(IDLE_SLEEP)
                    continue

                now = asyncio.get_running_loop().time()
                if now < self.state.next_poll_at:
                    await asyncio.sleep(min(IDLE_SLEEP, self.state.next_poll_at - now))
                    continue

                if self.snmp_client is None:
                    self.snmp_client = SnmpClient(self.switch_host, self.snmp_community)
                    await self.snmp_client.connect()
                    interface = await self.snmp_client.discover_interface(self.monitored_port)
                    self.interface_index = interface.index
                    self.previous_sample = await self.snmp_client.read_sample(self.interface_index)
                    self.state.next_poll_at = asyncio.get_running_loop().time() + self.interval
                    continue

                assert self.snmp_client is not None
                assert self.interface_index is not None
                assert self.previous_sample is not None
                current_sample = await self.snmp_client.read_sample(self.interface_index)
                observed = SnmpClient.out_mbps(self.previous_sample, current_sample)
                self.previous_sample = current_sample
                self.state.next_poll_at = asyncio.get_running_loop().time() + self.interval
                if observed is None:
                    continue

                estimate = self.estimator.observe(observed)
                logger.debug("observed=%.3f estimate=%.3f", observed, estimate)
                self.publish_bitrate(estimate)
        finally:
            self.publish_bitrate(0.0)
            self.client.loop_stop()
            self.client.disconnect()

        return 0

    def run(self) -> int:
        return asyncio.run(self.run_async())

    def stop(self, *_args):
        self.state.stopped = True
        logger.info("stopping streaming bridge")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Poll a configured UniFi switch port over SNMP, estimate streaming bitrate, and publish it to MQTT.",
    )
    parser.add_argument(
        "--switch-host",
        default=os.getenv("SWITCH_HOST"),
        help="IP address or hostname of the UniFi switch (default: SWITCH_HOST)",
    )
    parser.add_argument(
        "--mqtt-host",
        default=os.getenv("MQTT_HOST"),
        help="IP address or hostname of the MQTT broker (default: MQTT_HOST)",
    )
    parser.add_argument(
        "--snmp-community",
        default=os.getenv("SNMP_COMMUNITY", DEFAULT_COMMUNITY),
        help='SNMP v2c community string (default: SNMP_COMMUNITY or "public")',
    )
    parser.add_argument(
        "--monitored-port",
        type=int,
        default=os.getenv("MONITORED_PORT"),
        help="Physical switch port number to monitor (default: MONITORED_PORT)",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug logging",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    switch_host = args.switch_host or None
    monitored_port = args.monitored_port

    if not args.mqtt_host:
        raise SystemExit("--mqtt-host or MQTT_HOST is required")
    if not switch_host:
        raise SystemExit("--switch-host or SWITCH_HOST is required")
    if monitored_port is None:
        raise SystemExit("--monitored-port or MONITORED_PORT is required")

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    bridge = StreamingMqttBridge(
        switch_host=switch_host,
        monitored_port=monitored_port,
        mqtt_host=args.mqtt_host,
        snmp_community=args.snmp_community,
    )
    return bridge.run()


if __name__ == "__main__":
    sys.exit(main())
