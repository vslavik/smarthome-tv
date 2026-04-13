#!/usr/bin/env python3

import argparse
import logging
import os
from pathlib import Path
import signal
import sys
import tomllib
from typing import Literal

import msgspec
import paho.mqtt.client as mqtt

from scheduled_queue import ScheduledQueue


logger = logging.getLogger(__name__)


DEFAULT_MQTT_PORT = 1883
MQTT_KEEPALIVE = 30
QUEUE_TIMEOUT = 0.2

CEC_ACTIVE_TOPIC = "tvaux/internal/cec/active"
CEC_DEVICE_TOPIC_PREFIX = "tvaux/internal/cec/device/"
PS5_STATE_TOPIC = "tvaux/internal/ps5/ddp"
ANDROIDTV_STATE_TOPIC = "tvaux/internal/androidtv/state"
SYSTEM_STATE_TOPIC = "tvaux/state"
DEVICE_STATE_TOPIC_PREFIX = "tvaux/devices/"
CEC_COMMAND_TOPIC = "tvaux/internal/cec/command"

TV_LOGICAL_ADDRESS = 0
AVR_LOGICAL_ADDRESS = 5


class ConfigError(ValueError):
    pass


class CecDevice(msgspec.Struct):
    osd_name: str
    power_status: str
    logical_address: int
    kind: str | None
    physical_address: str
    vendor: str | None


class CecCommand(msgspec.Struct):
    action: str
    device_id: int | None = None


class CecEvent(msgspec.Struct):
    event: str
    device: CecDevice | None


class PS5StateEvent(msgspec.Struct, omit_defaults=True):
    state: str
    code: int | None
    error: str | None = None


class AndroidTVStateEvent(msgspec.Struct, omit_defaults=True):
    state: str | None
    current_app: str | None
    error: str | None = None


class DeviceConfig(msgspec.Struct, kw_only=True):
    monitor: bool = False
    cec_logical_address: int | None = None
    cec_osd_name: str | None = None
    androidtv_host: str | None = None
    ps5_host: str | None = None


class OrchestratorConfig(msgspec.Struct, kw_only=True):
    monitor_devices: list[str] = []
    devices: dict[str, DeviceConfig] = msgspec.field(default_factory=dict)


def load_config(path: Path) -> OrchestratorConfig:
    with path.open("rb") as file:
        config = msgspec.convert(tomllib.load(file), type=OrchestratorConfig)

    def _ensure_builtin_device(name: str, logical_address: int) -> None:
        if name not in config.devices:
            config.devices[name] = DeviceConfig(cec_logical_address=logical_address)
        if config.devices[name].cec_logical_address not in (None, logical_address):
            raise ConfigError(f"devices.{name}.cec_logical_address must be {logical_address}")
        if config.devices[name].cec_logical_address is None:
            config.devices[name].cec_logical_address = logical_address

    _ensure_builtin_device("tv", TV_LOGICAL_ADDRESS)
    _ensure_builtin_device("avr", AVR_LOGICAL_ADDRESS)

    monitor_devices = config.monitor_devices or list(config.devices.keys())
    unknown_monitor_devices = [device_id for device_id in monitor_devices if device_id not in config.devices]
    if unknown_monitor_devices:
        names = ", ".join(unknown_monitor_devices)
        raise ConfigError(f"monitor_devices references unknown devices: {names}")

    for device_id in monitor_devices:
        config.devices[device_id].monitor = True

    config.monitor_devices = monitor_devices
    return config


class TrackedDevice(msgspec.Struct):
    id: str
    power: Literal["on", "off", "standby", "unavailable"] = "unavailable"
    playback_state: Literal["idle", "playback", "paused"] | None = None

    def publish_to_json(self) -> bytes:
        return msgspec.json.encode({
            "id": self.id,
            "power": self.power,
        })


class DevicesManager:
    def __init__(self, config: OrchestratorConfig) -> None:
        self.config = config
        self.by_id = {
            device_id: TrackedDevice(id=device_id)
            for device_id in config.devices
        }
        self._cec_devices: dict[int, TrackedDevice] = {}
        self.ps5: TrackedDevice | None = None
        self.androidtv: TrackedDevice | None = None
        self.tv: TrackedDevice | None = self.by_id.get("tv")
        self.avr: TrackedDevice | None = self.by_id.get("avr")

        for device_id, cfg in config.devices.items():
            dev = self.by_id[device_id]
            if cfg.ps5_host:
                self.ps5 = dev
            if cfg.androidtv_host:
                self.androidtv = dev

    def match(self, cec: CecDevice) -> TrackedDevice:
        dev = self._cec_devices.get(cec.logical_address)
        if dev is None:
            devid, cfg = self._find_device_config(cec)
            if devid:
                dev = self.by_id[devid]
            else:
                devid = ''
                if cec.vendor:
                    devid = cec.vendor.lower() + '-'
                if cec.osd_name:
                    devid += cec.osd_name.lower()
                if not devid:
                    devid = f"cec-{cec.logical_address}"
                dev = TrackedDevice(id=devid)
                self.by_id[devid] = dev

            logger.info(f"discovered device '{devid}' at logical address {cec.logical_address}")
            self._cec_devices[cec.logical_address] = dev

        return dev

    def _find_device_config(self, cec: CecDevice):
        for id, cfg in self.config.devices.items():
            if cfg.cec_logical_address == cec.logical_address:
                return id, cfg
            elif cfg.cec_osd_name and cfg.cec_osd_name == cec.osd_name:
                return id, cfg
        return None, None


class RuntimeSystemState:
    def __init__(self, devices: DevicesManager) -> None:
        self.dev = devices
        self.active_source: TrackedDevice | None = None

    @property
    def state(self) -> Literal["off", "idle", "playback", "paused", "gaming"]:
        if self.dev.tv.power in ("off", "standby"):
            return "off"
        if self.active_source is None or self.active_source.power != "on":
            return "idle"
        if self.active_source == self.dev.ps5:
            return "gaming"
        if self.active_source == self.dev.androidtv:
            return self.dev.androidtv.playback_state or "idle"
        return "idle"

    def publish_to_json(self) -> bytes:
        return msgspec.json.encode({
            "state": self.state,
            "source": self.active_source.id if self.active_source else None,
        })


class TurnOffTvFixup(msgspec.Struct):
    pass


class Orchestrator:
    def __init__(self, *, mqtt_host: str, mqtt_port: int, config: OrchestratorConfig) -> None:
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
        self.config = config
        self.queue = ScheduledQueue()
        self.stopped = False
        self.mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt.on_connect = self.on_connect
        self.mqtt.on_message = self.on_message
        self.devices = DevicesManager(config)
        self.state = RuntimeSystemState(self.devices)
        self.turn_off_tv_fixup_pending = False

    def stop(self, *_args: object) -> None:
        self.stopped = True
        logger.info("stopping orchestrator")

    def on_connect(self, client, userdata, flags, reason_code, properties=None) -> None:
        client.subscribe(CEC_ACTIVE_TOPIC, qos=1)
        client.subscribe(f"{CEC_DEVICE_TOPIC_PREFIX}+", qos=1)
        client.subscribe(PS5_STATE_TOPIC, qos=1)
        client.subscribe(ANDROIDTV_STATE_TOPIC, qos=1)
        logger.info("subscribed to orchestrator MQTT topics")

    def on_message(self, client, userdata, msg) -> None:
        try:
            if msg.topic == CEC_ACTIVE_TOPIC or msg.topic.startswith(CEC_DEVICE_TOPIC_PREFIX):
                event = msgspec.json.decode(msg.payload, type=CecEvent)
            elif msg.topic == PS5_STATE_TOPIC:
                event = msgspec.json.decode(msg.payload, type=PS5StateEvent)
            elif msg.topic == ANDROIDTV_STATE_TOPIC:
                event = msgspec.json.decode(msg.payload, type=AndroidTVStateEvent)
            else:
                logger.warning("ignoring message on unexpected topic %s", msg.topic)
                return
        except msgspec.MsgspecError as error:
            logger.warning("ignoring invalid payload on %s: %s", msg.topic, error)
            return

        self.queue.put(event)

    def handle(self, event: object) -> None:
        match event:
            case CecEvent("active-source"):
                self.handle_cec_source_change(event)
            case CecEvent("power" | "scan"):
                self.handle_cec_power(event)
            case PS5StateEvent():
                self.handle_ps5_state(event)
            case AndroidTVStateEvent():
                self.handle_androidtv_state(event)
            case TurnOffTvFixup():
                self.handle_turn_off_tv_fixup()
            case _:
                raise TypeError(f"unexpected event: {event!r}")

    def handle_cec_source_change(self, event: CecEvent) -> None:
        if event.device is None:
            self.state.active_source = None
        else:
            dev = self.devices.match(event.device)
            self.state.active_source = dev
        self.publish_system_state()

    def handle_cec_power(self, event: CecEvent) -> None:
        dev = self.devices.match(event.device)
        if dev.power != event.device.power_status:
            dev.power = event.device.power_status
            self.handle_power_change(dev)

    def handle_ps5_state(self, event: PS5StateEvent) -> None:
        if not self.devices.ps5:
            return
        if event.state != self.devices.ps5.power:
            self.devices.ps5.power = event.state
            self.handle_power_change(self.devices.ps5)

    def handle_androidtv_state(self, event: AndroidTVStateEvent) -> None:
        if not self.devices.androidtv:
            return

        if event.state and event.state != "unavailable" and event.state != self.devices.androidtv.playback_state:
            self.devices.androidtv.playback_state = event.state
        else:
            self.devices.androidtv.playback_state = None

        if self.state.active_source == self.devices.androidtv:
            self.publish_system_state()

    def handle_power_change(self, device: TrackedDevice) -> None:
        self.publish_device_state(device)
        if device is self.state.active_source:
            self.state.active_source = None
            self.publish_system_state()
        elif device is self.devices.tv:
            self.publish_system_state()

        if device.id in self.config.monitor_devices and device.power in ("off", "standby"):
            self.schedule_turn_off_tv_fixup()

    def schedule_turn_off_tv_fixup(self) -> None:
        if self.turn_off_tv_fixup_pending:
            return
        if not self.should_turn_off_tv():
            return

        self.turn_off_tv_fixup_pending = True
        self.queue.put(TurnOffTvFixup(), delay=2.5)

    def handle_turn_off_tv_fixup(self) -> None:
        self.turn_off_tv_fixup_pending = False
        if not self.should_turn_off_tv():
            return

        logger.info("All monitored devices are off; turning TV off")
        command = CecCommand(action="standby", device_id=TV_LOGICAL_ADDRESS)
        self.mqtt.publish(CEC_COMMAND_TOPIC, msgspec.json.encode(command), qos=1)

    def should_turn_off_tv(self) -> bool:
        if not self.devices.tv or self.devices.tv.power != "on":
            return False

        for device_id in self.config.monitor_devices:
            device = self.devices.by_id[device_id]
            if device.power not in ("off", "standby"):
                return False

        return True

    def publish_system_state(self) -> None:
        payload = self.state.publish_to_json()
        self.mqtt.publish(SYSTEM_STATE_TOPIC, payload, qos=1, retain=True)

    def publish_device_state(self, device: TrackedDevice) -> None:
        payload = device.publish_to_json()
        self.mqtt.publish(f"{DEVICE_STATE_TOPIC_PREFIX}{device.id}", payload, qos=1, retain=True)

    def run(self) -> int:
        logger.info(
            "loaded %d devices (%d monitored), mqtt=%s:%d",
            len(self.config.devices),
            len(self.config.monitor_devices),
            self.mqtt_host,
            self.mqtt_port,
        )

        for device_id in sorted(self.config.devices):
            logger.info("device %s: %s", device_id, self.config.devices[device_id])

        self.mqtt.connect(self.mqtt_host, self.mqtt_port, keepalive=MQTT_KEEPALIVE)
        self.mqtt.loop_start()

        try:
            while not self.stopped:
                event = self.queue.get(timeout=QUEUE_TIMEOUT)
                if event is not None:
                    self.handle(event)
        finally:
            self.mqtt.loop_stop()
            self.mqtt.disconnect()

        return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--mqtt-host", default=os.getenv("MQTT_HOST"))
    parser.add_argument("--mqtt-port", type=int, default=DEFAULT_MQTT_PORT)
    parser.add_argument("--debug", action="store_true")
    return parser


def configure_logging(debug: bool) -> None:
    logging.basicConfig(
        level=logging.DEBUG if debug else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if not args.mqtt_host:
        raise SystemExit("--mqtt-host or MQTT_HOST is required")

    configure_logging(args.debug)

    try:
        config = load_config(Path(args.config))
    except FileNotFoundError:
        raise SystemExit(f"config file not found: {args.config}") from None
    except tomllib.TOMLDecodeError as error:
        raise SystemExit(f"invalid TOML in {args.config}: {error}") from error
    except (ConfigError, msgspec.ValidationError) as error:
        raise SystemExit(f"invalid config in {args.config}: {error}") from error

    orchestrator = Orchestrator(
        mqtt_host=args.mqtt_host,
        mqtt_port=args.mqtt_port,
        config=config,
    )
    signal.signal(signal.SIGINT, orchestrator.stop)
    signal.signal(signal.SIGTERM, orchestrator.stop)
    return orchestrator.run()


if __name__ == "__main__":
    sys.exit(main())
