#!/usr/bin/env python3

import argparse
import logging
import os
from pathlib import Path
import signal
import sys
import tomllib

import msgspec


logger = logging.getLogger(__name__)


DEFAULT_MQTT_PORT = 1883

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


class Orchestrator:
    def __init__(self, *, mqtt_host: str, mqtt_port: int, config: OrchestratorConfig) -> None:
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
        self.config = config
        self.stopped = False

    def stop(self, *_args: object) -> None:
        self.stopped = True
        logger.info("stopping orchestrator")

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
            case _:
                raise TypeError(f"unexpected event: {event!r}")

    def handle_cec_source_change(self, event: CecEvent) -> None:
        raise NotImplementedError

    def handle_cec_power(self, event: CecEvent) -> None:
        raise NotImplementedError

    def handle_ps5_state(self, state: PS5StateEvent) -> None:
        raise NotImplementedError

    def handle_androidtv_state(self, state: AndroidTVStateEvent) -> None:
        raise NotImplementedError

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
