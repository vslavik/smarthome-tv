#!/usr/bin/env python3

from __future__ import annotations

import argparse
import logging
import os
from queue import Empty, SimpleQueue
import signal
import time
from typing import Callable

import cec
import msgspec
import paho.mqtt.client as mqtt


IDLE_SLEEP = 0.2
LOGICAL_ADDRESS_COUNT = 15

MQTT_PORT = 1883
MQTT_KEEPALIVE = 30
MQTT_BASE_TOPIC = 'tvaux/internal/cec'
MQTT_COMMAND_TOPIC = f'{MQTT_BASE_TOPIC}/command'


logger = logging.getLogger(__name__)


class CECMessage(msgspec.Struct, frozen=True):
    """A message on the CEC bus"""
    raw: str
    source: int
    destination: int
    opcode: int | None = None
    parameters: tuple[int, ...] = ()


class CECDevice(msgspec.Struct):
    osd_name: str
    power_status: str
    logical_address: int
    kind: str | None
    physical_address: str
    vendor: str | None
    cec_version: str


class CECDeviceEvent(msgspec.Struct):
    event: str
    device: CECDevice | None


class CECCommand(msgspec.Struct):
    action: str
    device_id: int | None = None


def parse_command(raw_command: str) -> CECMessage | None:
    parts = raw_command.split(':')
    if not parts or len(parts[0]) != 2:
        return None

    if any(len(part) != 2 for part in parts[1:] if part):
        return None

    try:
        source = int(parts[0][0], 16)
        destination = int(parts[0][1], 16)
        opcode = int(parts[1], 16) if len(parts) >= 2 and parts[1] else None
        parameters = tuple(int(part, 16) for part in parts[2:])
    except ValueError:
        return None

    return CECMessage(
        raw=raw_command,
        source=source,
        destination=destination,
        opcode=opcode,
        parameters=parameters,
    )


def logical_address_kind(logical_address: int) -> str | None:
    match logical_address:
        case cec.CECDEVICE_TV:
            return 'tv'
        case cec.CECDEVICE_AUDIOSYSTEM:
            return 'audio'
        case cec.CECDEVICE_PLAYBACKDEVICE1 | cec.CECDEVICE_PLAYBACKDEVICE2 | cec.CECDEVICE_PLAYBACKDEVICE3:
            return 'playback'
        case _:
            return None


class CECClient:
    def __init__(self, publish: Callable[[str, CECDeviceEvent], None]) -> None:
        self.active_source_id: int | None = None
        self.active_source_known = False
        self.config: cec.libcec_configuration | None = None
        self.lib: cec.ICECAdapter | None = None
        self.devices: dict[int, CECDevice] = {}
        self.messages: SimpleQueue[CECMessage] = SimpleQueue()
        self.publish = publish

    def init(self) -> None:
        self.config = cec.libcec_configuration()
        self.config.strDeviceName = 'TVAUX'
        self.config.bActivateSource = 0
        self.config.clientVersion = cec.LIBCEC_VERSION_CURRENT
        self.config.deviceTypes.Add(cec.CEC_DEVICE_TYPE_RECORDING_DEVICE)
        self.config.SetCommandCallback(self.handle_command)
        self.lib = cec.ICECAdapter.Create(self.config)

        adapters = self.lib.DetectAdapters()
        if not adapters:
            raise RuntimeError('No CEC adapters found')

        port = adapters[0].strComName
        if not self.lib.Open(port):
            raise RuntimeError(f'Failed to open {port}')

        logger.info('Opened adapter %s', port)

    def handle_command(self, cmd) -> int:
        raw = str(cmd).strip()
        if not raw.startswith('>> '):
            return 0

        message = parse_command(raw[3:])
        if message is not None:
            self.messages.put(message)
        return 0

    def process_message(self, message: CECMessage) -> None:
        logger.debug(
            'CEC message src=%X dst=%X opcode=%s args=%s raw=%r',
            message.source,
            message.destination,
            '--' if message.opcode is None else f'{message.opcode:02X}',
            message.parameters,
            message.raw,
        )

        if message.source != cec.CECDEVICE_BROADCAST and message.source not in self.devices:
            self.scan_device(message.source)

        match message.opcode:
            case cec.CEC_OPCODE_ACTIVE_SOURCE:
                self.on_become_active(message.source)
            case cec.CEC_OPCODE_INACTIVE_SOURCE:
                self.on_become_inactive(message.source)
            case cec.CEC_OPCODE_STANDBY:
                self.on_power_status(message.source, cec.CEC_POWER_STATUS_STANDBY)
            case cec.CEC_OPCODE_REPORT_POWER_STATUS if message.parameters:
                self.on_power_status(message.source, message.parameters[0])

    def on_become_active(self, device_id: int | None) -> None:
        if self.active_source_known and self.active_source_id == device_id:
            return

        self.active_source_known = True
        if device_id is None:
            self.active_source_id = None
            logger.info('Active source cleared')
            self.publish(
                f'{MQTT_BASE_TOPIC}/active',
                CECDeviceEvent(event='active-source', device=None),
            )
            return

        self.active_source_id = device_id
        device = self.devices.get(device_id)
        logger.info('Active source is now %s', self.device_label(device_id))
        self.publish(
            f'{MQTT_BASE_TOPIC}/active',
            CECDeviceEvent(event='active-source', device=device),
        )

    def on_become_inactive(self, device_id: int) -> None:
        if device_id != self.active_source_id:
            return  # doesn't match our records
        # TODO: Enqueue a device_id power status check in near future
        self.on_become_active(None)

    def on_power_status(self, device_id: int, power_status: int) -> None:
        match power_status:
            case cec.CEC_POWER_STATUS_ON:
                power_status_name = 'on'
            case cec.CEC_POWER_STATUS_STANDBY:
                power_status_name = 'standby'
            case _:
                return

        device = self.devices.get(device_id)
        if device and device.power_status != power_status_name:
            device.power_status = power_status_name
            logger.info('Power status for %s -> %s', self.device_label(device_id), power_status_name)
            self.publish_device_event(device, 'power')

        if power_status == cec.CEC_POWER_STATUS_STANDBY and device_id == self.active_source_id:
            self.on_become_active(None)

    def close(self) -> None:
        if self.lib is not None:
            self.lib.Close()

    def execute_command(self, command: CECCommand) -> None:
        match command.action:
            case 'standby':
                self.standby(command.device_id)
            case _:
                logger.warning('Ignoring unknown CEC command action %r', command.action)

    def scan_devices(self) -> None:
        self.devices.clear()
        for logical_address in range(LOGICAL_ADDRESS_COUNT):
            self.scan_device(logical_address)

        active = self.lib.GetActiveSource()
        if active == -1:
            active = None
        self.on_become_active(active)

    def scan_device(self, logical_address: int) -> CECDevice | None:
        if logical_address >= LOGICAL_ADDRESS_COUNT:
            return None
        addresses = self.lib.GetActiveDevices()
        if not addresses.IsSet(logical_address):
            return None

        pa = self.lib.GetDevicePhysicalAddress(logical_address)
        vendor_id = self.lib.GetDeviceVendorId(logical_address)
        osd_name = self.lib.GetDeviceOSDName(logical_address)
        if osd_name == 'TVAUX':
            vendor_id = cec.CEC_VENDOR_UNKNOWN
        physical_nice = f'{(pa >> 12) & 0xF}.{(pa >> 8) & 0xF}.{(pa >> 4) & 0xF}.{pa & 0xF}'

        device = CECDevice(
            logical_address=logical_address,
            kind=logical_address_kind(logical_address),
            physical_address=physical_nice,
            vendor=None if vendor_id == cec.CEC_VENDOR_UNKNOWN else self.lib.VendorIdToString(vendor_id),
            osd_name=osd_name,
            cec_version=self.lib.CecVersionToString(self.lib.GetDeviceCecVersion(logical_address)),
            power_status=self.lib.PowerStatusToString(self.lib.GetDevicePowerStatus(logical_address)),
        )
        self.devices[logical_address] = device
        logger.info('Discovered device %s', device)
        self.publish_device_event(device, 'scan')
        return device

    def device_label(self, device_id: int) -> str:
        device = self.devices.get(device_id)
        if device is None:
            return f'{device_id}'
        return f'{device_id} ({device.osd_name})'

    def process_pending_messages(self) -> None:
        try:
            while True:
                message = self.messages.get_nowait()
                self.process_message(message)
        except Empty:
            pass

    def publish_device_event(self, device: CECDevice, event: str) -> None:
        self.publish(
            f'{MQTT_BASE_TOPIC}/device/{device.logical_address}',
            CECDeviceEvent(event=event, device=device),
        )

    def standby(self, device_id: int | None) -> None:
        if device_id is None:
            device_id = cec.CECDEVICE_BROADCAST

        if device_id < 0 or device_id > cec.CECDEVICE_BROADCAST:
            logger.warning('Ignoring standby for invalid device_id %r', device_id)
            return

        if not self.lib.StandbyDevices(device_id):
            logger.error('Standby command failed for %r', device_id)


class CECBridge:
    def __init__(self, mqtt_host: str) -> None:
        self.mqtt_host = mqtt_host
        self.mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt.on_connect = self.on_connect
        self.mqtt.on_message = self.on_message
        self.cec = CECClient(publish=self.publish_json)
        self.commands: SimpleQueue[CECCommand] = SimpleQueue()
        self.json_encoder = msgspec.json.Encoder()
        self.command_decoder = msgspec.json.Decoder(CECCommand)
        self.running = False

    def init(self) -> None:
        self.mqtt.connect(self.mqtt_host, MQTT_PORT, keepalive=MQTT_KEEPALIVE)
        self.cec.init()

    def publish_json(self, topic: str, payload: CECDeviceEvent) -> None:
        self.mqtt.publish(topic, self.json_encoder.encode(payload), qos=1, retain=True)

    def on_connect(self, client, userdata, flags, reason_code, properties=None) -> None:
        client.subscribe(MQTT_COMMAND_TOPIC, qos=1)
        logger.info('Subscribed to %s', MQTT_COMMAND_TOPIC)

    def on_message(self, client, userdata, msg) -> None:
        try:
            command = self.command_decoder.decode(msg.payload)
        except msgspec.MsgspecError as error:
            logger.warning('Ignoring invalid command payload on %s: %s', msg.topic, error)
            return

        self.commands.put(command)

    def stop(self, _sig=None, _frame=None) -> None:
        self.running = False

    def close(self) -> None:
        self.cec.close()
        self.mqtt.loop_stop()
        self.mqtt.disconnect()

    def run(self) -> None:
        self.mqtt.loop_start()
        self.cec.scan_devices()
        logger.info('Observing CEC traffic')

        self.running = True
        while self.running:
            self.process_pending_commands()
            self.cec.process_pending_messages()
            time.sleep(IDLE_SLEEP)

    def process_pending_commands(self) -> None:
        try:
            while True:
                command = self.commands.get_nowait()
                self.cec.execute_command(command)
        except Empty:
            pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--debug', action='store_true', help='Enable verbose CEC message logging')
    parser.add_argument('--mqtt-host', default=os.getenv('MQTT_HOST'))
    return parser.parse_args()


def configure_logging(debug: bool) -> None:
    logging.basicConfig(
        level=logging.DEBUG if debug else logging.INFO,
        format='%(asctime)s %(levelname)s %(message)s',
    )


def main() -> int:
    args = parse_args()
    configure_logging(args.debug)
    if not args.mqtt_host:
        raise SystemExit('--mqtt-host or MQTT_HOST is required')

    bridge = CECBridge(mqtt_host=args.mqtt_host)
    try:
        signal.signal(signal.SIGINT, bridge.stop)
        signal.signal(signal.SIGTERM, bridge.stop)
        bridge.init()
        bridge.run()
    finally:
        bridge.close()
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
