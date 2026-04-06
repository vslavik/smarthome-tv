#!/usr/bin/env python3

from __future__ import annotations

from queue import Empty, SimpleQueue
import signal
import time

import cec
import msgspec

IDLE_SLEEP = 0.2
LOGICAL_ADDRESS_COUNT = 15
BROADCAST_ADDRESS = 15


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
    logical_name: str
    physical_address: str
    vendor: str
    cec_version: str


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


class CECClient:
    def __init__(self) -> None:
        self.active_source: int | None = None
        self.config: cec.libcec_configuration | None = None
        self.command_callback = self.handle_command
        self.lib: cec.ICECAdapter | None = None
        self.devices: dict[int, CECDevice] = {}
        self.messages: SimpleQueue[CECMessage] = SimpleQueue()
        self.running = False

    def init(self) -> None:
        self.config = cec.libcec_configuration()
        self.config.strDeviceName = 'TVAUX'
        self.config.bActivateSource = 0
        self.config.clientVersion = cec.LIBCEC_VERSION_CURRENT
        self.config.deviceTypes.Add(cec.CEC_DEVICE_TYPE_RECORDING_DEVICE)
        self.config.SetCommandCallback(self.command_callback)
        self.lib = cec.ICECAdapter.Create(self.config)

        adapters = self.lib.DetectAdapters()
        if not adapters:
            raise RuntimeError('No CEC adapters found')

        port = adapters[0].strComName
        if not self.lib.Open(port):
            raise RuntimeError(f'Failed to open {port}')

        print(f'opened {port}', flush=True)

    def handle_command(self, cmd) -> int:
        raw = str(cmd).strip()
        if not raw.startswith('>> '):
            return 0

        message = parse_command(raw[3:])
        if message is not None:
            self.messages.put(message)
        return 0

    def process_message(self, message: CECMessage) -> None:
        if message.source != BROADCAST_ADDRESS and message.source not in self.devices:
            self.scan_device(message.source)

        opcode = '--' if message.opcode is None else f'{message.opcode:02x}'
        #print(
        #    f'dbg src={message.source:x} dst={message.destination:x} opcode={opcode} '
        #    f'args={message.parameters} raw="{message.raw}"',
        #    flush=True,
        #)

    def close(self) -> None:
        if self.lib is not None:
            self.lib.Close()

    def stop(self, _sig=None, _frame=None) -> None:
        self.running = False
        self.close()

    def scan_devices(self) -> None:
        self.devices.clear()
        self.active_source = self.lib.GetActiveSource()
        for logical_address in range(LOGICAL_ADDRESS_COUNT):
            self.scan_device(logical_address)

    def scan_device(self, logical_address: int) -> CECDevice | None:
        if logical_address >= LOGICAL_ADDRESS_COUNT:
            return None
        addresses = self.lib.GetActiveDevices()
        if not addresses.IsSet(logical_address):
            return None

        pa = self.lib.GetDevicePhysicalAddress(logical_address)
        physical_nice = f'{(pa >> 12) & 0xF}.{(pa >> 8) & 0xF}.{(pa >> 4) & 0xF}.{pa & 0xF}'

        device = CECDevice(
            logical_address=logical_address,
            logical_name=self.lib.LogicalAddressToString(logical_address),
            physical_address=physical_nice,
            vendor=self.lib.VendorIdToString(self.lib.GetDeviceVendorId(logical_address)),
            osd_name=self.lib.GetDeviceOSDName(logical_address),
            cec_version=self.lib.CecVersionToString(self.lib.GetDeviceCecVersion(logical_address)),
            power_status=self.lib.PowerStatusToString(self.lib.GetDevicePowerStatus(logical_address)),
        )
        print(f'device: {device}')
        self.devices[logical_address] = device
        return device

    def run(self) -> None:
        self.scan_devices()
        print('observing...', flush=True)

        self.running = True
        while self.running:
            try:
                while True:
                    message = self.messages.get_nowait()
                    self.process_message(message)
            except Empty:
                pass

            time.sleep(IDLE_SLEEP)


def main() -> int:
    client = CECClient()
    try:
        signal.signal(signal.SIGINT, client.stop)
        signal.signal(signal.SIGTERM, client.stop)
        client.init()
        client.run()
    finally:
        client.close()
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
