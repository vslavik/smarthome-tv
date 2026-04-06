#!/usr/bin/env python3

from __future__ import annotations

from queue import Empty, SimpleQueue
import signal
import time

import cec
import msgspec

IDLE_SLEEP = 0.2


class CECMessage(msgspec.Struct, frozen=True):
    """A message on the CEC bus"""
    raw: str
    source: int
    destination: int
    opcode: int | None = None
    parameters: tuple[int, ...] = ()


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
        self.config: cec.libcec_configuration | None = None
        self.command_callback = self.handle_command
        self.lib: cec.ICECAdapter | None = None
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
        opcode = '--' if message.opcode is None else f'{message.opcode:02x}'
        print(
            f'dbg src={message.source:x} dst={message.destination:x} opcode={opcode} '
            f'args={message.parameters} raw="{message.raw}"',
            flush=True,
        )

    def close(self) -> None:
        if self.lib is not None:
            self.lib.Close()

    def stop(self, _sig=None, _frame=None) -> None:
        self.running = False
        self.close()

    def run(self) -> None:
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
