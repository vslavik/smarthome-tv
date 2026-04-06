#!/usr/bin/env python3

from __future__ import annotations

import re
import signal
import sys
import time

import cec


# Adjust these.
TV = cec.CECDEVICE_TV

WATCHED = {
    cec.CECDEVICE_PLAYBACKDEVICE2,  # 0x8 = PS5
    cec.CECDEVICE_PLAYBACKDEVICE3,  # 0xB = SHIELD
}

DEVICE_NAMES = {
    cec.CECDEVICE_TV: 'tv',
    cec.CECDEVICE_PLAYBACKDEVICE2: 'ps5',
    cec.CECDEVICE_PLAYBACKDEVICE3: 'shield',
}

# In-memory state only.
is_on = {addr: False for addr in WATCHED}

# Optional: avoid repeated standby spam if multiple events arrive.
last_tv_off_at = 0.0
TV_OFF_DEBOUNCE_SEC = 2.0

lib = None


def all_watched_off() -> bool:
    return all(not is_on[addr] for addr in WATCHED)


def maybe_turn_off_tv() -> None:
    global last_tv_off_at

    if not all_watched_off():
        return

    now = time.monotonic()
    if now - last_tv_off_at < TV_OFF_DEBOUNCE_SEC:
        return

    print('all watched devices off -> TV standby', flush=True)
    lib.StandbyDevices(TV)
    last_tv_off_at = now


def parse_addr(cmd: str) -> int | None:
    # libCEC callback string is typically hex-ish like "4f:36" / "0f:82:10:00".
    m = re.match(r'^([0-9a-fA-F])([0-9a-fA-F]):', cmd)
    if not m:
        return None
    src = int(m.group(1), 16)
    return src if src in WATCHED else None


def parse_command(cmd):
    s = str(cmd).strip()

    # Remove libCEC direction prefix if present.
    if s.startswith('>> '):
        s = s[3:]
    elif s.startswith('<< '):
        s = s[3:]

    parts = s.split(':')
    if len(parts) < 2 or len(parts[0]) != 2:
        return None

    src = int(parts[0][0], 16)
    dst = int(parts[0][1], 16)
    opcode = int(parts[1], 16) if len(parts) >= 2 else None
    args = [int(x, 16) for x in parts[2:]]

    return src, dst, opcode, args, s


def handle_command(cmd) -> int:
    parsed = parse_command(cmd)
    if parsed is None:
        return 0

    src, dst, opcode, args, raw = parsed
    print(f'dbg src={src:x} dst={dst:x} opcode={opcode:02x} args={args} raw="{raw}"', flush=True)

    if src not in WATCHED:
        return 0

    changed = False

    if opcode == 0x36:  # Standby
        if is_on[src]:
            is_on[src] = False
            changed = True

    elif opcode in {0x82, 0x04, 0x0D}:  # Active Source / Image View On / Text View On
        if not is_on[src]:
            is_on[src] = True
            changed = True

    if changed:
        print(f'{DEVICE_NAMES.get(src, src)} -> {"on" if is_on[src] else "off"} ({raw})', flush=True)
        maybe_turn_off_tv()

    return 0


def open_adapter():
    adapters = lib.DetectAdapters()
    if not adapters:
        raise RuntimeError('No CEC adapters found')

    port = adapters[0].strComName
    if not lib.Open(port):
        raise RuntimeError(f'Failed to open {port}')

    print(f'opened {port}', flush=True)


def main() -> int:
    global lib

    config = cec.libcec_configuration()
    config.strDeviceName = 'TVAUX'
    config.bActivateSource = 0
    config.clientVersion = cec.LIBCEC_VERSION_CURRENT
    config.deviceTypes.Add(cec.CEC_DEVICE_TYPE_RECORDING_DEVICE)
    config.SetCommandCallback(handle_command)

    lib = cec.ICECAdapter.Create(config)
    open_adapter()

    print('observing...', flush=True)

    def stop(_sig, _frame):
        try:
            lib.Close()
        finally:
            sys.exit(0)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    while True:
        time.sleep(3600)


if __name__ == '__main__':
    raise SystemExit(main())