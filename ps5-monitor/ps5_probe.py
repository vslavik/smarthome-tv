#!/usr/bin/env python3

import argparse
import json
import socket
import re


MSG = b"SRCH * HTTP/1.1\ndevice-discovery-protocol-version:00030010\n"


def parse_reply(data):
    """Parse a PS5 DDP probe reply.

    Observed raw reply from a PS5 in standby:

    HTTP/1.1 620 Server Standby
    host-id:D4F7D51D9779
    host-type:PS5
    host-name:PS5
    host-request-port:997
    device-discovery-protocol-version:00030010
    system-version:13000040
    """
    text = data.decode("utf-8", "replace").replace("\r\n", "\n")
    lines = [line for line in text.split("\n") if line]
    if not lines:
        raise ValueError("empty reply")

    match = re.fullmatch(r"HTTP/1\.1 (\d{3}) (.+)", lines[0])
    if not match:
        raise ValueError(f"bad status line: {lines[0]!r}")

    code = int(match.group(1))
    match code:
        case 200:
            state = "on"
        case 620:
            state = "standby"
        case _:
            state = "unknown"

    info = {
        "state": state,
        "code": code,
        "status-string": match.group(2),
    }

    for line in lines[1:]:
        if ":" not in line:
            raise ValueError(f"bad header line: {line!r}")
        key, value = line.split(":", 1)
        info[key] = value

    return info


def probe(host, port=9302, timeout=1.0, tries=3):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        for _ in range(tries):
            sock.sendto(MSG, (host, port))
            try:
                data, _ = sock.recvfrom(4096)
            except socket.timeout:
                continue
            return parse_reply(data)
    finally:
        sock.close()
    return {"state": "off", "code": None}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-H", "--remote-host", required=True)
    ap.add_argument("-R", "--remote-port", type=int, default=9302)
    ap.add_argument("-t", "--timeout", type=float, default=1.0)
    ap.add_argument("-n", "--tries", type=int, default=3)
    ap.add_argument("-j", "--json", action="store_true")
    ap.add_argument("-P", "--probe", action="store_true")
    args = ap.parse_args()

    result = probe(args.remote_host, args.remote_port, args.timeout, args.tries)

    if args.json:
        print(json.dumps(result))
        return

    line = result["state"]
    if "host-name" in result:
        line += f" ({result['host-name']})"
    if "host-id" in result:
        line += f" [{result['host-id']}]"
    print(line)


if __name__ == "__main__":
    main()
