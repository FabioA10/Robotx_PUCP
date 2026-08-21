#!/usr/bin/env python3

import socket
import sys
from pathlib import Path


REPO_ROOT = (
    Path(__file__)
    .resolve()
    .parents[3]
)

DRIVER_PATH = (
    REPO_ROOT
    / "SYSTEM_ws"
    / "src"
    / "usbl"
    / "seatrac_ros2"
)

sys.path.insert(
    0,
    str(DRIVER_PATH),
)

from seatrac_ros2.seatrac_status_node import decode_status_frame


BLUEOS_IP = "192.168.2.3"
BLUEOS_PORT = 15000

STATUS_COMMAND = b"#10000DC0\r\n"


def main():

    sock = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM,
    )

    sock.settimeout(3.0)
    sock.connect((BLUEOS_IP, BLUEOS_PORT))

    print("=== SeaTrac X150 STATUS ===")
    print(f"BlueOS: {BLUEOS_IP}:{BLUEOS_PORT}")
    print(f"TX: {STATUS_COMMAND!r}")

    sock.send(STATUS_COMMAND)

    try:

        data = sock.recv(4096)

        text = data.decode(
            "ascii",
            errors="replace",
        ).strip()

        print(f"RX bytes: {len(data)}")
        print(f"RX ASCII: {text}")

        if not text.startswith("$10"):
            print("Respuesta inesperada.")
            return

        status = decode_status_frame(text)

        print()
        print("=== STATUS DECODED ===")

        for key, value in status.items():
            print(f"{key}: {value}")

    except socket.timeout:

        print("TIMEOUT: SeaTrac no respondió")

    finally:

        sock.close()


if __name__ == "__main__":
    main()
