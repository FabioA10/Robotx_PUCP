#!/usr/bin/env python3

import socket

BLUEOS_IP = "192.168.2.3"
BLUEOS_PORT = 15000

SYSTEM_INFO_COMMAND = b"#0281C1\r\n"


def main():

    sock = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM,
    )

    sock.settimeout(3.0)
    sock.connect((BLUEOS_IP, BLUEOS_PORT))

    print("=== SeaTrac X150 SYSTEM INFO ===")
    print(f"BlueOS: {BLUEOS_IP}:{BLUEOS_PORT}")
    print(f"TX: {SYSTEM_INFO_COMMAND!r}")

    sock.send(SYSTEM_INFO_COMMAND)

    try:

        data = sock.recv(4096)

        print(f"RX bytes: {len(data)}")
        print(f"RX raw: {data!r}")

        text = data.decode(
            "ascii",
            errors="replace",
        ).strip()

        print(f"RX ASCII: {text}")

        if text.startswith("$02"):
            print("SYSTEM_INFO: OK")
        else:
            print("SYSTEM_INFO: respuesta inesperada")

    except socket.timeout:

        print("TIMEOUT: SeaTrac no respondió")

    finally:

        sock.close()


if __name__ == "__main__":
    main()
