#!/usr/bin/env python3
"""Fail closed unless the local autopilot heartbeat reports disarmed."""

import json
from urllib.request import urlopen


URL = (
    'http://127.0.0.1:6040/v1/mavlink/vehicles/1/components/1/'
    'messages/HEARTBEAT'
)


def mask_bits(value):
    if isinstance(value, dict):
        return int(value.get('bits', 0))
    return int(value)


def main():
    try:
        with urlopen(URL, timeout=2.0) as response:
            payload = json.load(response)
        base_mode = mask_bits(payload['message']['base_mode'])
    except Exception as exc:
        raise SystemExit(
            'DETENIDO: no se pudo confirmar por MAVLink que el ROV esté desarmado: '
            + str(exc)
        ) from exc
    if base_mode & 128:
        raise SystemExit('DETENIDO: el ROV está ARMADO. Desármalo antes de instalar.')
    print('ARMADO=NO')


if __name__ == '__main__':
    main()
