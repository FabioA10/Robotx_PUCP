#!/usr/bin/env python3
"""Install or remove the small BlueOS startup hook idempotently."""

import argparse
from datetime import datetime, timezone
from pathlib import Path
import shutil


BEGIN = '# UUV_SEQUENCE_AGENT_V7_BEGIN'
END = '# UUV_SEQUENCE_AGENT_V7_END'
ENTRY = (
    BEGIN + '\n'
    + "'uuv_sequence_agent',180,50,0,0,"
    + '"python3 /usr/blueos/userdata/uuv_sequence_agent/main.py"\n'
    + END
)


def without_hook(text):
    """Remove one managed marker block without touching other services."""
    if BEGIN not in text and END not in text:
        return text
    if text.count(BEGIN) != 1 or text.count(END) != 1:
        raise ValueError('Marcadores V7 inconsistentes en start-blueos-core')
    before, rest = text.split(BEGIN, 1)
    _managed, after = rest.split(END, 1)
    return before.rstrip() + '\n' + after.lstrip('\n')


def backup(path):
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
    destination = path.with_name(path.name + '.before_uuv_v7_' + stamp)
    shutil.copy2(path, destination)
    return destination


def install(path):
    original = path.read_text(encoding='utf-8')
    clean = without_hook(original)
    marker = 'SERVICES=('
    if marker not in clean:
        raise ValueError('No se encontró SERVICES=( en start-blueos-core')
    updated = clean.replace(marker, marker + '\n' + ENTRY, 1)
    saved = backup(path)
    path.write_text(updated, encoding='utf-8')
    return saved


def uninstall(path):
    original = path.read_text(encoding='utf-8')
    updated = without_hook(original)
    if updated == original:
        return None
    saved = backup(path)
    path.write_text(updated, encoding='utf-8')
    return saved


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=('install', 'uninstall'))
    parser.add_argument('path', type=Path)
    args = parser.parse_args()
    if not args.path.is_file():
        raise SystemExit('No existe: ' + str(args.path))
    try:
        saved = install(args.path) if args.action == 'install' else uninstall(args.path)
    except (OSError, ValueError) as exc:
        raise SystemExit(str(exc)) from exc
    print('BACKUP=' + (str(saved) if saved else 'NO_NECESARIO'))


if __name__ == '__main__':
    main()
