#!/usr/bin/env python3
"""Train a gate YOLO-OBB model only after explicit environment checks."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path
import platform
import sys


def environment_status() -> dict[str, object]:
    """Inspect optional training dependencies without importing them eagerly."""
    status: dict[str, object] = {
        'python': platform.python_version(),
        'ultralytics_installed': importlib.util.find_spec('ultralytics') is not None,
        'torch_installed': importlib.util.find_spec('torch') is not None,
        'cuda_available': False,
        'cuda_device_count': 0,
    }
    if status['torch_installed']:
        import torch

        status['torch_version'] = torch.__version__
        status['cuda_available'] = bool(torch.cuda.is_available())
        status['cuda_device_count'] = int(torch.cuda.device_count())
        if torch.cuda.is_available():
            status['cuda_devices'] = [
                torch.cuda.get_device_name(index)
                for index in range(torch.cuda.device_count())
            ]
    return status


def build_parser() -> argparse.ArgumentParser:
    """Create the training CLI."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', required=True, type=Path)
    parser.add_argument('--model', default='yolo11n-obb.pt')
    parser.add_argument('--epochs', type=int, default=100)
    parser.add_argument('--imgsz', type=int, default=960)
    parser.add_argument('--batch', type=int, default=8)
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--patience', type=int, default=20)
    parser.add_argument('--seed', type=int, default=2026)
    parser.add_argument('--project', type=Path, default=Path('runs/gate_obb'))
    parser.add_argument('--name', default='baseline')
    parser.add_argument('--device', default='0')
    parser.add_argument('--allow-cpu', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    return parser


def _positive(name: str, value: int) -> None:
    if value <= 0:
        raise ValueError(f'{name} must be positive')


def main(argv=None) -> int:
    """Run an auditable OBB training job or fail with an actionable reason."""
    arguments = build_parser().parse_args(argv)
    for name in ('epochs', 'imgsz', 'batch', 'workers', 'patience'):
        value = getattr(arguments, name)
        if name == 'workers' and value == 0:
            continue
        _positive(name, value)
    if not arguments.data.is_file():
        print(f'ERROR: dataset config does not exist: {arguments.data}', file=sys.stderr)
        return 2

    status = environment_status()
    plan = {
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'environment': status,
        'data': str(arguments.data.resolve()),
        'model': arguments.model,
        'epochs': arguments.epochs,
        'imgsz': arguments.imgsz,
        'batch': arguments.batch,
        'workers': arguments.workers,
        'patience': arguments.patience,
        'seed': arguments.seed,
        'device': arguments.device,
        'project': str(arguments.project.resolve()),
        'name': arguments.name,
    }
    print(json.dumps(plan, indent=2, sort_keys=True))
    if arguments.dry_run:
        return 0
    if not status['ultralytics_installed'] or not status['torch_installed']:
        print(
            'ERROR: training dependencies are absent. Install a project-pinned '
            'PyTorch/CUDA and Ultralytics environment before training.',
            file=sys.stderr,
        )
        return 2
    if not status['cuda_available'] and not arguments.allow_cpu:
        print(
            'ERROR: CUDA is unavailable. Refusing slow accidental CPU training; '
            'use --allow-cpu only for an intentional smoke test.',
            file=sys.stderr,
        )
        return 2

    arguments.project.mkdir(parents=True, exist_ok=True)
    plan_path = arguments.project / f'{arguments.name}_training_plan.json'
    plan_path.write_text(json.dumps(plan, indent=2) + '\n', encoding='utf-8')

    from ultralytics import YOLO

    device = arguments.device if status['cuda_available'] else 'cpu'
    model = YOLO(arguments.model)
    model.train(
        data=str(arguments.data.resolve()),
        epochs=arguments.epochs,
        imgsz=arguments.imgsz,
        batch=arguments.batch,
        workers=arguments.workers,
        patience=arguments.patience,
        seed=arguments.seed,
        device=device,
        project=str(arguments.project),
        name=arguments.name,
        task='obb',
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
