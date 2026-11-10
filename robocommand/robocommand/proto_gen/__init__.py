"""Generated Protocol Buffers modules from official RoboNation RobotX 2026 schema."""

import os
import site
import sys

# Ensure user site-packages (containing protobuf>=7.35) precedes /usr/lib/python3/dist-packages
_USER_SITE = site.getusersitepackages()
if _USER_SITE and os.path.isdir(_USER_SITE):
    if _USER_SITE in sys.path:
        sys.path.remove(_USER_SITE)
    sys.path.insert(0, _USER_SITE)

_PROTO_GEN_DIR = os.path.dirname(os.path.abspath(__file__))
if _PROTO_GEN_DIR not in sys.path:
    sys.path.insert(0, _PROTO_GEN_DIR)

import common_pb2  # noqa: E402
from robotx import (  # noqa: E402
    rx_commands_pb2,
    rx_common_pb2,
    rx_course_pb2,
    rx_reports_pb2,
    rx_requests_pb2,
)

__all__ = [
    "common_pb2",
    "rx_commands_pb2",
    "rx_common_pb2",
    "rx_course_pb2",
    "rx_reports_pb2",
    "rx_requests_pb2",
]
