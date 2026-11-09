#!/usr/bin/env python3
"""Hardware-free checks for the BlueOS V7 bundle."""

import json
from pathlib import Path
import tempfile
import unittest

from compat import install_ros_shims, String


install_ros_shims()

from install_hook import BEGIN, END, install, uninstall  # noqa: E402
from main import JsonMavlinkMessage, mav_string, RestMav  # noqa: E402
from uuv_agent_core.guided_bridge import GuidedBridge  # noqa: E402
from uuv_agent_core.sequence_core import Sequence  # noqa: E402
from uuv_agent_core.sequence_executor_node import EMBEDDED_UI_GRACE_S  # noqa: E402


class ProtocolTests(unittest.TestCase):
    def test_integer_character_array_decodes(self):
        self.assertEqual(mav_string([70, 83, 95, 80, 0]), 'FS_P')

    def test_param_id_facade_decodes(self):
        message = JsonMavlinkMessage(
            {'message': {'type': 'PARAM_VALUE', 'param_id': [65, 66, 0]}}
        )
        self.assertEqual(message.param_id, 'AB')

    def test_only_required_command_long_messages_are_allowed(self):
        transport = RestMav()
        transport.templates['COMMAND_LONG'] = {
            'header': {},
            'message': {'type': 'COMMAND_LONG', 'command': {'type': ''}},
        }
        transport.command_long_send(1, 1, 511, 0, 32, 50_000, 0, 0, 0, 0, 0)
        packet = transport.outbox.get_nowait()
        self.assertEqual(packet['message']['command']['type'], 'MAV_CMD_SET_MESSAGE_INTERVAL')
        with self.assertRaises(ValueError):
            transport.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)

    def test_realtime_queue_keeps_only_fresh_packets(self):
        transport = RestMav()
        transport.templates['MANUAL_CONTROL'] = {
            'header': {},
            'message': {'type': 'MANUAL_CONTROL'},
        }
        for value in range(5):
            transport.manual_control_send(1, value, 0, 500, 0, 0)
        self.assertEqual(transport.stream_outbox.qsize(), 2)
        first = transport.stream_outbox.get_nowait()
        second = transport.stream_outbox.get_nowait()
        self.assertEqual(first['message']['x'], 3)
        self.assertEqual(second['message']['x'], 4)


class LinkPolicyTests(unittest.TestCase):
    def test_ui_grace_is_exactly_one_second(self):
        self.assertEqual(EMBEDDED_UI_GRACE_S, 1.0)

    def test_core_hold_freezes_step_timeout(self):
        core = Sequence()
        core.state = 'RUNNING'
        core.started = 10.0
        core.last_time = 12.0
        core.hold(15.5)
        self.assertEqual(core.started, 13.5)
        self.assertEqual(core.last_time, 15.5)

    def test_pause_and_resume_keep_owner(self):
        bridge = GuidedBridge.__new__(GuidedBridge)
        bridge.owner = 'run-1'
        bridge.paused = False
        bridge.pause_started = None
        bridge.pause_reason = None
        bridge.last_reason = ''
        bridge.clock = lambda: 20.0
        bridge.ticket_ok = lambda _packet: True
        bridge.capture_hold_target = lambda: True

        message = String()
        message.data = json.dumps({'action': 'pause'})
        bridge.action(message)
        self.assertTrue(bridge.paused)
        self.assertEqual(bridge.owner, 'run-1')

        message.data = json.dumps({'action': 'resume'})
        bridge.action(message)
        self.assertFalse(bridge.paused)
        self.assertEqual(bridge.owner, 'run-1')


class StartupHookTests(unittest.TestCase):
    def test_hook_is_idempotent_and_reversible(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'start-blueos-core'
            original = '#!/bin/bash\nSERVICES=(\n\'wifi\',0,0,0,0,"wifi"\n)\n'
            path.write_text(original, encoding='utf-8')
            install(path)
            install(path)
            installed = path.read_text(encoding='utf-8')
            self.assertEqual(installed.count(BEGIN), 1)
            self.assertEqual(installed.count(END), 1)
            uninstall(path)
            self.assertEqual(path.read_text(encoding='utf-8'), original)


if __name__ == '__main__':
    unittest.main(verbosity=2)
