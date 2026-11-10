"""Validate the source-exclusion contract of the shadow launch file."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node


def _load_launch_description():
    launch_path = (
        Path(__file__).parents[1]
        / 'launch'
        / 'uuv_follow_shadow.launch.py'
    )
    spec = importlib.util.spec_from_file_location('shadow_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_usbl_source_argument_is_explicit_and_restricted():
    """Only the supported mutually exclusive source modes may be selected."""
    description = _load_launch_description()
    arguments = {
        action.name: action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }

    source = arguments['usbl_source']
    assert source.choices == ['fake', 'external']
    assert source.default_value[0].text == 'fake'


def test_follower_and_shadow_have_separate_disabled_gates():
    """Starting candidate calculation cannot implicitly enable the arbiter."""
    description = _load_launch_description()
    arguments = {
        action.name: action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }

    assert arguments['follow_enabled'].default_value[0].text == 'false'
    assert arguments['shadow_enabled'].default_value[0].text == 'false'


def test_fake_usbl_node_is_conditionally_started():
    """The launch must not create a fake source in external-source mode."""
    description = _load_launch_description()
    fake_nodes = [
        action
        for action in description.entities
        if isinstance(action, Node)
        and action.node_executable == 'fake_usbl_node'
    ]

    assert len(fake_nodes) == 1
    assert isinstance(fake_nodes[0].condition, IfCondition)

    fake_context = LaunchContext()
    fake_context.launch_configurations['usbl_source'] = 'fake'
    assert fake_nodes[0].condition.evaluate(fake_context)

    external_context = LaunchContext()
    external_context.launch_configurations['usbl_source'] = 'external'
    assert not fake_nodes[0].condition.evaluate(external_context)
