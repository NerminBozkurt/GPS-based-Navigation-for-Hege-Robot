"""
Tests for the dry-run node's parameter loading.

Only load_bridge_params is tested here, and deliberately so: the rest of the
node is subscription plumbing around ackermann.limit_command(), which
test_ackermann.py already covers. What is worth pinning down is that the dry
run reads its limits out of the same file the bridge does - the whole point of
the node is that the numbers it shows are the numbers the bridge will use, and
that guarantee is one file read wide.

rclpy is imported by the module under test, so these skip rather than fail when
pytest runs without a sourced ROS environment.
"""

import os

import pytest

rclpy = pytest.importorskip("rclpy")

from hege_px4_bridge.dry_run_node import load_bridge_params  # noqa: E402

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config')


def test_loads_the_real_vehicle_limits():
    params = load_bridge_params(os.path.join(CONFIG, 'bridge_real.yaml'))
    assert params['wheel_base'] == 1.90
    assert params['max_speed'] == 0.3
    assert params['cmd_vel_topic'] == '/cmd_vel/selected'


def test_every_limit_the_node_needs_is_present_in_both_configs():
    needed = ('wheel_base', 'max_steering_angle', 'max_speed',
              'max_yaw_rate', 'min_moving_speed')
    for name in ('bridge_real.yaml', 'bridge_hege_sitl.yaml'):
        params = load_bridge_params(os.path.join(CONFIG, name))
        missing = [key for key in needed if key not in params]
        assert not missing, f"{name} is missing {missing}"


def test_a_file_without_a_parameter_block_is_rejected(tmp_path):
    bad = tmp_path / 'not_params.yaml'
    bad.write_text('wheel_base: 1.90\n')
    with pytest.raises(ValueError):
        load_bridge_params(str(bad))
