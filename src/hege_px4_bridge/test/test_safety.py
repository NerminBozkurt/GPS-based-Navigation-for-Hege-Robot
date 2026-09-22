"""Unit tests for the safety supervisor state machine."""

from hege_px4_bridge.safety import (ARMING_STATE_ARMED, NAVIGATION_STATE_OFFBOARD,
                                    BridgeState, SafetyConfig, SafetyInputs, evaluate)

CFG = SafetyConfig(cmd_timeout=0.5, status_timeout=2.0, attitude_timeout=0.5)


def inputs(**overrides):
    """A fully healthy situation at t = 10 s; tests override single fields."""
    base = dict(now=10.0, estop=False, status_time=9.9, attitude_time=9.9, cmd_time=9.8,
                nav_state=NAVIGATION_STATE_OFFBOARD, arming_state=ARMING_STATE_ARMED)
    base.update(overrides)
    return SafetyInputs(**base)


def test_active_when_everything_fresh():
    d = evaluate(inputs(), CFG)
    assert d.state == BridgeState.ACTIVE and d.allow_motion and d.stream_heartbeat


def test_estop_has_highest_priority():
    d = evaluate(inputs(estop=True), CFG)
    assert d.state == BridgeState.SOFTWARE_STOP
    assert not d.allow_motion and not d.stream_heartbeat


def test_waiting_for_px4_when_never_received():
    assert evaluate(inputs(status_time=None), CFG).state == BridgeState.WAITING_PX4
    assert evaluate(inputs(attitude_time=None), CFG).state == BridgeState.WAITING_PX4


def test_px4_stale():
    assert evaluate(inputs(status_time=7.9), CFG).state == BridgeState.PX4_STALE
    assert evaluate(inputs(attitude_time=9.4), CFG).state == BridgeState.PX4_STALE
    assert not evaluate(inputs(attitude_time=9.4), CFG).allow_motion
    assert not evaluate(inputs(attitude_time=9.4), CFG).stream_heartbeat


def test_status_and_attitude_have_independent_deadlines():
    # Status age 1.5 s is valid because VehicleStatus is normally only 2 Hz.
    assert evaluate(inputs(status_time=8.5), CFG).state == BridgeState.ACTIVE
    # The same age is not acceptable for the high-rate attitude stream.
    assert evaluate(inputs(attitude_time=8.5), CFG).state == BridgeState.PX4_STALE


def test_not_offboard_or_not_armed():
    assert evaluate(inputs(nav_state=0), CFG).state == BridgeState.NOT_OFFBOARD      # MANUAL
    assert evaluate(inputs(arming_state=1), CFG).state == BridgeState.NOT_OFFBOARD   # DISARMED


def test_cmd_timeout():
    d = evaluate(inputs(cmd_time=9.4), CFG)          # age 0.6 s > 0.5 s
    assert d.state == BridgeState.CMD_TIMEOUT and not d.allow_motion
    d = evaluate(inputs(cmd_time=None), CFG)         # never received
    assert d.state == BridgeState.CMD_TIMEOUT
    assert d.stream_heartbeat


def test_cmd_exactly_at_timeout_is_still_allowed():
    d = evaluate(inputs(cmd_time=9.5), CFG)          # age == 0.5 s (not > 0.5)
    assert d.allow_motion


def test_priority_order():
    """A stale PX4 link is reported before a missing command."""
    d = evaluate(inputs(status_time=7.0, cmd_time=None), CFG)
    assert d.state == BridgeState.PX4_STALE


def test_future_timestamp_is_treated_as_stale():
    assert evaluate(inputs(status_time=11.0), CFG).state == BridgeState.PX4_STALE


def test_config_validation():
    import pytest
    with pytest.raises(ValueError):
        SafetyConfig(cmd_timeout=0.0, status_timeout=2.0, attitude_timeout=0.5).validate()
    with pytest.raises(ValueError):
        SafetyConfig(cmd_timeout=0.5, status_timeout=-1.0, attitude_timeout=0.5).validate()
    with pytest.raises(ValueError):
        SafetyConfig(cmd_timeout=0.5, status_timeout=2.0, attitude_timeout=float("nan")).validate()
    CFG.validate()
