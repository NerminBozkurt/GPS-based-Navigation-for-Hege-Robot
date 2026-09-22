"""
Safety supervisor of the HEGE PX4 bridge.  Pure Python, no ROS imports.

The supervisor answers one question every control cycle:

        "May the bridge send a *moving* velocity setpoint to PX4 right now,
         and should the Offboard heartbeat continue?"

For an ordinary command timeout the bridge keeps Offboard alive and sends zero
velocity.  For a latched software stop or a stale PX4 feedback link it sends
zero and drops the heartbeat, allowing PX4's independently configured
COM_OF_LOSS_T / COM_OBL_RC_ACT failsafe to activate.

PX4 constants used (px4_msgs/VehicleStatus, PX4 v1.16.1):
    NAVIGATION_STATE_OFFBOARD = 14
    ARMING_STATE_ARMED        = 2
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

NAVIGATION_STATE_OFFBOARD = 14
ARMING_STATE_ARMED = 2


class BridgeState(str, Enum):
    SOFTWARE_STOP = "SOFTWARE_STOP"  # ROS stop request; not a hardware emergency stop
    WAITING_PX4 = "WAITING_PX4"     # no VehicleStatus received yet
    PX4_STALE = "PX4_STALE"         # VehicleStatus or attitude too old (link problem)
    NOT_OFFBOARD = "NOT_OFFBOARD"   # PX4 not in Offboard mode or not armed
    CMD_TIMEOUT = "CMD_TIMEOUT"     # PX4 ready but no fresh /cmd_vel
    ACTIVE = "ACTIVE"               # everything fresh -> motion allowed


@dataclass(frozen=True)
class SafetyInputs:
    now: float                     # current time [s]
    estop: bool                    # latched ROS software-stop flag
    status_time: float | None      # time of last VehicleStatus  [s] (None = never)
    attitude_time: float | None    # time of last VehicleAttitude [s]
    cmd_time: float | None         # time of last /cmd_vel        [s]
    nav_state: int | None          # VehicleStatus.nav_state
    arming_state: int | None       # VehicleStatus.arming_state


@dataclass(frozen=True)
class SafetyConfig:
    cmd_timeout: float = 0.5        # [s] max age of /cmd_vel before we stop
    status_timeout: float = 2.0     # [s] VehicleStatus is normally only 2 Hz
    attitude_timeout: float = 0.5   # [s] attitude should arrive much faster

    def validate(self) -> None:
        for name in ("cmd_timeout", "status_timeout", "attitude_timeout"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")


@dataclass(frozen=True)
class Decision:
    state: BridgeState
    allow_motion: bool
    stream_heartbeat: bool


def _age(now: float, t: float | None) -> float:
    if t is None or now < t:
        return float("inf")
    return now - t


def evaluate(inp: SafetyInputs, cfg: SafetyConfig) -> Decision:
    """
    Decide the bridge state.  Checks are ordered from most to least severe,
    so the reported state is always the *first* failing condition.
    """
    if inp.estop:
        return Decision(BridgeState.SOFTWARE_STOP, False, False)

    if inp.status_time is None or inp.attitude_time is None:
        return Decision(BridgeState.WAITING_PX4, False, True)

    if (_age(inp.now, inp.status_time) > cfg.status_timeout
            or _age(inp.now, inp.attitude_time) > cfg.attitude_timeout):
        return Decision(BridgeState.PX4_STALE, False, False)

    if inp.nav_state != NAVIGATION_STATE_OFFBOARD or inp.arming_state != ARMING_STATE_ARMED:
        return Decision(BridgeState.NOT_OFFBOARD, False, True)

    if _age(inp.now, inp.cmd_time) > cfg.cmd_timeout:
        return Decision(BridgeState.CMD_TIMEOUT, False, True)

    return Decision(BridgeState.ACTIVE, True, True)
