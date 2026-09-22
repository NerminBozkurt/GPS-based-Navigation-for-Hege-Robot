"""
Pure mathematics of the HEGE PX4 bridge.  No ROS imports here, so every
function can be unit-tested with plain pytest.

Conventions
-----------
ROS  (REP-103, REP-105):  body frame FLU (x forward, y left, z up),
                          world frame ENU, yaw positive = counter-clockwise.
PX4:                      body frame FRD (x forward, y right, z down),
                          world frame NED, yaw positive = clockwise (about Down).

The two yaw conventions are opposite, therefore:  omega_ned = -omega_ros.

Notation (matches the PX4 rover documentation):
    v          forward speed                        [m/s]
    omega      yaw rate  (psi_dot)                  [rad/s]
    delta      front-wheel steering angle           [rad]
    L          wheel base  (PX4 parameter RA_WHEEL_BASE)          [m]
    delta_max  maximum steering angle (PX4 parameter RA_MAX_STR_ANG) [rad]
    psi        vehicle yaw in NED                   [rad]
    K_psi      PX4 yaw P-gain (PX4 parameter RO_YAW_P)  [1/s]
"""

from __future__ import annotations

import math
from dataclasses import dataclass


# ----------------------------------------------------------------------------
# Small helpers
# ----------------------------------------------------------------------------

def wrap_pi(angle: float) -> float:
    """Wrap an angle to the interval [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, low: float, high: float) -> float:
    """Saturate value to [low, high]."""
    return max(low, min(high, value))


# ----------------------------------------------------------------------------
# Ackermann (bicycle model) kinematics
# ----------------------------------------------------------------------------

def steering_angle(v: float, omega: float, wheel_base: float) -> float:
    """
    Bicycle-model steering angle that produces yaw rate `omega` at speed `v`:

        delta = atan( L * omega / v )

    Derivation: a bicycle with wheel base L and front steering angle delta drives
    on a circle of radius R = L / tan(delta).  A vehicle on a circle of radius R
    at speed v has yaw rate omega = v / R  =>  omega = v * tan(delta) / L.
    Solving for delta gives the formula above.

    For v == 0 the steering angle is undefined (no motion); we return 0.
    """
    if abs(v) < 1e-9:
        return 0.0
    return math.atan(wheel_base * omega / v)


def max_yaw_rate_at_speed(v: float, wheel_base: float, max_steering_angle: float) -> float:
    """
    Largest yaw rate the front wheels can physically produce at speed v:

        omega_max(v) = |v| * tan(delta_max) / L

    This is the same expression PX4 uses internally
    (AckermannRateControl: max_possible_yaw_rate).
    """
    return abs(v) * math.tan(max_steering_angle) / wheel_base


# ----------------------------------------------------------------------------
# Command limiting
# ----------------------------------------------------------------------------

@dataclass(frozen=True)
class AckermannLimits:
    """Physical and safety limits of the rover.  All values must be > 0."""
    wheel_base: float            # L            [m]
    max_steering_angle: float    # delta_max    [rad]
    max_speed: float             # v_max        [m/s]  (keep <= PX4 RO_SPEED_LIM)
    max_yaw_rate: float          # omega_lim    [rad/s] (keep <= PX4 RO_YAW_RATE_LIM)
    min_moving_speed: float      # v_min        [m/s]  below this -> full stop

    def validate(self) -> None:
        for name in ("wheel_base", "max_steering_angle", "max_speed",
                     "max_yaw_rate", "min_moving_speed"):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"AckermannLimits.{name} must be > 0, got {getattr(self, name)}")
        if self.max_steering_angle >= math.pi / 2.0:
            raise ValueError("max_steering_angle must be < pi/2 rad")


@dataclass(frozen=True)
class LimitedCommand:
    v: float          # forward speed after limiting      [m/s]  (always >= 0)
    omega: float      # ROS yaw rate after limiting       [rad/s]
    stopped: bool     # True when the command was turned into a full stop
    reason: str       # human-readable explanation (for logging / status)


def limit_command(v_cmd: float, omega_cmd: float, lim: AckermannLimits) -> LimitedCommand:
    """
    Turn an arbitrary (v, omega) request from Nav2 into a command the rover can
    actually execute, applying the following rules in order:

    1. Non-finite input                     -> stop.
    2. Reverse (v < 0)                      -> stop.  PX4 offboard velocity
       control drives forwards only (AckermannVelControl sets backwards=false),
       and reversing an Ackermann rover under yaw-rate control is
       non-minimum-phase, so it is not allowed here.
    3. |v| below min_moving_speed           -> stop.  This also rejects
       "rotate in place" (v = 0, omega != 0), which a car cannot do.
    4. v  <- min(v, v_max)
    5. omega <- clamp(omega, +-min(omega_max(v), omega_lim))
       where omega_max(v) = v * tan(delta_max) / L is the kinematic limit.
    """
    if not (math.isfinite(v_cmd) and math.isfinite(omega_cmd)):
        return LimitedCommand(0.0, 0.0, True, "non-finite command")

    if v_cmd < 0.0:
        return LimitedCommand(0.0, 0.0, True, "reverse requested (not allowed)")

    if v_cmd < lim.min_moving_speed:
        if abs(omega_cmd) > 1e-6:
            return LimitedCommand(0.0, 0.0, True, "rotate-in-place requested (not possible)")
        return LimitedCommand(0.0, 0.0, True, "speed below min_moving_speed")

    v = min(v_cmd, lim.max_speed)

    omega_bound = min(max_yaw_rate_at_speed(v, lim.wheel_base, lim.max_steering_angle),
                      lim.max_yaw_rate)
    omega = clamp(omega_cmd, -omega_bound, omega_bound)

    reason = "ok"
    if v != v_cmd or omega != omega_cmd:
        reason = "saturated"
    return LimitedCommand(v, omega, False, reason)


# ----------------------------------------------------------------------------
# Frame conversions needed by the bridge
# ----------------------------------------------------------------------------

def ros_yaw_rate_to_ned(omega_ros: float) -> float:
    """
    ROS yaw rate (about +Z up, CCW positive) -> PX4 yaw rate (about +Z down,
    CW positive).  The z axes are anti-parallel, so the sign flips.
    """
    return -omega_ros


def yaw_from_px4_quaternion(q: tuple[float, float, float, float]) -> float:
    """
    Yaw psi (rotation about NED Down axis) from the PX4 attitude quaternion
    q = (w, x, y, z) that rotates FRD-body vectors into NED.

        psi = atan2( 2 (w z + x y),  1 - 2 (y^2 + z^2) )

    This is the ZYX (yaw-pitch-roll) Euler extraction; it is the same formula
    PX4 uses in matrix::Eulerf(q).psi(), so the bridge and PX4 agree on psi.
    """
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# ----------------------------------------------------------------------------
# Mapping (v, omega) -> PX4 offboard velocity setpoint
# ----------------------------------------------------------------------------

def velocity_setpoint_ned(v: float, omega_ned: float, psi: float,
                          px4_yaw_p: float) -> tuple[float, float, float]:
    """
    Build the NED velocity vector for px4_msgs/TrajectorySetpoint.velocity.

    Why this shape?  In PX4 v1.16 offboard *velocity* mode the Ackermann module
    (AckermannVelControl::generateAttitudeAndThrottleSetpoint) does:

        psi_sp   = atan2(v_E, v_N)                 # heading of the vector
        speed_sp = min(|v_NE|, RO_SPEED_LIM)       # its magnitude
        yaw_rate_sp = RO_YAW_P * wrap_pi(psi_sp - psi)   # (AckermannAttControl)

    So PX4 does not accept a yaw rate directly; it accepts a heading.  To make
    PX4's proportional heading controller request the nominal yaw rate, we choose

        psi_sp = psi + omega_ned / RO_YAW_P

    because nominally yaw_rate_sp = RO_YAW_P * (omega_ned / RO_YAW_P) = omega_ned.
    PX4's slew/rate/steering limits and the real vehicle dynamics can reduce the
    achieved response, so this is a setpoint mapping rather than an exact plant result.
    (PX4's own manual Position mode uses the identical construction:
     yaw_delta = stick * max_yaw_rate / RO_YAW_P.)

    Finally  v_N = v cos(psi_sp),  v_E = v sin(psi_sp),  v_D = 0.

    Inputs:  v >= 0 [m/s], omega_ned [rad/s], psi [rad, NED], px4_yaw_p [1/s].
    Output:  (v_N, v_E, v_D) in m/s, NED.
    """
    if px4_yaw_p <= 0.0:
        raise ValueError("px4_yaw_p (RO_YAW_P) must be > 0")
    psi_sp = wrap_pi(psi + omega_ned / px4_yaw_p)
    return (v * math.cos(psi_sp), v * math.sin(psi_sp), 0.0)


def stop_setpoint_ned() -> tuple[float, float, float]:
    """
    Zero velocity vector.  PX4 reacts to an exactly-zero vector by holding the
    current yaw and regulating speed to zero (AckermannVelControl), i.e. a
    controlled stop with the wheels straight.
    """
    return (0.0, 0.0, 0.0)
