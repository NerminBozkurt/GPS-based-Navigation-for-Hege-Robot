"""Unit tests for the pure math in hege_px4_bridge.ackermann (run with pytest)."""

import math

import pytest

from hege_px4_bridge import ackermann as ak

# SITL rover_ackermann values (PX4 airframe 4012)
L = 0.321
DELTA_MAX = 0.5236
LIM = ak.AckermannLimits(wheel_base=L, max_steering_angle=DELTA_MAX,
                         max_speed=1.0, max_yaw_rate=1.0, min_moving_speed=0.05)


# --------------------------------------------------------------------- helpers
def test_wrap_pi():
    assert math.isclose(ak.wrap_pi(0.0), 0.0)
    assert math.isclose(ak.wrap_pi(math.pi + 0.1), -math.pi + 0.1)
    assert math.isclose(ak.wrap_pi(-math.pi - 0.1), math.pi - 0.1)
    assert math.isclose(ak.wrap_pi(3 * math.pi), -math.pi)   # -pi is in range, +pi is not


# --------------------------------------------------------------------- kinematics
def test_steering_angle_round_trip():
    """delta = atan(L w / v)  <=>  w = v tan(delta) / L."""
    v, delta = 0.8, 0.25
    omega = v * math.tan(delta) / L
    assert math.isclose(ak.steering_angle(v, omega, L), delta, rel_tol=1e-12)


def test_steering_angle_zero_speed():
    assert ak.steering_angle(0.0, 1.0, L) == 0.0


def test_max_yaw_rate_at_speed_matches_px4_formula():
    v = 0.5
    assert math.isclose(ak.max_yaw_rate_at_speed(v, L, DELTA_MAX), v * math.tan(DELTA_MAX) / L)
    # reaching omega_max means steering exactly at delta_max
    omega_max = ak.max_yaw_rate_at_speed(v, L, DELTA_MAX)
    assert math.isclose(ak.steering_angle(v, omega_max, L), DELTA_MAX, rel_tol=1e-12)


# --------------------------------------------------------------------- limiting
def test_limits_validate():
    with pytest.raises(ValueError):
        ak.AckermannLimits(0.0, DELTA_MAX, 1.0, 1.0, 0.05).validate()
    with pytest.raises(ValueError):
        ak.AckermannLimits(L, math.pi / 2, 1.0, 1.0, 0.05).validate()
    LIM.validate()


def test_limit_ok_passthrough():
    c = ak.limit_command(0.3, 0.2, LIM)
    assert not c.stopped and c.v == 0.3 and c.omega == 0.2 and c.reason == "ok"


def test_limit_rejects_reverse_and_rotate_in_place_and_nan():
    assert ak.limit_command(-0.3, 0.0, LIM).stopped
    assert ak.limit_command(0.0, 0.5, LIM).stopped
    assert ak.limit_command(float("nan"), 0.0, LIM).stopped
    assert ak.limit_command(0.3, float("inf"), LIM).stopped
    for c in (ak.limit_command(-0.3, 0.0, LIM), ak.limit_command(0.0, 0.5, LIM)):
        assert c.v == 0.0 and c.omega == 0.0


def test_limit_speed_saturation():
    c = ak.limit_command(5.0, 0.0, LIM)
    assert c.v == LIM.max_speed and c.reason == "saturated"


def test_limit_yaw_rate_kinematic_bound():
    """At low speed the kinematic bound v tan(delta_max)/L is tighter than max_yaw_rate."""
    v = 0.2
    bound = v * math.tan(DELTA_MAX) / L          # = 0.36 rad/s < 1.0
    c = ak.limit_command(v, 2.0, LIM)
    assert math.isclose(c.omega, bound)
    c = ak.limit_command(v, -2.0, LIM)
    assert math.isclose(c.omega, -bound)
    # the resulting steering angle is exactly delta_max, never beyond
    assert math.isclose(abs(ak.steering_angle(v, c.omega, L)), DELTA_MAX, rel_tol=1e-12)


def test_limit_yaw_rate_config_bound():
    """At high speed the configured max_yaw_rate is the tighter bound."""
    v = 1.0
    assert v * math.tan(DELTA_MAX) / L > LIM.max_yaw_rate
    c = ak.limit_command(v, 5.0, LIM)
    assert math.isclose(c.omega, LIM.max_yaw_rate)


# --------------------------------------------------------------------- frames
def test_ros_yaw_rate_sign_flip():
    assert ak.ros_yaw_rate_to_ned(0.7) == -0.7


def q_from_yaw(psi):
    """Quaternion (w,x,y,z) for a pure yaw rotation about z."""
    return (math.cos(psi / 2), 0.0, 0.0, math.sin(psi / 2))


@pytest.mark.parametrize("psi", [0.0, 0.5, -1.2, 3.0, -3.0])
def test_yaw_from_quaternion_pure_yaw(psi):
    assert math.isclose(ak.yaw_from_px4_quaternion(q_from_yaw(psi)), psi, abs_tol=1e-12)


def test_yaw_from_quaternion_with_roll_pitch():
    """ZYX Euler: build q = qz(yaw) * qy(pitch) * qx(roll) and recover yaw."""
    roll, pitch, yaw = 0.3, -0.2, 1.1

    def qmul(a, b):
        w1, x1, y1, z1 = a
        w2, x2, y2, z2 = b
        return (w1*w2 - x1*x2 - y1*y2 - z1*z2,
                w1*x2 + x1*w2 + y1*z2 - z1*y2,
                w1*y2 - x1*z2 + y1*w2 + z1*x2,
                w1*z2 + x1*y2 - y1*x2 + z1*w2)

    qx = (math.cos(roll / 2), math.sin(roll / 2), 0.0, 0.0)
    qy = (math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0)
    qz = q_from_yaw(yaw)
    q = qmul(qz, qmul(qy, qx))
    assert math.isclose(ak.yaw_from_px4_quaternion(q), yaw, abs_tol=1e-12)


# --------------------------------------------------------------------- mapping to PX4
def px4_yaw_rate_from_setpoint(v_ned, psi, yaw_p):
    """Re-implementation of what PX4 does with our vector (AckermannVelControl + AttControl)."""
    psi_sp = math.atan2(v_ned[1], v_ned[0])
    return yaw_p * ak.wrap_pi(psi_sp - psi)


@pytest.mark.parametrize("psi", [0.0, 1.0, -2.5, 3.1])
@pytest.mark.parametrize("omega_ned", [0.0, 0.4, -0.4])
def test_velocity_setpoint_reproduces_requested_yaw_rate(psi, omega_ned):
    v, yaw_p = 0.6, 3.0
    v_ned = ak.velocity_setpoint_ned(v, omega_ned, psi, yaw_p)
    assert math.isclose(math.hypot(v_ned[0], v_ned[1]), v)          # speed magnitude preserved
    assert v_ned[2] == 0.0
    assert math.isclose(px4_yaw_rate_from_setpoint(v_ned, psi, yaw_p), omega_ned, abs_tol=1e-12)


def test_velocity_setpoint_straight_keeps_heading():
    v_ned = ak.velocity_setpoint_ned(1.0, 0.0, 0.7, 3.0)
    assert math.isclose(math.atan2(v_ned[1], v_ned[0]), 0.7)


def test_velocity_setpoint_rejects_bad_gain():
    with pytest.raises(ValueError):
        ak.velocity_setpoint_ned(1.0, 0.0, 0.0, 0.0)


def test_left_turn_in_ros_is_negative_yaw_rate_in_ned():
    """Full chain sign check: ROS +omega (left) must make PX4 turn left (yaw decreases in NED)."""
    psi = 0.0
    omega_ros = 0.3
    v_ned = ak.velocity_setpoint_ned(0.5, ak.ros_yaw_rate_to_ned(omega_ros), psi, 3.0)
    assert px4_yaw_rate_from_setpoint(v_ned, psi, 3.0) < 0.0      # negative NED yaw rate = CCW = left
    assert v_ned[1] < 0.0                                          # heading rotated towards West


def test_stop_setpoint():
    assert ak.stop_setpoint_ned() == (0.0, 0.0, 0.0)
