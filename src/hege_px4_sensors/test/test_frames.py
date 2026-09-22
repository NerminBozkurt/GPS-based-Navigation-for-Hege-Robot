"""Unit tests for hege_px4_sensors.frames -- every conversion is checked against
explicit rotation matrices built with numpy, so a wrong sign cannot hide."""

import math

import numpy as np
import pytest

from hege_px4_sensors import frames as fr

R_ENU_NED = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)
R_FRD_FLU = np.diag([1.0, -1.0, -1.0])


def rotmat(q):
    """Rotation matrix of unit quaternion (w,x,y,z)."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def random_unit_quat(rng):
    q = rng.normal(size=4)
    return tuple(q / np.linalg.norm(q))


RNG = np.random.default_rng(42)
QUATS = [random_unit_quat(RNG) for _ in range(10)]
VECS = [tuple(RNG.normal(size=3)) for _ in range(10)]


# ------------------------------------------------------------------ fixed rotations
def test_fixed_quaternions_match_matrices():
    assert np.allclose(rotmat(fr.Q_ENU_NED), R_ENU_NED)
    assert np.allclose(rotmat(fr.Q_FRD_FLU), R_FRD_FLU)
    # both are involutions
    assert np.allclose(R_ENU_NED @ R_ENU_NED, np.eye(3))
    assert np.allclose(R_FRD_FLU @ R_FRD_FLU, np.eye(3))


@pytest.mark.parametrize("v", VECS)
def test_vector_helpers(v):
    assert np.allclose(fr.ned_to_enu(v), R_ENU_NED @ np.array(v))
    assert np.allclose(fr.frd_to_flu(v), R_FRD_FLU @ np.array(v))


def test_ned_to_enu_examples():
    assert fr.ned_to_enu((1.0, 0.0, 0.0)) == (0.0, 1.0, 0.0)     # North -> +y
    assert fr.ned_to_enu((0.0, 1.0, 0.0)) == (1.0, 0.0, 0.0)     # East  -> +x
    assert fr.ned_to_enu((0.0, 0.0, 1.0)) == (0.0, 0.0, -1.0)    # Down  -> -z


# ------------------------------------------------------------------ quaternion algebra
@pytest.mark.parametrize("a", QUATS[:5])
@pytest.mark.parametrize("b", QUATS[5:])
def test_quat_multiply_composes_rotations(a, b):
    assert np.allclose(rotmat(fr.quat_multiply(a, b)), rotmat(a) @ rotmat(b))


@pytest.mark.parametrize("q", QUATS)
@pytest.mark.parametrize("v", VECS[:3])
def test_rotate_vector(q, v):
    assert np.allclose(fr.rotate_vector(q, v), rotmat(q) @ np.array(v))
    assert np.allclose(fr.rotate_vector(fr.quat_conjugate(q), v), rotmat(q).T @ np.array(v))


# ------------------------------------------------------------------ attitude conversion
@pytest.mark.parametrize("q", QUATS)
def test_px4_attitude_to_ros_matrix_identity(q):
    """R(q_ros) must equal R_ENU<-NED R(q) R_FRD<-FLU."""
    q_ros = fr.px4_attitude_to_ros(q)
    assert np.allclose(rotmat(q_ros), R_ENU_NED @ rotmat(q) @ R_FRD_FLU)
    assert math.isclose(sum(c * c for c in q_ros), 1.0)


def test_attitude_yaw_sign_and_offset():
    """
    Heading North in PX4 (yaw_ned = 0) is +90 deg yaw in ENU (x axis is East).
    Heading East  in PX4 (yaw_ned = +90 deg, clockwise) is yaw_enu = 0.
    """
    def q_yaw_ned(psi):
        return (math.cos(psi / 2), 0.0, 0.0, math.sin(psi / 2))

    q_ros = fr.px4_attitude_to_ros(q_yaw_ned(0.0))
    assert math.isclose(fr.yaw_enu_from_ros_quaternion(q_ros), math.pi / 2, abs_tol=1e-12)

    q_ros = fr.px4_attitude_to_ros(q_yaw_ned(math.pi / 2))
    assert math.isclose(fr.yaw_enu_from_ros_quaternion(q_ros), 0.0, abs_tol=1e-12)

    # general rule: yaw_enu = pi/2 - yaw_ned
    for psi in (0.3, -1.0, 2.0):
        q_ros = fr.px4_attitude_to_ros(q_yaw_ned(psi))
        expected = math.atan2(math.sin(math.pi / 2 - psi), math.cos(math.pi / 2 - psi))
        assert math.isclose(fr.yaw_enu_from_ros_quaternion(q_ros), expected, abs_tol=1e-12)


def test_attitude_forward_axis_maps_correctly():
    """Body x (forward) is the same physical axis in FRD and FLU; check it lands on
    the same world direction after conversion."""
    for q in QUATS:
        fwd_ned = fr.rotate_vector(q, (1.0, 0.0, 0.0))
        fwd_enu = fr.rotate_vector(fr.px4_attitude_to_ros(q), (1.0, 0.0, 0.0))
        assert np.allclose(fwd_enu, fr.ned_to_enu(fwd_ned))


# ------------------------------------------------------------------ velocity
@pytest.mark.parametrize("q", QUATS[:4])
def test_body_velocity_from_world_ned(q):
    v_body_frd = (1.2, -0.3, 0.1)
    v_ned = fr.rotate_vector(q, v_body_frd)                 # what PX4 would report in NED
    v_flu = fr.body_velocity_flu_from_px4(v_ned, 1, q)
    assert np.allclose(v_flu, fr.frd_to_flu(v_body_frd))


def test_arbitrary_heading_frd_world_velocity_is_rejected():
    q = (1.0, 0.0, 0.0, 0.0)
    assert fr.body_velocity_flu_from_px4((1.0, 2.0, 3.0), 2, q) is None
    assert fr.body_velocity_variance_flu_from_px4((1.0, 2.0, 3.0), 2, q) is None


def test_body_velocity_from_body_frd():
    assert fr.body_velocity_flu_from_px4((1.0, 0.2, -0.1), 3, (1, 0, 0, 0)) == (1.0, -0.2, 0.1)


@pytest.mark.parametrize("q", QUATS[:4])
def test_body_velocity_variance_from_reference_frame(q):
    variance = (1.0, 4.0, 9.0)
    r = rotmat(q)
    expected = np.diag(r.T @ np.diag(variance) @ r)
    got = fr.body_velocity_variance_flu_from_px4(variance, 1, q)
    assert np.allclose(got, expected)


def test_body_velocity_variance_body_frame_sign_flip_does_not_change_variance():
    got = fr.body_velocity_variance_flu_from_px4((1.0, 2.0, 3.0), 3, (1, 0, 0, 0))
    assert got == (1.0, 2.0, 3.0)


def test_body_velocity_unknown_frame():
    assert fr.body_velocity_flu_from_px4((1.0, 0.0, 0.0), 0, (1, 0, 0, 0)) is None


def test_is_finite():
    assert fr.is_finite((1.0, 2.0))
    assert not fr.is_finite((1.0, float("nan")))
