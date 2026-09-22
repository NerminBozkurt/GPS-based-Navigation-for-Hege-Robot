"""
Frame conversions PX4 (NED world, FRD body)  ->  ROS (ENU world, FLU body).
Pure Python, no ROS imports, fully unit-tested.

Quaternions are tuples (w, x, y, z), Hamilton convention, unit norm.
R(q) denotes the rotation matrix of q, so that  v_world = R(q) v_body.

Two fixed rotations do all the work:

  R_ENU<-NED = [[0, 1, 0],          swap North/East, flip Down -> Up.
                [1, 0, 0],          This is a 180 deg rotation about the axis (1,1,0)/sqrt(2),
                [0, 0,-1]]          quaternion Q_ENU_NED = (0, 1/sqrt2, 1/sqrt2, 0).
                                    It is its own inverse (R^2 = I).

  R_FRD<-FLU = diag(1, -1, -1)      180 deg about x, quaternion Q_FRD_FLU = (0, 1, 0, 0).
                                    Also its own inverse, so R_FLU<-FRD is the same matrix.
"""

from __future__ import annotations

import math

Quat = tuple[float, float, float, float]
Vec3 = tuple[float, float, float]

_S = 1.0 / math.sqrt(2.0)
Q_ENU_NED: Quat = (0.0, _S, _S, 0.0)   # rotates NED vectors into ENU (and ENU into NED)
Q_FRD_FLU: Quat = (0.0, 1.0, 0.0, 0.0)  # rotates FLU vectors into FRD (and FRD into FLU)


# --------------------------------------------------------------------------- quaternions
def quat_multiply(a: Quat, b: Quat) -> Quat:
    """Hamilton product a*b.  R(a*b) = R(a) R(b)  (apply b first, then a)."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def quat_conjugate(q: Quat) -> Quat:
    """Inverse rotation for a unit quaternion."""
    w, x, y, z = q
    return (w, -x, -y, -z)


def quat_normalize(q: Quat) -> Quat:
    n = math.sqrt(sum(c * c for c in q))
    if n == 0.0:
        raise ValueError("zero quaternion")
    return tuple(c / n for c in q)  # type: ignore[return-value]


def rotate_vector(q: Quat, v: Vec3) -> Vec3:
    """v' = R(q) v, computed as q * (0, v) * q^-1."""
    p = (0.0, v[0], v[1], v[2])
    r = quat_multiply(quat_multiply(q, p), quat_conjugate(q))
    return (r[1], r[2], r[3])


def rotation_matrix(q: Quat) -> tuple[Vec3, Vec3, Vec3]:
    """Rotation matrix R(q), with body axes in the rows/columns convention used here."""
    w, x, y, z = quat_normalize(q)
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


# --------------------------------------------------------------------------- world frame
def ned_to_enu(v: Vec3) -> Vec3:
    """(N, E, D) -> (E, N, -D).  Same for positions and velocities."""
    return (v[1], v[0], -v[2])


# --------------------------------------------------------------------------- body frame
def frd_to_flu(v: Vec3) -> Vec3:
    """(F, R, D) -> (F, -R, -D) = (F, L, U).  Same for accelerations and angular rates."""
    return (v[0], -v[1], -v[2])


# --------------------------------------------------------------------------- attitude
def px4_attitude_to_ros(q_ned_frd: Quat) -> Quat:
    """
    PX4 attitude q (FRD body -> NED world)  ->  ROS attitude (FLU body -> ENU world).

        R_ros = R_ENU<-NED * R(q) * R_FRD<-FLU
        q_ros = Q_ENU_NED  *  q   *  Q_FRD_FLU

    Reading right to left: take a FLU vector, express it in FRD, rotate it into
    NED with the PX4 attitude, then express that NED vector in ENU.
    """
    return quat_normalize(quat_multiply(quat_multiply(Q_ENU_NED, q_ned_frd), Q_FRD_FLU))


def body_velocity_flu_from_px4(velocity: Vec3, velocity_frame: int, q_ned_frd: Quat) -> Vec3 | None:
    """
    VehicleOdometry.velocity -> body-frame FLU velocity for nav_msgs/Odometry.twist.

    velocity_frame (px4_msgs/VehicleOdometry):
        1 = VELOCITY_FRAME_NED       world-fixed  -> rotate into body with R(q)^T, then FRD->FLU
        2 = VELOCITY_FRAME_FRD       world-fixed with arbitrary heading -> rejected: its reference
                                      heading is not encoded in VehicleOdometry, so q_ned_frd is
                                      insufficient for a correct transform
        3 = VELOCITY_FRAME_BODY_FRD  already body -> only FRD->FLU
        0 = unknown                  -> None
    """
    if velocity_frame == 1:
        v_body_frd = rotate_vector(quat_conjugate(q_ned_frd), velocity)   # R^T = R(q^-1)
        return frd_to_flu(v_body_frd)
    if velocity_frame == 3:
        return frd_to_flu(velocity)
    return None


def body_velocity_variance_flu_from_px4(variance: Vec3, velocity_frame: int,
                                        q_reference_frd: Quat) -> Vec3 | None:
    """Transform a diagonal velocity covariance into the body FLU frame.

    PX4 only provides the three diagonal entries. For a world/reference-frame
    velocity, diag(R.T @ diag(variance) @ R) is the best diagonal-only result.
    FRD->FLU changes signs but therefore does not change variances.
    """
    if velocity_frame == 1:
        r = rotation_matrix(q_reference_frd)
        return tuple(sum(float(variance[i]) * r[i][j] ** 2 for i in range(3))
                     for j in range(3))  # type: ignore[return-value]
    if velocity_frame == 3:
        return tuple(float(v) for v in variance)  # type: ignore[return-value]
    return None


def yaw_enu_from_ros_quaternion(q: Quat) -> float:
    """Yaw about ENU Up axis (CCW positive) from a ROS quaternion; handy for logging."""
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def is_finite(values) -> bool:
    return all(math.isfinite(float(v)) for v in values)
