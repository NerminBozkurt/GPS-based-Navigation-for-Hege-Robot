"""Small-distance WGS-84 approximations used by simulation evaluation."""

from __future__ import annotations

import math


EARTH_MEAN_RADIUS_M = 6_371_008.8


def geodetic_to_enu(latitude_deg: float, longitude_deg: float, altitude_m: float,
                    origin_latitude_deg: float, origin_longitude_deg: float,
                    origin_altitude_m: float) -> tuple[float, float, float]:
    """Convert a nearby WGS-84 coordinate to local East, North, Up metres."""
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    latitude_0 = math.radians(origin_latitude_deg)
    longitude_0 = math.radians(origin_longitude_deg)

    east = EARTH_MEAN_RADIUS_M * math.cos(latitude_0) * (longitude - longitude_0)
    north = EARTH_MEAN_RADIUS_M * (latitude - latitude_0)
    up = altitude_m - origin_altitude_m
    return east, north, up
