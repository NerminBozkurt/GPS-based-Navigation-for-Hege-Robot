import math

from hege_evaluation.geodesy import EARTH_MEAN_RADIUS_M, geodetic_to_enu


def test_origin_maps_to_zero():
    assert geodetic_to_enu(52.466, 12.958, 35.0, 52.466, 12.958, 35.0) == (0.0, 0.0, 0.0)


def test_small_north_and_east_displacements():
    lat0, lon0 = 52.466, 12.958
    north_one_m_deg = math.degrees(1.0 / EARTH_MEAN_RADIUS_M)
    east_one_m_deg = math.degrees(1.0 / (EARTH_MEAN_RADIUS_M * math.cos(math.radians(lat0))))

    east, north, up = geodetic_to_enu(lat0 + north_one_m_deg,
                                      lon0 + east_one_m_deg, 37.5,
                                      lat0, lon0, 35.0)
    assert math.isclose(east, 1.0, abs_tol=1.0e-9)
    assert math.isclose(north, 1.0, abs_tol=1.0e-9)
    assert math.isclose(up, 2.5, abs_tol=1.0e-12)
