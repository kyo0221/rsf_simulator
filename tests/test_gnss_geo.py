import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import rsf_gnss_geo as g  # noqa: E402

LAT0, LON0, ALT0 = 36.08288044139598, 140.07654638977903, 41.1224


def loc_formula(lat, lon, lat0, lon0):
    """rsf_3d_localization_node.cpp の on_gnss / init の式をそのまま再現(別実装)。"""
    a, f = 6378137.0, 1.0 / 298.257222101
    e2 = f * (2.0 - f)
    s = math.sin(math.radians(lat0))
    n = a / math.sqrt(1.0 - e2 * s * s) * math.cos(math.radians(lat0))
    m = a * (1.0 - e2) / (1.0 - e2 * s * s) ** 1.5
    return (lon - lon0) * math.pi / 180.0 * n, (lat - lat0) * math.pi / 180.0 * m


def test_origin_is_datum():
    lat, lon = g.xy_to_latlon(0.0, 0.0, LAT0, LON0)
    assert lat == pytest.approx(LAT0, abs=1e-15) and lon == pytest.approx(LON0, abs=1e-15)


def test_roundtrip_with_localization_formula_within_1mm():
    rng = np.random.default_rng(0)
    for _ in range(200):
        x, y = rng.uniform(-450, 600), rng.uniform(-500, 300)
        lat, lon = g.xy_to_latlon(x, y, LAT0, LON0)
        bx, by = loc_formula(lat, lon, LAT0, LON0)
        assert abs(bx - x) < 1e-3 and abs(by - y) < 1e-3


def test_latlon_to_xy_inverse():
    lat, lon = g.xy_to_latlon(123.4, -56.7, LAT0, LON0)
    x, y = g.latlon_to_xy(lat, lon, LAT0, LON0)
    assert abs(x - 123.4) < 1e-6 and abs(y + 56.7) < 1e-6


def test_altitude_adds_datum_altitude():
    assert g.altitude_from_z(0.5, ALT0) == pytest.approx(41.6224)
