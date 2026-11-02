"""GNSS の緯度経度 <-> map(=点群座標, 公式原点中心の局所 ENU) の変換(純 Python、ROS 非依存)。

rsf_3d_localization (src/rsf_3d_localization_node.cpp) の式と同一:
  N = a/sqrt(1-e2 sin^2 phi0) * cos phi0,  M = a(1-e2)/(1-e2 sin^2 phi0)^1.5   (GRS80)
  x = (lon-lon0) * pi/180 * N,  y = (lat-lat0) * pi/180 * M,  z = altitude - altitude0
エミュレータはこの逆を使う: lat = lat0 + y/M*180/pi,  lon = lon0 + x/N*180/pi
"""
import math

A_GRS80 = 6378137.0
F_GRS80 = 1.0 / 298.257222101
E2_GRS80 = F_GRS80 * (2.0 - F_GRS80)


def radii(lat0_deg):
    """(N, M): 経度方向・緯度方向の 1rad あたりの長さ[m]。localization の gnss_n_, gnss_m_ と同じ。"""
    s = math.sin(math.radians(lat0_deg))
    n = A_GRS80 / math.sqrt(1.0 - E2_GRS80 * s * s) * math.cos(math.radians(lat0_deg))
    m = A_GRS80 * (1.0 - E2_GRS80) / (1.0 - E2_GRS80 * s * s) ** 1.5
    return n, m


def xy_to_latlon(x, y, lat0, lon0):
    n, m = radii(lat0)
    return lat0 + math.degrees(y / m), lon0 + math.degrees(x / n)


def latlon_to_xy(lat, lon, lat0, lon0):
    n, m = radii(lat0)
    return (lon - lon0) * math.pi / 180.0 * n, (lat - lat0) * math.pi / 180.0 * m


def altitude_from_z(z, datum_alt):
    return z + datum_alt
