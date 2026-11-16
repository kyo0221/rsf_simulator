#ifndef GNSS_GEO_HPP_
#define GNSS_GEO_HPP_

#include <cmath>
#include <utility>

namespace rsf_simulator
{

constexpr double kGrs80A = 6378137.0;
constexpr double kGrs80F = 1.0 / 298.257222101;
constexpr double kGrs80E2 = kGrs80F * (2.0 - kGrs80F);
constexpr double kDegToRad = M_PI / 180.0;

inline std::pair<double, double> radii(double lat0_deg)
{
  const double s = std::sin(lat0_deg * kDegToRad);
  const double w = 1.0 - kGrs80E2 * s * s;
  const double n = kGrs80A / std::sqrt(w) * std::cos(lat0_deg * kDegToRad);
  const double m = kGrs80A * (1.0 - kGrs80E2) / std::pow(w, 1.5);
  return {n, m};
}

inline std::pair<double, double> xyToLatLon(double x, double y, double lat0, double lon0)
{
  const auto [n, m] = radii(lat0);
  return {lat0 + y / m / kDegToRad, lon0 + x / n / kDegToRad};
}

inline std::pair<double, double> latLonToXy(double lat, double lon, double lat0, double lon0)
{
  const auto [n, m] = radii(lat0);
  return {(lon - lon0) * kDegToRad * n, (lat - lat0) * kDegToRad * m};
}

inline double altitudeFromZ(double z, double datum_alt)
{
  return z + datum_alt;
}

}  // namespace rsf_simulator

#endif  // GNSS_GEO_HPP_
